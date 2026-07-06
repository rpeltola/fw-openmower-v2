/*
 * OpenMower V2 Firmware
 * Part of the OpenMower V2 Firmware (https://github.com/xtech/fw-openmower-v2)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "i2s6_audio.hpp"

#include <ch.h>
#include <etl/atomic.h>
#include <hal.h>
#include <ulog.h>

namespace xbot::driver::audio {

namespace {

// ============================================================================
// Clock derivation (RM0468 Ch. 51 "SPI2S", §51.6.9 SPI_I2SCFGR)
// ============================================================================
//
// SPI6/I2S6 kernel clock: boards/XCORE/mcuconf.h sets STM32_SPI6SEL = STM32_SPI6SEL_PCLK4, i.e.
// SPI6's kernel clock is APB4 (PCLK4, the D3-domain peripheral clock). From this board's actual
// clock tree (also in mcuconf.h):
//   HSE = 25 MHz (board.h STM32_HSECLK)
//   PLL1: /DIVM1=2, x DIVN1=44, /DIVP1=1  ->  PLL1_P = sys_ck = 25e6/2*44/1 = 550 MHz
//   D1CPRE=/1                              ->  rcc_c_ck        = 550 MHz
//   D1HPRE=/2                              ->  HCLK            = 275 MHz
//   D3PPRE4=/2                             ->  PCLK4           = 137.5 MHz
constexpr uint32_t kSpi6KernelClockHz = 137'500'000;

// I2SCFGR.CHLEN=0 selects a 16-bit channel length (matches DATLEN=0, 16-bit data - no padding).
// The wire format is always stereo (L+R slots) even though the audio source is mono; Play()
// duplicates every mono sample onto both slots (see RefillHalf()).
constexpr uint32_t kChannelLengthBits = 16;
constexpr uint32_t kChannelsPerFrame = 2;

// With MCKOE=0 (no master clock output - matches the MAX98357A, which needs none):
//   Fs = I2SxCLK / (kChannelLengthBits * kChannelsPerFrame * ((2 * I2SDIV) + ODD))
// Solving for (2*I2SDIV+ODD) closest to 16 kHz:
//   137'500'000 / (16 * 2 * 16000) = 268.55  ->  269 (odd, I2SDIV=134, ODD=1) is closer than 268
//   I2SDIV=134, ODD=1  ->  Fs = 137'500'000 / (16*2*269) = 15975.6 Hz  (-0.15%)
//   I2SDIV=134, ODD=0  ->  Fs = 137'500'000 / (16*2*268) = 16033.4 Hz  (+0.21%)
// 269/ODD=1 is chosen for the smaller error. A ~0.15% rate error is an inaudible pitch shift,
// but it is NOT exactly 16 kHz.
//
// TODO(hw bring-up): if an exact 16 kHz (or a cleanly-divisible rate) is ever required, retarget
// STM32_SPI6SEL to a PLL2/PLL3 "P" audio clock programmed for an exact multiple of 16 kHz
// (mirroring how STM32_SAI1SEL is already routed off PLL1_Q for the SAI1 audio peripheral in
// mcuconf.h), rather than silently shipping a wrong rate.
constexpr uint32_t kI2SDiv = 134;
constexpr uint32_t kOdd = 1;
static_assert(kI2SDiv >= 2 && kI2SDiv <= 0xFFU, "I2SDIV out of the 8-bit register field's range");

constexpr uint32_t kActualSampleRateHz =
    kSpi6KernelClockHz / (kChannelLengthBits * kChannelsPerFrame * (2 * kI2SDiv + kOdd));
static_assert(kActualSampleRateHz > 15900 && kActualSampleRateHz < 16100,
              "computed I2S rate has drifted away from ~16 kHz - recompute I2SDIV/ODD if the "
              "clock tree (mcuconf.h) changed");

// ============================================================================
// DMA buffer (must live in SRAM4 / D3 domain - see below) and playback state
// ============================================================================

// Mono samples per DMA half-buffer (32 ms at kActualSampleRateHz) - deliberately generous to
// give the feeder thread (which may block on a LittleFS/flash read to refill it) a wide margin
// before the other half runs out.
constexpr size_t kSamplesPerHalf = 512;
constexpr size_t kWordsPerHalf = kSamplesPerHalf * kChannelsPerFrame;  // interleaved L,R
constexpr size_t kTotalWords = kWordsPerHalf * 2;                      // two halves

// DMA target buffer. BDMA1 (the only DMA controller wired to SPI6 via DMAMUX2, since SPI6 is in
// the D3 power domain) can only address D3-domain RAM - SRAM4 at 0x38000000 - not AXI SRAM,
// DTCM or AHB SRAM1-3. The `.ram4` section is a stock ChibiOS linker feature
// (ext/ChibiOS_21.11.3/.../rules_memory.ld) already mapped onto this board's SRAM4 region in
// boards/XCORE/STM32H723xG_ITCM64k.ld, and already used elsewhere in this firmware (see
// CC_SECTION(".ram4") on board_info/carrier_board_info in globals.hpp) - so no linker changes
// are needed here, just the same attribute.
CC_SECTION(".ram4") int16_t dma_buffer_[kTotalWords];

const stm32_bdma_stream_t* dma_stream_ = nullptr;
SampleSource source_ = nullptr;
etl::atomic<uint16_t> volume_{256};  // unity gain by default; SetVolume() overrides
etl::atomic<bool> playing_{false};

thread_t* feeder_thread_ = nullptr;
THD_WORKING_AREA(feeder_wa_, 768);

constexpr eventmask_t kEvtRefillHalf0 = 1u << 0;
constexpr eventmask_t kEvtRefillHalf1 = 1u << 1;

inline int16_t ApplyVolume(int16_t sample) {
  return static_cast<int16_t>((static_cast<int32_t>(sample) * volume_.load()) / 256);
}

// Disables SPI6 + the BDMA stream and forgets the current source. Shared by the public Stop()
// and by RefillHalf() (which needs the same action once a fully-silent half confirms the
// source has drained, but can't call the public Stop() without name-clashing with itself).
void DisableHardware() {
  SPI6->CR1 &= ~SPI_CR1_SPE;
  if (dma_stream_ != nullptr) {
    bdmaStreamDisable(dma_stream_);
  }
  source_ = nullptr;
  playing_.store(false);
}

// Refills DMA half `half` (0 or 1) from source_, applying software volume. If source_ returns
// fewer than kSamplesPerHalf samples, the rest of this half is silence-padded and source_ is
// cleared (end-of-stream latched). If source_ was already cleared on a previous call (this half
// comes back fully silent) and we are actually playing, one whole buffer's worth of trailing
// silence has now been queued - safe to mute.
void RefillHalf(size_t half) {
  int16_t mono[kSamplesPerHalf];
  size_t n = 0;
  const bool was_active = (source_ != nullptr);
  if (was_active) {
    n = source_(mono, kSamplesPerHalf);
  }

  int16_t* half_base = dma_buffer_ + half * kWordsPerHalf;
  for (size_t i = 0; i < kSamplesPerHalf; i++) {
    const int16_t sample = (i < n) ? ApplyVolume(mono[i]) : 0;
    half_base[2 * i] = sample;      // Left
    half_base[2 * i + 1] = sample;  // Right (MAX98357A's hardwired SD_MODE strap decides
                                    // whether it plays L, R or their average; L==R makes the
                                    // result correct regardless of which).
  }

  if (was_active && n < kSamplesPerHalf) {
    source_ = nullptr;  // end-of-stream: stop pulling further data
  } else if (!was_active && playing_.load()) {
    DisableHardware();
  }
}

// BDMA1 stream ISR: only ever signals the feeder thread, never touches lfs/hardware itself.
// STM32_BDMA_ISR_TEIF (transfer error) is intentionally left unhandled here - it would indicate
// a BDMA/DMAMUX misconfiguration (e.g. a wrong request id) that on-hardware bring-up must catch
// on the bench, not something recoverable at runtime.
void BdmaIsr(void* p, uint32_t flags) {
  (void)p;
  chSysLockFromISR();
  if (feeder_thread_ != nullptr) {
    if (flags & STM32_BDMA_ISR_HTIF) chEvtSignalI(feeder_thread_, kEvtRefillHalf0);
    if (flags & STM32_BDMA_ISR_TCIF) chEvtSignalI(feeder_thread_, kEvtRefillHalf1);
  }
  chSysUnlockFromISR();
}

void FeederThreadFunc(void*) {
  while (true) {
    eventmask_t evt = chEvtWaitAny(kEvtRefillHalf0 | kEvtRefillHalf1);
    if (evt & kEvtRefillHalf0) RefillHalf(0);
    if (evt & kEvtRefillHalf1) RefillHalf(1);
  }
}

}  // namespace

bool Init() {
  if (dma_stream_ != nullptr) {
    return true;  // already initialized
  }

  // RM0468 §51.5: enable the SPI6 kernel + APB4 (RCC) clock before touching any SPI6 register.
  rccEnableSPI6(true);

  // ---- I2SCFGR: master TX, Philips standard, 16-bit data in a 16-bit channel slot, no MCLK ----
  // RM0468 §51.6.9 "SPI_I2SCFGR". I2SMOD=1 switches this SPI2S instance into I2S function.
  // I2SCFG field = 0b10 (I2SCFG_1 alone) is "Master - transmit" - the same encoding ChibiOS's own
  // I2Sv2 driver uses for SPI1-3 (see STM32_I2S*_CFGR_CFG in hal_i2s_lld.c), confirming the field
  // layout is unchanged on H7. I2SSTD=00 (Philips), DATLEN=00 (16-bit), CHLEN=0 (16-bit channel),
  // CKPOL=0 (clock idles low), MCKOE=0 (no master clock - unsupported/unneeded here) are all their
  // reset (zero) values, so they're simply left unset below.
  SPI6->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_1 | (kI2SDiv << SPI_I2SCFGR_I2SDIV_Pos) |
                  (kOdd != 0 ? SPI_I2SCFGR_ODD : 0);

  // ---- CFG1: enable the Tx DMA request ----
  // RM0468 §51.6.3 "SPI_CFG1". FTHLV (FIFO threshold, bits[8:5]) is left at its reset value of 0
  // = "1 data" - the DMA request stays asserted for every single 16-bit slot, which is what a
  // gapless audio stream needs (as opposed to bursting multiple slots per request).
  SPI6->CFG1 = SPI_CFG1_TXDMAEN;

  // ---- BDMA1 stream + DMAMUX2 request ----
  // SPI6 is in the D3 power domain; only BDMA1 (routed through DMAMUX2) can reach it - the
  // regular DMA1/DMA2 controllers cannot. mcuconf.h deliberately leaves STM32_SPI_USE_SPI6=FALSE
  // and its BDMA stream slots as STM32_BDMA_STREAM_ID_ANY so the (unused) ChibiOS SPI6 driver
  // never claims SPI6 or a BDMA1 stream out from under us. Passing STM32_BDMA_STREAM_ID_ANY here
  // asks the shared BDMA allocator (already initialized by hal_lld.c - STM32_BDMA_REQUIRED is
  // pulled in because the I2C4 driver, also D3-domain, needs BDMA) for whichever stream is free.
  dma_stream_ = bdmaStreamAlloc(STM32_BDMA_STREAM_ID_ANY,
                                10,  // matches STM32_SPI_SPI6_IRQ_PRIORITY in mcuconf.h
                                BdmaIsr, nullptr);
  if (dma_stream_ == nullptr) {
    ULOG_ERROR("I2S6Audio: no free BDMA1 stream available");
    return false;
  }
  bdmaSetRequestSource(dma_stream_, STM32_DMAMUX2_SPI6_TX);
  // SPI_TXDR is a 32-bit register, but for a 16-bit I2S frame it must be accessed as a halfword
  // (RM0468 §51.5.9): on this little-endian core, the register's own base address already is the
  // address of its low halfword, so the BDMA peripheral address is simply &SPI6->TXDR.
  bdmaStreamSetPeripheral(dma_stream_, &SPI6->TXDR);

  feeder_thread_ = chThdCreateStatic(feeder_wa_, sizeof(feeder_wa_), NORMALPRIO + 1, FeederThreadFunc, nullptr);
#ifdef USE_SEGGER_SYSTEMVIEW
  feeder_thread_->name = "I2S6Audio";
#endif

  return true;
}

void Play(SampleSource source) {
  if (dma_stream_ == nullptr) {
    ULOG_ERROR("I2S6Audio: Play() called before a successful Init()");
    return;
  }

  // Stop any stream already in progress first - also leaves SPI6/BDMA in a known-disabled state
  // while we re-prime the buffer below.
  Stop();

  source_ = source;
  // Pre-fill both halves synchronously, on the calling thread (not the feeder thread/ISR), so
  // the very first BDMA pass transmits real audio instead of stale/garbage buffer contents. If
  // the source is very short (drains within these two calls), RefillHalf() already silence-pads
  // the tail and clears source_; it won't call Stop() itself here since playing_ is still false
  // at this point (only set below), so the hardware still gets armed normally below and the
  // already-primed silence-padded buffer plays out before the feeder thread mutes it.
  RefillHalf(0);
  RefillHalf(1);

  bdmaStreamSetMemory0(dma_stream_, dma_buffer_);
  bdmaStreamSetTransactionSize(dma_stream_, kTotalWords);
  bdmaStreamSetMode(dma_stream_, STM32_BDMA_CR_DIR_M2P | STM32_BDMA_CR_CIRC | STM32_BDMA_CR_MINC |
                                     STM32_BDMA_CR_PSIZE_HWORD | STM32_BDMA_CR_MSIZE_HWORD |
                                     STM32_BDMA_CR_PL(1) |  // matches STM32_SPI_SPI6_DMA_PRIORITY
                                     STM32_BDMA_CR_HTIE | STM32_BDMA_CR_TCIE | STM32_BDMA_CR_TEIE);
  bdmaStreamEnable(dma_stream_);  // arms the BDMA stream, priming the SPI6 TX FIFO while SPE=0

  // RM0468 §51.5.2: SPE is the single enable bit for both plain-SPI and I2S function on the H7's
  // unified SPI2S peripheral (unlike legacy F4/F7 parts, there is no separate I2SE bit) - setting
  // it starts BCLK/WS generation.
  SPI6->CR1 |= SPI_CR1_SPE;
  playing_.store(true);
}

void Stop() {
  DisableHardware();
}

bool IsPlaying() {
  return playing_.load();
}

void SetVolume(uint16_t volume) {
  volume_.store(volume > 256 ? 256 : volume);
}

}  // namespace xbot::driver::audio
