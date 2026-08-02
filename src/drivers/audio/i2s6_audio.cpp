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

// Byte size of one half, i.e. exactly what cacheBufferFlush() must cover per RefillHalfLocked()
// call. Required to already be a whole number of D-cache lines (see cache.h's cacheBufferFlush()
// note) - true today (2048 B / 32 B lines) but asserted so a future kSamplesPerHalf change can't
// silently leave part of a half unflushed or spill the flush onto the other half.
constexpr size_t kHalfBufferBytes = kWordsPerHalf * sizeof(int16_t);
static_assert(kHalfBufferBytes % CACHE_LINE_SIZE == 0,
              "DMA half-buffer size must be a whole number of D-cache lines for cacheBufferFlush()");

// DMA target buffer. BDMA1 (the only DMA controller wired to SPI6 via DMAMUX2, since SPI6 is in
// the D3 power domain) can only address D3-domain RAM - SRAM4 at 0x38000000 - not AXI SRAM,
// DTCM or AHB SRAM1-3. The `.ram4` section is a stock ChibiOS linker feature
// (ext/ChibiOS_21.11.3/.../rules_memory.ld) already mapped onto this board's SRAM4 region in
// boards/XCORE/STM32H723xG_ITCM64k.ld, and already used elsewhere in this firmware (see
// CC_SECTION(".ram4") on board_info/carrier_board_info in globals.hpp) - so no linker changes
// are needed here, just the same attribute.
//
// SRAM4 is cacheable on this core (mcuconf.h leaves STM32_NOCACHE_ENABLE FALSE - the MPU's
// no-cache window covers AXI SRAM at 0x24000000, not SRAM4), and BDMA reads memory directly,
// bypassing the D-cache entirely. Without an explicit flush after every write, BDMA can read back
// stale/uninitialized SRAM4 content that never made it past the cache - total silence with an
// otherwise fully-correct pipeline. CC_ALIGN_DATA(CACHE_LINE_SIZE) plus the kHalfBufferBytes
// static_assert below (cache.h, pulled in transitively via <hal.h>) keep each half a whole
// number of cache lines at a cache-line-aligned address, which cacheBufferFlush() requires to
// avoid touching adjacent data. See RefillHalfLocked(), the only place dma_buffer_ is written.
CC_ALIGN_DATA(CACHE_LINE_SIZE) CC_SECTION(".ram4") int16_t dma_buffer_[kTotalWords];

const stm32_bdma_stream_t* dma_stream_ = nullptr;
SampleSource source_ = nullptr;
etl::atomic<uint16_t> volume_{256};  // unity gain by default; SetVolume() overrides
etl::atomic<bool> playing_{false};

thread_t* feeder_thread_ = nullptr;
THD_WORKING_AREA(feeder_wa_, 2048);

constexpr eventmask_t kEvtRefillHalf0 = 1u << 0;
constexpr eventmask_t kEvtRefillHalf1 = 1u << 1;

// Guards source_ and mono_scratch_ (below) against the feeder thread's RefillHalf() running
// concurrently with a caller's Play()/Stop(). RefillHalf() only ever runs in thread context -
// BdmaIsr() (below) merely signals events, it never calls RefillHalf() itself - so a mutex (as
// opposed to a critical section) is safe here.
MUTEX_DECL(audio_mutex_);

// Bounce buffer for RefillHalf()'s call into source_(): 1 KiB (kSamplesPerHalf * sizeof(int16_t))
// is too large to carry on a caller's stack (Play() can be reached from SoundService::ThreadFunc,
// whose whole working area is a fraction of that), so it lives here instead, serialized by
// audio_mutex_ like source_.
int16_t mono_scratch_[kSamplesPerHalf];

inline int16_t ApplyVolume(int16_t sample) {
  return static_cast<int16_t>((static_cast<int32_t>(sample) * volume_.load()) / 256);
}

// Disables SPI6 + the BDMA stream and forgets the current source. Shared by the public Stop()
// and by RefillHalfLocked() (which needs the same action once a fully-silent half confirms the
// source has drained). Assumes audio_mutex_ is already held.
//
// Just clearing SPE (no CSUSP/CSTART dance) is enough: ST's own stm32h7xx_hal_i2s.c's
// HAL_I2S_DMAStop() clears CFG1's DMA-enable bits, aborts the DMA, and calls __HAL_I2S_DISABLE()
// (CLEAR_BIT CR1, SPI_CR1_SPE) - it never touches CSUSP or waits on CSTART. That's specific to I2S
// master TX: CSUSP/"wait for CSTART to clear" is a generic-SPI abort-a-live-bus-transaction
// sequence (see ChibiOS SPIv3's spi_lld_suspend(), used for its polled-exchange abort path), not
// something I2S needs since there's no shared/addressed bus to leave in a clean state - clearing
// SPE simply stops BCLK/WS. Play()'s re-arm sequence (SPE then an unconditional CSTART) already
// matches HAL_I2S_Transmit_DMA()'s pattern regardless of whether CSTART was already set here.
void DisableHardwareLocked() {
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
// silence has now been queued - safe to mute. Assumes audio_mutex_ is already held.
void RefillHalfLocked(size_t half) {
  size_t n = 0;
  const bool was_active = (source_ != nullptr);
  if (was_active) {
    n = source_(mono_scratch_, kSamplesPerHalf);
  }

  int16_t* half_base = dma_buffer_ + half * kWordsPerHalf;
  for (size_t i = 0; i < kSamplesPerHalf; i++) {
    const int16_t sample = (i < n) ? ApplyVolume(mono_scratch_[i]) : 0;
    half_base[2 * i] = sample;      // Left
    half_base[2 * i + 1] = sample;  // Right (MAX98357A's hardwired SD_MODE strap decides
                                    // whether it plays L, R or their average; L==R makes the
                                    // result correct regardless of which).
  }
  // BDMA reads half_base directly from SRAM4, bypassing the D-cache - without this flush the
  // writes above could sit in cache indefinitely and BDMA would keep transmitting whatever was
  // there before (see the CC_ALIGN_DATA/kHalfBufferBytes comments above dma_buffer_'s definition).
  cacheBufferFlush(half_base, kHalfBufferBytes);

  if (was_active && n < kSamplesPerHalf) {
    source_ = nullptr;  // end-of-stream: stop pulling further data
  } else if (!was_active && playing_.load()) {
    DisableHardwareLocked();
  }
}

// Feeder-thread entry point: acquires audio_mutex_ before touching source_/mono_scratch_/the DMA
// state, so it can never interleave with a Play()/Stop() in progress on another thread.
void RefillHalf(size_t half) {
  chMtxLock(&audio_mutex_);
  RefillHalfLocked(half);
  chMtxUnlock(&audio_mutex_);
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
  //
  // Verified NOT an amplitude-loss bug (issue #119 follow-up): DATLEN=00/CHLEN=0 is exactly
  // ST's own I2S_DATAFORMAT_16B, defined as (0x00000000UL) in stm32h7xx_hal_i2s.h - i.e. ST's HAL
  // programs the *identical* bit pattern for a plain 16-bit stream. A 16-bit sample in a 16-bit
  // slot is transmitted MSB-first starting at the slot's own MSB (Philips protocol, RM0468
  // §51.4.9 "I2S Philips standard"), so it occupies the full slot - no padding, no truncation,
  // full 16-bit dynamic range. The classic failure this rules out is DATLEN=00 with CHLEN=1 (ST's
  // I2S_DATAFORMAT_16B_EXTENDED, which ORs in SPI_I2SCFGR_CHLEN): that packs a 16-bit sample
  // LSB-justified into a 32-bit slot, i.e. every sample plays back at 1/65536 of its true
  // amplitude unless the upper 16 bits are also written. This driver never sets CHLEN, so that
  // failure mode does not apply here.
  SPI6->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_1 | (kI2SDiv << SPI_I2SCFGR_I2SDIV_Pos) |
                  (kOdd != 0 ? SPI_I2SCFGR_ODD : 0);

  // ---- CFG1: enable the Tx DMA request ----
  // RM0468 §51.6.3 "SPI_CFG1". FTHLV (FIFO threshold, bits[8:5]) is left at its reset value of 0
  // = "1 data" - the DMA request stays asserted for every single 16-bit slot, which is what a
  // gapless audio stream needs (as opposed to bursting multiple slots per request).
  //
  // DSIZE[4:0] (bits[4:0]) is also left at its reset value of 0, which for plain SPI would be the
  // reserved/illegal "1-bit frame" encoding (DSIZE encodes bits-1, per SPIv3's own
  // spi_lld_polled_exchange(): `dsize = (CFG1 & SPI_CFG1_DSIZE_Msk) + 1`) - but this peripheral is
  // in I2S mode (I2SMOD=1 above), and ST's own stm32h7xx_hal_i2s.c never touches CFG1 at all in
  // HAL_I2S_Init(), and HAL_I2S_Transmit_DMA()/HAL_I2S_DMAStop() only ever set/clear
  // SPI_CFG1_TXDMAEN - DSIZE and FTHLV are untouched. Frame size in I2S mode is governed entirely
  // by I2SCFGR.DATLEN/CHLEN (set above); CFG1.DSIZE is a plain-SPI-only field the hardware ignores
  // once I2SMOD=1, so DSIZE=0 here is not the reserved-value bug it would be in SPI mode. Likewise
  // CR2.TSIZE (transfer size) is left at 0 = "unlimited" - ST's HAL never writes it for I2S either,
  // consistent with this being a free-running circular-DMA stream rather than a bounded transfer.
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

  // Held for the whole transition below (stop old source -> arm new one) so the feeder thread's
  // RefillHalf() can never run against a half-updated source_/DMA state - it either sees the old
  // source through to DisableHardwareLocked(), or the fully-armed new one, never a mix.
  chMtxLock(&audio_mutex_);

  // Stop any stream already in progress first - also leaves SPI6/BDMA in a known-disabled state
  // while we re-prime the buffer below.
  DisableHardwareLocked();

  source_ = source;
  // Pre-fill both halves synchronously, on the calling thread (not the feeder thread/ISR), so
  // the very first BDMA pass transmits real audio instead of stale/garbage buffer contents. If
  // the source is very short (drains within these two calls), RefillHalfLocked() already
  // silence-pads the tail and clears source_; it won't call DisableHardwareLocked() itself here
  // since playing_ is still false at this point (only set below), so the hardware still gets
  // armed normally below and the already-primed silence-padded buffer plays out before the
  // feeder thread mutes it.
  RefillHalfLocked(0);
  RefillHalfLocked(1);

  bdmaStreamSetMemory0(dma_stream_, dma_buffer_);
  bdmaStreamSetTransactionSize(dma_stream_, kTotalWords);
  bdmaStreamSetMode(dma_stream_, STM32_BDMA_CR_DIR_M2P | STM32_BDMA_CR_CIRC | STM32_BDMA_CR_MINC |
                                     STM32_BDMA_CR_PSIZE_HWORD | STM32_BDMA_CR_MSIZE_HWORD |
                                     STM32_BDMA_CR_PL(1) |  // matches STM32_SPI_SPI6_DMA_PRIORITY
                                     STM32_BDMA_CR_HTIE | STM32_BDMA_CR_TCIE | STM32_BDMA_CR_TEIE);
  bdmaStreamEnable(dma_stream_);  // arms the BDMA stream, priming the SPI6 TX FIFO while SPE=0

  // RM0468 §51.5.2: SPE is the single enable bit for both plain-SPI and I2S function on the H7's
  // unified SPI2S peripheral (unlike legacy F4/F7 parts, there is no separate I2SE bit) - setting
  // it powers up the peripheral, but on the unified SPI2S IP it does NOT by itself start BCLK/WS
  // generation in master mode. CSTART ("master transfer start", CR1 bit 9) also has to be set.
  // Confirmed against ST's own reference driver: stm32h7xx_hal_i2s.c's HAL_I2S_Transmit_DMA() does
  // `if (SPE clear) __HAL_I2S_ENABLE(hi2s);` (SET_BIT CR1, SPI_CR1_SPE) immediately followed by an
  // unconditional `SET_BIT(hi2s->Instance->CR1, SPI_CR1_CSTART);` before the transfer is considered
  // started - CSTART is not optional. In-tree corroboration for the same unified SPI2S IP: ChibiOS's
  // SPIv3 driver (ext/ChibiOS_21.11.3/os/hal/ports/STM32/LLD/SPIv3/hal_spi_v2_lld.c) sets SPE once
  // in spi_lld_configure() (`spip->spi->CR1 = SPI_CR1_MASRX | SPI_CR1_SPE;`) but only actually
  // starts a master transfer in the separate spi_lld_resume(), which does
  // `spip->spi->CR1 |= SPI_CR1_CSTART;`. Without this, SPE alone leaves the peripheral enabled but
  // idle - BCLK/WS never toggle and the amp never sees a clock, i.e. total silence independent of
  // (and in addition to) the D-cache/BDMA coherency bug fixed above.
  SPI6->CR1 |= SPI_CR1_SPE;
  SPI6->CR1 |= SPI_CR1_CSTART;
  playing_.store(true);

  chMtxUnlock(&audio_mutex_);
}

// Returns once no refill is in flight and source_ is cleared, so a caller can safely tear down
// whatever that source was reading from. Guarded like Play(): without a successful Init() the
// SPI6 kernel clock is off and touching CR1 would fault.
void Stop() {
  if (dma_stream_ == nullptr) {
    return;
  }
  chMtxLock(&audio_mutex_);
  DisableHardwareLocked();
  chMtxUnlock(&audio_mutex_);
}

bool IsPlaying() {
  return playing_.load();
}

void SetVolume(uint16_t volume) {
  volume_.store(volume > 256 ? 256 : volume);
}

}  // namespace xbot::driver::audio
