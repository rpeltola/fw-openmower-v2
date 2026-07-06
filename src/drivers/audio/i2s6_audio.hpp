/*
 * OpenMower V2 Firmware
 * Part of the OpenMower V2 Firmware (https://github.com/xtech/fw-openmower-v2)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file i2s6_audio.hpp
 * @brief Register-level SPI6/I2S6 -> MAX98357A I2S transmit driver.
 *
 * *** REQUIRES ON-HARDWARE BRING-UP ***
 * This driver has never been exercised against real silicon or the MAX98357A amp. Every
 * register value below is derived from RM0468 (STM32H723 reference manual) and this project's
 * actual clock-tree configuration (boards/XCORE/mcuconf.h), but it must still be verified with a
 * scope/logic analyzer before it is trusted:
 *  - BCLK / WS / DOUT waveforms on PG13 / PA15 / PB5 (frequency, polarity, bit alignment).
 *  - The actual audible sample rate/pitch - see i2s6_audio.cpp, it is NOT exactly 16 kHz.
 *  - That DMAMUX2 request id 12 (STM32_DMAMUX2_SPI6_TX) really is "SPI6_TX" on this silicon
 *    revision (cross-check against RM0468's DMAMUX2 request mapping table if audio sounds wrong
 *    or doesn't play at all).
 *  - No audible glitches/underrun at track start/stop, or across the half-buffer boundary.
 *
 * ---- Why a custom driver instead of the ChibiOS I2S HAL? ----
 * The STM32H723 SPI6 peripheral supports I2S ("SPI2S", RM0468 Ch. 51), but ChibiOS 21.11.3's
 * I2S HAL (hal_i2s_lld, SPIv1/SPIv2) only supports SPI1/2/3 via DMA1/DMA2. SPI6 lives in the D3
 * power domain and can only be reached by BDMA1 (via DMAMUX2), which the stock I2S HAL has no
 * knowledge of. mcuconf.h therefore leaves STM32_SPI_USE_SPI6 FALSE (its BDMA stream slots are
 * still reserved as STM32_BDMA_STREAM_ID_ANY so nothing else grabs them), and this driver
 * programs SPI6->I2SCFGR / CFG1 / CR1 and a BDMA1 stream directly.
 *
 * ---- Signal chain / muting ----
 * The MAX98357A needs no MCLK (it derives BCLK internally) and its SD_MODE pin is hardwired
 * (no GPIO on this board), so there is no software mute line: "mute" is simply "stop generating
 * BCLK/WS" (SPI6 disabled), at which point the amp auto-sleeps.
 */

#ifndef I2S6_AUDIO_HPP
#define I2S6_AUDIO_HPP

#include <cstddef>
#include <cstdint>

namespace xbot::driver::audio {

/**
 * @brief Pull-based mono sample source used to refill the DMA buffer.
 *
 * Invoked only from this driver's own feeder thread (never from ISR context), so it is free to
 * block (e.g. on a LittleFS read). Must write up to n_samples 16-bit mono samples into dst and
 * return the number actually written. Returning fewer than n_samples signals end-of-stream: the
 * driver silence-pads the remainder, plays out one further full buffer of silence, and then
 * stops (mutes) on its own.
 */
using SampleSource = size_t (*)(int16_t* dst, size_t n_samples);

// Nominal sample rate this driver (and its DMA buffer layout) is configured for. WAV files fed
// to it are expected to declare this rate. See i2s6_audio.cpp for the actual (slightly
// different) physical rate the hardware produces and why.
inline constexpr uint32_t kSampleRateHz = 16000;

/**
 * @brief One-time hardware init: RCC, I2SCFGR/CFG1, and a BDMA1 stream + DMAMUX2 request.
 *
 * Leaves SPI6 and the BDMA stream disabled (silent) until Play() is called. Safe to call once
 * from platform/service init; calling it again is a harmless no-op.
 * @return false if a BDMA1 stream could not be allocated (e.g. all streams already claimed).
 */
bool Init();

/**
 * @brief Start streaming audio pulled from source.
 *
 * Synchronously pre-fills both DMA half-buffers (in the calling thread) before arming the BDMA
 * stream and enabling SPI6, so playback starts click-free instead of transmitting stale buffer
 * contents. Safe to call while already playing (stops the current stream first).
 */
void Play(SampleSource source);

/** @brief Immediately mute (disable SPI6 + the BDMA stream) and forget the current source. */
void Stop();

/** @return true while SPI6 is actively being clocked (between Play() and exhaustion/Stop()). */
bool IsPlaying();

/** @brief Software volume applied to every sample as `sample * volume / 256`. 256 = unity gain. */
void SetVolume(uint16_t volume);

}  // namespace xbot::driver::audio

#endif  // I2S6_AUDIO_HPP
