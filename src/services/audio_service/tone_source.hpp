/*
 * OpenMower V2 Firmware
 * Part of the OpenMower V2 Firmware (https://github.com/xtech/fw-openmower-v2)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file tone_source.hpp
 * @brief Procedural tone generator exposed as an i2s6_audio SampleSource.
 *
 * Renders a small "program" of tone segments (fixed pitch or linear sweep, freq 0 =
 * silence/gap) with a short attack/release envelope per segment so beeps start and end
 * click-free. Programs are the built-in defaults for the UI beep sounds: any of them can
 * be overridden by a WAV in flash (see AudioService's sound resolution), in which case
 * this generator is never invoked for that name.
 *
 * Threading mirrors the file playback path in audio_service.cpp: SetToneProgram() runs on
 * the AudioService thread, FeedTone() on the I2S6 driver's feeder thread; a mutex guards
 * the program state. As with playback_mutex_ there, never call into i2s6_audio while
 * holding it.
 */

#ifndef TONE_SOURCE_HPP
#define TONE_SOURCE_HPP

#include <cstddef>
#include <cstdint>

namespace tone {

/** @brief One tone segment. freq 0 = silence. Differing start/end = linear sweep. */
struct Segment {
  uint16_t freq_start_hz;
  uint16_t freq_end_hz;
  uint16_t duration_ms;
};

inline constexpr size_t kMaxSegments = 8;

/**
 * @brief Load a new tone program (copied) and reset playback state.
 *
 * @param segments   Segments to play back-to-back.
 * @param count      Number of segments (clamped to kMaxSegments).
 * @param repeats    Total passes over the program (>= 1).
 * @param amplitude  Peak sample value (post-generator, pre-volume), e.g. 18000.
 */
void SetToneProgram(const Segment* segments, size_t count, uint8_t repeats, int16_t amplitude);

/**
 * @brief i2s6_audio SampleSource: renders the current program.
 *
 * Returns fewer than n_samples once the program (all repeats) is exhausted, which the
 * driver takes as end-of-stream.
 */
size_t FeedTone(int16_t* dst, size_t n_samples);

}  // namespace tone

#endif  // TONE_SOURCE_HPP
