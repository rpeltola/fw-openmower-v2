/*
 * OpenMower V2 Firmware
 * Part of the OpenMower V2 Firmware (https://github.com/xtech/fw-openmower-v2)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "tone_source.hpp"

#include <ch.h>
#include <etl/algorithm.h>

#include <cmath>
#include <cstring>
#include <drivers/audio/i2s6_audio.hpp>

namespace tone {

namespace {

constexpr uint32_t kSampleRate = xbot::driver::audio::kSampleRateHz;

// Attack/release ramp lengths inside every segment. Long enough to kill the start/stop
// click, short enough to be inaudible as a fade on even the shortest (60 ms) beep.
constexpr uint32_t kAttackSamples = kSampleRate * 4 / 1000;   // 4 ms
constexpr uint32_t kReleaseSamples = kSampleRate * 8 / 1000;  // 8 ms

// Quarter-wave would halve this but a full-cycle table keeps the lookup branch-free.
// 1024 * 2 bytes = 2 KiB in .bss, filled once on first use.
constexpr size_t kSineTableSize = 1024;
int16_t sine_table_[kSineTableSize];
bool sine_table_ready_ = false;

// Program + playback cursor. Guarded by mtx_: SetToneProgram() (AudioService thread)
// swaps the program while FeedTone() (feeder thread) renders from it.
MUTEX_DECL(mtx_);
Segment segments_[kMaxSegments];
size_t segment_count_ = 0;
uint8_t repeats_left_ = 0;
int16_t amplitude_ = 0;
size_t seg_index_ = 0;
uint32_t sample_in_seg_ = 0;
uint32_t seg_total_samples_ = 0;
uint32_t phase_ = 0;  // Q32 phase accumulator: increment = freq * 2^32 / fs

void EnterSegmentLocked(size_t index) {
  seg_index_ = index;
  sample_in_seg_ = 0;
  seg_total_samples_ = static_cast<uint32_t>(segments_[index].duration_ms) * kSampleRate / 1000;
  // Phase deliberately NOT reset between segments: a continuous accumulator keeps
  // back-to-back segments phase-coherent, which sounds cleaner on two-tone chimes.
}

}  // namespace

void SetToneProgram(const Segment* segments, size_t count, uint8_t repeats, int16_t amplitude) {
  chMtxLock(&mtx_);
  if (!sine_table_ready_) {
    for (size_t i = 0; i < kSineTableSize; i++) {
      sine_table_[i] = static_cast<int16_t>(
          32767.0f * sinf(2.0f * 3.14159265f * static_cast<float>(i) / static_cast<float>(kSineTableSize)));
    }
    sine_table_ready_ = true;
  }

  segment_count_ = etl::min(count, kMaxSegments);
  memcpy(segments_, segments, segment_count_ * sizeof(Segment));
  repeats_left_ = repeats > 0 ? repeats : 1;
  amplitude_ = amplitude;
  phase_ = 0;
  if (segment_count_ > 0) {
    EnterSegmentLocked(0);
  }
  chMtxUnlock(&mtx_);
}

size_t FeedTone(int16_t* dst, size_t n_samples) {
  chMtxLock(&mtx_);

  size_t written = 0;
  while (written < n_samples && segment_count_ > 0 && repeats_left_ > 0) {
    // Advance across segment/repeat boundaries first so a zero-length segment can't wedge
    // the loop: skip until we sit inside a segment with samples remaining.
    if (sample_in_seg_ >= seg_total_samples_) {
      if (seg_index_ + 1 < segment_count_) {
        EnterSegmentLocked(seg_index_ + 1);
      } else {
        repeats_left_--;
        if (repeats_left_ == 0) break;
        EnterSegmentLocked(0);
      }
      continue;
    }

    const Segment& seg = segments_[seg_index_];
    if (seg.freq_start_hz == 0) {
      dst[written++] = 0;
      sample_in_seg_++;
      continue;
    }

    // Linear frequency interpolation across the segment (fixed pitch when start == end).
    uint32_t freq = seg.freq_start_hz;
    if (seg.freq_end_hz != seg.freq_start_hz && seg_total_samples_ > 1) {
      const int32_t delta = static_cast<int32_t>(seg.freq_end_hz) - static_cast<int32_t>(seg.freq_start_hz);
      freq =
          static_cast<uint32_t>(static_cast<int32_t>(seg.freq_start_hz) + delta * static_cast<int32_t>(sample_in_seg_) /
                                                                              static_cast<int32_t>(seg_total_samples_));
    }
    phase_ += static_cast<uint32_t>((static_cast<uint64_t>(freq) << 32) / kSampleRate);

    int32_t sample = sine_table_[phase_ >> 22];  // top 10 bits index the 1024-entry table

    // Attack/release envelope, computed in integer sample counts.
    uint32_t env_num = 256;
    if (sample_in_seg_ < kAttackSamples) {
      env_num = 256 * sample_in_seg_ / kAttackSamples;
    }
    const uint32_t remaining = seg_total_samples_ - sample_in_seg_;
    if (remaining < kReleaseSamples) {
      env_num = etl::min(env_num, 256 * remaining / kReleaseSamples);
    }

    sample = sample * amplitude_ / 32767;
    sample = sample * static_cast<int32_t>(env_num) / 256;
    dst[written++] = static_cast<int16_t>(sample);
    sample_in_seg_++;
  }

  chMtxUnlock(&mtx_);
  return written;
}

}  // namespace tone
