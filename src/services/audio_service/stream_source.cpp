/*
 * OpenMower V2 Firmware
 * Part of the OpenMower V2 Firmware (https://github.com/xtech/fw-openmower-v2)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "stream_source.hpp"

#include <ch.h>
#include <etl/algorithm.h>
#include <etl/atomic.h>
#include <ulog.h>

#include <cstdlib>
#include <cstring>
#include <drivers/audio/i2s6_audio.hpp>

namespace stream {

namespace {

constexpr uint32_t kSampleRate = xbot::driver::audio::kSampleRateHz;

// ~1 s of mono 16-bit samples: generous headroom over the 200 ms prebuffer target and the
// ROS sender's 50-75% fill target (see the design doc), so ordinary network jitter drains
// and refills the ring instead of tripping the drop/underrun paths. Heap-allocated on
// Open() / freed on Release() (see below) rather than a static array: this would blow
// SRAM4 (the 16 KiB region i2s6_audio's own DMA buffer already lives in, and is nearly
// full) many times over, whereas the D1 AXI heap (ram0 in the linker script, ~170 KiB
// free) can carry it only while a stream is actually open.
constexpr size_t kRingCapacitySamples = 16384;  // 32768 bytes

constexpr size_t kPrebufferSamples = kNominalFrameSamples * 5;  // 200 ms @ 640 samples/frame
constexpr size_t kUnderrunAutoStopSamples = kSampleRate / 2;    // 500 ms, at the nominal rate

// A single lost/reordered frame is ordinary network jitter and worth concealing with
// silence; a gap this large means the stream restarted (or seq can't be made sense of) -
// resync instead of manufacturing a multi-second silence.
constexpr size_t kMaxConcealFrames = 25;  // ~1 s

// Guards every field below except have_seq_/expected_seq_, which only PushFrame() (always
// the AudioService thread) touches and are never read from the feeder thread - see
// PushFrame()'s comment. Mirrors tone_source.cpp's mtx_: AudioService thread (Open/
// PushFrame/Close/BufferedSamples/PrebufferReady/FreeBytes/UnderrunEvents) vs. the I2S6
// driver's feeder thread (FeedStream). Never call into i2s6_audio while this is held - same
// one-directional lock order audio_service.cpp's playback_mutex_ documents.
MUTEX_DECL(mtx_);

int16_t* ring_ = nullptr;
size_t ring_count_ = 0;
size_t read_idx_ = 0;
size_t write_idx_ = 0;
bool closing_ = false;

uint16_t underrun_events_ = 0;
size_t underrun_run_samples_ = 0;
bool underrun_active_ = false;
etl::atomic<bool> underrun_exceeded_{false};

bool open_ = false;  // AudioService thread only (Open()/Release() are its exclusive callers)

bool have_seq_ = false;  // AudioService thread only: PushFrame() is never called concurrently
uint16_t expected_seq_ = 0;

bool drop_active_ = false;  // guarded by mtx_; edge-tracked so a full ring logs once, not per frame

// Writes n samples (silence if samples == nullptr, used for gap concealment) into the ring,
// dropping - and counting - the whole call rather than partially filling it if they don't
// fit, so a dropped frame can't corrupt the framing of the one after it. Assumes mtx_ held.
void PushSamplesLocked(const int16_t* samples, size_t n) {
  if (ring_ == nullptr || n == 0) return;
  if (ring_count_ + n > kRingCapacitySamples) {
    if (!drop_active_) {
      drop_active_ = true;  // logged once per episode, not once per dropped frame at 25 pkt/s
      ULOG_WARNING("AudioService: stream ring full, dropping frame(s) (ERR_DROPPED)");
    }
    return;  // ring full: drop (never block)
  }
  drop_active_ = false;
  for (size_t i = 0; i < n; i++) {
    ring_[write_idx_] = (samples != nullptr) ? samples[i] : 0;
    write_idx_ = (write_idx_ + 1) % kRingCapacitySamples;
  }
  ring_count_ += n;
}

}  // namespace

bool Open() {
  if (open_) return true;

  auto* ring = static_cast<int16_t*>(malloc(kRingCapacitySamples * sizeof(int16_t)));
  if (ring == nullptr) {
    ULOG_ERROR("AudioService: stream ring allocation failed (%u samples)", static_cast<unsigned>(kRingCapacitySamples));
    return false;
  }

  chMtxLock(&mtx_);
  ring_ = ring;
  ring_count_ = 0;
  read_idx_ = 0;
  write_idx_ = 0;
  closing_ = false;
  underrun_events_ = 0;
  underrun_run_samples_ = 0;
  underrun_active_ = false;
  drop_active_ = false;
  chMtxUnlock(&mtx_);

  underrun_exceeded_.store(false);
  have_seq_ = false;
  expected_seq_ = 0;
  open_ = true;
  return true;
}

void Release() {
  if (!open_) return;

  chMtxLock(&mtx_);
  int16_t* old_ring = ring_;
  ring_ = nullptr;
  ring_count_ = 0;
  read_idx_ = 0;
  write_idx_ = 0;
  closing_ = false;
  chMtxUnlock(&mtx_);

  free(old_ring);
  underrun_exceeded_.store(false);
  have_seq_ = false;
  open_ = false;
}

void PushFrame(uint16_t seq, const int16_t* samples, size_t n_samples) {
  if (!open_) return;

  // Wraparound-safe gap size: unsigned subtraction mod 2^16 is correct regardless of
  // where seq/expected_seq_ sit relative to the 0/65535 wrap, unlike a plain `>` check.
  size_t missing = 0;
  if (!have_seq_) {
    have_seq_ = true;
  } else {
    missing = static_cast<uint16_t>(seq - expected_seq_);
    if (missing > kMaxConcealFrames) {
      missing = 0;  // implausible gap (restart / out-of-order) - resync, don't conceal it
    }
  }
  expected_seq_ = static_cast<uint16_t>(seq + 1);

  chMtxLock(&mtx_);
  if (!closing_) {
    for (size_t i = 0; i < missing; i++) {
      // Concealment for a frame we never received: we don't know how many samples it actually
      // held, so fill in the nominal (design-target) frame duration as the best estimate.
      PushSamplesLocked(nullptr, kNominalFrameSamples);
    }
    PushSamplesLocked(samples, n_samples);
  }
  chMtxUnlock(&mtx_);
}

void Close() {
  if (!open_) return;
  chMtxLock(&mtx_);
  closing_ = true;
  chMtxUnlock(&mtx_);
}

size_t BufferedSamples() {
  chMtxLock(&mtx_);
  const size_t n = ring_count_;
  chMtxUnlock(&mtx_);
  return n;
}

bool PrebufferReady() {
  chMtxLock(&mtx_);
  const bool ready = ring_count_ >= kPrebufferSamples;
  chMtxUnlock(&mtx_);
  return ready;
}

uint16_t FreeBytes() {
  chMtxLock(&mtx_);
  const size_t free_samples = (ring_ != nullptr) ? (kRingCapacitySamples - ring_count_) : 0;
  chMtxUnlock(&mtx_);
  return static_cast<uint16_t>(etl::min<size_t>(free_samples * sizeof(int16_t), UINT16_MAX));
}

uint16_t UnderrunEvents() {
  chMtxLock(&mtx_);
  const uint16_t n = underrun_events_;
  chMtxUnlock(&mtx_);
  return n;
}

bool TakeUnderrunExceeded() {
  return underrun_exceeded_.exchange(false);
}

size_t FeedStream(int16_t* dst, size_t n_samples) {
  chMtxLock(&mtx_);

  if (ring_ == nullptr) {
    chMtxUnlock(&mtx_);
    return 0;
  }

  size_t popped = 0;
  while (popped < n_samples && ring_count_ > 0) {
    dst[popped++] = ring_[read_idx_];
    read_idx_ = (read_idx_ + 1) % kRingCapacitySamples;
    ring_count_--;
  }

  const size_t shortfall = n_samples - popped;
  if (shortfall > 0 && closing_ && ring_count_ == 0) {
    // Deliberate end-of-stream: Stop Stream(flush=0) was called and every buffered sample
    // has now been played. Let the driver see the short read so its own end-of-stream
    // handling (silence-pad the remainder of this half, then auto-mute) takes over, exactly
    // as it would for an exhausted file or tone program.
    chMtxUnlock(&mtx_);
    return popped;
  }

  bool exceeded = false;
  if (shortfall > 0) {
    // Ring underrun while still open: pad with silence and NEVER report a short read here -
    // that would make the driver latch end-of-stream and mute for good. Track how long the
    // underrun has run so AudioService::OnLoop() can auto-stop it past 500 ms.
    memset(dst + popped, 0, shortfall * sizeof(int16_t));
    if (!underrun_active_) {
      underrun_active_ = true;
      underrun_run_samples_ = 0;
      if (underrun_events_ < UINT16_MAX) underrun_events_++;
    }
    underrun_run_samples_ += shortfall;
    if (underrun_run_samples_ >= kUnderrunAutoStopSamples) {
      exceeded = true;
    }
    popped = n_samples;
  } else {
    underrun_active_ = false;
    underrun_run_samples_ = 0;
  }

  chMtxUnlock(&mtx_);
  if (exceeded) underrun_exceeded_.store(true);
  return popped;
}

}  // namespace stream
