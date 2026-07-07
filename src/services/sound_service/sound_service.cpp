/*
 * OpenMower V2 Firmware
 * Part of the OpenMower V2 Firmware (https://github.com/xtech/fw-openmower-v2)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "sound_service.hpp"

#include <etl/algorithm.h>
#include <ulog.h>

#include <cstdio>
#include <cstring>
#include <drivers/audio/i2s6_audio.hpp>
#include <filesystem/file.hpp>
#include <filesystem/filesystem.hpp>

#include "services.hpp"

namespace {

// ---- Minimal WAV parsing (RIFF/WAVE, "fmt " + "data" chunks; PCM mono 16-bit only) ----

#pragma pack(push, 1)
struct WavRiffHeader {
  char riff_id[4];  ///< "RIFF"
  uint32_t riff_size;
  char wave_id[4];  ///< "WAVE"
};

struct WavChunkHeader {
  char id[4];
  uint32_t size;
};

struct WavFmtChunk {
  uint16_t audio_format;  ///< 1 = PCM
  uint16_t num_channels;
  uint32_t sample_rate;
  uint32_t byte_rate;
  uint16_t block_align;
  uint16_t bits_per_sample;
};
#pragma pack(pop)

constexpr uint16_t kWavFormatPcm = 1;
constexpr uint16_t kWavChannelsMono = 1;
constexpr uint16_t kWavBitsPerSample = 16;

// Streaming playback state. A single global instance can only ever play one track at a time, so
// this - like adc1's conv_id_to_cg_ table - lives as file-scope state next to the free function
// that uses it, rather than as class members.
File playback_file_;
size_t remaining_samples_ = 0;

/**
 * @brief xbot::driver::audio::SampleSource callback: streams PCM samples out of playback_file_.
 *
 * Runs on the I2S6 driver's own feeder thread (never ISR context), so the blocking LittleFS read
 * here is safe. Returns fewer than n_samples (down to 0) once the track's data chunk is
 * exhausted, which is how the driver detects end-of-stream.
 */
size_t FeedFromFile(int16_t* dst, size_t n_samples) {
  if (!playback_file_.isOpen() || remaining_samples_ == 0) {
    if (playback_file_.isOpen()) playback_file_.close();
    return 0;
  }

  const size_t to_read = etl::min(n_samples, remaining_samples_);
  int n = playback_file_.read(dst, to_read * sizeof(int16_t));
  if (n <= 0) {
    ULOG_WARNING("SoundService: read error mid-track (result %d)", n);
    playback_file_.close();
    remaining_samples_ = 0;
    return 0;
  }

  const size_t samples_read = static_cast<size_t>(n) / sizeof(int16_t);
  remaining_samples_ -= samples_read;
  if (remaining_samples_ == 0) {
    playback_file_.close();
  }
  return samples_read;
}

}  // namespace

void SoundService::Start() {
  if (thread_ != nullptr) {
    ULOG_ERROR("SoundService: Start() called twice!");
    return;
  }

  // LittleFS serializes all access internally (filesystem.cpp's lock_flash()/unlock_flash()
  // callbacks are invoked by lfs itself on every operation), so no extra locking is needed here
  // even though filesystem_service also touches lfs from its own thread.
  int result = lfs_mkdir(&lfs, "/user");
  if (result != LFS_ERR_OK && result != LFS_ERR_EXIST) {
    ULOG_ERROR("SoundService: failed to create /user: error=%d", result);
  }
  result = lfs_mkdir(&lfs, "/user/audio");
  if (result != LFS_ERR_OK && result != LFS_ERR_EXIST) {
    ULOG_ERROR("SoundService: failed to create /user/audio: error=%d", result);
  }

  if (!xbot::driver::audio::Init()) {
    ULOG_ERROR("SoundService: I2S6Audio::Init() failed, sound disabled");
    return;
  }
  SetVolume(205);  // ~80% default (205/256)

  thread_ = chThdCreateStatic(wa_, sizeof(wa_), NORMALPRIO, ThreadHelper, this);
#ifdef USE_SEGGER_SYSTEMVIEW
  thread_->name = "SoundService";
#endif
}

void SoundService::PlayTrack(uint8_t n) {
  char path[32];
  snprintf(path, sizeof(path), "/user/audio/%u.wav", n);

  if (playback_file_.isOpen()) {
    playback_file_.close();  // guard against re-entrant calls (e.g. a very fast re-pickup)
  }
  if (playback_file_.open(path, LFS_O_RDONLY) != LFS_ERR_OK) {
    ULOG_WARNING("SoundService: cannot open %s", path);
    return;
  }

  WavRiffHeader riff{};
  if (playback_file_.read(&riff, sizeof(riff)) != static_cast<int>(sizeof(riff)) ||
      memcmp(riff.riff_id, "RIFF", 4) != 0 || memcmp(riff.wave_id, "WAVE", 4) != 0) {
    ULOG_WARNING("SoundService: %s is not a RIFF/WAVE file", path);
    playback_file_.close();
    return;
  }

  // Walk chunks until "fmt " and "data" are both found (order in the file is not assumed beyond
  // "fmt " coming before "data", which is required to validate before streaming).
  bool have_fmt = false;
  WavFmtChunk fmt{};
  uint32_t data_size = 0;
  while (true) {
    WavChunkHeader chunk{};
    if (playback_file_.read(&chunk, sizeof(chunk)) != static_cast<int>(sizeof(chunk))) {
      break;  // EOF before a "data" chunk was found
    }
    if (memcmp(chunk.id, "fmt ", 4) == 0) {
      const size_t to_read = etl::min(chunk.size, static_cast<uint32_t>(sizeof(fmt)));
      if (playback_file_.read(&fmt, to_read) != static_cast<int>(to_read)) break;
      if (chunk.size > to_read) playback_file_.seek(static_cast<lfs_soff_t>(chunk.size - to_read), LFS_SEEK_CUR);
      if (chunk.size & 1) playback_file_.seek(1, LFS_SEEK_CUR);  // RIFF chunks are word-aligned
      have_fmt = true;
    } else if (memcmp(chunk.id, "data", 4) == 0) {
      data_size = chunk.size;  // file position is now at the first PCM sample
      break;
    } else {
      playback_file_.seek(static_cast<lfs_soff_t>(chunk.size + (chunk.size & 1)), LFS_SEEK_CUR);
    }
  }

  if (!have_fmt || data_size == 0) {
    ULOG_WARNING("SoundService: %s has no fmt/data chunk", path);
    playback_file_.close();
    return;
  }
  if (fmt.audio_format != kWavFormatPcm || fmt.num_channels != kWavChannelsMono ||
      fmt.bits_per_sample != kWavBitsPerSample || fmt.sample_rate != xbot::driver::audio::kSampleRateHz) {
    ULOG_WARNING(
        "SoundService: %s format mismatch (fmt=%u ch=%u bits=%u rate=%lu), expected PCM/mono/16-bit/%lu Hz - not "
        "playing it",
        path, fmt.audio_format, fmt.num_channels, fmt.bits_per_sample, static_cast<unsigned long>(fmt.sample_rate),
        static_cast<unsigned long>(xbot::driver::audio::kSampleRateHz));
    playback_file_.close();
    return;
  }

  remaining_samples_ = data_size / sizeof(int16_t);
  xbot::driver::audio::Play(&FeedFromFile);
}

void SoundService::SetVolume(uint16_t volume) {
  xbot::driver::audio::SetVolume(volume);
}

void SoundService::PlayEvent(SoundEvent event) {
  PlayTrack(static_cast<uint8_t>(event));
}

void SoundService::ThreadHelper(void* instance) {
  static_cast<SoundService*>(instance)->ThreadFunc();
}

// TODO(v1-sound-map): the following SoundEvents are ported (numbered) but not yet triggered from
// anywhere, pending the services that know about each condition:
//   kBootGreeting/kOpenMowerInitOk/kRosInit/kRosStartupOk/kRosStopped - high_level_service
//     (ROS connection/handshake state).
//   kMapRecordStart/kAutonomousStart/kMowDoneDock - high_level_service (mission/state changes).
//   kRtkWait/kGpsPoor/kGpsModerate/kGpsGood - gps_service (fix type/quality thresholds).
//   kRain - rain sensor state (see YFCoverUI::IsRainDetected(), not wired to sound yet).
//   kImuInitFailed - imu_service init result.
// Wire each by calling sound_service.PlayEvent(SoundEvent::k...) from the owning service once
// its state-change hook is available; no stub call sites are added here to avoid guessing at
// APIs those services don't expose yet.

void SoundService::ThreadFunc() {
  constexpr uint32_t kPollIntervalMs = 50;
  // emergency_service boots with TIMEOUT_INPUTS | TIMEOUT_HIGH_LEVEL already latched (and
  // SERVICE_NOT_READY while other services spin up), so without a startup grace period every
  // power-on would immediately "sound" like an emergency. Emergencies occurring during this
  // window are simply not evaluated (and don't update last_reasons), so a real, still-active
  // emergency is correctly announced the moment the grace period ends.
  constexpr uint32_t kStartupGraceMs = 10'000;

  const systime_t boot_time = chVTGetSystemTimeX();
  uint16_t last_reasons = 0;

  while (true) {
    chThdSleepMilliseconds(kPollIntervalMs);

    if (chVTTimeElapsedSinceX(boot_time) < TIME_MS2I(kStartupGraceMs)) {
      continue;
    }

    const uint16_t reasons = emergency_service.GetEmergencyReasons();
    if (last_reasons == 0 && reasons != 0) {
      // Rising edge: some emergency reason just became active. Fires once per edge - holding an
      // emergency active (or switching between reasons without ever clearing) doesn't retrigger;
      // it only re-arms once reasons drops back to 0 below.
      if (reasons & EmergencyReason::STOP) {
        PlayEvent(SoundEvent::kEmergencyStop);
      } else if (reasons & (EmergencyReason::LIFT | EmergencyReason::LIFT_MULTIPLE)) {
        PlayEvent(SoundEvent::kEmergencyLift);
      } else {
        PlayEvent(SoundEvent::kEmergencyOther);  // COLLISION, TIMEOUT_*, or any other reason
      }
    } else if (last_reasons != 0 && reasons == 0) {
      PlayEvent(SoundEvent::kEmergencyCleared);
    }
    last_reasons = reasons;
  }
}
