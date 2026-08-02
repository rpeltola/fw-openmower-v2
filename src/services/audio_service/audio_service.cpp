/*
 * OpenMower V2 Firmware
 * Part of the OpenMower V2 Firmware (https://github.com/xtech/fw-openmower-v2)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "audio_service.hpp"

#include <etl/algorithm.h>
#include <ulog.h>

#include <cctype>
#include <cstdio>
#include <cstring>
#include <drivers/audio/i2s6_audio.hpp>
#include <filesystem/file.hpp>
#include <filesystem/filesystem.hpp>

#include "services.hpp"
#include "tone_source.hpp"

using xbot::datatypes::RpcStatus;

namespace {

constexpr uint8_t U8(AudioClass c) {
  return static_cast<uint8_t>(c);
}
constexpr uint8_t Res(AudioResult r) {
  return static_cast<uint8_t>(r);
}
constexpr bool ValidClassByte(uint8_t c) {
  return c >= U8(AudioClass::AMBIENT) && c <= U8(AudioClass::ALARM);
}

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

// Guards playback_file_ and remaining_samples_ against PlayPath() (a re-trigger, called on
// the service thread) closing/reopening the handle while the I2S6 driver's feeder thread is
// mid-FeedFromFile() on it - concurrent use of one lfs_file_t is undefined behaviour in
// littlefs. The invariant to preserve is one-directional: NEVER acquire i2s6_audio's
// audio_mutex_ while holding this one. FeedFromFile(), reached through RefillHalf(), is the only
// place the two nest, and it nests them in that order - so wrapping any of the calls below in
// audio_mutex_ would close the cycle and deadlock.
MUTEX_DECL(playback_mutex_);

/**
 * @brief xbot::driver::audio::SampleSource callback: streams PCM samples out of playback_file_.
 *
 * Runs on the I2S6 driver's own feeder thread (never ISR context), so the blocking LittleFS read
 * here is safe. Returns fewer than n_samples (down to 0) once the track's data chunk is
 * exhausted, which is how the driver detects end-of-stream.
 */
size_t FeedFromFile(int16_t* dst, size_t n_samples) {
  chMtxLock(&playback_mutex_);

  if (!playback_file_.isOpen() || remaining_samples_ == 0) {
    if (playback_file_.isOpen()) playback_file_.close();
    chMtxUnlock(&playback_mutex_);
    return 0;
  }

  const size_t to_read = etl::min(n_samples, remaining_samples_);
  int n = playback_file_.read(dst, to_read * sizeof(int16_t));
  if (n <= 0) {
    ULOG_WARNING("AudioService: read error mid-track (result %d)", n);
    playback_file_.close();
    remaining_samples_ = 0;
    chMtxUnlock(&playback_mutex_);
    return 0;
  }

  const size_t samples_read = static_cast<size_t>(n) / sizeof(int16_t);
  remaining_samples_ -= samples_read;
  if (remaining_samples_ == 0) {
    playback_file_.close();
  }
  chMtxUnlock(&playback_mutex_);
  return samples_read;
}

// Releases the handle a preempted track was streaming from. FeedFromFile() closes the file
// itself at end-of-stream, but a track cut short by something else taking over the voice never
// gets there - and once i2s6_audio's source_ points somewhere else it is never re-entered, so
// the handle would stay open indefinitely. Must be called with the stream already retired
// (audio::Stop()), so no FeedFromFile() can be in flight, and with playback_mutex_ NOT held.
void CloseStream() {
  chMtxLock(&playback_mutex_);
  if (playback_file_.isOpen()) {
    playback_file_.close();
  }
  remaining_samples_ = 0;
  chMtxUnlock(&playback_mutex_);
}

// ---- Named-sound tables ----

// Legacy V1 numbered tracks: named sound -> /user/audio/<n>.wav fallback. Keeps every WAV
// uploaded for the numbered scheme working under the new names.
struct NameAlias {
  const char* name;
  uint8_t track;
};
constexpr NameAlias kTrackAliases[] = {
    {"boot", 1},          {"emergency_stop", 8}, {"emergency_lift", 9},     {"rain", 10},
    {"mow_done", 11},     {"ros_ok", 16},        {"ros_lost", 17},          {"gps_poor", 20},
    {"gps_moderate", 21}, {"gps_good", 22},      {"emergency_cleared", 23}, {"emergency_other", 24},
};

// Built-in tone fallbacks for names with a procedural default (the UI beep set). These make
// the service fully functional with an empty flash.
struct ToneDefault {
  const char* name;
  TonePattern pattern;
};
constexpr ToneDefault kToneDefaults[] = {
    {"button_ack", TonePattern::ACK},
    {"button_error", TonePattern::ERROR},
    {"charge_connect", TonePattern::CHARGE_CONNECT},
    {"charge_disconnect", TonePattern::CHARGE_DISCONNECT},
    {"boot", TonePattern::BOOT},
};

// Beeps sit noticeably below full scale so speech WAVs (mastered near full scale) and beeps
// land at a comparable loudness. The siren is the exception: deterrence wants every bit.
constexpr int16_t kToneAmplitude = 18000;
constexpr int16_t kSirenAmplitude = 32767;

// SAFETY sounds never drop below the pre-service-era default volume (205/256); ALARM always
// plays at unity regardless of what the master volume is set to.
constexpr uint16_t kSafetyVolumeFloor = 205;

constexpr uint32_t kStartupGraceMs = 10'000;  // see CheckEmergencyEdges()
constexpr uint32_t kBootChimeDelayMs = 1'500;

// Pure builder: fills the caller's program without touching the tone generator, so callers can
// validate a request BEFORE retiring whatever is currently playing - an invalid request must
// never cost the running sound its voice.
bool BuildToneProgram(TonePattern pattern, uint16_t freq, uint16_t duration_ms, uint8_t count, tone::Segment* segs,
                      size_t& n, uint8_t& repeats, int16_t& amplitude) {
  n = 0;
  repeats = 1;
  amplitude = kToneAmplitude;

  switch (pattern) {
    case TonePattern::ACK: segs[n++] = {880, 880, 60}; break;
    case TonePattern::ERROR:
      segs[n++] = {220, 220, 100};
      segs[n++] = {0, 0, 60};
      segs[n++] = {220, 220, 100};
      break;
    case TonePattern::WARN:
      segs[n++] = {660, 660, 120};
      segs[n++] = {0, 0, 80};
      segs[n++] = {660, 660, 120};
      break;
    case TonePattern::CHARGE_CONNECT:
      segs[n++] = {523, 523, 90};
      segs[n++] = {784, 784, 140};
      break;
    case TonePattern::CHARGE_DISCONNECT:
      segs[n++] = {784, 784, 90};
      segs[n++] = {523, 523, 140};
      break;
    case TonePattern::BOOT:
      segs[n++] = {523, 523, 80};
      segs[n++] = {659, 659, 80};
      segs[n++] = {784, 784, 80};
      segs[n++] = {1047, 1047, 160};
      break;
    case TonePattern::SIREN:
      segs[n++] = {1000, 3000, 400};
      segs[n++] = {3000, 1000, 400};
      repeats = 3;
      amplitude = kSirenAmplitude;
      break;
    case TonePattern::CUSTOM: {
      // Bounds keep a bad RPC from producing an ultrasonic/aliased scream or a minutes-long
      // blare: audible band well under Nyquist (8 kHz at our 16 kHz rate), bounded duration,
      // bounded repetition.
      if (freq < 100 || freq > 6000 || duration_ms < 20 || duration_ms > 2000 || count < 1 || count > 10) {
        return false;
      }
      segs[n++] = {freq, freq, duration_ms};
      if (count > 1) segs[n++] = {0, 0, 80};
      repeats = count;
      break;
    }
    default: return false;
  }

  return true;
}

// Names are bare identifiers, never paths: [a-z0-9_-] only. The ".wav" suffix and directory
// are appended by us, so path traversal can't be expressed at this layer at all.
bool ValidName(const char* name, size_t len) {
  if (len == 0 || len >= 64) return false;
  for (size_t i = 0; i < len; i++) {
    const char c = name[i];
    if (!(islower(static_cast<unsigned char>(c)) || isdigit(static_cast<unsigned char>(c)) || c == '_' || c == '-')) {
      return false;
    }
  }
  return true;
}

bool FileExists(const char* path) {
  struct lfs_info info;
  return lfs_stat(&lfs, path, &info) == LFS_ERR_OK && info.type == LFS_TYPE_REG;
}

/**
 * @brief Parse and validate f's RIFF/WAVE header; returns the data-chunk size in bytes (0 = bad).
 *
 * On success the file position is at the first PCM sample. Walks chunks until "fmt " and "data"
 * are both found (order in the file is not assumed beyond "fmt " coming before "data", which is
 * required to validate before streaming). Bounded three ways so a malformed file is rejected
 * instead of hanging the service thread (which also drives emergency sounds) forever: a chunk
 * claiming to be bigger than the whole file is rejected outright, every seek() return value is
 * checked, and the file position is required to strictly advance every pass - a seek that
 * silently failed to move it would otherwise re-read the same chunk header forever.
 */
uint32_t ParseWavHeader(File& f, const char* path) {
  WavRiffHeader riff{};
  if (f.read(&riff, sizeof(riff)) != static_cast<int>(sizeof(riff)) || memcmp(riff.riff_id, "RIFF", 4) != 0 ||
      memcmp(riff.wave_id, "WAVE", 4) != 0) {
    ULOG_WARNING("AudioService: %s is not a RIFF/WAVE file", path);
    return 0;
  }

  const int file_size = f.size();
  bool have_fmt = false;
  WavFmtChunk fmt{};
  uint32_t data_size = 0;
  constexpr int kMaxChunks = 32;  // real WAV files have a handful of chunks; this is a hang guard
  lfs_soff_t pos = f.seek(0, LFS_SEEK_CUR);
  for (int i = 0; file_size >= 0 && pos >= 0 && i < kMaxChunks; i++) {
    WavChunkHeader chunk{};
    if (f.read(&chunk, sizeof(chunk)) != static_cast<int>(sizeof(chunk))) {
      break;  // EOF before a "data" chunk was found
    }
    if (chunk.size > static_cast<uint32_t>(file_size)) {
      ULOG_WARNING("AudioService: %s has an implausible chunk size %lu", path, static_cast<unsigned long>(chunk.size));
      break;
    }

    bool seek_ok = true;
    if (memcmp(chunk.id, "fmt ", 4) == 0) {
      const size_t to_read = etl::min(chunk.size, static_cast<uint32_t>(sizeof(fmt)));
      if (f.read(&fmt, to_read) != static_cast<int>(to_read)) break;
      have_fmt = true;
      if (chunk.size > to_read) {
        seek_ok = f.seek(static_cast<lfs_soff_t>(chunk.size - to_read), LFS_SEEK_CUR) >= 0;
      }
      if (seek_ok && (chunk.size & 1)) {  // RIFF chunks are word-aligned
        seek_ok = f.seek(1, LFS_SEEK_CUR) >= 0;
      }
    } else if (memcmp(chunk.id, "data", 4) == 0) {
      data_size = chunk.size;  // file position is now at the first PCM sample
      break;
    } else {
      seek_ok = f.seek(static_cast<lfs_soff_t>(chunk.size + (chunk.size & 1)), LFS_SEEK_CUR) >= 0;
    }
    if (!seek_ok) {
      ULOG_WARNING("AudioService: %s seek failed while walking chunks", path);
      break;
    }

    const lfs_soff_t new_pos = f.seek(0, LFS_SEEK_CUR);
    if (new_pos <= pos) {
      ULOG_WARNING("AudioService: %s chunk walk stalled (position did not advance)", path);
      break;
    }
    pos = new_pos;
  }

  if (!have_fmt || data_size == 0) {
    ULOG_WARNING("AudioService: %s has no fmt/data chunk", path);
    return 0;
  }
  if (fmt.audio_format != kWavFormatPcm || fmt.num_channels != kWavChannelsMono ||
      fmt.bits_per_sample != kWavBitsPerSample || fmt.sample_rate != xbot::driver::audio::kSampleRateHz) {
    ULOG_WARNING(
        "AudioService: %s format mismatch (fmt=%u ch=%u bits=%u rate=%lu), expected PCM/mono/16-bit/%lu Hz - not "
        "playing it",
        path, fmt.audio_format, fmt.num_channels, fmt.bits_per_sample, static_cast<unsigned long>(fmt.sample_rate),
        static_cast<unsigned long>(xbot::driver::audio::kSampleRateHz));
    return 0;
  }
  return data_size;
}

/**
 * @brief Pre-flight check on its own scratch handle: is path a playable WAV?
 *
 * Runs before the current sound is retired, so a missing/corrupt/wrong-format file can be
 * rejected (or skipped over in the resolution chain) without silencing whatever is playing.
 * Uses a local File, so it is safe while the feeder thread streams from playback_file_.
 */
bool ValidateWav(const char* path) {
  File f;
  if (f.open(path, LFS_O_RDONLY) != LFS_ERR_OK) {
    ULOG_WARNING("AudioService: cannot open %s", path);
    return false;
  }
  const uint32_t data_size = ParseWavHeader(f, path);
  f.close();
  return data_size != 0;
}

}  // namespace

bool AudioService::OnStart() {
  // LittleFS serializes all access internally (filesystem.cpp's lock_flash()/unlock_flash()
  // callbacks are invoked by lfs itself on every operation), so no extra locking is needed here
  // even though filesystem_service also touches lfs from its own thread.
  int result = lfs_mkdir(&lfs, "/user");
  if (result != LFS_ERR_OK && result != LFS_ERR_EXIST) {
    ULOG_ERROR("AudioService: failed to create /user: error=%d", result);
  }
  result = lfs_mkdir(&lfs, "/user/audio");
  if (result != LFS_ERR_OK && result != LFS_ERR_EXIST) {
    ULOG_ERROR("AudioService: failed to create /user/audio: error=%d", result);
  }

  driver_ok_ = xbot::driver::audio::Init();
  if (!driver_ok_) {
    ULOG_ERROR("AudioService: I2S6Audio::Init() failed, sound disabled");
  }

  // OnStart() re-runs on every xbot CLAIM (this service has no registers, so the framework
  // restarts it whenever ROS reconnects), but the boot chime, the emergency startup grace,
  // and the last-seen reason mask are per-POWER-ON state: re-arming them on a claim would
  // chime twice per boot and swallow the ros_ok falling edge behind a fresh 10 s grace.
  if (!ever_started_) {
    ever_started_ = true;
    start_micros_ = xbot::service::system::getTimeMicros();
    boot_chime_played_ = false;
    grace_elapsed_ = false;
    last_reasons_ = 0;
  }
  return true;
}

uint32_t AudioService::OnLoop(uint32_t now_micros, uint32_t) {
  if (!driver_ok_) {
    return 1'000'000;
  }

  // Playback-finished edge: free the voice and tell the world. Fires once per completed
  // playback (playing_class_ only transitions to nonzero on a successful Start*), so this can't
  // spam the log the way a per-loop check would.
  if (playing_class_ != 0 && !xbot::driver::audio::IsPlaying()) {
    ULOG_INFO("AudioService: playback finished (class=%u)", playing_class_);
    playing_class_ = 0;
    SendPlayingClass(playing_class_);
  }

  const uint32_t elapsed_ms = (now_micros - start_micros_) / 1000;

  if (!boot_chime_played_ && elapsed_ms >= kBootChimeDelayMs) {
    boot_chime_played_ = true;
    StartNamed("boot", 4, AudioClass::UI);
  }

  // Latched, not re-evaluated: getTimeMicros() is a 32-bit microsecond counter, so elapsed_ms
  // wraps back to 0 every ~71.6 min. Re-testing the threshold every loop would re-arm the grace
  // period at every wrap and silently drop 10 s of emergency edges, indefinitely.
  if (!grace_elapsed_ && elapsed_ms >= kStartupGraceMs) {
    grace_elapsed_ = true;
  }
  if (grace_elapsed_) {
    CheckEmergencyEdges();
  }

  DrainPending();

  return 10'000;  // 10 ms: bounds RequestTone() latency for button-press feedback
}

void AudioService::CheckEmergencyEdges() {
  // emergency_service boots with TIMEOUT_INPUTS | TIMEOUT_HIGH_LEVEL already latched (and
  // SERVICE_NOT_READY while other services spin up), so without a startup grace period every
  // power-on would immediately "sound" like an emergency. Emergencies occurring during this
  // window are simply not evaluated (and don't update last_reasons_), so a real, still-active
  // emergency is correctly announced the moment the grace period ends.
  const uint16_t reasons = emergency_service.GetEmergencyReasons();
  constexpr uint16_t kPhysicalReasons =
      EmergencyReason::STOP | EmergencyReason::LIFT | EmergencyReason::LIFT_MULTIPLE | EmergencyReason::COLLISION;

  if (last_reasons_ == 0 && reasons != 0) {
    // Rising edge: some emergency reason just became active. Fires once per edge - holding an
    // emergency active (or switching between reasons without ever clearing) doesn't retrigger;
    // it only re-arms once reasons drops back to 0 below.
    if (reasons & EmergencyReason::STOP) {
      StartNamed("emergency_stop", 14, AudioClass::SAFETY);
    } else if (reasons & (EmergencyReason::LIFT | EmergencyReason::LIFT_MULTIPLE)) {
      StartNamed("emergency_lift", 14, AudioClass::SAFETY);
    } else if ((reasons & EmergencyReason::TIMEOUT_HIGH_LEVEL) && !(reasons & kPhysicalReasons)) {
      // ROS went away with nothing physically wrong: announce the connection loss as its own
      // sound instead of the generic emergency one, so a stack restart is recognizable by ear.
      StartNamed("ros_lost", 8, AudioClass::SAFETY);
    } else {
      StartNamed("emergency_other", 15, AudioClass::SAFETY);  // COLLISION, TIMEOUT_INPUTS, ...
    }
  } else if (last_reasons_ != 0 && reasons == 0) {
    if ((last_reasons_ & EmergencyReason::TIMEOUT_HIGH_LEVEL) && !(last_reasons_ & kPhysicalReasons)) {
      StartNamed("ros_ok", 6, AudioClass::SAFETY);
    } else {
      StartNamed("emergency_cleared", 17, AudioClass::SAFETY);
    }
  }
  last_reasons_ = reasons;
}

void AudioService::RequestTone(TonePattern pattern, AudioClass audio_class) {
  chMtxLock(&req_mtx_);
  if (!pending_.valid || U8(audio_class) >= pending_.audio_class) {
    pending_.valid = true;
    pending_.is_tone = true;
    pending_.audio_class = U8(audio_class);
    pending_.pattern = static_cast<uint8_t>(pattern);
  }
  chMtxUnlock(&req_mtx_);
}

void AudioService::RequestNamed(const char* name, AudioClass audio_class) {
  chMtxLock(&req_mtx_);
  if (!pending_.valid || U8(audio_class) >= pending_.audio_class) {
    pending_.valid = true;
    pending_.is_tone = false;
    pending_.audio_class = U8(audio_class);
    strncpy(pending_.name, name, sizeof(pending_.name) - 1);
    pending_.name[sizeof(pending_.name) - 1] = '\0';
  }
  chMtxUnlock(&req_mtx_);
}

void AudioService::RequestPath(const char* path, AudioClass audio_class) {
  chMtxLock(&req_mtx_);
  if (!pending_.valid || U8(audio_class) >= pending_.audio_class) {
    pending_.valid = true;
    pending_.is_tone = false;
    pending_.is_path = true;
    pending_.audio_class = U8(audio_class);
    strncpy(pending_.name, path, sizeof(pending_.name) - 1);
    pending_.name[sizeof(pending_.name) - 1] = '\0';
  }
  chMtxUnlock(&req_mtx_);
}

void AudioService::DrainPending() {
  Request req;
  chMtxLock(&req_mtx_);
  req = pending_;
  pending_.valid = false;
  chMtxUnlock(&req_mtx_);

  if (!req.valid || !ValidClassByte(req.audio_class)) return;
  if (req.is_tone) {
    StartTone(static_cast<TonePattern>(req.pattern), static_cast<AudioClass>(req.audio_class), 0, 0, 0);
  } else if (req.is_path) {
    StartPath(req.name, static_cast<AudioClass>(req.audio_class));
  } else {
    StartNamed(req.name, strnlen(req.name, sizeof(req.name)), static_cast<AudioClass>(req.audio_class));
  }
}

bool AudioService::ArbitrateStart(AudioClass audio_class) {
  // Equal class preempts (last-wins, matching the old PlayTrack behaviour); lower is dropped.
  return !(xbot::driver::audio::IsPlaying() && U8(audio_class) < playing_class_);
}

uint16_t AudioService::ApplyClassVolume(AudioClass audio_class) {
  uint16_t volume = master_volume_.load();
  if (audio_class == AudioClass::ALARM) {
    volume = 256;
  } else if (audio_class == AudioClass::SAFETY) {
    volume = etl::max(volume, kSafetyVolumeFloor);
  } else if (quiet_mode_.load()) {
    volume = volume / 4;
  }
  xbot::driver::audio::SetVolume(volume);
  return volume;
}

uint8_t AudioService::StartNamed(const char* name, size_t name_len, AudioClass audio_class) {
  if (!driver_ok_ || !ValidName(name, name_len)) {
    return Res(AudioResult::ERR_INVAL);
  }
  if (!ArbitrateStart(audio_class)) {
    return Res(AudioResult::ERR_DROPPED);
  }

  // Each candidate is fully validated (open + header parse on a scratch handle) BEFORE the
  // current sound is retired, and a bad file just drops through to the next fallback - a
  // corrupt custom override degrades to the numbered track / built-in default instead of
  // killing the running sound and going silent.
  char path[96];
  snprintf(path, sizeof(path), "/user/audio/%.*s.wav", static_cast<int>(name_len), name);
  if (!FileExists(path) || !ValidateWav(path)) {
    path[0] = '\0';
    for (const auto& alias : kTrackAliases) {
      if (strlen(alias.name) == name_len && memcmp(alias.name, name, name_len) == 0) {
        snprintf(path, sizeof(path), "/user/audio/%u.wav", alias.track);
        if (!FileExists(path) || !ValidateWav(path)) path[0] = '\0';
        break;
      }
    }
  }

  if (path[0] != '\0') {
    const uint16_t volume = ApplyClassVolume(audio_class);
    if (!PlayPath(path)) {
      return Res(AudioResult::ERR_NOENT);  // validated a moment ago; only a delete race gets here
    }
    playing_class_ = U8(audio_class);
    SendPlayingClass(playing_class_);
    ULOG_INFO("AudioService: playing '%.*s' (class=%u, volume=%u)", static_cast<int>(name_len), name, U8(audio_class),
              volume);
    return Res(AudioResult::OK);
  }

  // No file anywhere: fall back to the built-in tone default, if this name has one.
  for (const auto& def : kToneDefaults) {
    if (strlen(def.name) == name_len && memcmp(def.name, name, name_len) == 0) {
      return StartTone(def.pattern, audio_class, 0, 0, 0);
    }
  }

  ULOG_INFO("AudioService: no sound for '%.*s' (no file, no default)", static_cast<int>(name_len), name);
  return Res(AudioResult::ERR_NOENT);
}

uint8_t AudioService::StartPath(const char* path, AudioClass audio_class) {
  // No name resolution, no /user prefix: path is used exactly as given. Callers own keeping
  // it out of reach of anything RPC-driven (see RequestPath()).
  if (!driver_ok_) {
    return Res(AudioResult::ERR_INVAL);
  }
  if (!ArbitrateStart(audio_class)) {
    return Res(AudioResult::ERR_DROPPED);
  }
  if (!FileExists(path) || !ValidateWav(path)) {
    return Res(AudioResult::ERR_NOENT);
  }
  const uint16_t volume = ApplyClassVolume(audio_class);
  if (!PlayPath(path)) {
    return Res(AudioResult::ERR_NOENT);  // validated a moment ago; only a delete race gets here
  }
  playing_class_ = U8(audio_class);
  SendPlayingClass(playing_class_);
  ULOG_INFO("AudioService: playing path '%s' (class=%u, volume=%u)", path, U8(audio_class), volume);
  return Res(AudioResult::OK);
}

uint8_t AudioService::StartTone(TonePattern pattern, AudioClass audio_class, uint16_t freq, uint16_t duration_ms,
                                uint8_t count) {
  if (!driver_ok_) {
    return Res(AudioResult::ERR_INVAL);
  }
  tone::Segment segs[tone::kMaxSegments];
  size_t n = 0;
  uint8_t repeats = 1;
  int16_t amplitude = kToneAmplitude;
  if (!BuildToneProgram(pattern, freq, duration_ms, count, segs, n, repeats, amplitude)) {
    return Res(AudioResult::ERR_INVAL);
  }
  if (!ArbitrateStart(audio_class)) {
    return Res(AudioResult::ERR_DROPPED);
  }

  // Retire any running stream before loading the program: FeedTone() must never render a
  // half-swapped program, and Play() below re-primes the DMA buffers from scratch.
  xbot::driver::audio::Stop();
  CloseStream();  // a WAV we just preempted would otherwise keep its lfs handle open forever
  tone::SetToneProgram(segs, n, repeats, amplitude);
  const uint16_t volume = ApplyClassVolume(audio_class);
  xbot::driver::audio::Play(&tone::FeedTone);
  playing_class_ = U8(audio_class);
  SendPlayingClass(playing_class_);
  ULOG_INFO("AudioService: playing tone pattern=%u (class=%u, volume=%u)", static_cast<uint8_t>(pattern),
            U8(audio_class), volume);
  return Res(AudioResult::OK);
}

bool AudioService::PlayPath(const char* path) {
  // Retire any stream still running before touching the handle it is reading from. Both tracks
  // share one source_ (&FeedFromFile), so without this the feeder could be parked on
  // playback_mutex_ mid-refill and resume against the NEW file the moment this function unlocks
  // - splicing the new track's opening samples onto the old track's tail, and, for a track
  // shorter than one DMA half (most event sounds are), draining it entirely so the prefill below
  // finds nothing and the sound is never heard. Stop() blocks until any in-flight refill
  // finishes and clears source_, so no FeedFromFile() can be entered after it returns. It takes
  // only audio_mutex_ and is called with playback_mutex_ NOT held, which keeps the lock order
  // one-directional.
  xbot::driver::audio::Stop();

  // Held from the close of a possibly still-playing previous handle through to the point
  // remaining_samples_ is committed, so FeedFromFile() (feeder thread) can never observe the
  // handle mid-reopen.
  chMtxLock(&playback_mutex_);

  if (playback_file_.isOpen()) {
    playback_file_.close();  // guard against re-entrant calls (e.g. a very fast re-pickup)
  }
  if (playback_file_.open(path, LFS_O_RDONLY) != LFS_ERR_OK) {
    chMtxUnlock(&playback_mutex_);
    ULOG_WARNING("AudioService: cannot open %s", path);
    return false;
  }

  const uint32_t data_size = ParseWavHeader(playback_file_, path);
  if (data_size == 0) {
    playback_file_.close();
    chMtxUnlock(&playback_mutex_);
    return false;
  }

  remaining_samples_ = data_size / sizeof(int16_t);
  chMtxUnlock(&playback_mutex_);

  xbot::driver::audio::Play(&FeedFromFile);
  return true;
}

void AudioService::RPCPlayLocal(uint16_t call_id, const char* Name, uint32_t NameLen, uint8_t Class) {
  uint8_t r =
      ValidClassByte(Class) ? StartNamed(Name, NameLen, static_cast<AudioClass>(Class)) : Res(AudioResult::ERR_INVAL);
  SendRpcResponse(call_id, RpcStatus::SUCCESS, &r, 1);
}

void AudioService::RPCPlayTone(uint16_t call_id, uint8_t Pattern, uint8_t Class, uint16_t Freq, uint16_t DurationMs,
                               uint8_t Count) {
  uint8_t r = ValidClassByte(Class) ? StartTone(static_cast<TonePattern>(Pattern), static_cast<AudioClass>(Class), Freq,
                                                DurationMs, Count)
                                    : Res(AudioResult::ERR_INVAL);
  SendRpcResponse(call_id, RpcStatus::SUCCESS, &r, 1);
}

void AudioService::OnMasterVolumeChanged(const uint16_t& new_value) {
  master_volume_.store(etl::min<uint16_t>(new_value, 256));
  if (ValidClassByte(playing_class_)) {
    ApplyClassVolume(static_cast<AudioClass>(playing_class_));
  }
}

void AudioService::OnQuietModeChanged(const uint8_t& new_value) {
  quiet_mode_.store(new_value != 0);
}

void AudioService::SendStatus() {
  SendPlayingClass(playing_class_);
  SendAlarmState(0);
  // Streaming isn't implemented yet; the fields exist so the wire format is already stable.
  SendBufferFreeBytes(0);
  SendUnderruns(0);
  SendStreamActive(0);
}
