/*
 * OpenMower V2 Firmware
 * Part of the OpenMower V2 Firmware (https://github.com/xtech/fw-openmower-v2)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "security_service.hpp"

#include <etl/algorithm.h>
#include <ulog.h>

#include <cstring>
#include <filesystem/file.hpp>
#include <filesystem/filesystem.hpp>
#include <filesystem/versioned_struct.hpp>

#include "services.hpp"

using xbot::datatypes::RpcStatus;

namespace {

constexpr uint8_t Res(SecResult r) {
  return static_cast<uint8_t>(r);
}

// ============================================================================================
// Every timing/tuning constant this file uses lives here, in one place, so a bench/field
// tuning pass never has to go hunting through the state machine for a magic number.
// ============================================================================================

// ---- Ladder stage boundaries: DEFAULTS ONLY. The matching xbot inputs (Trigger/Deterrent/
// Siren Delay S, Siren Max S) let ROS override every one of these live, no reflash - see
// OnTriggerDelaySChanged() etc. below. NEEDS FIELD TUNING on a real robot. ----
constexpr uint8_t kDefaultTriggerDelayS = 3;     // sustained LIFT before the ladder starts at all
constexpr uint8_t kDefaultDeterrentDelayS = 10;  // trigger -> Stage 2 (spoken deterrent)
constexpr uint16_t kDefaultSirenDelayS = 60;     // trigger -> Stage 3 (full siren)
constexpr uint16_t kDefaultSirenMaxS = 600;      // Stage 3 continuous run before duty-cycling (10 min)

// Clamps applied to the same four values when set live from ROS. A 0 or absurd input must
// never make the ladder untriggerable (floor) or make the siren run forever without ever
// duty-cycling down (ceiling). NEEDS FIELD TUNING alongside the defaults above.
constexpr uint8_t kMinTriggerDelayS = 1, kMaxTriggerDelayS = 30;
constexpr uint8_t kMinDeterrentDelayS = 2, kMaxDeterrentDelayS = 120;
constexpr uint16_t kMinSirenDelayS = 5, kMaxSirenDelayS = 600;
constexpr uint16_t kMinSirenMaxS = 30, kMaxSirenMaxS = 3600;

// Stage 3 duty cycle once the continuous window (Siren Max S) has elapsed: a burst every
// minute instead of nonstop, so the alarm still nags at an empty field without cooking the
// speaker or the battery indefinitely. NEEDS FIELD TUNING.
constexpr uint32_t kSirenBurstPeriodMs = 60'000;
constexpr uint32_t kSirenBurstOnMs = 10'000;

// Stage 1 "warning chirps, rising cadence": the inter-chirp interval ramps linearly from
// Start down to End across the whole Stage-1 window (0 .. Deterrent Delay S). NEEDS FIELD
// TUNING.
constexpr uint32_t kWarnChirpIntervalStartMs = 2'500;
constexpr uint32_t kWarnChirpIntervalEndMs = 500;

// Stage 2 deterrent clip repeat interval ("looped ~every 10 s" per spec). NEEDS FIELD TUNING.
constexpr uint32_t kDeterrentRepeatMs = 10'000;

// Stage 3 sub-cycle: siren sweep, then the deterrent clip, then the owner-identity clip,
// then repeat. Each window just needs to be long enough for AudioService to finish a
// typical clip before the next Request* preempts it - the siren window in particular is
// sized to the ALARM/AudioClass TonePattern::SIREN program (2 segments x 400 ms x 3
// repeats = 2400 ms) with margin. NEEDS FIELD TUNING once real clip lengths exist.
constexpr uint32_t kSirenToneWindowMs = 2'800;
constexpr uint32_t kSirenDeterrentWindowMs = 4'000;
constexpr uint32_t kSirenOwnerWindowMs = 4'000;
constexpr uint32_t kSirenSubphaseWindowsMs[3] = {kSirenToneWindowMs, kSirenDeterrentWindowMs, kSirenOwnerWindowMs};

// ---- PIN policy ----
constexpr uint32_t kPinMinLen = 4, kPinMaxLen = 8;  // digits only; 8 matches the wire char[8]
// Attempts allowed before backoff kicks in; lockout then doubles from Base, capped at Max.
// NEEDS FIELD TUNING - first guess at "annoying enough to stop guessing, not so long a
// legitimate owner who fat-fingered it twice is locked out of their own mower."
constexpr uint16_t kFreeAttempts = 3;
constexpr uint32_t kLockoutBaseS = 5;
constexpr uint32_t kLockoutMaxS = 300;

// ---- /cfg paths (outside the FilesystemService /user sandbox - see file header) ----
constexpr const char* kOwnerTextPath = "/cfg/owner/owner.txt";
constexpr const char* kOwnerWavPath = "/cfg/owner/owner.wav";
constexpr uint32_t kOwnerTextMaxLen = 128;  // matches Set Owner Info's Text char[128]
// ROS can only write into /user (FilesystemService's sandbox), so the rendered owner clip
// lands here first; a successful, PIN-gated Set Owner Info promotes it into /cfg/owner - see
// PromoteOwnerWav(). The name only has to be stable and reserved; it is never resolved
// through AudioService's named-sound lookup.
constexpr const char* kOwnerWavStagingPath = "/user/audio/security_owner_pending.wav";
// Stage 2/3 spoken deterrent: an ordinary named sound (per-language file the owner/ROS
// uploads through the normal /user/audio resolution chain - see AudioService::StartNamed()).
constexpr const char* kDeterrentSoundName = "security_deterrent";

bool FileExistsAt(const char* path) {
  struct lfs_info info;
  return lfs_stat(&lfs, path, &info) == LFS_ERR_OK && info.type == LFS_TYPE_REG;
}

// Plain byte copy between two littlefs paths, used only for the one-directional /user ->
// /cfg owner.wav promotion. Synchronous/blocking (like every other flash op in this
// codebase); runs on SecurityService's own thread from inside a (rare, human-triggered)
// RPC handler, so it never blocks the audio feeder or any other service.
bool CopyFile(const char* src_path, const char* dst_path) {
  File src;
  if (src.open(src_path, LFS_O_RDONLY) != LFS_ERR_OK) {
    return false;
  }
  File dst;
  if (dst.mkdirp(dst_path) != LFS_ERR_OK ||
      dst.open(dst_path, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) != LFS_ERR_OK) {
    return false;
  }
  uint8_t buf[512];
  int n;
  bool ok = true;
  while ((n = src.read(buf, sizeof(buf))) > 0) {
    if (dst.write(buf, static_cast<size_t>(n)) != n) {
      ok = false;
      break;
    }
  }
  if (n < 0) ok = false;
  dst.sync();
  return ok;
}

// ---- PIN hashing --------------------------------------------------------------------------
// No crypto library in this build (out of scope for #123) and no HW RNG driver wired up
// either. HashPin() is a salted, twice-mixed CRC32 built on littlefs's own lfs_crc() (already
// linked in, zero new dependency). This is explicitly NOT a KDF - CRC32 is linear/invertible,
// so it will not resist an attacker who has both the salt and unlimited offline attempts.
// What it does buy: no plaintext PIN sits in a flash dump, and combined with the escalating
// lockout below there is no practical way to brute-force an 8-digit PIN against the live
// device. If this ever needs to resist more than a casual flash dump, swap this function for
// a real KDF - nothing else in security_service.cpp assumes the specific hash.
void HashPin(const char* pin, uint32_t pin_len, const uint8_t salt[SecurityService::kPinSaltBytes],
             uint8_t out_hash[SecurityService::kPinHashBytes]) {
  uint32_t crc = lfs_crc(0xFFFFFFFFu, salt, SecurityService::kPinSaltBytes);
  crc = lfs_crc(crc, pin, pin_len);
  const uint8_t mid[4] = {static_cast<uint8_t>(crc), static_cast<uint8_t>(crc >> 8), static_cast<uint8_t>(crc >> 16),
                          static_cast<uint8_t>(crc >> 24)};
  crc = lfs_crc(0xFFFFFFFFu, mid, sizeof(mid));
  crc = lfs_crc(crc, salt, SecurityService::kPinSaltBytes);
  out_hash[0] = static_cast<uint8_t>(crc);
  out_hash[1] = static_cast<uint8_t>(crc >> 8);
  out_hash[2] = static_cast<uint8_t>(crc >> 16);
  out_hash[3] = static_cast<uint8_t>(crc >> 24);
}

// Not a CSPRNG: mixes boot-relative timing jitter (the microsecond clock plus the
// free-running ChibiOS realtime counter, both already available - no new peripheral driver)
// through CRC32. Enough to defeat a precomputed rainbow table against a flash dump; not
// enough against an attacker who can also observe the exact PIN-set instant. Revisit if a HW
// RNG driver ever gets wired up.
void GenerateSalt(uint8_t salt[SecurityService::kPinSaltBytes]) {
  const uint32_t a = xbot::service::system::getTimeMicros();
  const uint32_t b = static_cast<uint32_t>(chSysGetRealtimeCounterX());
  uint32_t crc = lfs_crc(0xFFFFFFFFu, &a, sizeof(a));
  crc = lfs_crc(crc, &b, sizeof(b));
  uint8_t seed[4] = {static_cast<uint8_t>(crc), static_cast<uint8_t>(crc >> 8), static_cast<uint8_t>(crc >> 16),
                     static_cast<uint8_t>(crc >> 24)};
  for (size_t i = 0; i < SecurityService::kPinSaltBytes; i++) {
    crc = lfs_crc(crc, seed, sizeof(seed));
    salt[i] = static_cast<uint8_t>(crc);
    seed[0] = salt[i];  // fold forward so every output byte depends on the last
  }
}

// ---- Persisted state (survives a power cycle) -------------------------------------------
#pragma pack(push, 1)
struct SecurityPersistedState : public xbot::driver::filesystem::VersionedStruct<SecurityPersistedState> {
  VERSIONED_STRUCT_FIELDS(1);
  static constexpr const char* PATH = "/cfg/security_state.bin";

  uint8_t armed = 0;
  uint8_t alarm_latched = 0;  // the important one: survives to force straight-to-SIREN on reboot
  uint8_t pin_set = 0;
  uint8_t pin_hash[SecurityService::kPinHashBytes] = {};
  uint8_t pin_salt[SecurityService::kPinSaltBytes] = {};
  uint16_t failed_attempts = 0;  // never reset by a reboot - see RegisterFailedAttempt()
};
#pragma pack(pop)
static_assert(sizeof(SecurityPersistedState) == 2 + 1 + 1 + 1 + 4 + 8 + 2, "SecurityPersistedState layout changed");

}  // namespace

bool SecurityService::OnStart() {
  // OnStart() re-runs on every xbot CLAIM (ROS reconnect), same as AudioService's - the
  // persisted/latched state must only be loaded, and the boot-straight-to-SIREN decision
  // only made, once per power-on.
  if (!ever_started_) {
    ever_started_ = true;
    LoadPersisted();
    owner_set_ = FileExistsAt(kOwnerTextPath);

    trigger_delay_s_ = kDefaultTriggerDelayS;
    deterrent_delay_s_ = kDefaultDeterrentDelayS;
    siren_delay_s_ = kDefaultSirenDelayS;
    siren_max_s_ = kDefaultSirenMaxS;

    const uint32_t now = xbot::service::system::getTimeMicros();
    if (alarm_latched_) {
      // The latch survived a power cycle: most plausibly a thief pulling the battery to
      // silence it mid-ladder. Skip straight to maximum deterrence instead of re-running
      // the escalation from scratch.
      ULOG_ARG_WARNING(&service_id_, "SecurityService: alarm latch set at boot -> straight to SIREN");
      trigger_micros_ = now - static_cast<uint32_t>(siren_delay_s_) * 1'000'000u;
      forced_min_stage_ = AlarmState::SIREN;
      SetStage(AlarmState::SIREN, now);
    } else {
      SetStage(armed_ ? AlarmState::ARMED : AlarmState::DISARMED, now);
    }
  }

  SendArmed(armed_ ? 1 : 0);
  SendOwnerSet(owner_set_ ? 1 : 0);
  SendPinSet(pin_set_ ? 1 : 0);
  return true;
}

uint32_t SecurityService::OnLoop(uint32_t now_micros, uint32_t) {
  UpdateLadder(now_micros);
  return 200'000;  // 200 ms: plenty for chirp/siren-phase cadences, cheap on the service thread
}

void SecurityService::OnTriggerDelaySChanged(const uint8_t& new_value) {
  trigger_delay_s_ = etl::clamp(new_value, kMinTriggerDelayS, kMaxTriggerDelayS);
}

void SecurityService::OnDeterrentDelaySChanged(const uint8_t& new_value) {
  deterrent_delay_s_ = etl::clamp(new_value, kMinDeterrentDelayS, kMaxDeterrentDelayS);
}

void SecurityService::OnSirenDelaySChanged(const uint16_t& new_value) {
  siren_delay_s_ = etl::clamp(new_value, kMinSirenDelayS, kMaxSirenDelayS);
}

void SecurityService::OnSirenMaxSChanged(const uint16_t& new_value) {
  siren_max_s_ = etl::clamp(new_value, kMinSirenMaxS, kMaxSirenMaxS);
}

bool SecurityService::IsDocked() const {
  return power_service.IsAdapterPresent();
}

bool SecurityService::ArmGateActive() const {
  return armed_ && !IsDocked();
}

AlarmState SecurityService::ComputeStageForElapsed(uint32_t elapsed_ms) const {
  if (elapsed_ms < static_cast<uint32_t>(deterrent_delay_s_) * 1000) return AlarmState::WARNING;
  if (elapsed_ms < static_cast<uint32_t>(siren_delay_s_) * 1000) return AlarmState::DETERRENT;
  return AlarmState::SIREN;
}

void SecurityService::UpdateLadder(uint32_t now_micros) {
  switch (stage_) {
    case AlarmState::DISARMED: break;  // nothing to do; RPCSetArmed() drives DISARMED<->ARMED directly
    case AlarmState::ARMED:
      if (ArmGateActive()) {
        CheckLiftTrigger(now_micros);
      } else {
        lift_active_ = false;  // docked or disarmed: drop any partial lift-sustain timer
      }
      break;
    case AlarmState::WARNING:
    case AlarmState::DETERRENT:
    case AlarmState::SIREN: {
      const uint32_t elapsed_ms = (now_micros - trigger_micros_) / 1000;
      const AlarmState natural = ComputeStageForElapsed(elapsed_ms);
      // forced_min_stage_ is a floor set by Trigger Alarm RPCs; natural time-based
      // progression is the other floor. Whichever is higher wins - this can only ever
      // escalate within one trigger cycle (see TriggerLadder()), never step back down.
      const AlarmState target =
          static_cast<uint8_t>(natural) > static_cast<uint8_t>(forced_min_stage_) ? natural : forced_min_stage_;
      if (target != stage_) {
        SetStage(target, now_micros);
      }
      DriveStageSounds(now_micros, elapsed_ms);
      break;
    }
  }
}

void SecurityService::CheckLiftTrigger(uint32_t now_micros) {
  const uint16_t reasons = emergency_service.GetEmergencyReasons();
  const bool lifted = (reasons & (EmergencyReason::LIFT | EmergencyReason::LIFT_MULTIPLE)) != 0;
  if (!lifted) {
    lift_active_ = false;
    return;
  }
  if (!lift_active_) {
    lift_active_ = true;
    lift_since_micros_ = now_micros;
    return;
  }
  const uint32_t sustained_ms = (now_micros - lift_since_micros_) / 1000;
  if (sustained_ms >= static_cast<uint32_t>(trigger_delay_s_) * 1000) {
    TriggerLadder(now_micros, AlarmState::WARNING);
  }
}

void SecurityService::TriggerLadder(uint32_t now_micros, AlarmState requested_stage) {
  if (!armed_) return;  // nothing to escalate while the owner hasn't opted in
  if (stage_ == AlarmState::ARMED) {
    trigger_micros_ = now_micros;
    forced_min_stage_ = requested_stage;
    SetStage(requested_stage, now_micros);
  } else if (stage_ != AlarmState::DISARMED) {
    if (static_cast<uint8_t>(requested_stage) > static_cast<uint8_t>(forced_min_stage_)) {
      forced_min_stage_ = requested_stage;
    }
    if (static_cast<uint8_t>(forced_min_stage_) > static_cast<uint8_t>(stage_)) {
      SetStage(forced_min_stage_, now_micros);
    }
  }
  // stage_ == DISARMED: the owner hasn't armed at all (armed_ would be false too in that
  // case - defensive only, this branch should be unreachable).
}

void SecurityService::SetStage(AlarmState new_stage, uint32_t now_micros) {
  const AlarmState old = stage_;
  const bool was_idle = (old == AlarmState::DISARMED || old == AlarmState::ARMED);
  const bool now_active =
      (new_stage == AlarmState::WARNING || new_stage == AlarmState::DETERRENT || new_stage == AlarmState::SIREN);

  stage_ = new_stage;
  SendAlarmState(static_cast<uint8_t>(stage_));
  if (old != new_stage) {
    ULOG_ARG_INFO(&service_id_, "SecurityService: stage %u -> %u", static_cast<unsigned>(old),
                  static_cast<unsigned>(new_stage));
  }

  if (was_idle && now_active && !alarm_latched_) {
    // The single most safety-critical write in this file: must survive a battery pull that
    // happens the instant after this line runs, so it is not rate-limited or deferred.
    alarm_latched_ = true;
    Persist();
  }

  if (new_stage == AlarmState::SIREN && old != AlarmState::SIREN) {
    siren_phase_start_micros_ = now_micros;
    siren_subphase_start_micros_ = 0;
    siren_subphase_index_ = 0;
  }
  if (new_stage == AlarmState::WARNING && old != AlarmState::WARNING) {
    last_warn_chirp_micros_ = 0;  // force an immediate first chirp
  }
  if (new_stage == AlarmState::DETERRENT && old != AlarmState::DETERRENT) {
    last_deterrent_micros_ = 0;  // force an immediate first deterrent line
  }
}

void SecurityService::DriveStageSounds(uint32_t now_micros, uint32_t elapsed_ms) {
  switch (stage_) {
    case AlarmState::WARNING: {
      const uint32_t window_ms = static_cast<uint32_t>(deterrent_delay_s_) * 1000;
      const uint32_t frac_ms = etl::min(elapsed_ms, window_ms);
      const uint32_t interval_ms =
          window_ms == 0
              ? kWarnChirpIntervalEndMs
              : kWarnChirpIntervalStartMs - (kWarnChirpIntervalStartMs - kWarnChirpIntervalEndMs) * frac_ms / window_ms;
      if (last_warn_chirp_micros_ == 0 || (now_micros - last_warn_chirp_micros_) / 1000 >= interval_ms) {
        last_warn_chirp_micros_ = now_micros;
        audio_service.RequestTone(TonePattern::WARN, AudioClass::ALARM);
      }
      break;
    }
    case AlarmState::DETERRENT: {
      if (last_deterrent_micros_ == 0 || (now_micros - last_deterrent_micros_) / 1000 >= kDeterrentRepeatMs) {
        last_deterrent_micros_ = now_micros;
        audio_service.RequestNamed(kDeterrentSoundName, AudioClass::ALARM);
      }
      break;
    }
    case AlarmState::SIREN: {
      const uint32_t continuous_end_ms = static_cast<uint32_t>(siren_max_s_) * 1000;
      const uint32_t siren_run_ms = (now_micros - siren_phase_start_micros_) / 1000;
      bool burst_on = true;
      if (siren_run_ms > continuous_end_ms) {
        const uint32_t into_cycle_ms = (siren_run_ms - continuous_end_ms) % kSirenBurstPeriodMs;
        burst_on = into_cycle_ms < kSirenBurstOnMs;
      }
      if (!burst_on) {
        // Silent gap between bursts: freeze the sub-cycle so every burst reopens on the
        // siren tone instead of resuming mid-clip.
        siren_subphase_start_micros_ = 0;
        siren_subphase_index_ = 0;
        break;
      }
      if (siren_subphase_start_micros_ == 0) {
        siren_subphase_start_micros_ = now_micros;
        siren_subphase_index_ = 0;
        audio_service.RequestTone(TonePattern::SIREN, AudioClass::ALARM);
        break;
      }
      const uint32_t sub_elapsed_ms = (now_micros - siren_subphase_start_micros_) / 1000;
      if (sub_elapsed_ms >= kSirenSubphaseWindowsMs[siren_subphase_index_]) {
        siren_subphase_index_ = static_cast<uint8_t>((siren_subphase_index_ + 1) % 3);
        siren_subphase_start_micros_ = now_micros;
        switch (siren_subphase_index_) {
          case 0: audio_service.RequestTone(TonePattern::SIREN, AudioClass::ALARM); break;
          case 1: audio_service.RequestNamed(kDeterrentSoundName, AudioClass::ALARM); break;
          case 2: audio_service.RequestPath(kOwnerWavPath, AudioClass::ALARM); break;
          default: break;
        }
      }
      break;
    }
    default: break;
  }
}

SecurityService::AuthResult SecurityService::CheckPin(const char* pin, uint32_t pin_len) {
  if (!pin_set_) {
    return AuthResult::NO_PIN_SET;
  }

  const uint32_t now = xbot::service::system::getTimeMicros();
  if (lockout_active_) {
    // Signed subtraction handles the ~71.6 min getTimeMicros() wrap correctly as long as
    // the gap itself stays well under ~35 min, which the lockout cap (kLockoutMaxS) does.
    if (static_cast<int32_t>(lockout_until_micros_ - now) > 0) {
      return AuthResult::LOCKED_OUT;
    }
    lockout_active_ = false;
  }

  const uint32_t bounded_len = etl::min<uint32_t>(pin_len, kPinMaxLen);
  uint8_t hash[kPinHashBytes];
  HashPin(pin, bounded_len, pin_salt_, hash);
  if (memcmp(hash, pin_hash_, kPinHashBytes) != 0) {
    RegisterFailedAttempt(now);
    return AuthResult::WRONG_PIN;
  }
  RegisterSuccessfulAuth();
  return AuthResult::OK;
}

uint8_t SecurityService::AuthResultToWire(AuthResult auth) {
  switch (auth) {
    case AuthResult::OK: return Res(SecResult::OK);
    case AuthResult::WRONG_PIN: return Res(SecResult::ERR_AUTH);
    case AuthResult::LOCKED_OUT: return Res(SecResult::ERR_LOCKED);
    case AuthResult::NO_PIN_SET: return Res(SecResult::ERR_LOCKED);
  }
  return Res(SecResult::ERR_INVAL);
}

void SecurityService::RegisterFailedAttempt(uint32_t now_micros) {
  if (failed_attempts_ < 0xFFFF) failed_attempts_++;
  Persist();

  if (failed_attempts_ > kFreeAttempts) {
    uint32_t over = failed_attempts_ - kFreeAttempts;
    over = etl::min<uint32_t>(over, 20);  // bound the doubling loop; 2^20 s is already >> kLockoutMaxS
    uint32_t backoff_s = kLockoutBaseS;
    for (uint32_t i = 1; i < over && backoff_s < kLockoutMaxS; i++) {
      backoff_s *= 2;
    }
    backoff_s = etl::min(backoff_s, kLockoutMaxS);
    lockout_active_ = true;
    lockout_until_micros_ = now_micros + backoff_s * 1'000'000u;
    ULOG_ARG_WARNING(&service_id_, "SecurityService: wrong PIN (%u total) - locked out %lu s", failed_attempts_,
                     static_cast<unsigned long>(backoff_s));
  }
}

void SecurityService::RegisterSuccessfulAuth() {
  if (failed_attempts_ != 0) {
    failed_attempts_ = 0;
    Persist();
  }
  lockout_active_ = false;
}

bool SecurityService::SetPin(const char* new_pin, uint32_t new_pin_len) {
  if (new_pin_len < kPinMinLen || new_pin_len > kPinMaxLen) {
    return false;
  }
  for (uint32_t i = 0; i < new_pin_len; i++) {
    if (new_pin[i] < '0' || new_pin[i] > '9') {
      return false;
    }
  }
  GenerateSalt(pin_salt_);
  HashPin(new_pin, new_pin_len, pin_salt_, pin_hash_);
  pin_set_ = true;
  failed_attempts_ = 0;
  lockout_active_ = false;
  Persist();
  SendPinSet(1);
  return true;
}

bool SecurityService::WriteOwnerText(const char* text, uint32_t text_len) {
  text_len = etl::min(text_len, kOwnerTextMaxLen);
  File f;
  if (f.mkdirp(kOwnerTextPath) != LFS_ERR_OK ||
      f.open(kOwnerTextPath, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) != LFS_ERR_OK) {
    return false;
  }
  bool ok = true;
  if (text_len > 0) {
    // Text is raw wire bytes, bounded but not guaranteed NUL-terminated - write exactly what
    // came in, never strlen() it.
    ok = f.write(const_cast<char*>(text), text_len) == static_cast<int>(text_len);
  }
  f.sync();
  return ok;
}

void SecurityService::PromoteOwnerWav() {
  if (!FileExistsAt(kOwnerWavStagingPath)) {
    return;  // nothing staged this call - not an error, Text-only updates are valid
  }
  if (CopyFile(kOwnerWavStagingPath, kOwnerWavPath)) {
    lfs_remove(&lfs, kOwnerWavStagingPath);  // consume the staged upload
    ULOG_ARG_INFO(&service_id_, "SecurityService: promoted owner.wav from staging");
  } else {
    ULOG_ARG_WARNING(&service_id_, "SecurityService: owner.wav promotion failed mid-copy");
  }
}

void SecurityService::LoadPersisted() {
  SecurityPersistedState st{};
  if (xbot::driver::filesystem::VersionedStruct<SecurityPersistedState>::Load(st)) {
    armed_ = st.armed != 0;
    alarm_latched_ = st.alarm_latched != 0;
    pin_set_ = st.pin_set != 0;
    memcpy(pin_hash_, st.pin_hash, sizeof(pin_hash_));
    memcpy(pin_salt_, st.pin_salt, sizeof(pin_salt_));
    failed_attempts_ = st.failed_attempts;
  }
  // else: first boot ever, or a corrupt/missing file - the compiled-in defaults (disarmed,
  // no PIN, no latch) are already the safe ones.
}

void SecurityService::Persist() {
  SecurityPersistedState st{};
  st.armed = armed_ ? 1 : 0;
  st.alarm_latched = alarm_latched_ ? 1 : 0;
  st.pin_set = pin_set_ ? 1 : 0;
  memcpy(st.pin_hash, pin_hash_, sizeof(pin_hash_));
  memcpy(st.pin_salt, pin_salt_, sizeof(pin_salt_));
  st.failed_attempts = failed_attempts_;
  if (!xbot::driver::filesystem::VersionedStruct<SecurityPersistedState>::Save(st)) {
    ULOG_ARG_ERROR(&service_id_, "SecurityService: failed to persist security state");
  }
}

void SecurityService::SendStatus() {
  SendAlarmState(static_cast<uint8_t>(stage_));
  SendArmed(armed_ ? 1 : 0);
  SendOwnerSet(owner_set_ ? 1 : 0);
  SendPinSet(pin_set_ ? 1 : 0);
}

void SecurityService::RPCSetArmed(uint16_t call_id, const char* Pin, uint32_t PinLen, uint8_t Armed) {
  const AuthResult auth = CheckPin(Pin, PinLen);
  uint8_t r = AuthResultToWire(auth);
  if (auth == AuthResult::OK) {
    const bool want_armed = (Armed != 0);
    if (want_armed != armed_) {
      armed_ = want_armed;
      Persist();
      // If an alarm is actively escalating (WARNING/DETERRENT/SIREN), leave it running:
      // per the design, Clear Alarm is the only way to silence an active alarm. The new
      // armed_ intent simply takes effect once Clear Alarm resolves it.
      if (stage_ == AlarmState::ARMED || stage_ == AlarmState::DISARMED) {
        SetStage(armed_ ? AlarmState::ARMED : AlarmState::DISARMED, xbot::service::system::getTimeMicros());
      }
    }
    SendArmed(armed_ ? 1 : 0);
  }
  SendRpcResponse(call_id, RpcStatus::SUCCESS, &r, 1);
}

void SecurityService::RPCClearAlarm(uint16_t call_id, const char* Pin, uint32_t PinLen) {
  const AuthResult auth = CheckPin(Pin, PinLen);
  uint8_t r = AuthResultToWire(auth);
  if (auth == AuthResult::OK) {
    if (alarm_latched_) {
      alarm_latched_ = false;
      Persist();
    }
    if (stage_ == AlarmState::WARNING || stage_ == AlarmState::DETERRENT || stage_ == AlarmState::SIREN) {
      SetStage(armed_ ? AlarmState::ARMED : AlarmState::DISARMED, xbot::service::system::getTimeMicros());
    }
  }
  SendRpcResponse(call_id, RpcStatus::SUCCESS, &r, 1);
}

void SecurityService::RPCTriggerAlarm(uint16_t call_id, uint8_t Stage) {
  // Deliberately unauthenticated - see the file header. Only ever escalates, and only does
  // anything at all once the owner has armed the feature (TriggerLadder() re-checks armed_).
  uint8_t r = Res(SecResult::OK);
  AlarmState requested;
  switch (Stage) {
    case 1: requested = AlarmState::WARNING; break;
    case 2: requested = AlarmState::DETERRENT; break;
    case 3: requested = AlarmState::SIREN; break;
    default:
      r = Res(SecResult::ERR_INVAL);
      SendRpcResponse(call_id, RpcStatus::SUCCESS, &r, 1);
      return;
  }
  if (!armed_) {
    r = Res(SecResult::ERR_INVAL);  // nothing to escalate while disarmed
  } else {
    TriggerLadder(xbot::service::system::getTimeMicros(), requested);
  }
  SendRpcResponse(call_id, RpcStatus::SUCCESS, &r, 1);
}

void SecurityService::RPCSetOwnerInfo(uint16_t call_id, const char* Pin, uint32_t PinLen, const char* NewPin,
                                      uint32_t NewPinLen, const char* Text, uint32_t TextLen) {
  if (!pin_set_) {
    // Bootstrap: the very first PIN. Nothing to authenticate against yet, and a non-empty
    // New Pin is mandatory here - this is the only path that lifts the "no PIN -> can never
    // arm" safe default, so it must never leave owner info set with the PIN still unset.
    if (NewPinLen == 0 || !SetPin(NewPin, NewPinLen)) {
      uint8_t r = Res(SecResult::ERR_INVAL);
      SendRpcResponse(call_id, RpcStatus::SUCCESS, &r, 1);
      return;
    }
  } else {
    const AuthResult auth = CheckPin(Pin, PinLen);
    if (auth != AuthResult::OK) {
      uint8_t r = AuthResultToWire(auth);
      SendRpcResponse(call_id, RpcStatus::SUCCESS, &r, 1);
      return;
    }
    if (NewPinLen != 0 && !SetPin(NewPin, NewPinLen)) {
      uint8_t r = Res(SecResult::ERR_INVAL);
      SendRpcResponse(call_id, RpcStatus::SUCCESS, &r, 1);
      return;
    }
    // NewPinLen == 0: keep the current PIN, per contract.
  }

  uint8_t r = Res(SecResult::OK);
  if (!WriteOwnerText(Text, TextLen)) {
    r = Res(SecResult::ERR_IO);  // best-effort past this point: the PIN change above already stuck
  }
  PromoteOwnerWav();  // no-op, not an error, if nothing has been staged via the FS service
  owner_set_ = true;
  SendOwnerSet(1);
  SendRpcResponse(call_id, RpcStatus::SUCCESS, &r, 1);
}
