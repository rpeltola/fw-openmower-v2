//
// Created by clemens on 26.07.24.
//

#include "emergency_service.hpp"

#include <etl/algorithm.h>
#include <ulog.h>

#include <xbot-service/Lock.hpp>

#include "services.hpp"

using xbot::service::Lock;

void EmergencyService::OnStop() {
  // We won't be getting further updates from high level, so set that flag immediately.
  {
    Lock lk{&mtx_};
    active_unlock_mask_ = 0;
  }
  UpdateEmergency(EmergencyReason::TIMEOUT_HIGH_LEVEL);
}

uint32_t EmergencyService::OnLoop(uint32_t now_micros, uint32_t) {
  uint32_t next = etl::min(CheckInputs(now_micros), etl::min(CheckTimeouts(now_micros), CheckRequiredServices()));
  next = etl::min(next, CheckDriveUnlock(now_micros));
  next = etl::min(next, CheckTilt(now_micros));
  return next;
}

uint32_t EmergencyService::CheckInputs(uint32_t now) {
  constexpr uint16_t potential_reasons =
      EmergencyReason::STOP | EmergencyReason::LIFT | EmergencyReason::LIFT_MULTIPLE | EmergencyReason::COLLISION;
  auto [reasons, block_time] = input_service.GetEmergencyReasons(now);
  UpdateEmergency(reasons, potential_reasons);
  return block_time;
}

void EmergencyService::OnHighLevelEmergencyChanged(const uint16_t* new_value, uint32_t length) {
  (void)length;
  {
    Lock lk(&mtx_);
    last_high_level_emergency_message_ = xbot::service::system::getTimeMicros();
  }
  UpdateEmergency(new_value[0], new_value[1] & ~EmergencyReason::SERVICE_NOT_READY);
}

void EmergencyService::OnDriveUnlockRequestChanged(const uint16_t* new_value, uint32_t length) {
  if (length < 2) return;
  const uint16_t mask = new_value[0];
  const uint32_t now = xbot::service::system::getTimeMicros();

  bool state_changed = false;
  {
    Lock lk{&mtx_};
    if (mask == 0) {
      // Explicit release ends the session and the lockout.
      state_changed = active_unlock_mask_ != 0;
      active_unlock_mask_ = 0;
      unlock_session_lockout_ = false;
      unlock_session_start_micros_ = 0;
    } else if ((mask & ~kUnlockableMask) != 0) {
      // Request touches a non-unlockable reason: refuse entirely.
      ULOG_WARNING("emergency: drive unlock refused, mask 0x%04x not unlockable", mask);
    } else if (tilt_over_limit_) {
      // Not flat per our own IMU: refuse (and drop anything active).
      state_changed = active_unlock_mask_ != 0;
      active_unlock_mask_ = 0;
    } else if (unlock_session_lockout_) {
      // Session cap expired and renewals haven't stopped yet: ignore until the
      // requester lets go (gap handled in CheckDriveUnlock).
      last_unlock_renewal_micros_ = now;
    } else {
      uint32_t ttl = static_cast<uint32_t>(new_value[1]) * 1'000;
      if (ttl == 0) ttl = kUnlockDefaultTtlMicros;
      ttl = etl::clamp(ttl, kUnlockMinTtlMicros, kUnlockMaxTtlMicros);
      unlock_ttl_micros_ = ttl;
      if (active_unlock_mask_ == 0) {
        unlock_session_start_micros_ = now;
      }
      state_changed = active_unlock_mask_ != mask;
      active_unlock_mask_ = mask;
      last_unlock_renewal_micros_ = now;
    }
  }
  if (state_changed) {
    chEvtBroadcastFlags(&mower_events, MowerEvents::EMERGENCY_CHANGED);
    SendStatus();
  }
}

uint32_t EmergencyService::CheckDriveUnlock(uint32_t now) {
  bool dropped = false;
  {
    Lock lk{&mtx_};
    if (unlock_session_lockout_ && now - last_unlock_renewal_micros_ > kUnlockLockoutGapMicros) {
      // Requester stopped renewing; a fresh session may start again.
      unlock_session_lockout_ = false;
    }
    if (active_unlock_mask_ == 0) {
      return unlock_session_lockout_ ? 100'000 : UINT32_MAX;
    }
    if (now - last_unlock_renewal_micros_ > unlock_ttl_micros_) {
      // Dead-man expired: re-engage.
      active_unlock_mask_ = 0;
      dropped = true;
    } else if (now - unlock_session_start_micros_ > kUnlockSessionCapMicros) {
      // Session cap: re-engage and require a renewal gap before re-arming.
      active_unlock_mask_ = 0;
      unlock_session_lockout_ = true;
      dropped = true;
    } else if (tilt_over_limit_) {
      // No longer flat: re-engage.
      active_unlock_mask_ = 0;
      dropped = true;
    }
  }
  if (dropped) {
    ULOG_INFO("emergency: drive unlock re-engaged");
    chEvtBroadcastFlags(&mower_events, MowerEvents::EMERGENCY_CHANGED);
    SendStatus();
  }
  return 100'000;
}

uint32_t EmergencyService::CheckTilt(uint32_t now) {
  if (!imu_service.IsFound()) {
    return UINT32_MAX;
  }
  const float pitch = imu_service.GetPitch();

  // Coarse "not flat" tracker gating the drive unlock (sustained, invariant 5).
  if (pitch > kUnlockTiltLimitDeg) {
    if (tilt_over_limit_since_micros_ == 0) tilt_over_limit_since_micros_ = now;
    if (!tilt_over_limit_ && now - tilt_over_limit_since_micros_ > kUnlockTiltSustainMicros) {
      Lock lk{&mtx_};
      tilt_over_limit_ = true;
    }
  } else {
    tilt_over_limit_since_micros_ = 0;
    if (tilt_over_limit_) {
      Lock lk{&mtx_};
      tilt_over_limit_ = false;
    }
  }

  // Hard TILT emergency: robot picked up / rolled, independent of wheel sensors.
  // Latches like the input emergencies do; TILT itself clears when flat again but
  // LATCH stays until the normal reset path.
  if (pitch > kTiltEstopDeg) {
    tilt_estop_under_since_micros_ = 0;
    if (tilt_estop_over_since_micros_ == 0) tilt_estop_over_since_micros_ = now;
    if (!tilt_estop_over_ && now - tilt_estop_over_since_micros_ > kTiltEstopSustainMicros) {
      tilt_estop_over_ = true;
      UpdateEmergency(EmergencyReason::TILT | EmergencyReason::LATCH);
    }
  } else if (pitch < kTiltEstopClearDeg) {
    tilt_estop_over_since_micros_ = 0;
    if (tilt_estop_under_since_micros_ == 0) tilt_estop_under_since_micros_ = now;
    if (tilt_estop_over_ && now - tilt_estop_under_since_micros_ > kTiltEstopSustainMicros) {
      tilt_estop_over_ = false;
      UpdateEmergency(0, EmergencyReason::TILT);
    }
  } else {
    // Hysteresis band: hold current state, reset both sustain timers.
    tilt_estop_over_since_micros_ = 0;
    tilt_estop_under_since_micros_ = 0;
  }
  return 100'000;
}

uint32_t EmergencyService::CheckTimeouts(uint32_t now) {
  uint16_t reasons = 0;
  uint32_t block_time = UINT32_MAX;
  {
    Lock lk{&mtx_};
    if (TimeoutReached(now - last_high_level_emergency_message_, 1'000'000, block_time)) {
      reasons |= EmergencyReason::TIMEOUT_HIGH_LEVEL;
    }
  }
  constexpr uint16_t potential_reasons = EmergencyReason::TIMEOUT_HIGH_LEVEL | EmergencyReason::TIMEOUT_INPUTS;
  UpdateEmergency(reasons, potential_reasons);
  return block_time;
}

void EmergencyService::UpdateEmergency(uint16_t add, uint16_t clear) {
  {
    Lock lk{&mtx_};
    uint16_t old_reason = reasons_;
    reasons_ &= ~clear;
    reasons_ |= add;
    if (reasons_ == old_reason) {
      return;
    }
  }
  chEvtBroadcastFlags(&mower_events, MowerEvents::EMERGENCY_CHANGED);
  SendStatus();
}

uint16_t EmergencyService::GetEmergencyReasons() {
  Lock lk{&mtx_};
  return reasons_;
}

uint16_t EmergencyService::GetBladeBlockReasons() {
  Lock lk{&mtx_};
  return reasons_;
}

uint16_t EmergencyService::GetDriveBlockReasons() {
  Lock lk{&mtx_};
  return reasons_ & ~active_unlock_mask_;
}

bool EmergencyService::IsDriveUnlockActive() {
  Lock lk{&mtx_};
  return active_unlock_mask_ != 0;
}

void EmergencyService::RequireService(ServiceExt* svc) {
  Lock lk{&mtx_};
  required_services_.push_back(svc);
  reasons_ |= EmergencyReason::SERVICE_NOT_READY;
}

uint32_t EmergencyService::CheckRequiredServices() {
  if (required_services_.empty()) {
    // Nothing to do, no re-query
    return UINT32_MAX;
  }
  bool all_ready = true;
  for (auto* svc : required_services_) {
    if (!svc->IsHealthy()) {
      all_ready = false;
      break;
    }
  }
  // Retry in 100ms
  constexpr uint32_t retry_interval = 100'000;
  UpdateEmergency(all_ready ? 0 : EmergencyReason::SERVICE_NOT_READY, EmergencyReason::SERVICE_NOT_READY);
  return all_ready ? UINT32_MAX : retry_interval;
}

void EmergencyService::SendStatus() {
  xbot::service::Lock lk{&mtx_};
  StartTransaction();
  SendEmergencyReason(reasons_);
  SendDriveUnlockState(active_unlock_mask_);
  CommitTransaction();
}
