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
  // Tilt first: a newly detected tilt must drop the unlock in the same iteration that
  // notices it, not the next one.
  next = etl::min(next, CheckTilt(now_micros));
  next = etl::min(next, CheckDriveUnlock(now_micros));
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
    if (unlock_lockout_) {
      // Cooling down after the session cap. Refuse everything, a release included -
      // otherwise "release, re-arm" is an unbounded unlock one packet at a time.
      state_changed = active_unlock_mask_ != 0;
      active_unlock_mask_ = 0;
    } else if (mask == 0) {
      // Explicit release.
      state_changed = active_unlock_mask_ != 0;
      active_unlock_mask_ = 0;
    } else if ((mask & ~kUnlockableMask) != 0) {
      // Request touches a non-unlockable reason: refuse entirely.
      ULOG_WARNING("emergency: drive unlock refused, mask 0x%04x not unlockable", mask);
    } else if (tilt_over_limit_) {
      // Not flat per our own IMU: refuse (and drop anything active).
      state_changed = active_unlock_mask_ != 0;
      active_unlock_mask_ = 0;
    } else {
      uint32_t ttl = static_cast<uint32_t>(new_value[1]) * 1'000;
      if (ttl == 0) ttl = kUnlockDefaultTtlMicros;
      ttl = etl::clamp(ttl, kUnlockMinTtlMicros, kUnlockMaxTtlMicros);
      unlock_ttl_micros_ = ttl;
      if (active_unlock_mask_ == 0 && now - unlock_last_active_micros_ > kUnlockSessionGapMicros) {
        // Genuinely a new session. A shorter gap keeps the old session clock running,
        // so a dead-man that expires between renewals cannot reset the cap.
        unlock_session_start_micros_ = now;
      }
      state_changed = active_unlock_mask_ != mask;
      active_unlock_mask_ = mask;
      last_unlock_renewal_micros_ = now;
      unlock_last_active_micros_ = now;
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
    if (unlock_lockout_ && now - unlock_lockout_start_micros_ > kUnlockLockoutMicros) {
      unlock_lockout_ = false;
    }
    if (active_unlock_mask_ == 0) {
      return unlock_lockout_ ? 100'000 : UINT32_MAX;
    }
    unlock_last_active_micros_ = now;
    if (now - last_unlock_renewal_micros_ > unlock_ttl_micros_) {
      // Dead-man expired: re-engage.
      active_unlock_mask_ = 0;
      dropped = true;
    } else if (now - unlock_session_start_micros_ > kUnlockSessionCapMicros) {
      // Session cap: re-engage and hold everything off for the cool-down.
      active_unlock_mask_ = 0;
      unlock_lockout_ = true;
      unlock_lockout_start_micros_ = now;
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
  // Checked faster than the rest of the loop: this is the dead-man.
  return 50'000;
}

uint32_t EmergencyService::CheckTilt(uint32_t now) {
  if (!imu_service.IsFound()) {
    return UINT32_MAX;
  }
  const float pitch = imu_service.GetPitch();

  // Coarse "not flat" tracker gating the drive unlock (sustained, invariant 5).
  // Clearing needs its own threshold and its own sustain: a single noisy sample
  // dipping under the limit would otherwise re-open the gate on a robot wedged at
  // the threshold, granting a burst of driving per dip, forever.
  if (pitch > kUnlockTiltLimitDeg) {
    tilt_under_limit_since_micros_ = 0;
    if (tilt_over_limit_since_micros_ == 0) tilt_over_limit_since_micros_ = now;
    if (!tilt_over_limit_ && now - tilt_over_limit_since_micros_ > kUnlockTiltSustainMicros) {
      Lock lk{&mtx_};
      tilt_over_limit_ = true;
    }
  } else if (pitch < kUnlockTiltClearDeg) {
    tilt_over_limit_since_micros_ = 0;
    if (tilt_under_limit_since_micros_ == 0) tilt_under_limit_since_micros_ = now;
    if (tilt_over_limit_ && now - tilt_under_limit_since_micros_ > kUnlockTiltSustainMicros) {
      Lock lk{&mtx_};
      tilt_over_limit_ = false;
    }
  } else {
    // Hysteresis band: hold current state, reset both sustain timers.
    tilt_over_limit_since_micros_ = 0;
    tilt_under_limit_since_micros_ = 0;
  }

  // Hard TILT emergency: robot picked up / rolled, independent of wheel sensors.
  if (pitch > kTiltEstopDeg) {
    tilt_estop_under_since_micros_ = 0;
    if (tilt_estop_over_since_micros_ == 0) tilt_estop_over_since_micros_ = now;
    if (now - tilt_estop_over_since_micros_ > kTiltEstopSustainMicros) {
      tilt_estop_over_ = true;
    }
  } else if (pitch < kTiltEstopClearDeg) {
    tilt_estop_over_since_micros_ = 0;
    if (tilt_estop_under_since_micros_ == 0) tilt_estop_under_since_micros_ = now;
    if (now - tilt_estop_under_since_micros_ > kTiltEstopSustainMicros) {
      tilt_estop_over_ = false;
    }
  } else {
    // Hysteresis band: hold current state, reset both sustain timers.
    tilt_estop_over_since_micros_ = 0;
    tilt_estop_under_since_micros_ = 0;
  }

  // Asserted on the level, not on the edge: every other physical reason is re-applied
  // by CheckInputs each loop and self-heals if something clears it, and TILT must not
  // be the one reason a high-level clear can switch off while the robot is still on
  // its back. Latches like the input emergencies do - TILT drops when flat again, the
  // LATCH stays until the normal reset path. UpdateEmergency is a no-op when nothing
  // changed, so this costs nothing per tick.
  if (tilt_estop_over_) {
    UpdateEmergency(EmergencyReason::TILT | EmergencyReason::LATCH);
  } else {
    UpdateEmergency(0, EmergencyReason::TILT);
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
    // Remember what put the latch up, for as long as it stays up. Without this a
    // released stop button and a released wheel lift both look like a bare LATCH,
    // and the drive unlock would happily suppress either.
    if (add & EmergencyReason::LATCH) {
      latch_source_ |= add & static_cast<uint16_t>(~EmergencyReason::LATCH);
    }
    if (!(reasons_ & EmergencyReason::LATCH)) {
      latch_source_ = 0;
    }
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

uint16_t EmergencyService::EffectiveUnlockMaskLocked() const {
  uint16_t mask = active_unlock_mask_;
  // LATCH is only releasable when everything that raised it is itself unlockable.
  // A latch left standing by the stop button, or by a TILT that has since gone flat,
  // is the residue of a reason nobody may suppress.
  if ((latch_source_ & static_cast<uint16_t>(~kUnlockableMask)) != 0) {
    mask &= static_cast<uint16_t>(~EmergencyReason::LATCH);
  }
  return mask;
}

uint16_t EmergencyService::GetDriveBlockReasons() {
  Lock lk{&mtx_};
  return reasons_ & static_cast<uint16_t>(~EffectiveUnlockMaskLocked());
}

bool EmergencyService::IsDriveUnlockSuppressing() {
  Lock lk{&mtx_};
  return (reasons_ & EffectiveUnlockMaskLocked()) != 0;
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
  // What is actually suppressed right now, not what was asked for: an armed unlock
  // that is holding nothing back reports 0, so nothing downstream can claim the robot
  // is under override when it is simply driving.
  SendDriveUnlockState(reasons_ & EffectiveUnlockMaskLocked());
  CommitTransaction();
}
