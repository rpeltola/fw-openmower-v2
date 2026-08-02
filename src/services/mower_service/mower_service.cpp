//
// Created by clemens on 31.07.24.
//

#include "mower_service.hpp"

#include <cmath>
#include <xbot-service/portable/system.hpp>

#include "globals.hpp"
#include "services.hpp"

void MowerService::OnCreate() {
  chDbgAssert(mower_driver_ != nullptr, "Mower Motor Driver cannot be null!");
  mower_driver_->SetStateCallback(
      etl::delegate<void(const MotorDriver::ESCState&)>::create<MowerService, &MowerService::ESCCallback>(*this));
  mower_driver_->Start();
}

bool MowerService::OnStart() {
  mower_duty_ = 0;
  stall_latched_ = false;
  stall_condition_start_micros_ = 0;
  spinup_grace_until_micros_ = 0;
  return true;
}

void MowerService::OnStop() {
  mower_duty_ = 0;
  esc_ever_connected_ = false;
  stall_latched_ = false;
  stall_condition_start_micros_ = 0;
  spinup_grace_until_micros_ = 0;
}

void MowerService::tick() {
  chMtxLock(&mtx);

  // Check, if we recently received duty. If not, set to zero for safety
  if (xbot::service::system::getTimeMicros() - last_duty_received_micros_ > 10'000'000) {
    // it's ok to set it here, because we know that duty_set_ is false (we're in a timeout after all)
    mower_duty_ = 0;
  }

  // Stop commanding while power_service is intentionally idling the ESCs, so the
  // xESC command-timeout releases the motor and the gate driver can sleep.
  if (!duty_sent_ && !power_service.EscPowerIsOff()) {
    // Send motor speed to VESC, if we havent in the meantime
    // (e.g. due to new value or emergency)
    SetDuty();
  }

  mower_driver_->RequestStatus();

  // Rain detection: the platform's main-board analog sensor (threshold streamed
  // from ROS; 0 = disabled) OR'd with an optional Cover UI.
  const uint32_t rain_threshold = rain_threshold_.load();
  const int32_t rain_raw = robot->Mower_GetRainSensorRaw();  // -1 if no main-board sensor
  bool board_rain = false;
  if (rain_raw >= 0 && rain_threshold > 0) {
    board_rain = rain_detector_.Update(static_cast<uint16_t>(rain_raw), rain_threshold);
  } else {
    // No sensor or detection disabled: drop any latched state so a later
    // re-enable starts from a clean, debounced state.
    rain_detector_.Reset();
  }
  const bool rain_detected = board_rain || robot->Mower_IsCoverUiRainDetected();

  StartTransaction();
  SendRainDetected(rain_detected);
  if (rain_raw >= 0) {
    // Publish the live raw reading so the threshold can be set against it.
    SendRainValue(static_cast<uint32_t>(rain_raw));
  }

  // Check, if we have received ESC status updates recently. If not, send a disconnected message
  if (xbot::service::system::getTimeMicros() - last_valid_esc_state_micros_ > 1'000'000 || !esc_state_valid_) {
    // No recent update received (or none at all)
    mower_duty_ = 0;
    SendMowerStatus(static_cast<uint8_t>(power_service.EscPowerIsOff()
                                             ? MotorDriver::ESCState::ESCStatus::ESC_STATUS_POWERED_OFF
                                             : MotorDriver::ESCState::ESCStatus::ESC_STATUS_DISCONNECTED));
  } else {
    // We got recent data, send it
    StartTransaction();
    SendMowerESCTemperature(esc_state_.temperature_pcb);
    // Despite the name, this is the battery-side INPUT current, not the motor
    // phase current -- under a duty-mode stall it FALLS rather than rises (the
    // ESC's current controller cuts duty to hold phase current at its own
    // limit), so it is the wrong number to threshold a stall against. Kept as
    // "Mower Motor Current" (id 5) for compatibility; see id 10 below for the
    // real motor current.
    SendMowerMotorCurrent(esc_state_.current_input);
    SendMowerMotorPhaseCurrent(esc_state_.current_motor);

    // Stall detection. The ESC has no concept of stall: a duty-commanded
    // locked rotor regulates phase current down to its limit and sits below
    // every fault threshold it owns, indefinitely. Catch it here instead:
    // commanded duty above kStallDutyMin while eRPM stays under kStallRpmERpm
    // for kStallDwellUs, outside the post-spin-up grace window.
    const uint32_t now_us = xbot::service::system::getTimeMicros();
    const bool stall_condition = std::fabs(mower_duty_) > kStallDutyMin && std::fabs(esc_state_.rpm) < kStallRpmERpm;
    const bool in_spinup_grace = now_us < spinup_grace_until_micros_;
    if (stall_latched_) {
      // Latched: keep the blade off regardless of what tripped it before.
      // Only OnMowerSpeedChanged, on an explicit 0 command, may clear this.
      mower_duty_ = 0;
    } else if (!stall_condition || in_spinup_grace) {
      stall_condition_start_micros_ = 0;
    } else if (stall_condition_start_micros_ == 0) {
      stall_condition_start_micros_ = now_us;
    } else if (now_us - stall_condition_start_micros_ >= kStallDwellUs) {
      // Tripped: cut the blade so the stalled winding stops cooking, and
      // latch so firmware performs zero auto-retries (retry policy is ROS's).
      stall_latched_ = true;
      mower_duty_ = 0;
    }

    SendMowerStatus(stall_latched_ ? static_cast<uint8_t>(MotorDriver::ESCState::ESCStatus::ESC_STATUS_STALLED)
                                   : static_cast<uint8_t>(esc_state_.status));
    // The cause behind an ERROR status, when the ESC can name one (VESC mc_fault_code;
    // 0 = nothing reported). Sent beside the status, never folded into it.
    SendMowerESCFaultCode(esc_state_.fault_code);
    SendMowerMotorTemperature(esc_state_.temperature_motor);
    SendMowerRunning(std::fabs(esc_state_.rpm) > 0);
    SendMowerMotorRPM(std::fabs(esc_state_.rpm));
    // Actual motor direction reported by the ESC. Use esc_state_.direction, not
    // the sign of rpm: rpm sign is not portable across drivers (e.g. YFR4esc
    // reports an unsigned rpm). direction != 0 => reverse.
    SendMowerMotorDirection(esc_state_.direction > 0.5f ? MowerDirection::REVERSE : MowerDirection::FORWARD);
  }
  CommitTransaction();

  duty_sent_ = false;
  chMtxUnlock(&mtx);
}

void MowerService::ESCCallback(const MotorDriver::ESCState& state) {
  chMtxLock(&state_mutex_);
  esc_state_ = state;
  esc_state_valid_ = true;
  esc_ever_connected_ = true;
  last_valid_esc_state_micros_ = xbot::service::system::getTimeMicros();
  chMtxUnlock(&state_mutex_);
}

void MowerService::SetDuty() {
  // Get the current emergency state
  bool emergency = emergency_service.GetBladeBlockReasons() != 0;
  if (emergency) {
    mower_driver_->SetDuty(0);
  } else {
    mower_driver_->SetDuty(mower_duty_);
  }
  duty_sent_ = true;
}

void MowerService::OnMowerSpeedChanged(const float& new_value) {
  chMtxLock(&mtx);
  last_duty_received_micros_ = xbot::service::system::getTimeMicros();
  // Commanded normalized speed/duty in [-1, 1]: sign = direction, magnitude =
  // speed, 0 = off. Applied directly; any ramp on reversal is handled by the
  // ESC's own duty ramp (and ROS goes through 0 between mow sessions).
  const float clamped = new_value < -1.0f ? -1.0f : (new_value > 1.0f ? 1.0f : new_value);

  if (clamped == 0.0f) {
    // An explicit off is the only thing that releases a latched stall. ROS
    // re-sends the commanded speed every tick, so releasing on "any command
    // received" (or "any nonzero command") would clear the latch within one
    // tick and produce an infinite grind-retry loop -- exactly the failure
    // this exists to prevent. Firmware performs zero auto-retries; retry
    // policy lives in ROS.
    stall_latched_ = false;
  } else if (stall_latched_) {
    // Latched and still commanded on: ignore. mower_duty_ stays at 0 until
    // an explicit 0 arrives and re-arms detection.
    chMtxUnlock(&mtx);
    return;
  } else if (mower_duty_ == 0.0f) {
    // Rising edge 0 -> nonzero: (re)start the spin-up grace window so normal
    // ramp-up never trips stall detection.
    spinup_grace_until_micros_ = last_duty_received_micros_ + kSpinupGraceUs;
  }

  mower_duty_ = clamped;
  if (!duty_sent_) {
    SetDuty();
  }
  chMtxUnlock(&mtx);
}

void MowerService::OnRainThresholdChanged(const uint32_t& new_value) {
  rain_threshold_.store(new_value);
}

void MowerService::SetDriver(MotorDriver* motor_driver) {
  mower_driver_ = motor_driver;
}

bool MowerService::IsHealthy() {
  // Intentional ESC power-off (idle on the dock) is healthy, not a fault.
  return power_service.EscPowerIsOff() || (IsRunning() && esc_ever_connected_);
}
void MowerService::OnEmergencyChangedEvent() {
  bool emergency = emergency_service.GetBladeBlockReasons() != 0;
  if (!emergency) {
    // only set speed to 0 if the emergency happens, not if it's cleared
    return;
  }
  chMtxLock(&mtx);
  mower_duty_ = 0;
  // Instantly send the 0 duty cycle
  SetDuty();
  chMtxUnlock(&mtx);
}
