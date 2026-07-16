//
// Created by clemens on 26.07.24.
//

#include "diff_drive_service.hpp"

#include <ulog.h>

#include <drivers/motor/motor_driver.hpp>
#include <services.hpp>
#include <xbot-service/portable/system.hpp>

using namespace xbot::driver::motor;

void DiffDriveService::OnEmergencyChangedEvent() {
  bool emergency = emergency_service.GetEmergencyReasons() != 0;
  if (!emergency) {
    // only set speed to 0 if the emergency happens, not if it's cleared
    return;
  }
  chMtxLock(&state_mutex_);
  // Stop everything: command, limiter history, targets and loop state.
  ResetControlState();
  // Instantly send the 0 command
  SendMotorCommand();
  chMtxUnlock(&state_mutex_);
}

void DiffDriveService::ResetControlState() {
  cmd_v_ = cmd_w_ = 0;
  last0_v_ = last1_v_ = 0;
  last0_w_ = last1_w_ = 0;
  target_v_l_ = target_v_r_ = 0;
  speed_l_ = speed_r_ = 0;
  integ_l_ = integ_r_ = 0;
  meas_filt_l_ = meas_filt_r_ = 0;
  pi_prev_l_ = pi_prev_r_ = 0;
}
void DiffDriveService::UpdateCommand(float dt) {
  // Shape the commanded twist with velocity/acceleration limits (diff_drive_controller
  // SpeedLimiter), tracking the last two limited commands per axis for jerk limiting.
  // The linear acceleration limit is runtime-settable (Accel Limit register, <= 0 keeps
  // the built-in).
  const float accel_limit = AccelLimit.value > 0.0f ? AccelLimit.value : kAccelLimitMps2;
  limiter_lin_.max_acceleration = accel_limit;
  limiter_lin_.min_acceleration = -accel_limit;
  float v = cmd_v_;
  float w = cmd_w_;
  limiter_lin_.limit(v, last0_v_, last1_v_, dt);
  limiter_ang_.limit(w, last0_w_, last1_w_, dt);
  last1_v_ = last0_v_;
  last0_v_ = v;
  last1_w_ = last0_w_;
  last0_w_ = w;
  // Inverse kinematics: per-wheel linear speed [m/s]. The right wheel is mounted
  // mirrored, so its target is negated (same frame as its measured speed).
  const float half_track = 0.5f * static_cast<float>(WheelDistance.value);
  target_v_l_ = v - w * half_track;
  target_v_r_ = -(v + w * half_track);
}

void DiffDriveService::UpdateOpenLoopCommand() {
  if (ControlMode.value == kControlModeDutyLoop) {
    // Closed loop: speed_l_/speed_r_ are computed by the PI in ProcessStatusUpdate().
    return;
  }
  // Open-loop duty: duty proportional to the desired speed (feedforward; stalls at
  // low speed - that is the open-loop behaviour, unchanged). Clamp to [-1, 1].
  const float duty_l = target_v_l_ / kMaxWheelSpeedMps;
  const float duty_r = target_v_r_ / kMaxWheelSpeedMps;
  speed_l_ = duty_l > 1.0f ? 1.0f : (duty_l < -1.0f ? -1.0f : duty_l);
  speed_r_ = duty_r > 1.0f ? 1.0f : (duty_r < -1.0f ? -1.0f : duty_r);
}

void DiffDriveService::SetDrivers(MotorDriver* left_driver, MotorDriver* right_driver) {
  left_esc_driver_ = left_driver;
  right_esc_driver_ = right_driver;
}

bool DiffDriveService::IsHealthy() {
  // Intentional ESC power-off (idle on the dock) is healthy, not a fault.
  return power_service.EscPowerIsOff() || (IsRunning() && (escs_connected_.load() == (ESC_LEFT | ESC_RIGHT)));
}

bool DiffDriveService::OnStart() {
  // Check, if configuration is valid, if not retry
  if (WheelDistance.value == 0) {
    ULOG_ARG_ERROR(&service_id_, "WheelDistance was 0, cannot start service!");
    return false;
  }

  if (WheelTicksPerMeter.value == 0.0) {
    ULOG_ARG_ERROR(&service_id_, "WheelTicksPerMeter was 0, cannot start service!");
    return false;
  }

  ResetControlState();
  // A configuration transaction restarts the service, so any live Loop Tuning override
  // is dropped here: the newest full reconfigure wins over an older live tune.
  tune_kp_ = tune_ki_ = tune_ks_ = tune_kv_ = tune_out_max_ = tune_slew_ = -1.0f;
  last_ticks_valid = false;
  return true;
}

void DiffDriveService::OnCreate() {
  chDbgAssert(left_esc_driver_ != nullptr, "Left Motor Driver cannot be null!");
  chDbgAssert(right_esc_driver_ != nullptr, "Right Motor Driver cannot be null!");

  // Register callbacks
  left_esc_driver_->SetStateCallback(
      etl::delegate<void(const MotorDriver::ESCState&)>::create<DiffDriveService, &DiffDriveService::LeftESCCallback>(
          *this));
  right_esc_driver_->SetStateCallback(
      etl::delegate<void(const MotorDriver::ESCState&)>::create<DiffDriveService, &DiffDriveService::RightESCCallback>(
          *this));

  left_esc_driver_->Start();
  right_esc_driver_->Start();
}

void DiffDriveService::OnStop() {
  ResetControlState();
  last_ticks_valid = false;
  escs_connected_ = 0;
}

void DiffDriveService::tick() {
  chMtxLock(&state_mutex_);

  // Check, if we recently received a command. If not, hard-stop for safety (bypass the
  // limiter ramp - a comms loss should stop, not coast down).
  if (xbot::service::system::getTimeMicros() - last_duty_received_micros_ > 1'000'000) {
    ResetControlState();
  } else {
    // Shape the commanded twist (SpeedLimiter) and run the inverse kinematics at the
    // fixed tick rate, then compute the open-loop actuator command. The closed-loop
    // modes compute speed_l_/speed_r_ from these targets in ProcessStatusUpdate().
    UpdateCommand(kTickPeriodS);
    UpdateOpenLoopCommand();
  }

  // Stop commanding while power_service is intentionally idling the ESCs, so the
  // xESC command-timeout releases the motor and the gate driver can sleep.
  if (!command_sent_ && !power_service.EscPowerIsOff()) {
    SendMotorCommand();
  }

  left_esc_driver_->RequestStatus();
  right_esc_driver_->RequestStatus();

  // Check, if we have received ESC status updates recently. If not, send a disconnected message
  if (xbot::service::system::getTimeMicros() - last_valid_esc_state_micros_ > 1'000'000) {
    const auto no_data_status = power_service.EscPowerIsOff()
                                    ? MotorDriver::ESCState::ESCStatus::ESC_STATUS_POWERED_OFF
                                    : MotorDriver::ESCState::ESCStatus::ESC_STATUS_DISCONNECTED;
    StartTransaction();
    if (!left_esc_state_valid_) {
      SendLeftESCStatus(static_cast<uint8_t>(no_data_status));
    }
    if (!right_esc_state_valid_) {
      SendRightESCStatus(static_cast<uint8_t>(no_data_status));
    }
    CommitTransaction();
  }

  command_sent_ = false;
  chMtxUnlock(&state_mutex_);
}

void DiffDriveService::SendMotorCommand() {
  // Get the current emergency state. On emergency we always command 0.
  bool emergency = emergency_service.GetEmergencyReasons() != 0;
  // Both duty and duty_loop output a duty cycle in [-1, 1].
  const float cmd_l = emergency ? 0.0f : speed_l_;
  const float cmd_r = emergency ? 0.0f : speed_r_;
  left_esc_driver_->SetDuty(cmd_l);
  right_esc_driver_->SetDuty(cmd_r);
  command_sent_ = true;
}

float DiffDriveService::RunSpeedLoop(float target_v, float measured_v, float& integ, float& prev_pi, float kp, float ki,
                                     float ks, float kv, float out_max, float max_slew, float dt, bool& saturated,
                                     bool& slew_limited) {
  saturated = false;
  slew_limited = false;
  // Clean stop: don't hold torque against a standstill (avoids energizing/heating
  // the ESC and low-speed hunting when the mower is meant to be still).
  if (target_v > -1e-3f && target_v < 1e-3f) {
    integ = 0.0f;
    prev_pi = 0.0f;
    return 0.0f;
  }
  // Feedforward: the duty the wheel is expected to need at this speed. This is what
  // gets the wheel moving; the PI only trims what the model gets wrong.
  const float ff = (target_v > 0.0f ? ks : -ks) + kv * target_v;

  float err = target_v - measured_v;
  // Tolerance band around the target.
  if (err < kSpeedDeadbandMps && err > -kSpeedDeadbandMps) {
    err = 0.0f;
  }
  // Conditional integration (anti-windup): integrate only while the output is off the
  // rail, or while the error is pushing back off it. Clamping the integrator instead
  // would let it sit wound up against a rail and overshoot on release.
  const float out_pre = ff + kp * err + integ;
  const bool sat_hi = out_pre > out_max;
  const bool sat_lo = out_pre < -out_max;
  if ((!sat_hi && !sat_lo) || (sat_hi && err < 0.0f) || (sat_lo && err > 0.0f)) {
    integ += ki * err * dt;
  }

  float pi = kp * err + integ;
  // Slew-rate limit the PI TERM ONLY: the feedforward must be free to jump on a command
  // step (the setpoint accel limit shapes that), while the correction stays smooth so a
  // stick-slip overshoot cannot be dumped in one cycle.
  if (max_slew > 0.0f) {
    const float max_delta = max_slew * dt;
    if (pi > prev_pi + max_delta) {
      pi = prev_pi + max_delta;
      slew_limited = true;
    } else if (pi < prev_pi - max_delta) {
      pi = prev_pi - max_delta;
      slew_limited = true;
    }
  }
  prev_pi = pi;

  float out = ff + pi;
  if (out > out_max) {
    out = out_max;
    saturated = true;
  } else if (out < -out_max) {
    out = -out_max;
    saturated = true;
  }
  return out;
}

void DiffDriveService::LeftESCCallback(const MotorDriver::ESCState& state) {
  chMtxLock(&state_mutex_);
  left_esc_state_ = state;
  left_esc_state_valid_ = true;
  escs_connected_ |= ESC_LEFT;
  if (right_esc_state_valid_) {
    ProcessStatusUpdate();
  }
  chMtxUnlock(&state_mutex_);
}

void DiffDriveService::RightESCCallback(const MotorDriver::ESCState& state) {
  chMtxLock(&state_mutex_);
  right_esc_state_ = state;
  right_esc_state_valid_ = true;
  escs_connected_ |= ESC_RIGHT;
  if (left_esc_state_valid_) {
    ProcessStatusUpdate();
  }
  chMtxUnlock(&state_mutex_);
}

void DiffDriveService::ProcessStatusUpdate() {
  uint32_t micros = xbot::service::system::getTimeMicros();
  last_valid_esc_state_micros_ = micros;
  StartTransaction();
  SendLeftESCTemperature(left_esc_state_.temperature_pcb);
  SendLeftESCCurrent(left_esc_state_.current_input);
  SendLeftESCStatus(static_cast<uint8_t>(left_esc_state_.status));

  SendRightESCTemperature(right_esc_state_.temperature_pcb);
  SendRightESCCurrent(right_esc_state_.current_input);
  SendRightESCStatus(static_cast<uint8_t>(right_esc_state_.status));

  // Forward the remaining ESC telemetry already parsed into ESCState. Direction is not
  // sent: it is sign(rpm) (see the VESC driver, which derives it exactly that way), and
  // the service description has to fit a single advertise datagram, so the consumer
  // derives it from Rpm instead.
  SendLeftESCRpm(left_esc_state_.rpm);
  SendLeftESCDutyCycle(left_esc_state_.duty_cycle);
  SendLeftESCInputVoltage(left_esc_state_.voltage_input);
  SendLeftESCMotorTemperature(left_esc_state_.temperature_motor);
  SendLeftESCTachoAbsolute(left_esc_state_.tacho_absolute);

  SendRightESCRpm(right_esc_state_.rpm);
  SendRightESCDutyCycle(right_esc_state_.duty_cycle);
  SendRightESCInputVoltage(right_esc_state_.voltage_input);
  SendRightESCMotorTemperature(right_esc_state_.temperature_motor);
  SendRightESCTachoAbsolute(right_esc_state_.tacho_absolute);

  // Calculate the twist according to wheel ticks
  if (last_ticks_valid) {
    float dt = static_cast<float>(micros - last_ticks_micros_) / 1'000'000.0f;
    int32_t d_left = static_cast<int32_t>(left_esc_state_.tacho - last_ticks_left);
    int32_t d_right = static_cast<int32_t>(right_esc_state_.tacho - last_ticks_right);
    float vx = static_cast<float>(d_left - d_right) / (2.0f * dt * static_cast<float>(WheelTicksPerMeter.value));
    float vr = -static_cast<float>(d_left + d_right) /
               (static_cast<float>(WheelDistance.value) * dt * static_cast<float>(WheelTicksPerMeter.value));
    double data[6]{};
    data[0] = vx;
    data[5] = vr;
    SendActualTwist(data, 6);
    uint32_t ticks[2];
    ticks[0] = left_esc_state_.tacho;
    ticks[1] = right_esc_state_.tacho;
    SendWheelTicks(ticks, 2);

    // Firmware speed loop: close a per-wheel feedforward+PI on measured wheel speed.
    // Runs at the ESC telemetry rate (dt from the status window).
    if (ControlMode.value == kControlModeDutyLoop) {
      // Feedback is the ESC's own eRPM, not the tacho delta: the tacho only ticks 6 times
      // per electrical revolution, so over one window it quantizes the speed far too
      // coarsely to close a loop at walking pace. eRPM/60 elec-rev/s * 6 ticks/elec-rev
      // = eRPM/10 ticks/s; divided by ticks-per-meter that is m/s, with the pole pairs
      // cancelling out. The sign convention is the ESC's own, i.e. the same one the tacho
      // delta above uses, so the mirrored right wheel needs no extra negation (its target
      // is already mirrored to match).
      // Odometry (Actual Twist / Wheel Ticks) deliberately stays tacho-based.
      const float meas_v_l = left_esc_state_.rpm * 0.1f / static_cast<float>(WheelTicksPerMeter.value);
      const float meas_v_r = right_esc_state_.rpm * 0.1f / static_cast<float>(WheelTicksPerMeter.value);
      // Low-pass the measured speed before the loop sees it (EMA).
      meas_filt_l_ += kMeasFilterAlpha * (meas_v_l - meas_filt_l_);
      meas_filt_r_ += kMeasFilterAlpha * (meas_v_r - meas_filt_r_);
      // Gain resolution, in order: Loop Tuning input (live, negative = not set) overrides
      // the register (sentinel: Kp/Ki/Ks/Kv < 0, Max/Slew <= 0), which overrides the
      // built-in. So gains can be tuned live via the input, or set per-robot via
      // ll/services/diff_drive/loop_kp etc.
      const float kp = tune_kp_ >= 0.0f ? tune_kp_ : (LoopKp.value >= 0.0f ? LoopKp.value : kDutyLoopKp);
      const float ki = tune_ki_ >= 0.0f ? tune_ki_ : (LoopKi.value >= 0.0f ? LoopKi.value : kDutyLoopKi);
      const float ks = tune_ks_ >= 0.0f ? tune_ks_ : (LoopKs.value >= 0.0f ? LoopKs.value : kDutyLoopKs);
      const float kv = tune_kv_ >= 0.0f ? tune_kv_ : (LoopKv.value >= 0.0f ? LoopKv.value : kDutyLoopKv);
      const float out_max =
          tune_out_max_ > 0.0f ? tune_out_max_ : (LoopMaxOutput.value > 0.0f ? LoopMaxOutput.value : 1.0f);
      const float max_slew = tune_slew_ > 0.0f ? tune_slew_ : (LoopSlew.value > 0.0f ? LoopSlew.value : kDutyLoopSlew);
      bool sat_l = false, sat_r = false, slew_l = false, slew_r = false;
      speed_l_ = RunSpeedLoop(target_v_l_, meas_filt_l_, integ_l_, pi_prev_l_, kp, ki, ks, kv, out_max, max_slew, dt,
                              sat_l, slew_l);
      speed_r_ = RunSpeedLoop(target_v_r_, meas_filt_r_, integ_r_, pi_prev_r_, kp, ki, ks, kv, out_max, max_slew, dt,
                              sat_r, slew_r);
      SendMotorCommand();

      // Debug frame for the tuning harness: everything needed to reconstruct one loop
      // iteration, stamped with the firmware clock so it can be aligned without guessing
      // at transport latency.
      uint16_t flags = kFlagDutyLoop;
      if (sat_l) flags |= kFlagSatLeft;
      if (sat_r) flags |= kFlagSatRight;
      if (slew_l) flags |= kFlagSlewLeft;
      if (slew_r) flags |= kFlagSlewRight;
      double debug[10];
      debug[0] = static_cast<double>(micros) * 1e-6;
      debug[1] = target_v_l_;
      debug[2] = target_v_r_;
      debug[3] = meas_filt_l_;
      debug[4] = meas_filt_r_;
      debug[5] = speed_l_;
      debug[6] = speed_r_;
      debug[7] = integ_l_;
      debug[8] = integ_r_;
      debug[9] = flags;
      SendControlDebug(debug, 10);
    }
  }
  last_ticks_valid = true;
  last_ticks_left = left_esc_state_.tacho;
  last_ticks_right = right_esc_state_.tacho;
  last_ticks_micros_ = micros;

  right_esc_state_valid_ = left_esc_state_valid_ = false;

  CommitTransaction();
}

void DiffDriveService::OnControlTwistChanged(const double* new_value, uint32_t length) {
  if (length != 6) return;
  chMtxLock(&state_mutex_);
  last_duty_received_micros_ = xbot::service::system::getTimeMicros();
  // The Control Twist is now a PHYSICAL velocity: linear.x [m/s], angular.z [rad/s].
  // Just store it here; tick() applies the SpeedLimiter and inverse kinematics at a
  // fixed rate (so the limiter's dt is consistent) and dispatches the command.
  cmd_v_ = static_cast<float>(new_value[0]);
  cmd_w_ = static_cast<float>(new_value[5]);
  chMtxUnlock(&state_mutex_);
}

void DiffDriveService::OnLoopTuningChanged(const double* new_value, uint32_t length) {
  if (length != 6) return;
  // [kp, ki, ks, kv, out_max, slew]. A negative (or NaN) element leaves that gain alone,
  // so a single gain can be swept without restating the rest. Unlike a register write
  // this takes effect on the next loop iteration and does not restart the service -
  // that is the point: it is what the tuning harness sweeps gains with while driving.
  chMtxLock(&state_mutex_);
  float* targets[6] = {&tune_kp_, &tune_ki_, &tune_ks_, &tune_kv_, &tune_out_max_, &tune_slew_};
  for (uint32_t i = 0; i < 6; i++) {
    const double v = new_value[i];
    // NaN compares false against itself; skip it along with the negative sentinel.
    if (v >= 0.0) {
      *targets[i] = static_cast<float>(v);
    }
  }
  chMtxUnlock(&state_mutex_);
}
