//
// Created by clemens on 26.07.24.
//

#ifndef DIFF_DRIVE_SERVICE_HPP
#define DIFF_DRIVE_SERVICE_HPP

#include <drivers/gps/nmea_gps_driver.h>
#include <etl/atomic.h>

#include <DiffDriveServiceBase.hpp>
#include <drivers/motor/motor_driver.hpp>
#include <globals.hpp>
#include <xbot-service/portable/socket.hpp>

#include "speed_limiter.hpp"

using namespace xbot::service;
using namespace xbot::driver::motor;

class DiffDriveService : public DiffDriveServiceBase {
 private:
  THD_WORKING_AREA(wa, 1024){};
  MotorDriver *left_esc_driver_ = nullptr;
  MotorDriver *right_esc_driver_ = nullptr;

  MotorDriver::ESCState left_esc_state_{};
  MotorDriver::ESCState right_esc_state_{};
  bool left_esc_state_valid_ = false;
  bool right_esc_state_valid_ = false;
  uint32_t last_valid_esc_state_micros_ = 0;
  static constexpr uint8_t ESC_LEFT = 1 << 0;
  static constexpr uint8_t ESC_RIGHT = 1 << 1;
  etl::atomic<uint8_t> escs_connected_{0};
  uint32_t last_duty_received_micros_ = 0;

  uint32_t last_ticks_left = 0;
  uint32_t last_ticks_right = 0;
  bool last_ticks_valid = false;
  uint32_t last_ticks_micros_ = 0;

  // Per-wheel duty cycle [-1, 1] actually sent to the ESC (both modes output duty).
  float speed_l_ = 0;
  float speed_r_ = 0;
  // True once the command has been sent in the current tick window.
  bool command_sent_ = false;

  // Control Mode register values.
  //   duty       : open-loop duty (the mapped command IS the duty)  -- default, unchanged
  //   duty_loop  : firmware PI on measured wheel speed -> duty
  static constexpr uint8_t kControlModeDuty = 0;
  static constexpr uint8_t kControlModeDutyLoop = 1;
  // Wheel speed [m/s] that maps to full duty (1.0) in the open-loop duty mode. Only used
  // as the open-loop feedforward gain now that the command is a real m/s: duty = v / this.
  static constexpr float kMaxWheelSpeedMps = 0.5f;

  // --- Commanded twist (PHYSICAL units, from ROS) ---------------------------
  // The Control Twist is a real velocity: linear.x [m/s], angular.z [rad/s].
  float cmd_v_ = 0;  // commanded body linear velocity [m/s]
  float cmd_w_ = 0;  // commanded body angular velocity [rad/s]

  // --- diff_drive_controller command shaping (ported 1:1) -------------------
  // SpeedLimiter needs the previous two limited commands per axis for jerk limiting.
  float last0_v_ = 0, last1_v_ = 0;
  float last0_w_ = 0, last1_w_ = 0;
  // Velocity limits act as a safety cap; max angular = 2*v_max/track for in-place spins.
  // Acceleration limiting shapes the SETPOINT: with the feedforward in place the duty now
  // jumps on a command step, so the ramp belongs on the target, not on the output. The
  // limits sit above the ROS velocity_smoother's (2.5 / 3.2), so they never bind during
  // normal navigation - they exist to protect direct /ll/cmd_vel writers (drive_tune, joy).
  // The linear limit is runtime-settable via the Accel Limit register (see kAccelLimitMps2).
  static constexpr float kMaxLinVelMps = 0.6f;    // m/s
  static constexpr float kAccelLimitMps2 = 3.0f;  // m/s^2, built-in when Accel Limit <= 0
  static constexpr float kMaxAngVelRps = 3.0f;    // rad/s
  static constexpr float kMaxAngAccRps2 = 4.0f;   // rad/s^2
  SpeedLimiter limiter_lin_{true, true, false, -kMaxLinVelMps, kMaxLinVelMps, -kAccelLimitMps2, kAccelLimitMps2,
                            0.0f, 0.0f};
  SpeedLimiter limiter_ang_{true,           true, false, -kMaxAngVelRps, kMaxAngVelRps, -kMaxAngAccRps2,
                            kMaxAngAccRps2, 0.0f, 0.0f};
  // tick() period [s], must match tick_schedule_ (20 ms). Used as dt for the limiter.
  static constexpr float kTickPeriodS = 0.02f;

  // --- Firmware speed loop (duty_loop) --------------------------------------
  // Per-wheel target wheel speed [m/s] from the inverse kinematics. Right wheel is
  // mirrored, so its target is negated (same frame as the measured right speed).
  float target_v_l_ = 0;
  float target_v_r_ = 0;
  // PI integrator state, in actuator units (duty or amps).
  float integ_l_ = 0;
  float integ_r_ = 0;
  // Low-pass filtered measured wheel speed [m/s]. The feedback is the ESC's own eRPM,
  // which is already filtered, so this only takes the last of the noise off.
  float meas_filt_l_ = 0;
  float meas_filt_r_ = 0;
  // Previous PI term, for slew-rate limiting. Only the PI term is slewed - the
  // feedforward must be free to jump on a command step.
  float pi_prev_l_ = 0;
  float pi_prev_r_ = 0;
  // Built-in per-mode gains, used when the Loop Kp/Ki/Max registers are left at their
  // "unset" sentinel (Kp/Ki < 0, Max <= 0). Field-tune live via those registers
  // (ll/services/diff_drive/loop_kp etc.) without reflashing. These defaults are sized
  // to break stiction: the error is in m/s (~0.05) so the m/s->actuator gain is large.
  // duty_loop: output is duty [-1,1]; breakaway is ~0.16 duty on grass.
  static constexpr float kDutyLoopKp = 2.0f;  // duty per (m/s of error)
  static constexpr float kDutyLoopKi = 6.0f;  // duty per (m/s of error * s)
  // Feedforward: duty = ks*sign(target) + kv*target. ks is the static-friction (breakaway)
  // term and stays 0 until it is characterized on the robot; kv is the inverse of the
  // open-loop duty->speed gain, i.e. the same 1/kMaxWheelSpeedMps map the open-loop mode
  // uses, so duty_loop starts out at least as strong as open loop and the PI only trims.
  static constexpr float kDutyLoopKs = 0.0f;  // duty
  static constexpr float kDutyLoopKv = 2.0f;  // duty per (m/s)

  // Measured-speed low-pass filter coefficient (EMA): filt += alpha * (meas - filt).
  // Lower = smoother but more lag.
  static constexpr float kMeasFilterAlpha = 0.5f;
  // Built-in slew-rate limit for the PI term [duty per second], used when the Loop Slew
  // register is <= 0. Caps how fast the correction can move, so an overshoot cannot be
  // dumped in one cycle; the feedforward is not slewed.
  static constexpr float kDutyLoopSlew = 3.0f;  // duty per second
  // Bounds on the loop's dt [s], around the nominal 0.02 (50 Hz). A late or missed ESC
  // status must not turn into one giant integration + slew step; a suspiciously early one
  // must not divide by ~0. Only the control loop is clamped, never the odometry.
  static constexpr float kMinLoopDtS = 0.005f;
  static constexpr float kMaxLoopDtS = 0.06f;
  // Tolerance band: within this speed error we command no correction, to avoid
  // hunting on the coarse low-speed hall feedback. Keep this WELL below the slowest
  // wheel speed we want to hold, or slow commands get swallowed and never drive.
  static constexpr float kSpeedDeadbandMps = 0.005f;

  // Live gain overrides from the Loop Tuning input. Negative = not overridden, fall back
  // to the register (and, if that is at its sentinel, to the built-in). A register write
  // restarts the service, which clears these - the last full reconfigure wins.
  float tune_kp_ = -1;
  float tune_ki_ = -1;
  float tune_ks_ = -1;
  float tune_kv_ = -1;
  float tune_out_max_ = -1;
  float tune_slew_ = -1;

  // Control Debug flag bits (output id20, element 9).
  static constexpr uint16_t kFlagSatLeft = 1 << 0;
  static constexpr uint16_t kFlagSatRight = 1 << 1;
  static constexpr uint16_t kFlagSlewLeft = 1 << 2;
  static constexpr uint16_t kFlagSlewRight = 1 << 3;
  static constexpr uint16_t kFlagDutyLoop = 1 << 4;

  // Runs one per-wheel feedforward + PI step. Returns the duty command, clamped to
  // +/-out_max. The feedforward (ks*sign(target) + kv*target) is applied directly; only
  // the PI term is slew-limited (max_slew per second) so a command step still gets its
  // full feedforward immediately. The integrator uses conditional integration: it only
  // integrates while the output is unsaturated or the error points out of saturation.
  // measured_v should be the filtered wheel speed. A near-zero target cleanly stops
  // (output 0, integrator and slew memory reset) rather than holding torque at standstill.
  // saturated/slew_limited report which limits bound this step, for the debug frame.
  static float RunSpeedLoop(float target_v, float measured_v, float &integ, float &prev_pi, float kp, float ki,
                            float ks, float kv, float out_max, float max_slew, float dt, bool &saturated,
                            bool &slew_limited);

 public:
  explicit DiffDriveService(uint16_t service_id) : DiffDriveServiceBase(service_id, wa, sizeof(wa)) {
  }

  void OnEmergencyChangedEvent();

  void SetDrivers(MotorDriver *left_driver, MotorDriver *right_driver);

  // Defined in the .cpp so it can consult power_service (intentional ESC
  // power-off is healthy, not a fault).
  bool IsHealthy() override;

 protected:
  bool OnStart() override;
  void OnCreate() override;
  void OnStop() override;

 private:
  void tick();
  ServiceSchedule tick_schedule_{*this, 20'000,
                                 XBOT_FUNCTION_FOR_METHOD(DiffDriveService, &DiffDriveService::tick, this)};

  // Apply the SpeedLimiter to the commanded twist and run the inverse kinematics to
  // produce the per-wheel target speeds [m/s]. Called at the fixed tick rate.
  void UpdateCommand(float dt);
  // In open-loop duty mode, map the per-wheel target speed to the duty command. In
  // duty_loop the command is computed by the PI in ProcessStatusUpdate() instead.
  void UpdateOpenLoopCommand();
  // Zero the whole control chain (command, limiter history, targets, loop state).
  void ResetControlState();

  // Sends the current per-wheel command to both ESCs, dispatching to duty / speed /
  // current control based on the Control Mode register. Forces 0 on emergency.
  void SendMotorCommand();

  void LeftESCCallback(const MotorDriver::ESCState &state);
  void RightESCCallback(const MotorDriver::ESCState &state);
  void ProcessStatusUpdate();

 protected:
  void OnControlTwistChanged(const double *new_value, uint32_t length) override;
  void OnLoopTuningChanged(const double *new_value, uint32_t length) override;
};

#endif  // DIFF_DRIVE_SERVICE_HPP
