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
  // Acceleration/jerk limiting is DISABLED for now so the transient response matches the
  // pre-physical firmware exactly (the FTC planner is not tuned for command ramping). The
  // accel limits stay here to enable + tune later as a smoothing step.
  static constexpr float kMaxLinVelMps = 0.6f;   // m/s
  static constexpr float kMaxLinAccMps2 = 0.5f;  // m/s^2  (unused while accel limiting off)
  static constexpr float kMaxAngVelRps = 3.0f;   // rad/s
  static constexpr float kMaxAngAccRps2 = 2.0f;  // rad/s^2 (unused while accel limiting off)
  SpeedLimiter limiter_lin_{true,           false, false, -kMaxLinVelMps, kMaxLinVelMps, -kMaxLinAccMps2,
                            kMaxLinAccMps2, 0.0f,  0.0f};
  SpeedLimiter limiter_ang_{true,           false, false, -kMaxAngVelRps, kMaxAngVelRps, -kMaxAngAccRps2,
                            kMaxAngAccRps2, 0.0f,  0.0f};
  // tick() period [s], must match tick_schedule_ (40 ms). Used as dt for the limiter.
  static constexpr float kTickPeriodS = 0.04f;

  // --- Firmware speed loop (duty_loop) --------------------------------------
  // Per-wheel target wheel speed [m/s] from the inverse kinematics. Right wheel is
  // mirrored, so its target is negated (same frame as the measured right speed).
  float target_v_l_ = 0;
  float target_v_r_ = 0;
  // PI integrator state, in actuator units (duty or amps).
  float integ_l_ = 0;
  float integ_r_ = 0;
  // Low-pass filtered measured wheel speed [m/s] (coarse 25 Hz tacho is noisy at low
  // speed; filtering stops the loop chasing quantization).
  float meas_filt_l_ = 0;
  float meas_filt_r_ = 0;
  // Previous actuator output, for slew-rate limiting (ramps through breakaway and caps
  // the dump on overshoot -> kills the stick-slip surge).
  float out_prev_l_ = 0;
  float out_prev_r_ = 0;
  // Built-in per-mode gains, used when the Loop Kp/Ki/Max registers are left at their
  // "unset" sentinel (Kp/Ki < 0, Max <= 0). Field-tune live via those registers
  // (ll/services/diff_drive/loop_kp etc.) without reflashing. These defaults are sized
  // to break stiction: the error is in m/s (~0.05) so the m/s->actuator gain is large.
  // duty_loop: output is duty [-1,1]; breakaway is ~0.16 duty on grass.
  static constexpr float kDutyLoopKp = 2.0f;  // duty per (m/s of error)
  static constexpr float kDutyLoopKi = 6.0f;  // duty per (m/s of error * s)

  // Measured-speed low-pass filter coefficient (EMA): filt += alpha * (meas - filt).
  // Lower = smoother but more lag. ~0.3 tames tacho quantization without much lag.
  static constexpr float kMeasFilterAlpha = 0.3f;
  // Built-in output slew-rate limits [actuator units per second], used when the Loop
  // Slew register is <= 0. Ramps the command through breakaway and caps the overshoot
  // dump, converting the stick-slip surge into a smooth creep.
  static constexpr float kDutyLoopSlew = 3.0f;  // duty per second
  // Tolerance band: within this speed error we command no correction, to avoid
  // hunting on the coarse low-speed hall feedback. Keep this WELL below the slowest
  // wheel speed we want to hold, or slow commands get swallowed and never drive.
  static constexpr float kSpeedDeadbandMps = 0.005f;

  // Runs one per-wheel PI step. Returns the actuator command (duty or amps),
  // clamped to +/-out_max, updated at no more than max_slew per second from prev_out,
  // and updates the integrator/prev_out in-place (with anti-windup). measured_v should
  // be the filtered wheel speed. A near-zero target cleanly stops (output 0, integrator
  // and slew memory reset) rather than holding torque at standstill.
  static float RunSpeedLoop(float target_v, float measured_v, float &integ, float &prev_out, float kp, float ki,
                            float out_max, float max_slew, float dt);

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
  ServiceSchedule tick_schedule_{*this, 40'000,
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
};

#endif  // DIFF_DRIVE_SERVICE_HPP
