//
// Velocity/acceleration/jerk limiter for the diff-drive command twist.
//
// Ported 1:1 from ROS ros_controllers diff_drive_controller (BSD-3-Clause,
// Copyright (c) 2013, PAL Robotics, S.L.), so the firmware shapes the commanded
// twist exactly the way the canonical ROS controller does. Only the type (float)
// and namespacing differ from the original speed_limiter.{h,cpp}.
//

#ifndef DIFF_DRIVE_SPEED_LIMITER_HPP
#define DIFF_DRIVE_SPEED_LIMITER_HPP

namespace xbot::service {

class SpeedLimiter {
 public:
  SpeedLimiter(bool has_velocity_limits = false, bool has_acceleration_limits = false, bool has_jerk_limits = false,
               float min_velocity = 0.0f, float max_velocity = 0.0f, float min_acceleration = 0.0f,
               float max_acceleration = 0.0f, float min_jerk = 0.0f, float max_jerk = 0.0f)
      : has_velocity_limits(has_velocity_limits),
        has_acceleration_limits(has_acceleration_limits),
        has_jerk_limits(has_jerk_limits),
        min_velocity(min_velocity),
        max_velocity(max_velocity),
        min_acceleration(min_acceleration),
        max_acceleration(max_acceleration),
        min_jerk(min_jerk),
        max_jerk(max_jerk) {
  }

  // Limit the velocity and acceleration. v is the desired command (modified in place),
  // v0 the previous command, v1 the command before that, dt the time step [s].
  float limit(float& v, float v0, float v1, float dt) {
    const float tmp = v;
    limit_jerk(v, v0, v1, dt);
    limit_acceleration(v, v0, dt);
    limit_velocity(v);
    return tmp != 0.0f ? v / tmp : 1.0f;
  }

  float limit_velocity(float& v) {
    const float tmp = v;
    if (has_velocity_limits) {
      v = clamp(v, min_velocity, max_velocity);
    }
    return tmp != 0.0f ? v / tmp : 1.0f;
  }

  float limit_acceleration(float& v, float v0, float dt) {
    const float tmp = v;
    if (has_acceleration_limits) {
      const float dv_min = min_acceleration * dt;
      const float dv_max = max_acceleration * dt;
      const float dv = clamp(v - v0, dv_min, dv_max);
      v = v0 + dv;
    }
    return tmp != 0.0f ? v / tmp : 1.0f;
  }

  float limit_jerk(float& v, float v0, float v1, float dt) {
    const float tmp = v;
    if (has_jerk_limits) {
      const float dv = v - v0;
      const float dv0 = v0 - v1;
      const float dt2 = 2.0f * dt * dt;
      const float da_min = min_jerk * dt2;
      const float da_max = max_jerk * dt2;
      const float da = clamp(dv - dv0, da_min, da_max);
      v = v0 + dv0 + da;
    }
    return tmp != 0.0f ? v / tmp : 1.0f;
  }

  bool has_velocity_limits;
  bool has_acceleration_limits;
  bool has_jerk_limits;
  float min_velocity;
  float max_velocity;
  float min_acceleration;
  float max_acceleration;
  float min_jerk;
  float max_jerk;

 private:
  static float clamp(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
  }
};

}  // namespace xbot::service

#endif  // DIFF_DRIVE_SPEED_LIMITER_HPP
