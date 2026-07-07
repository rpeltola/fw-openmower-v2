//
// Created by clemens on 31.07.24.
//

#ifndef MOWER_SERVICE_HPP
#define MOWER_SERVICE_HPP

#include <ch.h>
#include <etl/atomic.h>

#include <MowerServiceBase.hpp>
#include <debug/debug_tcp_interface.hpp>
#include <drivers/motor/motor_driver.hpp>
#include <globals.hpp>

using namespace xbot::driver::motor;
using namespace xbot::service;

// Threshold + hysteresis + debounce for the analog main-board rain sensor.
// Wet when the raw ADC reading exceeds the (per-platform) threshold; it only
// clears once the reading falls well below it, so the sensor stays "raining"
// until it genuinely dries out (a wet resistive sensor holds a moisture film
// and decays slowly). A separate debounce guards against single-sample spikes.
class RainDetector {
 public:
  // Schmitt trigger with debounce over the raw ADC reading: turns wet above
  // wet_threshold and dry again below 75% of it (the hysteresis band in between
  // holds the current state), flipping only after kDebounceSamples consecutive
  // samples past the far edge so single-sample spikes are ignored.
  // wet_threshold 0 disables detection. Returns the current raining state.
  bool Update(uint16_t raw, uint32_t wet_threshold) {
    if (wet_threshold == 0) {
      Reset();
      return false;
    }
    const uint32_t dry_threshold = wet_threshold - wet_threshold / 4;
    // Is the reading past the edge that would flip the current state?
    const bool wants_flip = raining_ ? (raw < dry_threshold) : (raw > wet_threshold);
    if (!wants_flip) {
      flip_streak_ = 0;
    } else if (++flip_streak_ >= kDebounceSamples) {
      raining_ = !raining_;
      flip_streak_ = 0;
    }
    return raining_;
  }

  bool IsRaining() const {
    return raining_;
  }

  // Clear any latched state (e.g. when detection is disabled at runtime) so a
  // later re-enable starts from a clean, debounced state.
  void Reset() {
    raining_ = false;
    flip_streak_ = 0;
  }

 private:
  static constexpr uint8_t kDebounceSamples = 2;
  bool raining_ = false;
  uint8_t flip_streak_ = 0;  // consecutive samples arguing to flip the current state
};

class MowerService : public MowerServiceBase {
 public:
  explicit MowerService(const uint16_t service_id) : MowerServiceBase(service_id, wa, sizeof(wa)) {
  }

  MotorDriver::ESCState GetESCState() const {
    return esc_state_;
  }

  void SetDriver(MotorDriver* motor_driver);

  void OnEmergencyChangedEvent();

  // Defined in the .cpp so it can consult power_service (intentional ESC
  // power-off is healthy, not a fault).
  bool IsHealthy() override;

 protected:
  void OnCreate() override;
  bool OnStart() override;
  void OnStop() override;

 private:
  void tick();
  ServiceSchedule tick_schedule_{*this, 500'000, XBOT_FUNCTION_FOR_METHOD(MowerService, &MowerService::tick, this)};

  void SetDuty();
  MUTEX_DECL(mtx);

  void ESCCallback(const MotorDriver::ESCState& state);

 protected:
  void OnMowerEnabledChanged(const uint8_t& new_value) override;
  void OnRainThresholdChanged(const uint32_t& new_value) override;

 private:
  THD_WORKING_AREA(wa, 1024){};
  MotorDriver::ESCState esc_state_{};
  bool esc_state_valid_ = false;
  uint32_t last_duty_received_micros_ = 0;
  uint32_t last_valid_esc_state_micros_ = 0;

  float mower_duty_ = 0;
  bool duty_sent_ = false;
  etl::atomic<bool> esc_ever_connected_{false};
  MotorDriver* mower_driver_ = nullptr;

  RainDetector rain_detector_;
  // Wet threshold (raw ADC counts) streamed from ROS via the "Rain Threshold"
  // input; 0 = disabled. Defaults to disabled until ROS provides a value.
  etl::atomic<uint32_t> rain_threshold_{0};
};

#endif  // MOWER_SERVICE_HPP
