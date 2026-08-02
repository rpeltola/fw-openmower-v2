//
// Created by clemens on 26.07.24.
//

#ifndef EMERGENCY_SERVICE_HPP
#define EMERGENCY_SERVICE_HPP

#include <etl/string.h>
#include <etl/vector.h>

#include <EmergencyServiceBase.hpp>

#include "globals.hpp"

using namespace xbot::service;

class EmergencyService : public EmergencyServiceBase {
 private:
  THD_WORKING_AREA(wa, 1024){};

 public:
  explicit EmergencyService(uint16_t service_id) : EmergencyServiceBase(service_id, wa, sizeof(wa)) {
  }

  uint16_t GetEmergencyReasons();
  // Raw reasons - the blade path reads this; a drive unlock can never reach it.
  uint16_t GetBladeBlockReasons();
  // Reasons minus the currently active drive-unlock mask. Only
  // LIFT/LIFT_MULTIPLE/LATCH are ever suppressible; everything else always
  // blocks drive too.
  uint16_t GetDriveBlockReasons();
  bool IsDriveUnlockActive();
  uint32_t CheckInputs(uint32_t now);

  void RequireService(ServiceExt* svc);

 protected:
  void OnStop() override;
  uint32_t OnLoop(uint32_t now_micros, uint32_t last_tick_micros) override;
  void OnHighLevelEmergencyChanged(const uint16_t* new_value, uint32_t length) override;
  void OnDriveUnlockRequestChanged(const uint16_t* new_value, uint32_t length) override;

 private:
  uint32_t CheckTimeouts(uint32_t now);
  uint32_t CheckRequiredServices();
  uint32_t CheckDriveUnlock(uint32_t now);
  uint32_t CheckTilt(uint32_t now);
  void SendStatus();
  ServiceSchedule status_schedule_{*this, 1'000'000,
                                   XBOT_FUNCTION_FOR_METHOD(EmergencyService, &EmergencyService::SendStatus, this)};

  MUTEX_DECL(mtx_);

  void UpdateEmergency(uint16_t add, uint16_t clear = 0);

  uint16_t reasons_ = EmergencyReason::TIMEOUT_INPUTS | EmergencyReason::TIMEOUT_HIGH_LEVEL;
  uint32_t last_high_level_emergency_message_ = 0;

  // Drive unlock session state. The mask suppresses drive-side consequences of
  // the listed reasons only; it is dead-man renewed, session-capped and dropped
  // on tilt. All transitions send Drive Unlock State so consumers render truth.
  uint16_t active_unlock_mask_ = 0;
  uint32_t unlock_ttl_micros_ = kUnlockDefaultTtlMicros;
  uint32_t last_unlock_renewal_micros_ = 0;
  uint32_t unlock_session_start_micros_ = 0;
  bool unlock_session_lockout_ = false;
  uint32_t tilt_over_limit_since_micros_ = 0;
  bool tilt_over_limit_ = false;
  uint32_t tilt_estop_over_since_micros_ = 0;
  bool tilt_estop_over_ = false;
  uint32_t tilt_estop_under_since_micros_ = 0;

  static constexpr uint16_t kUnlockableMask =
      EmergencyReason::LIFT | EmergencyReason::LIFT_MULTIPLE | EmergencyReason::LATCH;
  static constexpr uint32_t kUnlockDefaultTtlMicros = 500'000;
  static constexpr uint32_t kUnlockMaxTtlMicros = 1'000'000;
  static constexpr uint32_t kUnlockMinTtlMicros = 100'000;
  static constexpr uint32_t kUnlockSessionCapMicros = 15'000'000;
  static constexpr uint32_t kUnlockLockoutGapMicros = 1'000'000;
  static constexpr float kUnlockTiltLimitDeg = 25.0f;
  static constexpr uint32_t kUnlockTiltSustainMicros = 300'000;
  static constexpr float kTiltEstopDeg = 45.0f;
  static constexpr float kTiltEstopClearDeg = 40.0f;
  static constexpr uint32_t kTiltEstopSustainMicros = 500'000;

  etl::vector<ServiceExt*, 16> required_services_{};
};

#endif  // EMERGENCY_SERVICE_HPP
