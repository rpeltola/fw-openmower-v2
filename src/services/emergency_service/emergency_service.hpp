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

  // Raw reasons, for display and announcement: what the operator should be told is
  // wrong. Anything that GATES an actuator must use one of the two getters below
  // instead, so it is explicit about which side of the split it is on.
  uint16_t GetEmergencyReasons();
  // Raw reasons - the blade path reads this; a drive unlock can never reach it.
  uint16_t GetBladeBlockReasons();
  // Reasons minus the currently active drive-unlock mask. Only
  // LIFT/LIFT_MULTIPLE/LATCH are ever suppressible; everything else always
  // blocks drive too.
  uint16_t GetDriveBlockReasons();
  // True only while an unlock is actually suppressing a reason that would otherwise
  // block the drive. An unlock armed against a robot with no emergency suppresses
  // nothing, and must not silently throttle normal driving.
  bool IsDriveUnlockSuppressing();
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
  // The unlock mask actually in force. Caller must hold mtx_.
  uint16_t EffectiveUnlockMaskLocked() const;
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
  // What was set at the moment LATCH went up. input_service latches EVERY input
  // carrying an `emergency` block, stop button included, so a released e-stop leaves
  // LATCH standing alone in reasons_ - indistinguishable from a released wheel lift
  // without this. Suppressing that would drive a robot whose e-stop the operator
  // believes is still holding.
  uint16_t latch_source_ = 0;
  uint32_t unlock_ttl_micros_ = kUnlockDefaultTtlMicros;
  uint32_t last_unlock_renewal_micros_ = 0;
  uint32_t unlock_session_start_micros_ = 0;
  uint32_t unlock_last_active_micros_ = 0;
  uint32_t unlock_lockout_start_micros_ = 0;
  bool unlock_lockout_ = false;
  uint32_t tilt_over_limit_since_micros_ = 0;
  uint32_t tilt_under_limit_since_micros_ = 0;
  bool tilt_over_limit_ = false;
  uint32_t tilt_estop_over_since_micros_ = 0;
  bool tilt_estop_over_ = false;
  uint32_t tilt_estop_under_since_micros_ = 0;

  static constexpr uint16_t kUnlockableMask =
      EmergencyReason::LIFT | EmergencyReason::LIFT_MULTIPLE | EmergencyReason::LATCH;
  static constexpr uint32_t kUnlockDefaultTtlMicros = 500'000;
  static constexpr uint32_t kUnlockMaxTtlMicros = 1'000'000;
  // Floor sits well above the 50 ms check interval, so the dead-man is enforced with
  // useful resolution rather than rounded up by the loop period.
  static constexpr uint32_t kUnlockMinTtlMicros = 200'000;
  static constexpr uint32_t kUnlockSessionCapMicros = 15'000'000;
  // Forced cool-down once the session cap trips. Nothing re-arms during it, not even
  // an explicit release: cycling the mask must not buy an unbounded unlock.
  static constexpr uint32_t kUnlockLockoutMicros = 3'000'000;
  // A gap shorter than this continues the current session instead of starting a new
  // one, so a requester whose dead-man expires between renewals cannot reset the
  // session clock every cycle and stay unlocked forever.
  static constexpr uint32_t kUnlockSessionGapMicros = 1'000'000;
  static constexpr float kUnlockTiltLimitDeg = 25.0f;
  static constexpr float kUnlockTiltClearDeg = 20.0f;
  static constexpr uint32_t kUnlockTiltSustainMicros = 300'000;
  static constexpr float kTiltEstopDeg = 45.0f;
  static constexpr float kTiltEstopClearDeg = 40.0f;
  static constexpr uint32_t kTiltEstopSustainMicros = 500'000;

  etl::vector<ServiceExt*, 16> required_services_{};
};

#endif  // EMERGENCY_SERVICE_HPP
