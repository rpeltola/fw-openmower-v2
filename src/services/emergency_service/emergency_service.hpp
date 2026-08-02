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
  // The blade path reads this: the raw reasons, which no unlock mask can ever reach.
  // "A lift sensor is active => the blade is dead" is therefore structural, not a
  // convention that a later feature could quietly break.
  uint16_t GetBladeBlockReasons();
  // The drive path reads this: the reasons minus whatever a scoped drive unlock is
  // currently suppressing. The mask is always 0 until the unlock primitive exists, so
  // this is behaviour-identical to GetEmergencyReasons() for now.
  uint16_t GetDriveBlockReasons();
  bool IsDriveUnlockActive();
  uint32_t CheckInputs(uint32_t now);

  void RequireService(ServiceExt* svc);

 protected:
  void OnStop() override;
  uint32_t OnLoop(uint32_t now_micros, uint32_t last_tick_micros) override;
  void OnHighLevelEmergencyChanged(const uint16_t* new_value, uint32_t length) override;

 private:
  uint32_t CheckTimeouts(uint32_t now);
  uint32_t CheckRequiredServices();
  void SendStatus();
  ServiceSchedule status_schedule_{*this, 1'000'000,
                                   XBOT_FUNCTION_FOR_METHOD(EmergencyService, &EmergencyService::SendStatus, this)};

  MUTEX_DECL(mtx_);

  void UpdateEmergency(uint16_t add, uint16_t clear = 0);

  uint16_t reasons_ = EmergencyReason::TIMEOUT_INPUTS | EmergencyReason::TIMEOUT_HIGH_LEVEL;
  uint32_t last_high_level_emergency_message_ = 0;

  // Reasons whose DRIVE-side consequence is currently suppressed. Nothing sets this
  // yet; it exists so the two getters above are the only place the distinction lives.
  uint16_t active_unlock_mask_ = 0;

  etl::vector<ServiceExt*, 16> required_services_{};
};

#endif  // EMERGENCY_SERVICE_HPP
