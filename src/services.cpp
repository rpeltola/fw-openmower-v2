#include "services.hpp"

#include <service_ids.h>

#include "drivers/input/gpio_input_driver.hpp"
#ifdef DEBUG_BUILD
#include "drivers/input/simulated_input_driver.hpp"
#endif
#include "globals.hpp"

EmergencyService emergency_service{xbot::service_ids::EMERGENCY};
DiffDriveService diff_drive{xbot::service_ids::DIFF_DRIVE};
MowerService mower_service{xbot::service_ids::MOWER};
ImuService imu_service{xbot::service_ids::IMU};
BmsService bms_service{xbot::service_ids::BMS};
PowerService power_service{xbot::service_ids::POWER};
GpsService gps_service{xbot::service_ids::GPS};
InputService input_service{xbot::service_ids::INPUT};
HighLevelService high_level_service{xbot::service_ids::HIGH_LEVEL};
FilesystemService filesystem_service{xbot::service_ids::FILESYSTEM};
AudioService audio_service{xbot::service_ids::AUDIO};
SecurityService security_service{xbot::service_ids::SECURITY};

void StartServices() {
#define START_IF_NEEDED(service, id)                \
  if (robot->NeedsService(xbot::service_ids::id)) { \
    service.start();                                \
  }

  if (robot->NeedsService(xbot::service_ids::INPUT)) {
#ifndef ROBOT_PLATFORM_Sabo
    input_service.RegisterInputDriver("gpio", new GpioInputDriver{});
#endif
#ifdef DEBUG_BUILD
    input_service.RegisterInputDriver("simulated", new SimulatedInputDriver{});
#endif
  }

  START_IF_NEEDED(bms_service, BMS)

  if (robot->NeedsService(xbot::service_ids::INPUT)) {
    emergency_service.RequireService(&input_service);
  }
  if (robot->NeedsService(xbot::service_ids::DIFF_DRIVE)) {
    emergency_service.RequireService(&diff_drive);
  }
  if (robot->NeedsService(xbot::service_ids::MOWER)) {
    emergency_service.RequireService(&mower_service);
  }
  if (robot->NeedsService(xbot::service_ids::IMU)) {
    emergency_service.RequireService(&imu_service);
  }
  if (robot->NeedsService(xbot::service_ids::POWER)) {
    emergency_service.RequireService(&power_service);
  }

  START_IF_NEEDED(emergency_service, EMERGENCY)
  // Depends on EmergencyService (emergency-reason sound triggers) being up.
  START_IF_NEEDED(audio_service, AUDIO)
  START_IF_NEEDED(imu_service, IMU)
  START_IF_NEEDED(power_service, POWER)
  // Depends on EmergencyService (LIFT reasons) and PowerService (dock/charger presence) being up.
  START_IF_NEEDED(security_service, SECURITY)
  START_IF_NEEDED(diff_drive, DIFF_DRIVE)
  START_IF_NEEDED(mower_service, MOWER)
  START_IF_NEEDED(gps_service, GPS)
  START_IF_NEEDED(input_service, INPUT)
  START_IF_NEEDED(high_level_service, HIGH_LEVEL)
  START_IF_NEEDED(filesystem_service, FILESYSTEM)
}
