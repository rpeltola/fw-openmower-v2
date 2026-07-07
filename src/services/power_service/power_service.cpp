//
// Created by clemens on 09.09.24.
//

#include "power_service.hpp"

#include <ulog.h>

#include <cstdio>
#include <cstring>
#include <globals.hpp>
#include <services.hpp>

#include "board.h"
#include "drivers/adc/adc1.hpp"

using namespace xbot::driver;

namespace {
// The xESC SHUTDOWN line (GPIO0/PD4, shared by all three ESCs) is active-low:
// driving it low powers the ESCs off. The boot default is high-Z (held at the
// "on" level on the ESC side), so the ESCs are powered unless we drive this.
constexpr int ESC_SHUTDOWN_OFF_LEVEL = PAL_HIGH;  // drive high to idle the ESC; low = run (safe default)
constexpr int ESC_SHUTDOWN_ON_LEVEL = PAL_LOW;
}  // namespace

// Static assertions to ensure ChargerDriver::ReChargeVoltage enum matches PowerService ReChargeVoltages enum
static_assert(static_cast<uint8_t>(ChargerDriver::ReChargeVoltage::PERCENT_93_0) == 0, "ReChargeVoltage enum mismatch");
static_assert(static_cast<uint8_t>(ChargerDriver::ReChargeVoltage::PERCENT_94_3) == 1, "ReChargeVoltage enum mismatch");
static_assert(static_cast<uint8_t>(ChargerDriver::ReChargeVoltage::PERCENT_95_2) == 2, "ReChargeVoltage enum mismatch");
static_assert(static_cast<uint8_t>(ChargerDriver::ReChargeVoltage::PERCENT_97_6) == 3, "ReChargeVoltage enum mismatch");

void PowerService::SetDriver(ChargerDriver* charger_driver) {
  charger_ = charger_driver;
}

bool PowerService::OnStart() {
  charger_configured_ = false;
  if (DangerouslyOverrideHardwareChargeCurrentLimit.valid && DangerouslyOverrideHardwareChargeCurrentLimit.value) {
    ULOG_ARG_WARNING(
        &service_id_,
        "DangerouslyOverrideHardwareChargeCurrentLimit is set - hardware current limits will be bypassed!");
  }
  return true;
}

void PowerService::service_tick_() {
  xbot::service::Lock lk{&mtx_};
  // Send the sensor values
  StartTransaction();
  if (charger_configured_) {
    const char* status_text = ChargerDriver::statusToString(charger_status_);
    SendChargingStatus(status_text, strlen(status_text));
  } else {
    SendChargingStatus(CHARGE_STATUS_NOT_FOUND_STR, strlen(CHARGE_STATUS_NOT_FOUND_STR));
  }
  SendBatteryVoltage(battery_volts_);
  SendChargeVoltage(adapter_volts_);
  SendChargeCurrent(charge_current_);
  SendChargerEnabled(true);
  if (BatteryFullVoltage.valid && BatteryEmptyVoltage.valid) {
    battery_percent_ =
        (battery_volts_ - BatteryEmptyVoltage.value) / (BatteryFullVoltage.value - BatteryEmptyVoltage.value);
  } else {
    battery_percent_ = (battery_volts_ - robot->Power_GetDefaultBatteryEmptyVoltage()) /
                       (robot->Power_GetDefaultBatteryFullVoltage() - robot->Power_GetDefaultBatteryEmptyVoltage());
  }
  SendBatteryPercentage(etl::max(0.0f, etl::min(1.0f, battery_percent_)));
  SendChargerInputCurrent(adapter_current_);

  // ADC values
  SendBatteryVoltageADC(battery_volts_adc_);
  SendChargeVoltageADC(adapter_volts_adc_);
  SendDCDCInputCurrent(dcdc_current_);

  CommitTransaction();

  update_esc_power_();
}

void PowerService::update_esc_power_() {
  // ESC idle power-cut: when parked (high-level IDLE) on level ground, idle all
  // three xESCs via GPIO0/PD4 so they cool down - the motors are not needed while
  // idle. Leaving IDLE restores power, giving the xESCs time to wake before moving.
  // Pitch-gated because a released drive motor has no holding torque, so this is
  // only safe on (near) level ground.
  const uint8_t max_pitch = ShutdownESCMaxPitch.value;
  const bool idle = high_level_service.GetStateId() == HighLevelStatus::IDLE;
  const bool imu_ok = imu_service.IsFound();
  const float pitch = imu_service.GetPitch();
  const bool ready = max_pitch != 0 && idle && imu_ok && pitch <= static_cast<float>(max_pitch);

  // Edge-triggered diagnostic: when the cut is held off, log the (first) blocking
  // reason whenever it changes, so it is clear why the ESCs stay powered.
  // reason 0 = ready (the cut/restore action below logs that case).
  uint8_t reason;
  if (max_pitch == 0) {
    reason = 1;
  } else if (!idle) {
    reason = 2;
  } else if (!imu_ok) {
    reason = 3;
  } else if (pitch > static_cast<float>(max_pitch)) {
    reason = 4;
  } else {
    reason = 0;
  }
  if (reason != esc_block_reason_) {
    esc_block_reason_ = reason;
    switch (reason) {
      case 1: ULOG_ARG_INFO(&service_id_, "ESC power-cut disabled (shutdown_esc_max_pitch=0)"); break;
      case 2: ULOG_ARG_INFO(&service_id_, "ESC power-cut held off: high-level not IDLE"); break;
      case 3: ULOG_ARG_INFO(&service_id_, "ESC power-cut held off: IMU not available"); break;
      case 4:
        ULOG_ARG_INFO(&service_id_, "ESC power-cut held off: tilt %d > limit %u deg", static_cast<int>(pitch),
                      static_cast<unsigned>(max_pitch));
        break;
      default: break;  // reason 0 (ready): the cut action logs it
    }
  }

  // Drive the shutdown line: high = idle the xESCs, low = run. A custom xESC
  // sleeps its gate driver on this line; a stock xESC releases via its kill
  // switch (the default app config wires it here), so this degrades gracefully.
  palSetLineMode(LINE_GPIO0, PAL_MODE_OUTPUT_PUSHPULL);
  palWriteLine(LINE_GPIO0, ready ? ESC_SHUTDOWN_OFF_LEVEL : ESC_SHUTDOWN_ON_LEVEL);
  if (ready != esc_power_off_) {
    esc_power_off_ = ready;
    if (ready) {
      ULOG_ARG_INFO(&service_id_, "ESC power cut (idle, tilt %d deg <= %u); ESCs off to cool",
                    static_cast<int>(imu_service.GetPitch()), static_cast<unsigned>(max_pitch));
    } else {
      ULOG_ARG_INFO(&service_id_, "ESC power restored");
    }
  }
}

void PowerService::driver_tick_() {
  update_charger_();
  read_adc_();

  if (charger_configured_ && power_management_callback_) {
    power_management_callback_();
  }
}

void PowerService::read_adc_() {
  // Debugging: adc1::DumpBenchmarkMeasurement(Adc1ConversionId::V_BATTERY, "V-BAT");
  xbot::service::Lock lk{&mtx_};
  adapter_volts_adc_ = adc1::GetValueOrNaN(adc1::Adc1ConversionId::V_CHARGER, 100);
  battery_volts_adc_ = adc1::GetValueOrNaN(adc1::Adc1ConversionId::V_BATTERY, 100);
  dcdc_current_ = adc1::GetValueOrNaN(adc1::Adc1ConversionId::I_IN_DCDC, 100);
}

void PowerService::update_charger_() {
  if (charger_ == nullptr) {
    ULOG_ARG_ERROR(&service_id_, "Charger is null!");
    return;
  }

  if (!charger_configured_) {
    // charger not configured, configure it
    if (charger_->init()) {
      // Set the currents low
      bool success = true;
      if (PreChargeCurrent.valid && PreChargeCurrent.value > 0) {
        success &= charger_->setPreChargeCurrent(PreChargeCurrent.value);
      } else {
        success &= charger_->setPreChargeCurrent(robot->Power_GetDefaultPreChargeCurrent());
      }

      float software_charge_current = robot->Power_GetDefaultChargeCurrent();

      // Check, if the user has provided custom current. If so, use it
      if (ChargeCurrent.valid && ChargeCurrent.value > 0) {
        software_charge_current = ChargeCurrent.value;
      }

      // Check, if the user feels dangerous and allows higher charging currents
      bool override_limit =
          DangerouslyOverrideHardwareChargeCurrentLimit.valid && DangerouslyOverrideHardwareChargeCurrentLimit.value;

      if (!override_limit) {
        // Limit the current to the max value provided by the robot
        software_charge_current = std::min(robot->Power_GetMaxChargeCurrent(), software_charge_current);
      }

      // Hardware resistor is the default setting when not using software-control.
      // It's a very conservative choice so that charging without firmware is safe.
      // Therefore, we set "overwrite_hardware_limit" to true here, so that we can go for a higher current.
      // On watchdog timeout, the charger will automatically switch to hardware resistor.
      success &= charger_->setChargingCurrent(software_charge_current, true);

      if (ChargeVoltage.valid && ChargeVoltage.value > 0) {
        success &= charger_->setChargingVoltage(ChargeVoltage.value);
      } else {
        // Only set a custom value, if the robot implementation provides one.
        if (robot->Power_GetDefaultChargeVoltage() > 0.0) {
          success &= charger_->setChargingVoltage(robot->Power_GetDefaultChargeVoltage());
        }
      }
      if (TerminationCurrent.valid && TerminationCurrent.value > 0) {
        success &= charger_->setTerminationCurrent(TerminationCurrent.value);
      } else {
        success &= charger_->setTerminationCurrent(robot->Power_GetDefaultTerminationCurrent());
      }
      if (ReChargeVoltage.valid) {
        success &= charger_->setReChargeVoltage(static_cast<ChargerDriver::ReChargeVoltage>(ReChargeVoltage.value));
      } else {
        success &= charger_->setReChargeVoltage(robot->Power_GetDefaultReChargeVoltage());
      }

      // Disable temperature sense, the battery doesn't have it
      success &= charger_->setTsEnabled(false);
      charger_configured_ = success;
    }

    if (charger_configured_) {
      ULOG_ARG_INFO(&service_id_, "Successfully Configured Charger");
    } else {
      ULOG_ARG_ERROR(&service_id_, "Unable to Configure Charger");
    }
  } else {
    xbot::service::Lock lk{&mtx_};
    // charger is configured, do monitoring
    bool success = true;
    {
      bool s = charger_->resetWatchdog();
      if (!s) {
        ULOG_ARG_WARNING(&service_id_, "Error Resetting Watchdog");
      }
      success &= s;
    }
    {
      bool s = charger_->readChargeCurrent(charge_current_);
      if (!s) {
        ULOG_ARG_WARNING(&service_id_, "Error Reading Charge Current");
      }
      success &= s;
    }
    {
      bool s = charger_->readBatteryVoltage(battery_volts_);
      if (!s) {
        ULOG_ARG_WARNING(&service_id_, "Error Reading Battery Voltage");
      }
      success &= s;
    }
    {
      bool s = charger_->readAdapterVoltage(adapter_volts_);
      if (!s) {
        ULOG_ARG_WARNING(&service_id_, "Error Reading Adapter Voltage");
      }
      success &= s;
    }
    {
      bool s = charger_->readAdapterCurrent(adapter_current_);
      if (!s) {
        ULOG_ARG_WARNING(&service_id_, "Error Reading Adapter Current");
      }
      success &= s;
    }
    charger_status_ = charger_->getChargerStatus();

    if (!success || charger_status_ == CHARGER_STATUS::COMMS_ERROR) {
      // Error during comms or watchdog timer expired, reconfigure charger
      charger_configured_ = false;
      ULOG_ARG_ERROR(&service_id_, "Error during charging comms - reconfiguring");
    } else {
      if (battery_volts_ < robot->Power_GetAbsoluteMinVoltage()) {
        critical_count_++;
        if (critical_count_ > 10) {
          palClearLine(LINE_HIGH_LEVEL_GLOBAL_EN);
        }
      } else {
        critical_count_ = 0;
        palSetLine(LINE_HIGH_LEVEL_GLOBAL_EN);
      }
    }
  }
}

void PowerService::OnChargingAllowedChanged(const uint8_t& new_value) {
  (void)new_value;
}
