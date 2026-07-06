#ifndef YARDFORCE_ROBOT_HPP
#define YARDFORCE_ROBOT_HPP

#include <drivers/charger/bq_2576/bq_2576.hpp>
#include <drivers/ui/yf_cover_ui/yf_cover_ui.hpp>

#include "robot.hpp"

class YardForceRobot : public MowerRobot {
 public:
  void InitPlatform() override;
  bool IsHardwareSupported() override;

  UARTDriver* GPS_GetUartPort() override {
#ifndef STM32_UART_USE_USART6
#error STM32_UART_USE_USART6 must be enabled for the YardForce build to work
#endif
    return &UARTD6;
  }

  float Power_GetDefaultBatteryFullVoltage() override {
    return 7.0f * 4.2f;
  }

  float Power_GetDefaultBatteryEmptyVoltage() override {
    return 7.0f * 3.3f;
  }

  float Power_GetDefaultChargeCurrent() override {
    return 1.0;
  }

  float Power_GetMaxChargeCurrent() override {
    return 1.0;
  }

  float Power_GetAbsoluteMinVoltage() override {
    // 3.3V min, 7s pack
    return 7.0f * 3.0;
  }

  // Main-board rain sensor: AGPIO3 / PA6 / ADC1_IN3 (passive 2-plate resistive,
  // wet raises the reading). The wet threshold is provided at runtime from ROS
  // (MowerService "Rain Threshold" input, 0 = disabled).
  int32_t Mower_GetRainSensorRaw() override;

  bool Mower_IsCoverUiRainDetected() override {
    // Only trust the rain flag while the panel is actually responding: the
    // driver doesn't clear rain_detected_ when the UI goes unavailable, so a
    // disconnected panel that last reported rain would otherwise latch it on.
    return yf_cover_ui_.IsAvailable() && yf_cover_ui_.IsRainDetected();
  }

 private:
  BQ2576 charger_{249000, 14040};
  xbot::driver::ui::YFCoverUI yf_cover_ui_{};
};

#endif  // YARDFORCE_ROBOT_HPP
