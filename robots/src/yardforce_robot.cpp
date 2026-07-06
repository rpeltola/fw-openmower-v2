#include "../include/yardforce_robot.hpp"

#include <etl/array_view.h>
#include <ulog.h>

#include <drivers/adc/adc1.hpp>
#include <services.hpp>

using namespace xbot::driver::adc1;

void YardForceRobot::InitPlatform() {
  InitMotors();
  charger_.setI2C(&I2CD1);
  power_service.SetDriver(&charger_);

#if !STM32_UART_USE_UART7
#error STM32_UART_USE_UART7 must be enabled for the YF Cover UI to work
#endif
  input_service.RegisterInputDriver("yf_cover_ui", &yf_cover_ui_);
  yf_cover_ui_.Start(&UARTD7);

  // Bring up ADC1 for the main-board rain sensor (read in Mower_GetRainSensorRaw()).
  adc1::Init();
  if (!adc1::Start()) {
    ULOG_ERROR("YardForce: ADC1 start failed; main-board rain sensor unavailable");
  }
}

int32_t YardForceRobot::Mower_GetRainSensorRaw() {
  // AGPIO3 / PA6 / ADC1_IN3. Long sample time + 16x hardware oversampling to
  // tame the noisy high-impedance resistive reading. The group is converted
  // directly (never registered), so the id/delegate are unused; raw counts are
  // read straight from the buffer.
  static const Adc1Sensor sensors[] = {{.channel = ADC_CHANNEL_IN3, .sample_rate = ADC_SMPR_SMP_384P5}};
  static adcsample_t buffer[1];
  static Adc1ConversionGroup cg = Adc1ConversionGroup::Create(
      Adc1ConversionId::RAIN, etl::array_view<const Adc1Sensor>(sensors), buffer, Resolution::BITS_16, 16, 4,
      [](const Adc1ConversionGroup*, float) -> float { return 0.0f; });

  if (!adc1::Convert(&cg)) {
    return -1;
  }
  return static_cast<int32_t>(buffer[0]);
}

bool YardForceRobot::IsHardwareSupported() {
  // Accept YardForce 1.x.x boards
  if (strncmp("hw-openmower-yardforce", carrier_board_info.board_id, sizeof(carrier_board_info.board_id)) == 0 &&
      carrier_board_info.version_major == 1) {
    return true;
  }

  // Accept early testing boards
  if (strncmp("hw-xbot-devkit", carrier_board_info.board_id, sizeof(carrier_board_info.board_id)) == 0) {
    return true;
  }

  return false;
}
