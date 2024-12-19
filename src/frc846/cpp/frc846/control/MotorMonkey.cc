#include "frc846/control/MotorMonkey.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>

#include "frc846/control/hardware/TalonFX_interm.h"

namespace frc846::control {

size_t MotorMonkey::slot_counter_{0};
std::map<size_t, frc846::control::base::MotorMonkeyType>
    MotorMonkey::slot_id_to_type_{};

frc846::control::hardware::IntermediateController*
    MotorMonkey::controller_registry[CONTROLLER_REGISTRY_SIZE]{};

units::volt_t MotorMonkey::battery_voltage{0_V};

void MotorMonkey::Tick() {
  battery_voltage = frc::RobotController::GetBatteryVoltage();

  for (size_t i = 0; i < CONTROLLER_REGISTRY_SIZE; i++) {
    if (controller_registry[i] != nullptr) {
      // TODO: DO STUFF HERE
    }
  }
}

size_t MotorMonkey::ConstructController(
    frc846::control::base::MotorMonkeyType type,
    frc846::control::config::MotorConstructionParameters params) {
  slot_counter_++;
  slot_id_to_type_[slot_counter_] = type;

  if (frc::RobotBase::IsSimulation()) {
  } else if (frc846::control::base::MotorMonkeyTypeHelper::is_talon_fx(type)) {
    frc846::control::hardware::IntermediateController* this_controller =
        controller_registry[slot_counter_] =
            new frc846::control::hardware::TalonFX_interm{
                params.can_id, params.bus, params.max_wait_time};

    this_controller->SetInverted(params.inverted);
    this_controller->SetNeutralMode(params.brake_mode);
    this_controller->SetCurrentLimit(params.motor_current_limit);
    this_controller->SetVoltageCompensation(params.voltage_compensation);
  }

  // if (frc::RobotBase::IsSimulation()) {
  //   controller_registry[slot_counter_] = nullptr;  // TODO: implement
  // } else if (frc846::control::base::MotorMonkeyTypeHelper::is_spark_flex(
  //                type)) {
  //   controller_registry[slot_counter_] = new rev::CANSparkFlex{
  //       params.can_id,
  //       rev::CANSparkLowLevel::MotorType::kBrushless};  // TODO: implement
  // } else if
  // (frc846::control::base::MotorMonkeyTypeHelper::is_spark_max(type)) {
  //   controller_registry[slot_counter_] = new rev::CANSparkMax{
  //       params.can_id,
  //       rev::CANSparkLowLevel::MotorType::kBrushless};  // TODO: implement
  // } else if (frc846::control::base::MotorMonkeyTypeHelper::is_talon_fx(type))
  // {
  //   controller_registry[slot_counter_] = new
  //   ctre::phoenix6::hardware::TalonFX{
  //       params.can_id, params.bus};  // TODO: implement
  //   ctre::phoenix6::hardware::TalonFX* this_device =
  //       static_cast<ctre::phoenix6::hardware::TalonFX*>(
  //           controller_registry[slot_counter_]);

  //   this_device->SetInverted(params.inverted);
  //   this_device->SetNeutralMode(
  //       params.brake_mode ? ctre::phoenix6::signals::NeutralModeValue::Brake
  //                         :
  //                         ctre::phoenix6::signals::NeutralModeValue::Coast);
  // }

  // TODO: Construct controller here

  return slot_counter_;
}

units::volt_t MotorMonkey::GetBatteryVoltage() { return battery_voltage; }

void MotorMonkey::SetLoad(size_t slot_id, units::newton_meter_t load) {}

void MotorMonkey::SetGains(size_t slot_id,
                           frc846::control::base::MotorGains gains) {}

void MotorMonkey::WriteDC(size_t slot_id, double duty_cycle) {}

void MotorMonkey::WriteVelocity(size_t slot_id,
                                units::radians_per_second_t velocity) {}

void MotorMonkey::WritePosition(size_t slot_id, units::radian_t position) {}

void MotorMonkey::ZeroEncoder(size_t slot_id, units::radian_t position) {}

units::radians_per_second_t MotorMonkey::GetVelocity(size_t slot_id) {
  return 0_rad_per_s;
}

units::radian_t MotorMonkey::GetPosition(size_t slot_id) { return 0_rad; }

units::ampere_t MotorMonkey::GetCurrent(size_t slot_id) { return 0_A; }

}  // namespace frc846::control