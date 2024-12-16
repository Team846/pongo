#include "frc846/control/MotorMonkey.h"

namespace frc846::control {

ctre::phoenix6::hardware::TalonFX*
    MotorMonkey::talonFXRegistry[MAX_CAPACITY_PER_CONTROLLER_REGISTRY];
rev::CANSparkMax*
    MotorMonkey::sparkMaxRegistry[MAX_CAPACITY_PER_CONTROLLER_REGISTRY];
rev::CANSparkFlex*
    MotorMonkey::sparkFlexRegistry[MAX_CAPACITY_PER_CONTROLLER_REGISTRY];

void MotorMonkey::Tick() {}

size_t MotorMonkey::ConstructController(
    frc846::control::base::MotorMonkeyType type,
    frc846::control::config::MotorConstructionParameters params) {
  return 0;
}

units::volt_t MotorMonkey::GetBatteryVoltage() { return 0_V; }

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