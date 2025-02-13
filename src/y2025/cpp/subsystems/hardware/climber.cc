#include "subsystems/hardware/climber.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/robot_constants.h"

ClimberSubsystem::ClimberSubsystem()
    : WristSubsystem("climber",
          frc846::control::base::MotorMonkeyType::SPARK_FLEX_VORTEX,
          frc846::control::config::MotorConstructionParameters{
              .can_id = ports::climber_::kClimber_CANID,
              .inverted = false,
              .brake_mode = true,
              .motor_current_limit = 40_A,
              .smart_current_limit = 30_A,
              .voltage_compensation = 12_V,
              .circuit_resistance = robot_constants::climber_::wire_resistance,
              .rotational_inertia = frc846::wpilib::unit_kg_m_sq{1.0}},
          225_tr / 1_tr) {
  RegisterPreference("pre_climb_setpoint", 270_deg);
  RegisterPreference("climb_setpoint", 100_deg);
  RegisterPreference("stow_setpoint", 0_deg);
}

WristTarget ClimberSubsystem::ZeroTarget() const {
  return WristTarget{0.0_deg};
}

void ClimberSubsystem::ExtendedSetup() {}

std::pair<units::degree_t, bool> ClimberSubsystem::GetSensorPos() {
  return {0_deg, false};
}