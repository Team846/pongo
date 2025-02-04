#include "subsystems/hardware/climber.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/robot_constants.h"

ClimberSubsystem::ClimberSubsystem()
    : WristSubsystem("climber",
          frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO,
          frc846::control::config::MotorConstructionParameters{
              .can_id = ports::climber_::kClimber_CANID,
              .inverted = false,
              .brake_mode = true,
              .motor_current_limit = 40_A,
              .smart_current_limit =
                  30_A,  // TODO: prefify current limits (only)
              .voltage_compensation = 12_V,
              .circuit_resistance = robot_constants::climber_::wire_resistance,
              .rotational_inertia = frc846::wpilib::unit_kg_m_sq{1.0}

          },
          1.0_tr / 2.0_tr) {}

void ClimberSubsystem::ExtendedSetup() {
  // TODO: implement
}

std::pair<units::degree_t, bool> ClimberSubsystem::GetSensorPos() {
  return {0_deg, false};
}