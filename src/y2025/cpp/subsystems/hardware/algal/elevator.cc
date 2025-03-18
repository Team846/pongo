#include "subsystems/hardware/algal/elevator.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/SubsystemHelper.h"
#include "subsystems/robot_constants.h"

ElevatorSubsystem::ElevatorSubsystem()
    : LinearSubsystem("elevator",
          frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO,
          frc846::control::config::MotorConstructionParameters{
              .can_id = ports::algal_ss_::elevator_::kElevator_CANID,
              .inverted = true,
              .brake_mode = true,
              .motor_current_limit = 90_A,
              .smart_current_limit = 120_A,
              .voltage_compensation = 12_V,
              .circuit_resistance =
                  robot_constants::algae_ss_::wire_resistance_base,
              .rotational_inertia = frc846::wpilib::unit_kg_m_sq{1.0}},
          37_in / 64.579_tr, robot_constants::elevator::elevator_hall_effect) {
  REGISTER_PIDF_CONFIG(0.0, 0.0, 0.0, 0.0);
  REGISTER_SOFTLIMIT_CONFIG(true, 104_in, 40_in, 94_in, 29_in, 0.3);
}

LinearSubsystemTarget ElevatorSubsystem::ZeroTarget() const {
  return LinearSubsystemTarget{0_in};
}

void ElevatorSubsystem::ExtendedSetup() {}