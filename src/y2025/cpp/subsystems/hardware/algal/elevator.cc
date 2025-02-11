#include "subsystems/hardware/algal/elevator.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/robot_constants.h"

ElevatorSubsystem::ElevatorSubsystem()
    : LinearSubsystem("elevator",
          frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO,
          frc846::control::config::MotorConstructionParameters{
              .can_id = ports::algal_ss_::elevator_::kElevator_CANID,
              .inverted = false,
              .brake_mode = true,
              .motor_current_limit = 40_A,
              .smart_current_limit = 30_A,
              .voltage_compensation = 12_V,
              .circuit_resistance =
                  robot_constants::algae_ss_::wire_resistance_base,
              .rotational_inertia = frc846::wpilib::unit_kg_m_sq{1.0}},
          1.0_in / 2.0_tr, robot_constants::elevator::elevator_hall_effect) {}

LinearSubsystemTarget ElevatorSubsystem::ZeroTarget() const {
  return LinearSubsystemTarget{0_in};
}

void ElevatorSubsystem::ExtendedSetup() {
  // TODO: implement
}

std::pair<units::inch_t, bool> ElevatorSubsystem::GetSensorPos() {
  // TODO: implement
  return {0_in, false};
}