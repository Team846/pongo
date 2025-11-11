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
              .motor_current_limit = 60_A,
              .smart_current_limit = 80_A,
              .voltage_compensation = 12_V,
              .circuit_resistance =
                  robot_constants::algae_ss_::wire_resistance_base,
              .rotational_inertia = frc846::wpilib::unit_kg_m_sq{0.0005}},
          37_in / 64.579_tr, robot_constants::elevator::elevator_hall_effect) {
  REGISTER_PIDF_CONFIG(0.01, 0.0, 0.0, 0.0);
  REGISTER_SOFTLIMIT_CONFIG(true, 73_in, 29.12_in, 65_in, 40_in, 0.3);

  RegisterPreference("hall_effect_plus_height", 1.7_in);
  RegisterPreference("hall_effect_counter", 40);

  RegisterPreference("sm_target_range", 34_in);
  RegisterPreference("sm_vel_thresh", 0.02_fps);
  RegisterPreference("sm_current_thresh", 30_A);
  RegisterPreference("sm_counter", 100);
}

LinearSubsystemTarget ElevatorSubsystem::ZeroTarget() const {
  return LinearSubsystemTarget{0_in};
}

void ElevatorSubsystem::ExtendedSetup() {}

void ElevatorSubsystem::RHExtension() {}