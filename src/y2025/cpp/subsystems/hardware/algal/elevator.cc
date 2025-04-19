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
              .rotational_inertia = frc846::wpilib::unit_kg_m_sq{1.0}},
          37_in / 64.579_tr, robot_constants::elevator::elevator_hall_effect) {
  REGISTER_PIDF_CONFIG(0.01, 0.0, 0.0, 0.0);
  REGISTER_SOFTLIMIT_CONFIG(true, 73_in, 29.12_in, 65_in, 40_in, 0.3);

  RegisterPreference("hall_effect_plus_height", 1.7_in);
  RegisterPreference("hall_effect_counter", 40);

  RegisterPreference("sm_target_range", 34_in);
  RegisterPreference("sm_vel_thresh", 0.02_fps);
  RegisterPreference("sm_current_thresh", 30_A);
  RegisterPreference("sm_counter", 10);
}

LinearSubsystemTarget ElevatorSubsystem::ZeroTarget() const {
  return LinearSubsystemTarget{0_in};
}

void ElevatorSubsystem::ExtendedSetup() {
  // linear_esc_.ConfigReverseLimitSwitch(
  //     true, frc846::control::base::LimitSwitchDefaultState::kNormallyOff);
  // homing_counter_ = 0;
}

void ElevatorSubsystem::RHExtension() {
  // homing_counter_++;
  // bool sensor_triggered = linear_esc_.GetReverseLimitSwitchState();
  // Graph("elevator/homing_sensor", sensor_triggered);
  // if (sensor_triggered &&
  //     homing_counter_ > GetPreferenceValue_int("hall_effect_counter") &&
  //     GetReadings().position < 34_in) {
  //   linear_esc_helper_.SetPosition(
  //       robot_constants::elevator::min_height_off_base +
  //       GetPreferenceValue_unit_type<units::inch_t>("hall_effect_plus_height"));
  //   homing_counter_ = 0;
  // }

  // if (GetTarget().position <
  //         GetPreferenceValue_unit_type<units::inch_t>("sm_target_range") &&
  //     units::math::abs(linear_esc_helper_.GetVelocity()) <
  //         GetPreferenceValue_unit_type<units::feet_per_second_t>(
  //             "sm_vel_thresh") &&
  //     GetReadings().position > 29_in && GetReadings().position < 34_in &&
  //     linear_esc_.GetCurrent() >
  //         GetPreferenceValue_unit_type<units::ampere_t>("sm_current_thresh"))
  //         {
  //   safety_counter_ += 1;
  //   if (safety_counter_ > GetPreferenceValue_int("sm_counter")) {
  //     linear_esc_helper_.SetPosition(
  //         robot_constants::elevator::min_height_off_base);
  //   }
  // } else {
  //   safety_counter_ = 0;
  // }
}