#include "subsystems/hardware/algal/algal_end_effector.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/SubsystemHelper.h"
#include "subsystems/robot_constants.h"

AlgalEESubsystem::AlgalEESubsystem()
    : GenericSubsystem("algal_end_effector"),
      motor_configs_{
          .can_id = ports::algal_ss_::end_effector_::kEE1_CANID,
          .inverted = false,
          .brake_mode = true,
          .motor_current_limit = 40_A,
          .smart_current_limit = 30_A,
          .voltage_compensation = 12_V,
          .circuit_resistance = robot_constants::algae_ss_::wire_resistance,
          .rotational_inertia = frc846::wpilib::unit_kg_m_sq{3.0},
      },
      esc_1_{frc846::control::base::SPARK_MAX_NEO550,
          GetCurrentConfig(motor_configs_)},
      esc_2_{frc846::control::base::SPARK_MAX_NEO550,
          (GetCurrentConfig(GetModifiedConfig(motor_configs_,
              ports::algal_ss_::end_effector_::kEE2_CANID, true)))} {
  RegisterPreference("idle_speed", 0.04);
  RegisterPreference("piece_thresh", 2_tps);
}

frc846::control::config::MotorConstructionParameters
AlgalEESubsystem::GetCurrentConfig(
    frc846::control::config::MotorConstructionParameters original_config) {
  frc846::control::config::MotorConstructionParameters modifiedConfig =
      original_config;
  REGISTER_MOTOR_CONFIG(
      original_config.motor_current_limit, original_config.smart_current_limit);
  modifiedConfig.motor_current_limit =
      GetPreferenceValue_unit_type<units::ampere_t>(
          "motor_configs/current_limit");
  modifiedConfig.smart_current_limit =
      GetPreferenceValue_unit_type<units::ampere_t>(
          "motor_configs/smart_current_limit");
  return modifiedConfig;
}

void AlgalEESubsystem::Setup() {
  esc_1_.Setup();
  esc_1_.EnableStatusFrames({});

  esc_2_.Setup();
  esc_2_.EnableStatusFrames({frc846::control::config::kFaultFrame});

  esc_2_.ConfigReverseLimitSwitch(
      false, frc846::control::base::LimitSwitchDefaultState::kNormallyOff);
}

bool AlgalEESubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_1_.VerifyConnected(), ok, "Could not verify esc 1");
  FRC846_VERIFY(esc_2_.VerifyConnected(), ok, "Could not verify esc 2");
  return ok;
}

AlgalEEReadings AlgalEESubsystem::ReadFromHardware() {
  AlgalEEReadings readings;
  readings.has_piece_ =
      esc_2_.GetReverseLimitSwitchState() &&
      units::math::abs(esc_2_.GetVelocity()) <=
          GetPreferenceValue_unit_type<units::turns_per_second_t>(
              "piece_thresh");
  Graph("readings/has_piece", readings.has_piece_);
  return readings;
}

void AlgalEESubsystem::WriteToHardware(AlgalEETarget target) {
  Graph("target/duty_cycle", target.duty_cycle_);

  if (GetReadings().has_piece_ && target.duty_cycle_ > 0.0) {
    target.duty_cycle_ = GetPreferenceValue_double("idle_speed");
  }
  esc_1_.WriteDC(target.duty_cycle_);
  esc_2_.WriteDC(target.duty_cycle_);
}