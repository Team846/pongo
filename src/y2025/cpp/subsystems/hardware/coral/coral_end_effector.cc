#include "subsystems/hardware/coral/coral_end_effector.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "ports.h"
#include "subsystems/SubsystemHelper.h"
#include "subsystems/robot_constants.h"

CoralEESubsystem::CoralEESubsystem()
    : GenericSubsystem("coral_end_effector"),
      motor_configs_{
          .can_id = ports::coral_ss_::end_effector_::kEE_CANID,
          .inverted = false,
          .brake_mode = true,
          .motor_current_limit = 10_A,
          .smart_current_limit = 5_A,
          .voltage_compensation = 12_V,
          .circuit_resistance = robot_constants::coral_ss_::wire_resistance,
          .rotational_inertia = frc846::wpilib::unit_kg_m_sq{0.5},
      },
      esc_{frc846::control::base::SPARK_MAX_NEO550,
          GetCurrentConfig(motor_configs_)} {
  RegisterPreference("idle_speed", 0.05);
}

frc846::control::config::MotorConstructionParameters
CoralEESubsystem::GetCurrentConfig(
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

void CoralEESubsystem::Setup() {
  esc_.Setup();
  esc_.EnableStatusFrames({frc846::control::config::kFaultFrame});

  esc_.ConfigForwardLimitSwitch(
      false, frc846::control::base::LimitSwitchDefaultState::kNormallyOff);
  esc_.ConfigReverseLimitSwitch(
      false, frc846::control::base::LimitSwitchDefaultState::kNormallyOff);
}

bool CoralEESubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_.VerifyConnected(), ok, "Could not verify esc");
  return ok;
}

CoralEEReadings CoralEESubsystem::ReadFromHardware() {
  CoralEEReadings readings;
  readings.has_piece_ = esc_.GetForwardLimitSwitchState();
  readings.see_reef = esc_.GetReverseLimitSwitchState();
  Graph("readings/has_piece", readings.has_piece_);
  Graph("readings/see_reef", readings.see_reef);
  return readings;
}

void CoralEESubsystem::WriteToHardware(CoralEETarget target) {
  Graph("target/duty_cycle", target.duty_cycle_);
  if (GetReadings().has_piece_)
    esc_.WriteDC(GetPreferenceValue_double("idle_speed"));
  else
    esc_.WriteDC(target.duty_cycle_);
}