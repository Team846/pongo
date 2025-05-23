#include "subsystems/hardware/generic/linear_subsystem.h"

#include "subsystems/SubsystemHelper.h"

LinearSubsystem::LinearSubsystem(std::string name,
    frc846::control::base::MotorMonkeyType mmtype,
    frc846::control::config::MotorConstructionParameters motor_configs_,
    linear_pos_conv_t conversion, units::inch_t hall_effect_loc_)
    : frc846::robot::GenericSubsystem<LinearSubsystemReadings,
          LinearSubsystemTarget>(name),
      linear_esc_(mmtype, GetCurrentConfig(motor_configs_)),
      hall_effect_loc_(hall_effect_loc_) {
  linear_esc_helper_.SetConversion(conversion);

  linear_esc_helper_.bind(&linear_esc_);

  RegisterPreference("pidf_deadband", 0.35_in);
}

frc846::control::config::MotorConstructionParameters
LinearSubsystem::GetCurrentConfig(
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

void LinearSubsystem::Setup() {
  linear_esc_.Setup();

  linear_esc_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame,
          frc846::control::config::StatusFrame::kVelocityFrame,
          frc846::control::config::StatusFrame::kFaultFrame});

  linear_esc_helper_.SetPosition(28.5_in);

  linear_esc_helper_.SetSoftLimits(GET_SOFTLIMITS(units::inch_t));
  // linear_esc_helper_.SetControllerSoftLimits(GET_SOFTLIMITS(units::inch_t));

  // linear_esc_.ConfigForwardLimitSwitch(
  //     false, frc846::control::base::LimitSwitchDefaultState::kNormallyOff);

  ExtendedSetup();
}

void LinearSubsystem::HomeSubsystem(units::inch_t pos) {
  if (is_initialized()) linear_esc_helper_.SetPosition(pos);
  is_homed_ = true;
}

bool LinearSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(linear_esc_.VerifyConnected(), ok, "Could not verify esc");
  return ok;
}

LinearSubsystemReadings LinearSubsystem::ReadFromHardware() {
  LinearSubsystemReadings readings;
  readings.position = linear_esc_helper_.GetPosition();

  linear_esc_.SetLoad(1_Nm);

  Graph("readings/position", readings.position);
  Graph("readings/current_draw", linear_esc_.GetCurrent());

  Graph("readings/error", GetTarget().position - readings.position);

  RHExtension();

  // bool forward_limit = linear_esc_.GetForwardLimitSwitchState();

  // Graph("readings/homing_sensor", forward_limit);

  // if (forward_limit && !is_homed_) {
  //   is_homed_ = true;
  //   linear_esc_helper_.SetPosition(hall_effect_loc_);
  // }

  return readings;
}

void LinearSubsystem::OverrideSoftLimits(bool overrideLimits) {
  frc846::control::SoftLimitsConfig limits = GET_SOFTLIMITS(units::inch_t);

  limits.using_limits = !overrideLimits;
  linear_esc_helper_.SetSoftLimits(limits);
}

void LinearSubsystem::WriteToHardware(LinearSubsystemTarget target) {
  // Graph("target/position", target.position);
  linear_esc_.SetGains(GET_PIDF_GAINS());

  linear_esc_.SetLoad(1_Nm);

  if (units::math::abs(GetReadings().position - target.position) >
      GetPreferenceValue_unit_type<units::inch_t>("pidf_deadband")) {
    Graph("within_deadband", false);
    linear_esc_helper_.WritePosition(target.position);
  } else {
    Graph("within_deadband", true);
    linear_esc_helper_.WriteDC(0.0);
  }
}

void LinearSubsystem::BrakeSubsystem() {
  if (is_initialized()) linear_esc_.SetNeutralMode(true);
}

void LinearSubsystem::CoastSubsystem() {
  if (is_initialized()) linear_esc_.SetNeutralMode(false);
}