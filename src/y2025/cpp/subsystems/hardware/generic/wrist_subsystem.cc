#include "subsystems/hardware/generic/wrist_subsystem.h"

#include "subsystems/SubsystemHelper.h"

WristSubsystem::WristSubsystem(std::string name,
    frc846::control::base::MotorMonkeyType mmtype,
    frc846::control::config::MotorConstructionParameters motor_configs_,
    wrist_pos_conv_t conversion)
    : frc846::robot::GenericSubsystem<WristReadings, WristTarget>(name),
      wrist_esc_(mmtype, GetCurrentConfig(motor_configs_)) {
  wrist_esc_helper_.SetConversion(conversion);

  RegisterPreference("rezero_thresh", 2_deg);

  wrist_esc_helper_.bind(&wrist_esc_);
}

frc846::control::config::MotorConstructionParameters
WristSubsystem::GetCurrentConfig(
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

void WristSubsystem::Setup() {
  wrist_esc_.Setup();

  wrist_esc_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame,
          frc846::control::config::StatusFrame::kVelocityFrame,
          frc846::control::config::StatusFrame::kFaultFrame,
          frc846::control::config::StatusFrame::kAbsoluteFrame});

  wrist_esc_helper_.SetSoftLimits(GET_SOFTLIMITS(units::degree_t));
  // wrist_esc_helper_.SetControllerSoftLimits(GET_SOFTLIMITS(units::degree_t));

  const auto [sensor_pos, is_valid] = GetSensorPos();
  wrist_esc_helper_.SetPosition(sensor_pos);

  ExtendedSetup();
}

bool WristSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(
      wrist_esc_.VerifyConnected(), ok, "Could not verify wrist motor");
  return ok;
}

WristReadings WristSubsystem::ReadFromHardware() {
  WristReadings readings;
  readings.position = wrist_esc_helper_.GetPosition();
  readings.velocity = wrist_esc_helper_.GetVelocity();
  readings.absolute_position = wrist_esc_.GetAbsoluteEncoderPosition();

  units::degree_t cg_pos =
      readings.position *
          (GetPreferenceValue_bool("flip_position_load_sign") ? -1 : 1) +
      GetPreferenceValue_unit_type<units::degree_t>("cg_offset");

  wrist_esc_.SetLoad(1_Nm * units::math::cos(cg_pos));

  Graph("readings/cg_pos", cg_pos);

  Graph("readings/position", readings.position);
  Graph("readings/error", GetTarget().position - readings.position);
  Graph("readings/velocity", readings.velocity);
  units::degree_t abs_pos_deg = readings.absolute_position;
  Graph("readings/absolute_position", abs_pos_deg);

  Graph("readings/current_draw", wrist_esc_.GetCurrent());

  const auto [sensor_pos, is_valid] = GetSensorPos();
  if (is_valid &&
      units::math::abs(sensor_pos - readings.position) >
          GetPreferenceValue_unit_type<units::degree_t>("rezero_thresh")) {
    wrist_esc_helper_.SetPosition(sensor_pos);
  }

  Graph("readings/sensor_pos", sensor_pos);
  Graph("readings/sensor_pos_valid", is_valid);

  return readings;
}

void WristSubsystem::WriteToHardware(WristTarget target) {
  Graph("target/position", target.position);

  wrist_esc_.SetGains(GET_PIDF_GAINS());
  wrist_esc_helper_.WritePosition(target.position);
}

void WristSubsystem::BrakeSubsystem() {
  if (is_initialized()) wrist_esc_.SetNeutralMode(true);
}

void WristSubsystem::CoastSubsystem() {
  if (is_initialized()) wrist_esc_.SetNeutralMode(false);
}