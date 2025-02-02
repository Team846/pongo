#include "subsystems/hardware/generic/wrist_subsystem.h"

#include "subsystems/SubsystemHelper.h"

WristSubsystem::WristSubsystem(std::string name,
    frc846::control::base::MotorMonkeyType mmtype,
    frc846::control::config::MotorConstructionParameters motor_configs_,
    wrist_pos_conv_t conversion)
    : frc846::robot::GenericSubsystem<WristReadings, WristTarget>(name),
      wrist_esc_(mmtype, motor_configs_) {
  REGISTER_PIDF_CONFIG("gains", 0.0, 0.0, 0.0, 0.0);
  REGISTER_SOFTLIMIT_CONFIG("limits", true, 90_deg, 0_deg, 90_deg, 0_deg, 0.3);

  wrist_esc_helper_.SetConversion(conversion);
  wrist_esc_helper_.SetSoftLimits(GET_SOFTLIMITS("limits", units::degree_t));
  wrist_esc_helper_.SetControllerSoftLimits(
      GET_SOFTLIMITS("limits", units::degree_t));

  wrist_esc_helper_.bind(&wrist_esc_);
}

void WristSubsystem::Setup() {
  wrist_esc_.Setup();

  wrist_esc_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame,
          frc846::control::config::StatusFrame::kVelocityFrame});

  const auto [sensor_pos, is_valid] = GetSensorPos();
  if (is_valid) { wrist_esc_helper_.SetPosition(sensor_pos); }

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

  Graph("readings/position", readings.position);

  const auto [sensor_pos, is_valid] = GetSensorPos();
  if (is_valid) { wrist_esc_helper_.SetPosition(sensor_pos); }

  Graph("readings/sensor_pos", sensor_pos);
  Graph("readings/sensor_valid", is_valid);

  return readings;
}

void WristSubsystem::WriteToHardware(WristTarget target) {
  wrist_esc_.SetGains(GET_PIDF_GAINS("gains"));
  wrist_esc_helper_.WritePosition(target.position);
}