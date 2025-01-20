#include "subsystems/hardware/coral_wrist.h"

#include "subsystems/SubsystemHelper.h"

CoralWristSubsystem::CoralWristSubsystem()
    : frc846::robot::GenericSubsystem<CoralWristReadings, CoralWristTarget>(
          "coral_wrist"),
      motor_configs(GET_MOTOR_CONFIG("coral_wrist/coral_wrist_one_",
          ports::coral_wrist_::kCoralWristOne_CANID,
          frc846::wpilib::unit_ohm{0.0}, frc846::wpilib::unit_kg_m_sq{0.0})),
      coral_wrist_(frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO550,
          motor_configs) {
  RegisterPreference("coral_wrist/coral_wrist_tolerance_", 0.25_in);

  REGISTER_MOTOR_CONFIG(
      "coral_wrist/coral_wrist_one_", false, true, 40_A, 40_A, 16.0_V);
  REGISTER_PIDF_CONFIG("coral_wrist/coral_wrist_gains_", 0.0, 0.0, 0.0, 0.0);
  REGISTER_SOFTLIMIT_CONFIG("coral_wrist/coral_wrist_softlimits", true, 1.0);

  motor_helper_.SetConversion(coral_wrist_reduction);

  motor_helper_.bind(&coral_wrist_);
}

void CoralWristSubsystem::Setup() { coral_wrist_.Setup(); }

CoralWristTarget CoralWristSubsystem::ZeroTarget() const {
  CoralWristTarget target;
  target.position = 0.0_deg;
  return target;
}

bool CoralWristSubsystem::VerifyHardware() {
  bool ok = true;

  FRC846_VERIFY(
      coral_wrist_.VerifyConnected(), ok, "Coral Wrist not connected");

  return ok;
  return true;
}

CoralWristReadings CoralWristSubsystem::ReadFromHardware() {
  CoralWristReadings readings;
  readings.position = motor_helper_.GetPosition();

  Graph("Coral Wrist extension", readings.position);

  return readings;
}

void CoralWristSubsystem::WriteToHardware(CoralWristTarget target) {
  coral_wrist_.SetGains(GET_PIDF_GAINS("coral_wrist/coral_wrist_gains_"));
  motor_helper_.WritePosition(target.position);
}
