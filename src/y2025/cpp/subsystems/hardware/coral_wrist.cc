#include "subsystems/hardware/coral_wrist.h"

#include "subsystems/SubsystemHelper.h"

CoralWristSubsystem::CoralWristSubsystem()
    : frc846::robot::GenericSubsystem<CoralWristReadings, CoralWristTarget>(
          "coral_wrist"),
      motor_configs_one_(GET_MOTOR_CONFIG("motor_configs",
          ports::coral_wrist_::kCoralWristOne_CANID,
          frc846::wpilib::unit_ohm{0.0}, frc846::wpilib::unit_kg_m_sq{0.0})),
      motor_configs_two_(GET_MOTOR_CONFIG("motor_configs",
          ports::coral_wrist_::kCoralWristTwo_CANID,
          frc846::wpilib::unit_ohm{0.0}, frc846::wpilib::unit_kg_m_sq{0.0})),
      coral_wrist_one_(frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO550,
          motor_configs_one_),
      coral_wrist_two_(frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO550,
          motor_configs_two_) {
  RegisterPreference("coral_wrist_tolerance", 0.25_in);

  REGISTER_MOTOR_CONFIG("motor_configs", false, true, 40_A, 40_A, 16.0_V);
  REGISTER_PIDF_CONFIG("coral_wrist_gains", 0.0, 0.0, 0.0, 0.0);
  REGISTER_SOFTLIMIT_CONFIG("coral_wrist_softlimits", true, 1.0);

  motor_helper_one_.SetConversion(coral_wrist_reduction);
  motor_helper_two_.SetConversion(coral_wrist_reduction);

  wrist_calculator_.setConstants({20.0_kg, 3.0_in, 0.0_deg, -1.0, 1.0});

  motor_helper_one_.bind(&coral_wrist_one_);
  motor_helper_two_.bind(&coral_wrist_two_);
}

void CoralWristSubsystem::Setup() {
  coral_wrist_one_.Setup();
  coral_wrist_two_.Setup();

  coral_wrist_one_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame,
          frc846::control::config::StatusFrame::kVelocityFrame});
  coral_wrist_one_.ConfigForwardLimitSwitch(
      false, frc846::control::base::LimitSwitchDefaultState::kNormallyOff);

  motor_helper_one_.SetPosition(0.0_deg);
  motor_helper_two_.SetPosition(0.0_deg);
}

CoralWristTarget CoralWristSubsystem::ZeroTarget() const {
  CoralWristTarget{0.0_deg};
}

bool CoralWristSubsystem::VerifyHardware() {
  bool ok = true;

  FRC846_VERIFY(
      coral_wrist_one_.VerifyConnected(), ok, "Coral Wrist One not connected");
  FRC846_VERIFY(
      coral_wrist_two_.VerifyConnected(), ok, "Coral Wrist Two not connected");

  return ok;
}

CoralWristReadings CoralWristSubsystem::ReadFromHardware() {
  CoralWristReadings readings;
  readings.position = motor_helper_one_.GetPosition();
  readings.gp_detected = coral_wrist_two_.GetForwardLimitSwitchState();

  Graph("readings/position", readings.position);
  Graph("readings/game_piece_detected", readings.gp_detected);

  return readings;
}

void CoralWristSubsystem::WriteToHardware(CoralWristTarget target) {
  coral_wrist_one_.SetGains(GET_PIDF_GAINS("coral_wrist_gains_"));
  coral_wrist_two_.SetGains(GET_PIDF_GAINS("coral_wrist_gains_"));
  motor_helper_one_.WriteDC(
      wrist_calculator_.calculate({motor_helper_one_.GetPosition(),
          target.position, motor_helper_one_.GetVelocity(),
          GET_PIDF_GAINS("coral_wrist_gains")}));
  motor_helper_two_.WriteDC(
      wrist_calculator_.calculate({motor_helper_one_.GetPosition(),
          target.position, motor_helper_one_.GetVelocity(),
          GET_PIDF_GAINS("coral_wrist_gains")}));
}
