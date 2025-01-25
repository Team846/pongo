#include "subsystems/hardware/climber.h"

#include "subsystems/SubsystemHelper.h"
#include "subsystems/hardware/climber.h"

ClimberSubsystem::ClimberSubsystem()
    : frc846::robot::GenericSubsystem<ClimberReadings, ClimberTarget>(
          "climber"),
      motor_configs(GET_MOTOR_CONFIG("climber/climber_one_",
          ports::climber_::kClimberOne_CANID, frc846::wpilib::unit_ohm{0.0},
          frc846::wpilib::unit_kg_m_sq{0.0})),
      climber_(frc846::control::base::MotorMonkeyType::TALON_FX_KRAKENX44,
          motor_configs),
      motor_two_configs(GET_MOTOR_CONFIG("climber/climber_two_",
          ports::climber_::kClimberTwo_CANID, frc846::wpilib::unit_ohm{0.0},
          frc846::wpilib::unit_kg_m_sq{0.0})),
      climber_two_(frc846::control::base::MotorMonkeyType::TALON_FX_KRAKENX44,
          motor_two_configs) {
  RegisterPreference("climber/climber_tolerance_", 0.25_in);

  REGISTER_MOTOR_CONFIG(
      "climber/climber_one_", false, true, 40_A, 40_A, 16.0_V);
  REGISTER_MOTOR_CONFIG(
      "climber/climber_two_", false, true, 40_A, 40_A, 16.0_V);
  REGISTER_PIDF_CONFIG("climber/climber_gains_", 0.0, 0.0, 0.0, 0.0);
  REGISTER_SOFTLIMIT_CONFIG("climber/climber_softlimits", true, 1.0);

  motor_helper_.SetConversion(climber_reduction_);
  motor_helper_two_.SetConversion(climber_reduction_);
  motor_helper_.bind(&climber_);
  motor_helper_two_.bind(&climber_two_);
}

void ClimberSubsystem::Setup() {
  climber_.Setup();
  climber_two_.Setup();
  climber_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame,
          frc846::control::config::StatusFrame::kVelocityFrame});
  climber_two_.EnableStatusFrames(
      {frc846::control::config::StatusFrame::kPositionFrame,
          frc846::control::config::StatusFrame::kVelocityFrame});
}

ClimberTarget ClimberSubsystem::ZeroTarget() const {
  return {0.0_deg, ClimberState::kClimberIdle};
}

bool ClimberSubsystem::VerifyHardware() {
  bool ok = true;

  FRC846_VERIFY(climber_.VerifyConnected(), ok, "climber one not connected");
  FRC846_VERIFY(
      climber_two_.VerifyConnected(), ok, "climber two not connected");

  return ok;
}

ClimberReadings ClimberSubsystem::ReadFromHardware() {
  ClimberReadings readings;
  readings.position = motor_helper_.GetPosition();

  Graph("climber_position", readings.position);

  return readings;
}

void ClimberSubsystem::WriteToHardware(ClimberTarget target) {
  climber_.SetGains(GET_PIDF_GAINS("climber/climber_gains_"));
  climber_two_.SetGains(GET_PIDF_GAINS("climber/climber_two_gains_"));

  if (target.target_state == kPreClimb) {
    target.position = 30.0_deg;
  } else if (target.target_state == kClimb) {
    target.position = 60.0_deg;
  } else {
    target.position = 90.0_deg;
  }

  motor_helper_.WritePosition(target.position);
  motor_helper_two_.WritePosition(target.position);
}
