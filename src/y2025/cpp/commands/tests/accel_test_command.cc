#include "commands/tests/accel_test_command.h"

#include "subsystems/robot_container.h"

AccelTestCommand::AccelTestCommand(RobotContainer& container,
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain)
    : frc846::robot::GenericCommand<RobotContainer, AccelTestCommand>(
          container, "DisplacementTestCommand"),
      drivetrain_(drivetrain) {
  AddRequirements({drivetrain_});
}

void AccelTestCommand::OnInit() {
  timer_.Reset();
  timer_.Start();
}

void AccelTestCommand::Periodic() {
  double duty_cycle =
      drivetrain_->GetPreferenceValue_double("displacement_test/duty_cycle");

  double target_velocity = drivetrain_->GetPreferenceValue_double(
      "displacement_test/target_velocity");

  auto pose = drivetrain_->GetReadings().pose;
  double vel = pose.velocity.magnitude().value();

  // drivetrain_->WriteToHardware(duty_cycle);
  timer_.Start();
  if (vel >= target_velocity) {
    Log("Acceleration to {} done in {}", target_velocity, timer_.Get().value());
    accel_logged_ = true;
  }
}

bool AccelTestCommand::IsFinished() { return accel_logged_; }

void AccelTestCommand::OnEnd(bool interrupted) {
  // drivetrain_->WriteToHardware(0.0);
}
