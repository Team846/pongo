#include "commands/tests/braking_test_command.h"

#include "subsystems/robot_container.h"

BrakingTestCommand::BrakingTestCommand(RobotContainer& container,
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain)
    : frc846::robot::GenericCommand<RobotContainer, BrakingTestCommand>(
          container, "BrakingTestCommand"),
      drivetrain_(drivetrain) {
  AddRequirements({drivetrain_});
}

void BrakingTestCommand::OnInit() {
  timer_.Reset();
  timer_.Start();
  Log("Starting velocity {}",
      drivetrain_->GetReadings().pose.velocity.magnitude().value());
}

void BrakingTestCommand::Periodic() {
  double duty_cycle =
      drivetrain_->GetPreferenceValue_double("displacement_test/duty_cycle");

  auto pose = drivetrain_->GetReadings().pose;

  double target_velocity = drivetrain_->GetPreferenceValue_double(
      "displacement_test/target_velocity");
  units::inch_t target_distance =
      drivetrain_->GetPreferenceValue_unit_type<units::inch_t>(
          "displacement_test/target_distance");

  double vel = pose.velocity.magnitude().value();

  // drivetrain_->WriteToHardware(-duty_cycle);

  if (vel <= 0.01) {
    brake_logged_ = true;
    Log("Braking done {}", timer_.Get().value());

    // drivetrain_->WriteToHardware(0.0);
  }
}

bool BrakingTestCommand::IsFinished() { return brake_logged_; }

void BrakingTestCommand::OnEnd(bool interrupted) {
  // drivetrain_->WriteToHardware(0.0);
}
