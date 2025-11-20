#include "tests/displacement_test_command.h"
#include "subsystems/robot_container.h"

DisplacementTestCommand::DisplacementTestCommand(
    RobotContainer& container,
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain)
    : frc846::robot::GenericCommand<RobotContainer, DisplacementTestCommand>(container, "DisplacementTestCommand"),
      drivetrain_(drivetrain) {
  AddRequirements({drivetrain_});
}

void DisplacementTestCommand::OnInit() {
  timer_.Reset();
  timer_.Start();
  start_pos_ = drivetrain_->GetReadings().pose.position;

  accel_logged_ = false;
  brake_logged_ = false;
  distance_logged_ = false;
}

void DisplacementTestCommand::Periodic() {
  double duty_cycle = drivetrain_->GetPreferenceValue_double("displacement_test/duty_cycle");

  auto pose = drivetrain_->GetReadings().pose;
  frc846::math::Vector2D delta{
      pose.position[0] - start_pos_[0],
      pose.position[1] - start_pos_[1]};
  units::inch_t distance_traveled = delta.magnitude();
  
  double target_velocity = drivetrain_->GetPreferenceValue_double("displacement_test/target_velocity");
  units::inch_t target_distance = drivetrain_->GetPreferenceValue_unit_type<units::inch_t>("displacement_test/target_distance");
  
  double vel = pose.velocity.magnitude().value();

  // acceleration
  if (!accel_logged_) {
    drivetrain_->WriteToHardware(duty_cycle);
    timer_.Start();
    
    if (vel >= target_velocity) {
      accel_logged_ = true;
      Log("Acceleration done", timer_.Get().value());

      drivetrain_->WriteToHardware(0.0);
    }
    return;
  }

  // displacement
  if (accel_logged_ && !distance_logged_) {
    drivetrain_->WriteToHardware(0.0);
    if (distance_traveled >= target_distance) {
      distance_logged_ = true;
      Log("Displacement done", timer_.Get().value());

      drivetrain_->WriteToHardware(-duty_cycle);
    }
    return;
  }

  // braking
  if (distance_logged_ && !brake_logged_) {
    drivetrain_->WriteToHardware(-duty_cycle);

    if (vel <= 0.01) {
      brake_logged_ = true;
      Log("Braking done", timer_.Get().value());

      drivetrain_->WriteToHardware(0.0);
    }
    return;
  }
}

bool DisplacementTestCommand::IsFinished() {
  return accel_logged_ && distance_logged_ && brake_logged_;
}

void DisplacementTestCommand::OnEnd(bool interrupted) {
  drivetrain_->WriteToHardware(0.0);
  timer_.Stop();
  Log("Total time", timer_.Get().value());
}
