#include "tests/displacement_test_command.h"

#include "subsystems/robot_container.h"

DisplacementTestCommand::DisplacementTestCommand(RobotContainer& container,
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain)
    : frc846::robot::GenericCommand<RobotContainer, DisplacementTestCommand>(
          container, "DisplacementTestCommand"),
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
  double duty_cycle =
      drivetrain_->GetPreferenceValue_double("displacement_test/duty_cycle");
  auto pose = drivetrain_->GetReadings().pose;

  frc846::math::Vector2D delta{
      pose.position[0] - start_pos_[0], pose.position[1] - start_pos_[1]};
  units::inch_t distance_traveled = delta.magnitude();

  double vel = pose.velocity.magnitude().value();
  double target_velocity = drivetrain_->GetPreferenceValue_double(
      "displacement_test/target_velocity");
  units::inch_t target_distance =
      drivetrain_->GetPreferenceValue_unit_type<units::inch_t>(
          "displacement_test/target_distance");

  auto send_move = [&](double dc) {
    frc846::robot::swerve::DrivetrainOLControlTarget target;
    target.velocity = {units::feet_per_second_t(dc), 0_fps};
    target.angular_velocity = 0_deg_per_s;
    drivetrain_->SetTarget(target);
  };

  if (!accel_logged_) {
    send_move(duty_cycle);
    if (vel >= target_velocity) {
      accel_logged_ = true;
      Log("Acceleration phase complete at {}s", timer_.Get().value());
    }
  } else if (!distance_logged_) {
    send_move(duty_cycle);
    if (distance_traveled >= target_distance) {
      distance_logged_ = true;
      Log("Displacement phase complete at {}s", timer_.Get().value());
    }
  } else if (!brake_logged_) {
    send_move(-duty_cycle);
    if (vel <= 0.05) {
      brake_logged_ = true;
      Log("Braking phase complete at {}s", timer_.Get().value());
    }
  }
}

bool DisplacementTestCommand::IsFinished() {
  return accel_logged_ && distance_logged_ && brake_logged_;
}

void DisplacementTestCommand::OnEnd(bool interrupted) {
  frc846::robot::swerve::DrivetrainOLControlTarget stop_target;
  stop_target.velocity = {0_fps, 0_fps};
  stop_target.angular_velocity = 0_deg_per_s;

  drivetrain_->SetTarget(stop_target);
  timer_.Stop();
  Log("Test Finished. Total time: {}s", timer_.Get().value());
}
