#include "commands/tests/displacement_test_command.h"

#include <frc846/robot/GenericCommand.h>

#include "subsystems/robot_container.h"

DisplacementTestCommand::DisplacementTestCommand(RobotContainer& container)
    : frc846::robot::GenericCommand<RobotContainer, DisplacementTestCommand>(
          container, "DisplacementTestCommand") {
  AddRequirements({&container.drivetrain_});
}

void DisplacementTestCommand::OnInit() {
  timer_.Reset();
  timer_.Start();
  start_pos_ = container_.drivetrain_.GetReadings().pose.position;
  distance_logged_ = false;
  Log("Starting velocity {}",
      container_.drivetrain_.GetReadings().pose.velocity.magnitude().value());
}

void DisplacementTestCommand::Periodic() {
  

  auto pose = container_.drivetrain_.GetReadings().pose;
  frc846::math::Vector2D delta{
      pose.position[0] - start_pos_[0], pose.position[1] - start_pos_[1]};
  units::inch_t distance_traveled = delta.magnitude();

  units::inch_t target_distance =
      container_.drivetrain_.GetPreferenceValue_unit_type<units::inch_t>(
          "displacement_test/target_distance");

  container_.drivetrain_.SetTarget({{0.0_fps,0.0_fps},0.0_deg_per_s,-1_fps_sq,false, true, container_.drivetrain_.GetPreferenceValue_double("displacement_test/duty_cycle")});

  if (distance_traveled >= target_distance) {
    distance_logged_ = true;
    Log("Displacement done {}", timer_.Get().value());
  }

  Graph("distancetraveled", distance_traveled);
}

bool DisplacementTestCommand::IsFinished() { return distance_logged_; }

void DisplacementTestCommand::OnEnd(bool interrupted) {
  container_.drivetrain_.SetTarget({{0.0_fps,0.0_fps},0.0_deg_per_s,-1_fps_sq,false, false});
}
