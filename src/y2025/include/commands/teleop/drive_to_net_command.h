#pragma once

#include "frc846/robot/swerve/drive_to_point_command.h"
#include "subsystems/robot_container.h"

class DriveToNetCommand : public frc846::robot::swerve::DriveToPointCommand {
public:
  DriveToNetCommand(RobotContainer& container, bool is_scoring,
      units::feet_per_second_t max_speed,
      units::feet_per_second_squared_t max_acceleration,
      units::feet_per_second_squared_t max_deceleration);

  std::pair<frc846::math::FieldPoint, bool> GetTargetPoint() override;

private:
  RobotContainer& container_;
  bool is_scoring_;
};