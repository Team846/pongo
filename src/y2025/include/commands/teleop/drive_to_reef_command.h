#pragma once

#include "frc846/robot/swerve/drive_to_point_command.h"
#include "subsystems/robot_container.h"

class DriveToReefCommand : public frc846::robot::swerve::DriveToPointCommand {
public:
  DriveToReefCommand(RobotContainer& container, bool is_left, bool is_pre_point,
      units::feet_per_second_t max_speed,
      units::feet_per_second_squared_t max_acceleration,
      units::feet_per_second_squared_t max_deceleration);

  std::pair<frc846::math::FieldPoint, bool> GetTargetPoint() override;

private:
  RobotContainer& container_;
  bool is_left_;
  bool is_pre_point_;
};