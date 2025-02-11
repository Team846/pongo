#pragma once

#include "frc846/robot/swerve/drive_to_point_command.h"

class DriveToReefCommand : public frc846::robot::swerve::DriveToPointCommand {
public:
  DriveToReefCommand(frc846::robot::swerve::DrivetrainSubsystem* drivetrain,
      bool is_left, units::feet_per_second_t max_speed,
      units::feet_per_second_squared_t max_acceleration,
      units::feet_per_second_squared_t max_deceleration);

  std::pair<frc846::math::FieldPoint, bool> GetTargetPoint() override;

private:
  bool is_left_;
};