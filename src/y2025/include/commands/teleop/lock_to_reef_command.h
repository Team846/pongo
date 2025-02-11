#pragma once

#include "frc846/base/Loggable.h"
#include "frc846/math/fieldpoints.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "frc846/robot/swerve/lock_to_point_command.h"
#include "subsystems/robot_container.h"

class LockToReefCommand : public frc846::robot::swerve::LockToPointCommand {
public:
  LockToReefCommand(RobotContainer& container, bool is_left);

private:
  RobotContainer& container_;
  bool is_left_;

  frc846::math::Vector2D base_adj{0_in, 0_in};
};
