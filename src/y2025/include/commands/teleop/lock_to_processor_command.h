#pragma once

#include "frc846/base/Loggable.h"
#include "frc846/math/fieldpoints.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "frc846/robot/swerve/lock_to_point_command.h"
#include "subsystems/robot_container.h"

class LockToProcessorCommand
    : public frc846::robot::swerve::LockToPointCommand {
public:
  LockToProcessorCommand(
      RobotContainer& container, frc846::math::Vector2D& base_adj);
};
