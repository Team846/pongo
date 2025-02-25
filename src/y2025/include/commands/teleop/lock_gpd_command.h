#pragma once

#include "frc846/base/Loggable.h"
#include "frc846/math/fieldpoints.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "frc846/robot/swerve/lock_to_point_command.h"
#include "subsystems/robot_container.h"

class LockGPDCommand : public frc846::robot::swerve::LockToPointCommand {
public:
  LockGPDCommand(RobotContainer& container);
};
