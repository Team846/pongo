#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "frc846/base/Loggable.h"
#include "frc846/math/fieldpoints.h"
#include "frc846/robot/GenericCommand.h"
#include "frc846/robot/swerve/drive_to_point_command.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "lock_to_point_command.h"
#include "subsystems/robot_container.h"

class DriveToReefCommand
    : public frc846::robot::GenericCommandGroup<RobotContainer,
          DriveToReefCommand, frc2::SequentialCommandGroup> {
public:
  DriveToReefCommand(RobotContainer& container, bool is_left);
  std::vector<std::unique_ptr<frc2::Command>> getCommandChain(
      RobotContainer& container, bool is_left);

private:
  bool is_left_;
};
