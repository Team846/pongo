#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "frc846/base/Loggable.h"
#include "frc846/math/fieldpoints.h"
#include "frc846/robot/GenericCommand.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "subsystems/robot_container.h"

class LockToReefCommand
    : public frc846::robot::GenericCommand<RobotContainer, LockToReefCommand> {
public:
  LockToReefCommand(RobotContainer& container, bool is_left);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
  bool is_left_;

  frc846::math::Vector2D base_adj{0_in, 0_in};
};
