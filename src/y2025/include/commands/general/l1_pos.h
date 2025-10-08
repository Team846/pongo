#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class L1PosCommand
    : public frc846::robot::GenericCommand<RobotContainer, L1PosCommand> {
public:
  L1PosCommand(RobotContainer& container, bool score);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
  bool score_;
};