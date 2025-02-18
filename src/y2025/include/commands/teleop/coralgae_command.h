#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class CoralgaeCommand
    : public frc846::robot::GenericCommand<RobotContainer, CoralgaeCommand> {
public:
  CoralgaeCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;
};