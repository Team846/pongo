#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class IdleCoralCommand
    : public frc846::robot::GenericCommand<RobotContainer, IdleCoralCommand> {
public:
  IdleCoralCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
};