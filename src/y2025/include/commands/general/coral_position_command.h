#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class CoralPositionCommand
    : public frc846::robot::GenericCommand<RobotContainer,
          CoralPositionCommand> {
public:
  CoralPositionCommand(RobotContainer& container, CoralStates where);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
  CoralStates where_;
};