#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class AlgalPositionCommand
    : public frc846::robot::GenericCommand<RobotContainer,
          AlgalPositionCommand> {
public:
  AlgalPositionCommand(
      RobotContainer& container, AlgalStates where, bool score);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
  AlgalStates where_;
  bool score_;
};