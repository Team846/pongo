#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class DinosaurClimberCommand
    : public frc846::robot::GenericCommand<RobotContainer,
          DinosaurClimberCommand> {
public:
  DinosaurClimberCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
  bool isA;
};