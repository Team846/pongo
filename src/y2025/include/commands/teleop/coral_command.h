#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class CoralCommand
    : public frc846::robot::GenericCommand<RobotContainer, CoralCommand> {
public:
  CoralCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;
};