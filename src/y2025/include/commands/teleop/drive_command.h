#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class DriveCommand
    : public frc846::robot::GenericCommand<RobotContainer, DriveCommand> {
public:
  DriveCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
};