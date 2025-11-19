#pragma once
#include <frc/Timer.h>

#include "frc846/robot/GenericCommand.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "subsystems/robot_container.h"

class BrakingTestCommand
    : public frc846::robot::GenericCommand<RobotContainer, BrakingTestCommand> {
public:
  BrakingTestCommand(RobotContainer& container,
      frc846::robot::swerve::DrivetrainSubsystem* drivetrain);

  void OnInit() override;
  void Periodic() override;
  void OnEnd(bool interrupted) override;
  bool IsFinished() override;

private:
  frc846::robot::swerve::DrivetrainSubsystem* drivetrain_;

  frc::Timer timer_;

  bool brake_logged_ = false;
};
