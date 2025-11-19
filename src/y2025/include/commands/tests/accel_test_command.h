#pragma once
#include <frc/Timer.h>

#include "frc846/robot/GenericCommand.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "subsystems/robot_container.h"

class AccelTestCommand
    : public frc846::robot::GenericCommand<RobotContainer, AccelTestCommand> {
public:
  AccelTestCommand(RobotContainer& container,
      frc846::robot::swerve::DrivetrainSubsystem* drivetrain);

  void OnInit() override;
  void Periodic() override;
  void OnEnd(bool interrupted) override;
  bool IsFinished() override;

private:
  frc846::robot::swerve::DrivetrainSubsystem* drivetrain_;

  frc::Timer timer_;

  bool accel_logged_ = false;
};