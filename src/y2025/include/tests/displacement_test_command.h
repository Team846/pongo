#pragma once

#include "frc846/robot/GenericCommand.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "subsystems/robot_container.h"
#include <frc/Timer.h>

class DisplacementTestCommand
    : public frc846::robot::GenericCommand<RobotContainer, DisplacementTestCommand> {
 public:
  DisplacementTestCommand(RobotContainer& container,
                          frc846::robot::swerve::DrivetrainSubsystem* drivetrain);

  void OnInit() override;
  void Periodic() override;
  void OnEnd(bool interrupted) override;
  bool IsFinished() override;

 private:
  frc846::robot::swerve::DrivetrainSubsystem* drivetrain_;

  frc::Timer timer_;
  frc::Timer accel_timer_;
  frc::Timer brake_timer_;
  frc::Timer distance_timer_;

  frc846::math::Vector2D start_pos_;

  bool accel_logged_ = false;
  bool brake_logged_ = false;
  bool distance_logged_ = false;
};
