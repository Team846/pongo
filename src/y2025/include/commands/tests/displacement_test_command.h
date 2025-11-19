#pragma once
#include <frc/Timer.h>

#include "frc846/robot/GenericCommand.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "subsystems/robot_container.h"

class DisplacementTestCommand
    : public frc846::robot::GenericCommand<RobotContainer,
          DisplacementTestCommand> {
public:
  DisplacementTestCommand(RobotContainer& container);

  void OnInit() override;
  void Periodic() override;
  void OnEnd(bool interrupted) override;
  bool IsFinished() override;

private:
  frc::Timer timer_;

  frc846::math::Vector2D start_pos_;

  bool distance_logged_;
};
