#pragma once

#include "frc846/math/fieldpoints.h"
#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class SimDriveCommand
    : public frc846::robot::GenericCommand<RobotContainer, SimDriveCommand> {
public:
  SimDriveCommand(RobotContainer& container, frc846::math::FieldPoint where);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
  frc846::math::Vector2D where_pos;
  units::degree_t where_bearing;
};