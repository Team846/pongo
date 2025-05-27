#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class NetAutoAlignCommand
    : public frc846::robot::GenericCommandGroup<RobotContainer,
          NetAutoAlignCommand, frc2::SequentialCommandGroup> {
public:
  NetAutoAlignCommand(
      RobotContainer& container, frc846::math::Vector2D& base_adj);
};
