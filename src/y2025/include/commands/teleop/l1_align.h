#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class L1AutoAlignCommand
    : public frc846::robot::GenericCommandGroup<RobotContainer,
          L1AutoAlignCommand, frc2::SequentialCommandGroup> {
public:
  L1AutoAlignCommand(RobotContainer& container, bool is_left,
      units::feet_per_second_t max_speed,
      units::feet_per_second_t lower_max_speed,
      units::feet_per_second_squared_t max_acceleration,
      units::feet_per_second_squared_t max_deceleration,
      frc846::math::Vector2D& base_adj);
};