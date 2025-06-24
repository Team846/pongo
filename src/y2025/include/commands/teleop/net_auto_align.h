#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class NetAutoAlignCommand
    : public frc846::robot::GenericCommandGroup<RobotContainer,
          NetAutoAlignCommand, frc2::SequentialCommandGroup> {
public:
  NetAutoAlignCommand(RobotContainer& container,
      units::feet_per_second_t max_speed,
      units::feet_per_second_t max_net_speed,
      units::feet_per_second_squared_t max_acceleration,
      units::feet_per_second_squared_t max_net_acceleration,
      units::feet_per_second_squared_t max_deceleration,
      units::feet_per_second_squared_t max_net_deceleration);
};
