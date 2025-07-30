#pragma once

#include "frc846/math/fieldpoints.h"
#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class ReefAutoAutoAlignCommand
    : public frc846::robot::GenericCommandGroup<RobotContainer,
          ReefAutoAutoAlignCommand, frc2::SequentialCommandGroup> {
public:
  ReefAutoAutoAlignCommand(RobotContainer& container, int numberOnRight,
      bool blueSide, bool leftSide, bool isRetry);

  static frc846::math::FieldPoint getModifiedPrePose(
      int numberOnRight, bool blueSide, bool leftSide, bool isRetry);
};