#pragma once

#include <frc2/command/ParallelDeadlineGroup.h>

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class CompleteGPDCommand
    : public frc846::robot::GenericCommandGroup<RobotContainer,
          CompleteGPDCommand, frc2::ParallelDeadlineGroup> {
public:
  CompleteGPDCommand(RobotContainer& container);
};