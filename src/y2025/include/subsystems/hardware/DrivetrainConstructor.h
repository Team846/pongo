#pragma once

#include "frc846/base/Loggable.h"
#include "frc846/robot/swerve/drivetrain.h"

/*
DrivetrainConstructor

A class providing methods to aid construction of a DrivetrainSubsystem
object.
*/
class DrivetrainConstructor : public frc846::base::Loggable {
public:
  DrivetrainConstructor();

  frc846::robot::swerve::DrivetrainConfigs getDrivetrainConfigs();
};