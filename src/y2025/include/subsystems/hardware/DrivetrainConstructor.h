#pragma once

#include "frc846/robot/swerve/drivetrain.h"

/*
DrivetrainConstructor

A class providing static methods to aid construction of a DrivetrainSubsystem
object.
*/
class DrivetrainConstructor {
public:
  static frc846::robot::swerve::DrivetrainConfigs getDrivetrainConfigs();
};