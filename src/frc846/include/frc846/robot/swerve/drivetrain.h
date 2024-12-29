#pragma once

#include <units/acceleration.h>
#include <units/angular_acceleration.h>

#include "frc846/robot/GenericSubsystem.h"
#include "frc846/robot/swerve/odometry/swerve_pose.h"
#include "frc846/robot/swerve/swerve_module.h"

namespace frc846::robot::swerve {

/*
DrivetrainConfigs

Contains all configs related to the specific drivetrain in use.
*/
struct DrivetrainConfigs {};

struct DrivetrainReadings {
  frc846::robot::swerve::odometry::SwervePose pose;
};

// Open-loop control, for use during teleop
struct DrivetrainOLControlTarget {
  frc846::math::VectorND<units::feet_per_second, 2> velocity;
};

// Allows for acceleration-based control of the drivetrain
struct DrivetrainAccelerationControlTarget {
  frc846::math::VectorND<units::feet_per_second_squared, 2> linear_acceleration;
  units::degrees_per_second_squared_t angular_acceleration;
};

using DrivetrainTarget = std::variant<DrivetrainOLControlTarget,
    DrivetrainAccelerationControlTarget>;

/*
DrivetrainSubsystem

A generic class to control a 4-module Kraken x60 swerve drive with CANCoders.
*/
class DrivetrainSubsystem
    : public frc846::robot::GenericSubsystem<DrivetrainReadings,
          DrivetrainTarget> {
public:
  DrivetrainSubsystem(DrivetrainConfigs configs, bool enable);

  void Setup() override;

  DrivetrainTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  DrivetrainReadings ReadFromHardware() override;

  void WriteToHardware(DrivetrainTarget target) override;

  DrivetrainConfigs configs_;
  std::array<SwerveModule*, 4> modules_;
};

}  // namespace frc846::robot::swerve