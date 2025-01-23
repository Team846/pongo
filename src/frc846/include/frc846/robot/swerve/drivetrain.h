#pragma once

#include <units/acceleration.h>
#include <units/angular_acceleration.h>

#include "frc846/robot/GenericSubsystem.h"
#include "frc846/robot/swerve/control/swerve_ol_calculator.h"
#include "frc846/robot/swerve/odometry/swerve_odometry_calculator.h"
#include "frc846/robot/swerve/odometry/swerve_pose.h"
#include "frc846/robot/swerve/swerve_module.h"
#include "studica/AHRS.h"

namespace frc846::robot::swerve {

/*
DrivetrainConfigs

Contains all configs related to the specific drivetrain in use.
*/
struct DrivetrainConfigs {
  studica::AHRS::NavXComType navX_connection_mode;

  SwerveModuleCommonConfig module_common_config;
  std::array<SwerveModuleUniqueConfig, 4> module_unique_configs;

  units::inch_t wheelbase_horizontal_dim;
  units::inch_t wheelbase_forward_dim;

  units::feet_per_second_squared_t max_accel;
};

struct DrivetrainReadings {
  frc846::robot::swerve::odometry::SwervePose pose;
  units::degrees_per_second_t yaw_rate;
};

// Open-loop control, for use during teleop
struct DrivetrainOLControlTarget {
  frc846::math::VectorND<units::feet_per_second, 2> velocity;
  units::degrees_per_second_t angular_velocity;
};

// Allows for acceleration-based control of the drivetrain
struct DrivetrainAccelerationControlTarget {
  units::feet_per_second_squared_t linear_acceleration;
  units::degree_t accel_dir;
  units::degrees_per_second_t angular_velocity;
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
  DrivetrainSubsystem(DrivetrainConfigs configs);

  void Setup() override;

  DrivetrainTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroBearing();

  void SetCANCoderOffsets();

private:
  DrivetrainReadings ReadFromHardware() override;

  frc846::math::VectorND<units::feet_per_second, 2> compensateForSteerLag(
      frc846::math::VectorND<units::feet_per_second, 2> uncompensated);

  void WriteVelocitiesHelper(
      frc846::math::VectorND<units::feet_per_second, 2> velocity,
      units::degrees_per_second_t angular_velocity, bool cut_excess_steering);
  void WriteToHardware(DrivetrainTarget target) override;

  DrivetrainConfigs configs_;
  std::array<SwerveModuleSubsystem*, 4> modules_;

  studica::AHRS navX_;

  frc846::robot::swerve::odometry::SwerveOdometryCalculator odometry_;
  frc846::robot::swerve::control::SwerveOpenLoopCalculator ol_calculator_;
};

}  // namespace frc846::robot::swerve