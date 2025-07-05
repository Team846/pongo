#pragma once

#include <units/angle.h>
#include <units/base.h>
#include <units/torque.h>
#include <units/velocity.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <variant>

#include "frc846/control/HMCHelper.h"
#include "frc846/control/HigherMotorController.h"
#include "frc846/robot/GenericSubsystem.h"

namespace frc846::robot::swerve {

struct SwerveModuleReadings {
  units::feet_per_second_t vel;
  units::foot_t drive_pos;
  units::degree_t steer_pos;
};

// For open-loop control by DrivetrainSubsystem
struct SwerveModuleOLControlTarget {
  units::feet_per_second_t drive;
  units::degree_t steer;
};

using SwerveModuleTarget = SwerveModuleOLControlTarget;

struct SwerveModuleUniqueConfig {
  std::string loc;

  int cancoder_id;
  int drive_id;
  int steer_id;

  frc846::wpilib::unit_ohm circuit_resistance;
};

using steer_conv_unit = units::dimensionless::scalar_t;
using drive_conv_unit = units::unit_t<
    units::compound_unit<units::foot, units::inverse<units::turn>>>;

struct SwerveModuleCommonConfig {
  frc846::control::config::MotorConstructionParameters drive_params;
  frc846::control::config::MotorConstructionParameters steer_params;

  frc846::control::base::MotorMonkeyType motor_types;
  steer_conv_unit steer_reduction;
  drive_conv_unit drive_reduction;

  units::unit_t<units::compound_unit<units::meter, units::kilogram>>
      steer_load_factor;

  std::string_view bus = "";
};

/*
SwerveModuleSubsystem

A class representing a single swerve module. Controls a drive and steer motor
and a CANcoder. Meant to be constructed as a child subsystem of
DrivetrainSubsystem.
*/
class SwerveModuleSubsystem
    : public frc846::robot::GenericSubsystem<SwerveModuleReadings,
          SwerveModuleTarget> {
public:
  /*
  SwerveModuleSubsystem()

  Constructs a SwerveModuleSubsystem object with the given parameters. For use
  by DrivetrainSubsystem.
  */
  SwerveModuleSubsystem(Loggable& parent,
      SwerveModuleUniqueConfig unique_config,
      SwerveModuleCommonConfig common_config);

  void Setup() override;

  SwerveModuleTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void SetCANCoderOffset();
  void SetCANCoderOffset(units::degree_t offset);

  void ZeroWithCANcoder();

  /*
  SetSteerGains()

  Sets the gains for the steer motor controller. Should be called after
  SwerveModuleSubsystem Setup, in DrivetrainSubsystem Setup.
  */
  void SetSteerGains(frc846::control::base::MotorGains gains);

private:
  int last_rezero = 101;

  /*
  getMotorParams()

  Static helper function modifies the drive and steer motor parameters provided
  in the common configuration using the unique configuration.
  */
  static std::pair<frc846::control::config::MotorConstructionParameters,
      frc846::control::config::MotorConstructionParameters>
  getMotorParams(SwerveModuleUniqueConfig unique_config,
      SwerveModuleCommonConfig common_config);

  SwerveModuleReadings ReadFromHardware() override;

  void WriteToHardware(SwerveModuleTarget target) override;

  /*
  calculateSteerPosition()

  Calculates the direction for the steer motor, based on a normalized target.
  Also returns a boolean that represents the inversion of the drive motor.
  */
  std::pair<units::degree_t, bool> calculateSteerPosition(
      units::degree_t target_norm, units::degree_t current);

  frc846::control::HigherMotorController drive_;
  frc846::control::HigherMotorController steer_;

  frc846::control::HMCHelper<units::foot> drive_helper_;
  frc846::control::HMCHelper<units::degree> steer_helper_;

  ctre::phoenix6::hardware::CANcoder cancoder_;

  units::feet_per_second_t max_speed_;

  units::unit_t<units::compound_unit<units::meter, units::kilogram>>
      steer_load_factor_;
};

}  // namespace frc846::robot::swerve