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

// Allows for torque-based control by DrivetrainSubsystem
struct SwerveModuleTorqueControlTarget {
  units::newton_meter_t drive;
  units::newton_meter_t steer;
};

// For open-loop control by DrivetrainSubsystem
struct SwerveModuleOLControlTarget {
  double drive;
  units::degree_t steer;
};

using SwerveModuleTarget =
    std::variant<SwerveModuleTorqueControlTarget, SwerveModuleOLControlTarget>;

/*
SwerveModule

A class representing a single swerve module. Controls a drive and steer motor
and a CANcoder. Meant to be constructed as a child subsystem of
DrivetrainSubsystem.
*/
class SwerveModule
    : public frc846::robot::GenericSubsystem<SwerveModuleReadings,
          SwerveModuleTarget> {
  using steer_conv_unit = units::dimensionless::dimensionless_t;
  using drive_conv_unit = units::unit_t<
      units::compound_unit<units::foot, units::inverse<units::turn>>>;

public:
  /*
  SwerveModule()

  Constructs a SwerveModule object with the given parameters. For use by
  DrivetrainSubsystem.
  */
  SwerveModule(Loggable& parent, std::string loc,
      frc846::control::config::MotorConstructionParameters drive_params,
      frc846::control::config::MotorConstructionParameters steer_params,
      frc846::control::base::MotorMonkeyType motor_types, int cancoder_id,
      steer_conv_unit steer_reduction, drive_conv_unit drive_reduction,
      std::string cancoder_bus = "");

  void Setup() override;

  SwerveModuleTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  /*
  SetSteerGains()

  Sets the gains for the steer motor controller. Should be called after
  SwerveModuleSubsystem Setup, in DrivetrainSubsystem Setup.
  */
  void SetSteerGains(frc846::control::base::MotorGains gains);

private:
  SwerveModuleReadings ReadFromHardware() override;

  void WriteToHardware(SwerveModuleTarget target) override;

  frc846::control::HigherMotorController drive_;
  frc846::control::HigherMotorController steer_;

  frc846::control::HMCHelper<units::foot> drive_helper_;
  frc846::control::HMCHelper<units::degree> steer_helper_;

  ctre::phoenix6::hardware::CANcoder cancoder_;
};

}  // namespace frc846::robot::swerve