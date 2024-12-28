#pragma once

#include "frc846/robot/GenericSubsystem.h"
#include "frc846/control/HigherMotorController.h"
#include "frc846/control/HMCHelper.h"

#include <units/angle.h>
#include <units/torque.h>
#include <units/velocity.h>
#include <units/base.h>

#include <variant>

#include <ctre/phoenix6/CANcoder.hpp>

namespace frc846::robot::swerve {

struct SwerveModuleReadings {
  units::feet_per_second_t vel;
  units::foot_t drive_pos;
  units::degree_t steer_pos;
};

/*
SwerveModuleTorqueControlTarget

Allows to control drive and steer motors with torque control.
*/
struct SwerveModuleTorqueControlTarget {
  units::newton_meter_t drive;
  units::newton_meter_t steer;
};

/*
SwerveModuleOLControlTarget

Allows to control drive and steer motors with open loop control.
*/
struct SwerveModuleOLControlTarget {
  double drive;
  units::degree_t steer;
};

using SwerveModuleTarget =
    std::variant<SwerveModuleTorqueControlTarget, SwerveModuleOLControlTarget>;

/*
SwerveModule

A class that represents a swerve module. This subsystem should be a child of
Drivetrain.
*/
class SwerveModuleSubsystem
    : public frc846::robot::GenericSubsystem<SwerveModuleReadings,
          SwerveModuleTarget> {
  using steer_conv_unit = units::dimensionless::scalar_t;
  using drive_conv_unit = units::unit_t<
      units::compound_unit<units::foot, units::inverse<units::turn>>>;

public:
  SwerveModuleSubsystem(Loggable& parent, std::string loc,
      frc846::control::config::MotorConstructionParameters drive_params,
      frc846::control::config::MotorConstructionParameters steer_params,
      frc846::control::base::MotorMonkeyType motor_types, int cancoder_id,
      steer_conv_unit steer_reduction, drive_conv_unit drive_reduction,
      std::string cancoder_bus = "");

  void Setup() override;

  SwerveModuleTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void SetCANCoderOffset();
  void SetCANCoderOffset(units::degree_t offset);

  void ZeroWithCANcoder();

  void SetGains(frc846::control::base::MotorGains gains);

private:
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
};

}  // namespace frc846::robot::swerve