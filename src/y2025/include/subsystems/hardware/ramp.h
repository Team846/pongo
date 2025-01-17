#pragma once

#include <units/math.h>
#include <units/velocity.h>

#include "frc846/base/Loggable.h"
#include "frc846/control/HMCHelper.h"
#include "frc846/control/HigherMotorController.h"
#include "frc846/control/base/motor_control_base.h"
#include "frc846/control/config/construction_params.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/robot/calculators/VerticalArmCalculator.h"
#include "frc846/wpilib/units.h"
#include "ports.h"

enum RampState { kIntake, kIdle };

struct RampReadings {
  RampState state;
  units::feet_per_second_t vel;
};

struct RampTarget {
  RampState state;
  units::feet_per_second_t vel;
};

using ramp_pos_conv_t = units::unit_t<
    units::compound_unit<units::feet, units::inverse<units::turn>>>;

class RampSubsystem
    : public frc846::robot::GenericSubsystem<RampReadings, RampTarget> {
public:
  RampSubsystem();

  void Setup() override;

  RampTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  bool hasZeroed = false;

  // TODO: Set to correct reduction later
  ramp_pos_conv_t ramp_reduction_ = 1.0_ft / 1.0_tr;

  frc846::control::config::MotorConstructionParameters motor_configs;
  frc846::control::HigherMotorController ramp_;
  frc846::control::HMCHelper<units::feet> motor_helper_;

  frc846::robot::calculators::VerticalArmCalculator arm_calculator_;

  RampReadings ReadFromHardware() override;

  void WriteToHardware(RampTarget target) override;
};