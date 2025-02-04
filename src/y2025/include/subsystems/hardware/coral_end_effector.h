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

enum CoralEndEffectorState { kCoralScore, kCoralIntake, kCoralIdle };

struct CoralEndEffectorReadings {
  units::feet_per_second_t vel;
  bool reef_detected;
};

struct CoralEndEffectorTarget {
  CoralEndEffectorState state;
  units::feet_per_second_t vel;
};

using coral_end_effector_pos_conv_t = units::unit_t<
    units::compound_unit<units::feet, units::inverse<units::turn>>>;

class CoralEndEffectorSubsystem
    : public frc846::robot::GenericSubsystem<CoralEndEffectorReadings,
          CoralEndEffectorTarget> {
public:
  CoralEndEffectorSubsystem();

  void Setup() override;

  CoralEndEffectorTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  bool hasZeroed = false;

  // TODO: Set to correct reduction later
  coral_end_effector_pos_conv_t coral_pivot_reduction_ = 1.0_ft / 1.0_tr;

  frc846::control::config::MotorConstructionParameters motor_configs;
  frc846::control::HigherMotorController coral_end_effector;
  frc846::control::HMCHelper<units::feet> motor_helper_;

  CoralEndEffectorReadings ReadFromHardware() override;

  void WriteToHardware(CoralEndEffectorTarget target) override;
};