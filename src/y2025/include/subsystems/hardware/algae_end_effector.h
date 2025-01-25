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

enum AlgaeEndEffectorState { kScore, kIdle };

struct AlgaeEndEffectorReadings {
  units::feet_per_second_t vel;
  bool gp_detected;
};

struct AlgaeEndEffectorTarget {
  AlgaeEndEffectorState state;
  units::feet_per_second_t vel;
};

using algae_end_effector_pos_conv_t = units::unit_t<
    units::compound_unit<units::feet, units::inverse<units::turn>>>;

class AlgaeEndEffectorSubsystem
    : public frc846::robot::GenericSubsystem<AlgaeEndEffectorReadings,
          AlgaeEndEffectorTarget> {
public:
  AlgaeEndEffectorSubsystem();

  void Setup() override;

  AlgaeEndEffectorTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  bool hasZeroed = false;

  // TODO: Set to correct reduction later
  algae_end_effector_pos_conv_t algae_pivot_reduction_ = 1.0_ft / 1.0_tr;

  frc846::control::config::MotorConstructionParameters motor_configs;
  frc846::control::HigherMotorController algae_end_effector;
  frc846::control::HMCHelper<units::feet> motor_helper_;

  AlgaeEndEffectorReadings ReadFromHardware() override;

  void WriteToHardware(AlgaeEndEffectorTarget target) override;
};