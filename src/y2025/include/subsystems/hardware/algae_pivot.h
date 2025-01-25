#pragma once

#include <units/math.h>

#include "frc846/base/Loggable.h"
#include "frc846/control/HMCHelper.h"
#include "frc846/control/HigherMotorController.h"
#include "frc846/control/base/motor_control_base.h"
#include "frc846/control/config/construction_params.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/robot/calculators/VerticalArmCalculator.h"
#include "frc846/wpilib/units.h"
#include "ports.h"

struct AlgaePivotReadings {
  units::degree_t position;
  bool gp_detected;
};

struct AlgaePivotTarget {
  units::degree_t position;
};

using algae_pivot_pos_conv_t = units::unit_t<
    units::compound_unit<units::degree, units::inverse<units::turn>>>;

class AlgaePivotSubsystem
    : public frc846::robot::GenericSubsystem<AlgaePivotReadings,
          AlgaePivotTarget> {
public:
  AlgaePivotSubsystem();

  void Setup() override;

  AlgaePivotTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  bool WithinTolerance(units::degree_t pos) {
    return units::math::abs(pos - GetReadings().position) <
           GetPreferenceValue_unit_type<units::degree_t>(
               "algae_pivot_tolerance_");
  }

private:
  bool hasZeroed = false;

  // TODO: Set to correct reduction later
  algae_pivot_pos_conv_t algae_pivot_reduction_ = 1.0_deg / 1.0_tr;

  frc846::control::config::MotorConstructionParameters motor_configs;
  frc846::control::HigherMotorController pivot_;
  frc846::control::HMCHelper<units::degree> motor_helper_;

  frc846::robot::calculators::VerticalArmCalculator arm_calculator_;

  AlgaePivotReadings ReadFromHardware() override;

  void WriteToHardware(AlgaePivotTarget target) override;
};