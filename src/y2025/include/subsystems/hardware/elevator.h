#pragma once

#include <units/angle.h>
#include <units/constants.h>
#include <units/length.h>
#include <units/math.h>
#include <units/torque.h>
#include <units/velocity.h>

#include "frc846/base/Loggable.h"
#include "frc846/control/HMCHelper.h"
#include "frc846/control/HigherMotorController.h"
#include "frc846/control/base/motor_control_base.h"
#include "frc846/control/config/construction_params.h"
#include "frc846/control/config/soft_limits.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/wpilib/units.h"
#include "ports.h"

struct ElevatorReadings {
  units::inch_t height;
};

struct ElevatorTarget {
  units::inch_t height;
};

using elevator_pos_conv_t = units::unit_t<
    units::compound_unit<units::inch, units::inverse<units::turn>>>;

class ElevatorSubsystem
    : public frc846::robot::GenericSubsystem<ElevatorReadings, ElevatorTarget> {
public:
  ElevatorSubsystem();

  void Setup() override;

  ElevatorTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  bool WithinTolerance(units::inch_t pos) {
    return units::math::abs(pos - GetReadings().height) <
           GetPreferenceValue_unit_type<units::inch_t>("elevator_tolerance_");
  }

private:
  bool hasZeroed = false;

  elevator_pos_conv_t elevator_reduction_ = 1.0_in / 1.0_tr;

  frc846::control::config::MotorConstructionParameters motor_configs;
  frc846::control::HigherMotorController elevator_;
  frc846::control::HMCHelper<units::inch> motor_helper_;

  ElevatorReadings ReadFromHardware() override;

  void WriteToHardware(ElevatorTarget target) override;
};
