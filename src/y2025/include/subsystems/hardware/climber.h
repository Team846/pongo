#pragma once

#include <time.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/velocity.h>

#include "frc846/base/Loggable.h"
#include "frc846/control/HMCHelper.h"
#include "frc846/control/HigherMotorController.h"
#include "frc846/robot/GenericSubsystem.h"
#include "ports.h"
#include "units/length.h"
#include "units/math.h"

struct ClimberReadings {
  units::degree_t position;
};

enum ClimberState { kClimberIdle, kPreClimb, kClimb };

struct ClimberTarget {
  units::degree_t position;
  ClimberState target_state;
};

using climber_pos_conv_t = units::unit_t<
    units::compound_unit<units::degree, units::inverse<units::turn>>>;

class ClimberSubsystem
    : public frc846::robot::GenericSubsystem<ClimberReadings, ClimberTarget> {
public:
  ClimberSubsystem();

  void Setup() override;

  ClimberTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  ClimberReadings ReadFromHardware() override;

  void WriteToHardware(ClimberTarget target) override;

  climber_pos_conv_t climber_reduction_ = 1.0_deg / 1.0_tr;

  frc846::control::config::MotorConstructionParameters motor_configs;
  frc846::control::config::MotorConstructionParameters motor_two_configs;
  frc846::control::HigherMotorController climber_;
  frc846::control::HigherMotorController climber_two_;
  frc846::control::HMCHelper<units::degree> motor_helper_;
  frc846::control::HMCHelper<units::degree> motor_helper_two_;

  bool has_zeroed_ = false;
};
