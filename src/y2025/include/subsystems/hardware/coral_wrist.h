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
#include "frc846/robot/calculators/VerticalArmCalculator.h"
#include "ports.h"
#include "units/length.h"
#include "units/math.h"

struct CoralWristReadings {
  units::degree_t position;
  bool gp_detected;
};

struct CoralWristTarget {
  units::degree_t position;
};

using CoralWrist_pos_conv_t = units::unit_t<
    units::compound_unit<units::deg, units::inverse<units::turn>>>;

class CoralWristSubsystem
    : public frc846::robot::GenericSubsystem<CoralWristReadings,
          CoralWristTarget> {
public:
  CoralWristSubsystem();

  void Setup() override;

  CoralWristTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  CoralWrist_pos_conv_t coral_wrist_reduction = 1.0_deg / 1.0_tr;

  frc846::control::config::MotorConstructionParameters motor_configs;

  frc846::control::HigherMotorController coral_wrist_one_;
  frc846::control::HigherMotorController coral_wrist_two_;

  frc846::control::HMCHelper<units::degree> motor_helper_one_;
  frc846::control::HMCHelper<units::degree> motor_helper_two_;

  frc846::robot::calculators::VerticalArmCalculator wrist_calculator_;

  CoralWristReadings ReadFromHardware() override;

  void WriteToHardware(CoralWristTarget target) override;

  bool has_zeroed_ = false;
};
