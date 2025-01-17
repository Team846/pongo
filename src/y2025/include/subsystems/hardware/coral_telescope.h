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

struct TelescopeReadings {
  units::inch_t extension;
};

struct TelescopeTarget {
  units::inch_t extension;
};

using telescope_pos_conv_t = units::unit_t<
    units::compound_unit<units::inch, units::inverse<units::turn>>>;

class TelescopeSubsystem
    : public frc846::robot::GenericSubsystem<TelescopeReadings,
          TelescopeTarget> {
public:
  TelescopeSubsystem();

  void Setup() override;

  TelescopeTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  TelescopeReadings ReadFromHardware() override;

  void WriteToHardware(TelescopeTarget target) override;

  telescope_pos_conv_t telescope_reduction = 1.0_in / 0.0_tr;

  frc846::control::config::MotorConstructionParameters motor_configs;
  frc846::control::HigherMotorController telescope_;
  frc846::control::HMCHelper<units::inch> motor_helper_;

  bool has_zeroed_ = false;
};
