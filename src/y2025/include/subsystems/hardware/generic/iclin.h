#pragma once

#include <time.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/math.h>
#include <units/velocity.h>

#include "frc846/base/Loggable.h"
#include "frc846/control/HMCHelper.h"
#include "frc846/control/HigherMotorController.h"
#include "frc846/robot/GenericSubsystem.h"
#include "pdcsu.h"
#include "subsystems/robot_constants.h"

using namespace pdcsu::units;
using namespace pdcsu::control;
using namespace pdcsu::util;

struct IclinReadings {
  units::inch_t position;
};

struct IclinTarget {
  units::inch_t position;
};

using linear_pos_conv_t = units::unit_t<
    units::compound_unit<units::inch, units::inverse<units::turn>>>;

class IclinSubsystem
    : public frc846::robot::GenericSubsystem<IclinReadings, IclinTarget> {
public:
  IclinSubsystem(std::string name,
      frc846::control::base::MotorMonkeyType mmtype,
      frc846::control::config::MotorConstructionParameters motor_configs_,
      linear_pos_conv_t conversion, units::inch_t hall_effect_loc);

  frc846::control::config::MotorConstructionParameters GetCurrentConfig(
      frc846::control::config::MotorConstructionParameters original_config);

  void Setup() override final;

  bool VerifyHardware() override final;

  void HomeSubsystem(units::inch_t pos);

  bool isHomed() { return is_homed_; }

  void OverrideSoftLimits(bool overrideLimits);

  void BrakeSubsystem();
  void CoastSubsystem();

protected:
  virtual void ExtendedSetup() = 0;

  frc846::control::HigherMotorController linear_esc_;
  frc846::control::HMCHelper<units::inch> linear_esc_helper_;

  IclinReadings ReadFromHardware() override final;

  void WriteToHardware(IclinTarget target) override final;

  bool is_homed_ = false;

  DefBLDC bldc;
  DefLinearSys lin_sys;
  ICNORPositionControl icnor;
};
