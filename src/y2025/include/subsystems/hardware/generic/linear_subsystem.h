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
#include "subsystems/robot_constants.h"

struct LinearSubsystemReadings {
  units::inch_t position;
};

struct LinearSubsystemTarget {
  units::inch_t position;
};

using linear_pos_conv_t = units::unit_t<
    units::compound_unit<units::inch, units::inverse<units::turn>>>;

class LinearSubsystem
    : public frc846::robot::GenericSubsystem<LinearSubsystemReadings,
          LinearSubsystemTarget> {
public:
  LinearSubsystem(std::string name,
      frc846::control::base::MotorMonkeyType mmtype,
      frc846::control::config::MotorConstructionParameters motor_configs_,
      linear_pos_conv_t conversion, units::inch_t hall_effect_loc);

  void Setup() override final;

  bool VerifyHardware() override final;

protected:
  virtual void ExtendedSetup() = 0;

  virtual std::pair<units::inch_t, bool> GetSensorPos() {
    return {0_in, false};
  }

  frc846::control::config::MotorConstructionParameters motor_configs_;

  frc846::control::HigherMotorController linear_esc_;
  frc846::control::HMCHelper<units::inch> linear_esc_helper_;

  LinearSubsystemReadings ReadFromHardware() override final;

  void WriteToHardware(LinearSubsystemTarget target) override final;
  bool is_homed_ = false;
  units::inch_t hall_effect_loc_;
};
