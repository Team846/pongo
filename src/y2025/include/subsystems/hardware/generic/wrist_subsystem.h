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
#include "frc846/math/collection.h"
#include "frc846/robot/GenericSubsystem.h"

struct WristReadings {
  units::degree_t position;
  units::degrees_per_second_t velocity;
  units::turn_t absolute_position;
};

struct WristTarget {
  units::degree_t position;
};

using wrist_pos_conv_t = units::unit_t<
    units::compound_unit<units::deg, units::inverse<units::turn>>>;

class WristSubsystem
    : public frc846::robot::GenericSubsystem<WristReadings, WristTarget> {
public:
  WristSubsystem(std::string name,
      frc846::control::base::MotorMonkeyType mmtype,
      frc846::control::config::MotorConstructionParameters motor_configs_,
      wrist_pos_conv_t conversion);

  frc846::control::config::MotorConstructionParameters GetCurrentConfig(
      frc846::control::config::MotorConstructionParameters original_config);

  void Setup() override final;

  bool VerifyHardware() override final;

protected:
  virtual void ExtendedSetup() = 0;

  virtual std::pair<units::degree_t, bool> GetSensorPos() {
    return {0_deg, false};
  }

  frc846::control::config::MotorConstructionParameters motor_configs_;

  frc846::control::HigherMotorController wrist_esc_;
  frc846::control::HMCHelper<units::degree> wrist_esc_helper_;

  WristReadings ReadFromHardware() override final;

  void WriteToHardware(WristTarget target) override final;
};
