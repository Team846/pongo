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

struct ShooterReadings {};

struct ShooterTarget {
  double duty_cycle_;
};

class ShooterSubsystem
    : public frc846::robot::GenericSubsystem<ShooterReadings, ShooterTarget> {
public:
  ShooterSubsystem();

  ShooterTarget ZeroTarget() const override;

  frc846::control::config::MotorConstructionParameters GetCurrentConfig(
      frc846::control::config::MotorConstructionParameters original_config);

  void Setup() override;

  bool VerifyHardware() override;

  void BrakeSubsystem();
  void CoastSubsystem();

  void ZeroClimber();

protected:
  frc846::control::config::MotorConstructionParameters motor_configs_;

  frc846::control::HigherMotorController esc_1_;
  frc846::control::HigherMotorController esc_2_;
  frc846::control::HMCHelper<units::degree> esc_helper_1_;
  frc846::control::HMCHelper<units::degree> esc_helper_2_;

  ShooterReadings ReadFromHardware() override;

  void WriteToHardware(ShooterTarget target) override;
};

inline frc846::control::config::MotorConstructionParameters GetModifiedConfig(
    frc846::control::config::MotorConstructionParameters original_config,
    int can_id, bool inverted) {
  frc846::control::config::MotorConstructionParameters modifiedConfig =
      original_config;
  modifiedConfig.can_id = can_id;
  modifiedConfig.inverted = inverted;
  return modifiedConfig;
}