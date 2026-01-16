#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>

#include "frc846/control/HMCHelper.h"
#include "frc846/control/HigherMotorController.h"
#include "frc846/robot/GenericSubsystem.h"

struct ShooterReadings {};

struct ShooterTarget {
  double percent;
};

class ShooterSubsystem
    : public frc846::robot::GenericSubsystem<ShooterReadings, ShooterTarget> {
public:
  ShooterSubsystem();

  void Setup() override;

  ShooterTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  frc846::control::HigherMotorController esc_;
  frc846::control::HigherMotorController esc_2_;

  frc846::control::HMCHelper<units::degree> esc_helper_;
  frc846::control::HMCHelper<units::degree> esc_helper_2_;

  ShooterReadings ReadFromHardware() override;

  void WriteToHardware(ShooterTarget target) override;
};
