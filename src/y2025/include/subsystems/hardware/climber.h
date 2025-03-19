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

struct ClimberReadings {};

struct ClimberTarget {
  double duty_cycle_;
};

class ClimberSubsystem
    : public frc846::robot::GenericSubsystem<ClimberReadings, ClimberTarget> {
public:
  ClimberSubsystem();

  ClimberTarget ZeroTarget() const override;

  frc846::control::config::MotorConstructionParameters GetCurrentConfig(
      frc846::control::config::MotorConstructionParameters original_config);

  void Setup() override;

  bool VerifyHardware() override;

  void BrakeSubsystem();
  void CoastSubsystem();

protected:
  frc846::control::config::MotorConstructionParameters motor_configs_;

  frc846::control::HigherMotorController esc_;
  frc846::control::HMCHelper<units::degree> esc_helper_;

  ClimberReadings ReadFromHardware() override;

  void WriteToHardware(ClimberTarget target) override;
};
