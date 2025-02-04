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

struct AlgalEEReadings {
  bool has_piece_;
};

struct AlgalEETarget {
  double duty_cycle_;
};

class AlgalEESubsystem
    : public frc846::robot::GenericSubsystem<AlgalEEReadings, AlgalEETarget> {
public:
  AlgalEESubsystem();

  AlgalEETarget ZeroTarget() const override { return {0.0}; }

  void Setup() override;

  bool VerifyHardware() override;

protected:
  frc846::control::config::MotorConstructionParameters motor_configs_;

  frc846::control::HigherMotorController esc_1_;
  frc846::control::HigherMotorController esc_2_;

  AlgalEEReadings ReadFromHardware() override;

  void WriteToHardware(AlgalEETarget target) override;
};
