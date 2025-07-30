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

struct CoralEEReadings {
  bool has_piece_;
  bool see_reef;
};

struct CoralEETarget {
  double duty_cycle_;
};

class CoralEESubsystem
    : public frc846::robot::GenericSubsystem<CoralEEReadings, CoralEETarget> {
public:
  CoralEESubsystem();

  CoralEETarget ZeroTarget() const override { return {0.0}; }

  frc846::control::config::MotorConstructionParameters GetCurrentConfig(
      frc846::control::config::MotorConstructionParameters original_config);

  void SetPieceOverride(bool override_piece);
  void SetReefOverride(bool override_reef);

  void Setup() override;

  bool VerifyHardware() override;

protected:
  frc846::control::config::MotorConstructionParameters motor_configs_;

  frc846::control::HigherMotorController esc_;

  bool piece_override_ = false;
  bool reef_override_ = false;

  CoralEEReadings ReadFromHardware() override;

  void WriteToHardware(CoralEETarget target) override;
};
