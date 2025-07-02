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

  frc846::control::config::MotorConstructionParameters GetCurrentConfig(
      frc846::control::config::MotorConstructionParameters original_config);

  void SetPieceOverride(bool override_piece);

  void Setup() override;

  bool VerifyHardware() override;

protected:
  frc846::control::config::MotorConstructionParameters motor_configs_;

  frc846::control::HigherMotorController esc_1_;
  frc846::control::HigherMotorController esc_2_;

  bool piece_override_ = false;

  AlgalEEReadings ReadFromHardware() override;

  void WriteToHardware(AlgalEETarget target) override;
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