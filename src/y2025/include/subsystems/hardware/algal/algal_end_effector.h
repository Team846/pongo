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
  units::feet_per_second_t velocity_;
  bool use_back_spin_ = false;
  bool coral_mode_ = false;
  bool pick_mode = false;
  bool cm = false;
};

using roller_pos_conv_t = units::unit_t<
    units::compound_unit<units::feet, units::inverse<units::turn>>>;

class AlgalEESubsystem
    : public frc846::robot::GenericSubsystem<AlgalEEReadings, AlgalEETarget> {
public:
  AlgalEESubsystem();

  AlgalEETarget ZeroTarget() const override { return {0.0_fps}; }

  frc846::control::config::MotorConstructionParameters GetCurrentConfig(
      frc846::control::config::MotorConstructionParameters original_config);

  void SetPieceOverride(bool override_piece);

  void Setup() override;

  bool VerifyHardware() override;

protected:
  frc846::control::config::MotorConstructionParameters motor_configs_;

  frc846::control::HigherMotorController esc_1_;
  frc846::control::HigherMotorController esc_2_;

  frc846::control::HMCHelper<units::feet> esc_helper_1_;
  frc846::control::HMCHelper<units::feet> esc_helper_2_;

  roller_pos_conv_t roller_reduction_ = 0.5_ft / 1_tr;

  bool piece_override_ = false;

  double counter_ = 0;

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