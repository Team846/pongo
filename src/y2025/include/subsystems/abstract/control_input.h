#pragma once

#include "frc846/math/vectors.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/robot/xbox.h"
#include "subsystems/hardware/algal/algal_ss.h"
#include "subsystems/hardware/coral/coral_ss.h"

struct ControlInputReadings {
  // Driving
  double translate_x;
  double translate_y;

  // Reef adjustment
  bool rc_p_y;
  bool rc_p_x;
  bool rc_n_y;
  bool rc_n_x;
  bool rc_control;

  // Driving
  double rotation;

  // Alignment
  bool lock_left_reef;
  bool lock_right_reef;
  bool auto_align;

  // Superstructure
  bool position_algal;
  AlgalStates algal_state;
  CoralStates coral_state;

  // Scoring
  bool score_coral;
  bool score_algae;

  // Climb: 1: Preclimb, 2: Climb, 3: Reset
  int climb_state;

  // Resets
  bool zero_bearing;

  // Overrides
  bool override_autostow;

  // Adjustments
  bool inc_telescope;
  bool dec_telescope;
  bool inc_c_wrist;
  bool dec_c_wrist;
  bool inc_elevator;
  bool dec_elevator;
  bool inc_a_wrist;
  bool dec_a_wrist;

  bool targeting_algae;
};

struct ControlInputTarget {
  bool driver_rumble;
  bool operator_rumble;
};

class ControlInputSubsystem
    : public frc846::robot::GenericSubsystem<ControlInputReadings,
          ControlInputTarget> {
public:
  ControlInputSubsystem(CoralSuperstructure* coral_ss);

  void Setup() override;

  ControlInputTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  ControlInputReadings UpdateWithInput();

  frc846::math::Vector2D base_adj{0_in, 0_in};

private:
  ControlInputReadings previous_readings_{};

  CoralSuperstructure* coral_ss_;

  frc846::robot::XboxReadings previous_driver_{};
  frc846::robot::XboxReadings previous_operator_{};

  frc::XboxController driver_{0};
  frc::XboxController operator_{1};

  int climb_state_ = 0;

  ControlInputReadings ReadFromHardware() override;

  void WriteToHardware(ControlInputTarget target) override;
};
