#pragma once

#include "frc846/math/vectors.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/robot/xbox.h"

struct ControlInputReadings {
  double translate_x;
  double translate_y;

  bool rc_p_y;
  bool rc_p_x;
  bool rc_n_y;
  bool rc_n_x;
  bool rc_control;

  double rotation;

  bool test_move_10_ft;  // TODO: remove when not needed
  bool test_bearing_pid;
  bool lock_left_reef;
  bool lock_right_reef;
  bool lock_processor;

  bool zero_bearing;

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
  ControlInputSubsystem();

  void Setup() override;

  ControlInputTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  ControlInputReadings UpdateWithInput();

  frc846::math::Vector2D base_adj{0_in, 0_in};

private:
  ControlInputReadings previous_readings_{};

  frc846::robot::XboxReadings previous_driver_{};
  frc846::robot::XboxReadings previous_operator_{};

  frc::XboxController driver_{0};
  frc::XboxController operator_{1};

  ControlInputReadings ReadFromHardware() override;

  void WriteToHardware(ControlInputTarget target) override;
};
