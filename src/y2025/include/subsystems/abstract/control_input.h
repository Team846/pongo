#pragma once

#include "frc846/robot/GenericSubsystem.h"
#include "frc846/robot/xbox.h"

struct ControlInputReadings {
  double translate_x;
  double translate_y;
  double rotation;

  bool zero_bearing;
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

 private:
  ControlInputReadings previous_readings_{};

  frc846::robot::XboxReadings previous_driver_{};
  frc846::robot::XboxReadings previous_operator_{};

  frc::XboxController driver_{0};
  frc::XboxController operator_{1};

  ControlInputReadings ReadFromHardware() override;

  void WriteToHardware(ControlInputTarget target) override;
};
