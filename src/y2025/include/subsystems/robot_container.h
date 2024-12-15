#pragma once

#include "frc846/robot/GenericRobotContainer.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/hardware/leds.h"

class RobotContainer : public frc846::robot::GenericRobotContainer {
 public:
  ControlInputSubsystem control_input_{};
  LEDsSubsystem leds_{};

  RobotContainer() {
    RegisterPreference("init_drivetrain", true);
    RegisterPreference("init_leds", true);

    control_input_.Init();
    if (GetPreferenceValue_bool("init_leds")) leds_.Init();

    RegisterSubsystems({&control_input_, &leds_});
  }
};
