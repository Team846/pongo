#pragma once

#include "frc846/robot/GenericRobotContainer.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/hardware/leds.h"

class RobotContainer : public frc846::robot::GenericRobotContainer {
public:
  ControlInputSubsystem control_input_{};
  LEDsSubsystem leds_{};
  frc846::robot::swerve::DrivetrainSubsystem drivetrain_{
      {frc846::robot::swerve::NavX_connection_type::kSerial}};

  RobotContainer() {
    RegisterPreference("init_drivetrain", true);
    RegisterPreference("init_leds", true);

    control_input_.Init();
    if (GetPreferenceValue_bool("init_drivetrain")) drivetrain_.Init();
    if (GetPreferenceValue_bool("init_leds")) leds_.Init();

    RegisterSubsystems({&control_input_, &drivetrain_, &leds_});
  }
};
