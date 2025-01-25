#pragma once

#include "frc846/robot/GenericRobotContainer.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/hardware/DrivetrainConstructor.h"
#include "subsystems/hardware/leds.h"

class RobotContainer : public frc846::robot::GenericRobotContainer {
public:
  ControlInputSubsystem control_input_{};
  LEDsSubsystem leds_{};

  DrivetrainConstructor drivetrain_constructor_{};
  frc846::robot::swerve::DrivetrainSubsystem drivetrain_{
      drivetrain_constructor_.getDrivetrainConfigs()};

  RobotContainer() {
    RegisterPreference("init_drivetrain", true);
    RegisterPreference("init_leds", true);

    control_input_.Init();
    bool drivetrain_init = (GetPreferenceValue_bool("init_drivetrain"));
    bool leds_init = (GetPreferenceValue_bool("init_leds"));

    RegisterSubsystemGroupA({{&control_input_, true}});
    RegisterSubsystemGroupB({{&leds_, leds_init}});
    RegisterSubsystemGroupAB({{&drivetrain_, drivetrain_init}});
  }
};
