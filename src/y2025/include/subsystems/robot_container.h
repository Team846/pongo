#pragma once

#include "frc846/robot/GenericRobotContainer.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/abstract/gpd.h"
#include "subsystems/hardware/DrivetrainConstructor.h"
#include "subsystems/hardware/leds.h"

class RobotContainer : public frc846::robot::GenericRobotContainer {
public:
  ControlInputSubsystem control_input_{};
  LEDsSubsystem leds_{};

  DrivetrainConstructor drivetrain_constructor_{};
  frc846::robot::swerve::DrivetrainSubsystem drivetrain_{
      drivetrain_constructor_.getDrivetrainConfigs()};

  GPDSubsystem GPD_{&drivetrain_};

  RobotContainer() {
    RegisterPreference("init_drivetrain", true);
    RegisterPreference("init_leds", true);
    RegisterPreference("init_gpd", true);

    bool drivetrain_init = (GetPreferenceValue_bool("init_drivetrain"));
    bool leds_init = (GetPreferenceValue_bool("init_leds"));
    bool gpd_init = (GetPreferenceValue_bool("init_gpd"));

    RegisterSubsystemGroupA({{&control_input_, true}});
    RegisterSubsystemGroupAB({{&drivetrain_, drivetrain_init}});
    RegisterSubsystemGroupB({{&leds_, leds_init}});
    RegisterSubsystemGroupAB({{&GPD_, gpd_init}});
  }
};
