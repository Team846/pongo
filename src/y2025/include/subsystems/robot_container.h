#pragma once

#include "frc846/robot/GenericRobotContainer.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/hardware/DrivetrainConstructor.h"
#include "subsystems/hardware/algae_pivot.h"
#include "subsystems/hardware/climber.h"
#include "subsystems/hardware/coral_telescope.h"
#include "subsystems/hardware/coral_wrist.h"
#include "subsystems/hardware/elevator.h"
#include "subsystems/hardware/leds.h"
#include "subsystems/hardware/ramp.h"

class RobotContainer : public frc846::robot::GenericRobotContainer {
public:
  ControlInputSubsystem control_input_{};
  LEDsSubsystem leds_{};

  DrivetrainConstructor drivetrain_constructor_{};
  frc846::robot::swerve::DrivetrainSubsystem drivetrain_{
      drivetrain_constructor_.getDrivetrainConfigs()};

  AlgaePivotSubsystem algae_pivot_{};
  ClimberSubsystem climber_{};
  TelescopeSubsystem telescope_{};
  CoralWristSubsystem coral_wrist_{};
  ElevatorSubsystem elevator_{};
  RampSubsystem ramp_{};

  RobotContainer() {
    RegisterPreference("init_drivetrain", true);
    RegisterPreference("init_leds", true);
    RegisterPreference("init_algae_pivot", true);
    RegisterPreference("init_climber", true);
    RegisterPreference("init_telescope", true);
    RegisterPreference("init_coral_wrist", true);
    RegisterPreference("init_elevator", true);
    RegisterPreference("init_ramp", true);

    control_input_.Init();

    bool drivetrain_init = (GetPreferenceValue_bool("init_drivetrain"));
    bool algae_pivot_init = (GetPreferenceValue_bool("init_algae_pivot"));
    bool leds_init = (GetPreferenceValue_bool("init_leds"));
    bool climber_init = (GetPreferenceValue_bool("init_climber"));
    bool telescope_init = (GetPreferenceValue_bool("init_telescope"));
    bool coral_wrist_init = (GetPreferenceValue_bool("init_coral_wrist"));
    bool elevator_init = (GetPreferenceValue_bool("init_elevator"));
    bool ramp_init = (GetPreferenceValue_bool("init_ramp"));

    RegisterSubsystemGroupA({{&control_input_, true}});
    RegisterSubsystemGroupB({{&leds_, leds_init}});

    RegisterSubsystemGroupA({{&ramp_, &ramp_init}});
    RegisterSubsystemGroupB({{&climber_, &climber_init}});

    RegisterSubsystemGroupA({{&algae_pivot_, &algae_pivot_init}});
    RegisterSubsystemGroupA({{&elevator_, &elevator_init}});

    RegisterSubsystemGroupB({{&telescope_, &telescope_init}});
    RegisterSubsystemGroupB({{&coral_wrist_, &coral_wrist_init}});

    RegisterSubsystemGroupAB({{&drivetrain_, drivetrain_init}});
  }
};

//,&climber_, &telescope_, &climber_, &coral_wrist_, &elevator_, &ramp_
