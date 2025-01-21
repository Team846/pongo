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
    if (GetPreferenceValue_bool("init_drivetrain")) drivetrain_.Init();
    if (GetPreferenceValue_bool("init_leds")) leds_.Init();
    if (GetPreferenceValue_bool("init_algae_pivot")) algae_pivot_.Init();
    if (GetPreferenceValue_bool("init_climber")) climber_.Init();
    if (GetPreferenceValue_bool("init_telescope")) telescope_.Init();
    if (GetPreferenceValue_bool("init_coral_wrist")) coral_wrist_.Init();
    if (GetPreferenceValue_bool("init_elevator")) elevator_.Init();
    if (GetPreferenceValue_bool("init_ramp")) ramp_.Init();

    RegisterSubsystems({&control_input_, &drivetrain_, &leds_, &algae_pivot_,
        &telescope_, &coral_wrist_, &elevator_, &ramp_, &climber_});
  }
};

//,&climber_, &telescope_, &climber_, &coral_wrist_, &elevator_, &ramp_
