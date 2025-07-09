#pragma once

#include "frc846/robot/GenericRobotContainer.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/abstract/gpd.h"
#include "subsystems/hardware/DrivetrainConstructor.h"
#include "subsystems/hardware/algal/algal_ss.h"
#include "subsystems/hardware/climber.h"
#include "subsystems/hardware/coral/coral_ss.h"
#include "subsystems/hardware/leds.h"

class RobotContainer : public frc846::robot::GenericRobotContainer {
public:
  LEDsSubsystem leds_{};

  DrivetrainConstructor drivetrain_constructor_{};
  frc846::robot::swerve::DrivetrainSubsystem drivetrain_{
      drivetrain_constructor_.getDrivetrainConfigs()};

  GPDSubsystem GPD_{&drivetrain_};

  CoralSuperstructure coral_ss_{};
  AlgalSuperstructure algal_ss_{};

  ControlInputSubsystem control_input_{&coral_ss_, &algal_ss_, &drivetrain_};

  ClimberSubsystem climber_{};

  RobotContainer() {
    RegisterPreference("init_drivetrain", true);
    RegisterPreference("init_leds", true);
    RegisterPreference("init_gpd", true);

    RegisterPreference("init_coral_ss", true);
    RegisterPreference("init_algal_ss", true);
    RegisterPreference("init_climber", false);

    bool drivetrain_init = (GetPreferenceValue_bool("init_drivetrain"));
    bool leds_init = (GetPreferenceValue_bool("init_leds"));
    bool gpd_init = (GetPreferenceValue_bool("init_gpd"));

    bool coral_ss_init = (GetPreferenceValue_bool("init_coral_ss"));
    bool algal_ss_init = (GetPreferenceValue_bool("init_algal_ss"));
    bool climber_init = (GetPreferenceValue_bool("init_climber"));

    RegisterSubsystemGroupAB({{&control_input_, true}});
    RegisterSubsystemGroupA({{&leds_, leds_init}});

    RegisterSubsystemGroupAB({{&drivetrain_, drivetrain_init}});
    RegisterSubsystemGroupAB({{&GPD_, gpd_init}});

    // TODO: undo
    // NVM // Superstructures not necessary - updated with extensions
    RegisterSubsystemGroupA({{&coral_ss_, coral_ss_init}});
    RegisterSubsystemGroupB({{&algal_ss_, algal_ss_init}});

    // if (coral_ss_init) {
    //   coral_ss_.Init();
    //   all_subsystems_.push_back(&coral_ss_);
    // }

    // if (algal_ss_init) {
    //   algal_ss_.Init();
    //   all_subsystems_.push_back(&algal_ss_);
    // }

    // TODO: undo
    RegisterSubsystemGroupB({{&climber_, climber_init}});
  }

  // void GroupAUpdateHardwareExtension() override {
  //   // if (control_input_.GetReadings().first_enable_exception) return;
  //   // coral_ss_.UpdateHardware();
  // }

  // void GroupBUpdateHardwareExtension() override {
  //   // if (control_input_.GetReadings().first_enable_exception) return;
  //   // algal_ss_.UpdateHardware();
  // }

  // void
  // GroupAUpdateReadingsExtension() override { /*coral_ss_.UpdateReadings();*/
  // }

  // void
  // GroupBUpdateReadingsExtension() override { /*algal_ss_.UpdateReadings();*/
  // }
};
