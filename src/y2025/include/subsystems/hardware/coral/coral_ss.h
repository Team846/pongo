#pragma once

#include "subsystems/hardware/coral/coral_end_effector.h"
#include "subsystems/hardware/coral/coral_wrist.h"
#include "subsystems/hardware/coral/telescope.h"

struct CoralSetpoint {
  units::inch_t height;
  units::degree_t angle;
  double ee_dc;
};

enum CoralStates {
  kCoral_StowNoPiece,
  kCoral_StowWithPiece,
  kCoral_ScoreL2,
  kCoral_ScoreL3,
  kCoral_ScoreL4,
};

struct CoralSSReadings {};

struct CoralSSTarget {
  CoralStates state;
  bool score;
};

class CoralSuperstructure
    : public frc846::robot::GenericSubsystem<CoralSSReadings, CoralSSTarget> {
public:
  CoralSuperstructure();

  void Setup() override;

  bool VerifyHardware() override;

  TelescopeSubsystem telescope;
  CoralWristSubsystem coral_wrist;
  CoralEESubsystem coral_end_effector;

  CoralSetpoint getSetpoint(CoralStates state);

protected:
  CoralSSReadings ReadFromHardware() override;

  void WriteToHardware(CoralSSTarget target) override;
};
