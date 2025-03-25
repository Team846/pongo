#pragma once

#include <frc/DigitalInput.h>

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
  kCoral_DINOSAUR_A,
  kCoral_DINOSAUR_B,
};

struct CoralSSReadings {
  bool autostow_valid;
  bool piece_entered;
};

struct CoralSSTarget {
  CoralStates state;
  bool score;
};

class CoralSuperstructure
    : public frc846::robot::GenericSubsystem<CoralSSReadings, CoralSSTarget> {
public:
  CoralSuperstructure();

  CoralSSTarget ZeroTarget() const override { return {}; };

  void Setup() override;

  bool VerifyHardware() override;

  TelescopeSubsystem telescope;
  CoralWristSubsystem coral_wrist;
  CoralEESubsystem coral_end_effector;

  CoralSetpoint getSetpoint(CoralStates state);

  bool isHomed() { return telescope.isHomed(); }

  bool hasReached(CoralStates state);
  bool hasReachedTelescope(CoralStates state);
  bool hasReachedWrist(CoralStates state);

  void adjustTelescope(bool upwards);
  void adjustWrist(bool upwards);
  void clearAdjustments();

  CoralStates last_state;

protected:
  CoralSSReadings ReadFromHardware() override;

  void WriteToHardware(CoralSSTarget target) override;

  units::inch_t telescope_adjustment_ = 0_in;
  units::degree_t wrist_adjustment_ = 0_deg;

  int no_piece_count_;
  int see_reef_count_;

  frc::DigitalInput chute_sensor_{4};

private:
};
