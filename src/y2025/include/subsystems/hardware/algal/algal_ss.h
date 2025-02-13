#pragma once

#include "subsystems/hardware/algal/algal_end_effector.h"
#include "subsystems/hardware/algal/algal_wrist.h"
#include "subsystems/hardware/algal/elevator.h"

struct AlgalSetpoint {
  units::inch_t height;
  units::degree_t angle;
  double ee_dc;
};

enum AlgalStates {
  kAlgae_Stow,
  kAlgae_Processor,
  kAlgae_GroundIntake,
  kAlgae_OnTopIntake,
  kAlgae_Net,
  kAlgae_L2Pick,
  kAlgae_L3Pick,
};

struct AlgalSSReadings {};

struct AlgalSSTarget {
  AlgalStates state;
  bool score;
  std::optional<AlgalStates> separate_wrist_state = std::nullopt;
};

class AlgalSuperstructure
    : public frc846::robot::GenericSubsystem<AlgalSSReadings, AlgalSSTarget> {
public:
  AlgalSuperstructure();

  AlgalSSTarget ZeroTarget() const override { return {}; };

  void Setup() override;

  bool VerifyHardware() override;

  ElevatorSubsystem elevator;
  AlgalWristSubsystem algal_wrist;
  AlgalEESubsystem algal_end_effector;

  AlgalSetpoint getSetpoint(AlgalStates state);

  bool isHomed() { return elevator.isHomed(); }

  bool hasReached(AlgalStates state);

protected:
  AlgalSSReadings ReadFromHardware() override;

  void WriteToHardware(AlgalSSTarget target) override;
};
