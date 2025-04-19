#pragma once

#include "subsystems/hardware/generic/linear_subsystem.h"

class ElevatorSubsystem : public LinearSubsystem {
public:
  ElevatorSubsystem();

  LinearSubsystemTarget ZeroTarget() const override;

protected:
  void ExtendedSetup() override;

  void RHExtension() override;

private:
  int homing_counter_ = 0;
  int safety_counter_ = 0;
};