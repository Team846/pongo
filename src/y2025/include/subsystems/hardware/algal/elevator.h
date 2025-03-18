#pragma once

#include "subsystems/hardware/generic/linear_subsystem.h"

class ElevatorSubsystem : public LinearSubsystem {
public:
  ElevatorSubsystem();

  LinearSubsystemTarget ZeroTarget() const override;

protected:
  void ExtendedSetup() override;

private:
};