#pragma once

#include "subsystems/hardware/generic/wrist_subsystem.h"

class AlgalWristSubsystem : public WristSubsystem {
public:
  AlgalWristSubsystem();

  WristTarget ZeroTarget() const override;

protected:
  void ExtendedSetup() override;
  std::pair<units::degree_t, bool> GetSensorPos() override;

private:
};