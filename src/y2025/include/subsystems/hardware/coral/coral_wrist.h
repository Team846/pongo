#pragma once

#include "subsystems/hardware/generic/wrist_subsystem.h"

class CoralWristSubsystem : public WristSubsystem {
public:
  CoralWristSubsystem();

protected:
  void ExtendedSetup() override;
  std::pair<units::degree_t, bool> GetSensorPos() override;

private:
};