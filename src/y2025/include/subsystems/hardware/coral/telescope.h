#pragma once

#include "subsystems/hardware/generic/linear_subsystem.h"

class TelescopeSubsystem : public LinearSubsystem {
public:
  TelescopeSubsystem();

protected:
  void ExtendedSetup() override;
  std::pair<units::inch_t, bool> GetSensorPos() override;

private:
};