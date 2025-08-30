#pragma once

#include "subsystems/hardware/generic/iclin.h"

class TelescopeSubsystem : public IclinSubsystem {
public:
  TelescopeSubsystem();

  IclinTarget ZeroTarget() const override;

protected:
  void ExtendedSetup() override;

private:
};