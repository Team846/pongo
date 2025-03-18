#pragma once

#include "subsystems/hardware/generic/wrist_subsystem.h"

class CoralWristSubsystem : public WristSubsystem {
public:
  CoralWristSubsystem();

  WristTarget ZeroTarget() const override;

protected:
  void ExtendedSetup() override;
  std::pair<units::degree_t, bool> GetSensorPos() override;

private:
  static constexpr wrist_pos_conv_t subsystem_reduction =
      18_tr / 84_tr * 1_tr / 21_tr;
};