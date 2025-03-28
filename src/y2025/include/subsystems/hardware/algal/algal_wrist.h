#pragma once

#include "subsystems/hardware/generic/wrist_subsystem.h"

class AlgalWristSubsystem : public WristSubsystem {
public:
  AlgalWristSubsystem();

  WristTarget ZeroTarget() const override;

protected:
  void ExtendedSetup() override;
  std::pair<units::degree_t, bool> GetSensorPos(
      units::degree_t sensor_pos) override;

private:
  static constexpr wrist_pos_conv_t encoder_reduction = 1_tr / 81_tr;
  static constexpr wrist_pos_conv_t encoder_to_subsystem_reduction =
      16_tr / 40_tr;
};