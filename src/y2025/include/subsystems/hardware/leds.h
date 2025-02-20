#pragma once

#include <frc/AddressableLED.h>

#include "frc846/robot/GenericRobot.h"
#include "frc846/robot/GenericSubsystem.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "ports.h"

enum LEDsState {
  kLEDsUnready,
  kLEDsDisabled,
  kLEDsAutonomous,
  kLEDsSequencing,
  kLEDsTeleop,
  kLEDsClimbing,
  kLEDsHavePiece,
  kisLinedUp,
};

struct LEDsReadings {};

struct LEDsTarget {
  LEDsState state;
};

class LEDsSubsystem
    : public frc846::robot::GenericSubsystem<LEDsReadings, LEDsTarget> {
public:
  LEDsSubsystem();

  void Setup() override;

  LEDsTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  void SetRainbow();
  void SetStrip(int R, int G, int B);
  void Flash(int loops_on);

  // Number of LEDs.
  static constexpr int kLength = 23;

  std::array<frc::AddressableLED::LEDData, kLength> leds_buffer_;

  frc::AddressableLED leds_{ports::leds_::kLEDStrip1};

  LEDsReadings ReadFromHardware() override;

  int loops = 0;
  int first_pixel_hue_ = 0;

  void WriteToHardware(LEDsTarget target) override;
};
