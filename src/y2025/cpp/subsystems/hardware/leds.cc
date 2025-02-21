#include "subsystems/hardware/leds.h"

LEDsSubsystem::LEDsSubsystem()
    : frc846::robot::GenericSubsystem<LEDsReadings, LEDsTarget>("leds") {}

void LEDsSubsystem::Setup() {
  leds_.SetLength(kLength);
  leds_.SetData(leds_buffer_);
  leds_.Start();
}

LEDsTarget LEDsSubsystem::ZeroTarget() const { return {kLEDsUnready}; }

bool LEDsSubsystem::VerifyHardware() { return true; }

LEDsReadings LEDsSubsystem::ReadFromHardware() { return {}; }

void LEDsSubsystem::SetRainbow() {
  int kRange = 180;
  int kStart = 0;
  for (int i = 0; i < kLength; i++) {
    const auto pixelHue =
        ((first_pixel_hue_ + (i * kRange / kLength)) % kRange + kStart) % 180;
    leds_buffer_[i].SetHSV(pixelHue, 255, 255);
  }
  first_pixel_hue_ += 7;
  first_pixel_hue_ %= kRange;
}

void LEDsSubsystem::SetStrip(int R, int G, int B) {
  for (int i = 0; i < kLength; i++) {
    leds_buffer_[i].SetRGB(G, R, B);
  }
}

void LEDsSubsystem::Flash(int loops_on) {
  for (int i = 0; i < kLength; i++) {
    if (loops % loops_on < loops_on / 2) leds_buffer_[i].SetRGB(0, 0, 0);
  }
}

#define RED 255, 0, 0
#define BLUE 0, 0, 255
#define GREEN 0, 255, 0
#define ORANGE 255, 17, 0

#define SLOW_FLASH 30
#define MED_FLASH 12
#define RAPID_FLASH 7

void LEDsSubsystem::WriteToHardware(LEDsTarget target) {
  if (target.state == kLEDsUnready) {
    SetStrip(RED);
  } else if (target.state == kLEDsDisabled) {
    SetStrip(ORANGE);
  } else if (target.state == kLEDsAutonomous) {
    SetStrip(BLUE);
  } else if (target.state == kLEDsSequencing) {
    SetStrip(GREEN);
    Flash(RAPID_FLASH);
  } else if (target.state == kLEDsTeleop) {
    SetStrip(ORANGE);
    Flash(SLOW_FLASH);
  } else if (target.state == kLEDsClimbing) {
    SetStrip(GREEN);
    Flash(SLOW_FLASH);
  } else if (target.state == kLEDsHavePiece) {
    SetRainbow();
  } else if (target.state == kisLinedUp) {
    SetStrip(GREEN);
    Flash(MED_FLASH);
  }

  loops++;
  loops %= 100;

  leds_.SetData(leds_buffer_);
}
