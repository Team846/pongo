#include "subsystems/hardware/leds.h"

LEDsSubsystem::LEDsSubsystem()
    : frc846::robot::GenericSubsystem<LEDsReadings, TLTGT>("leds") {}

void LEDsSubsystem::Setup() {
  leds_.SetLength(kLength);
  leds_.SetData(leds_buffer_);
  leds_.Start();
}

TLTGT LEDsSubsystem::ZeroTarget() const { return LEDsTarget{kLEDsUnready}; }

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
#define LIGHT_BLUE 25, 25, 255
#define GREEN 0, 255, 0
#define ORANGE 255, 17, 0
#define PURPLE 255, 0, 255

#define SLOW_FLASH 30
#define MED_FLASH 12
#define RAPID_FLASH 7

void LEDsSubsystem::WriteToHardware(TLTGT target) {
  if (auto* tgt = std::get_if<LEDsTarget>(&target)) {
    if (tgt->state == kLEDsHoming) {
      SetStrip(PURPLE);
      Flash(RAPID_FLASH);
    } else if (tgt->state == kLEDsHomingGyro) {
      SetStrip(GREEN);
      Flash(RAPID_FLASH);
    } else if (tgt->state == kLEDsUnready) {
      SetStrip(RED);
    } else if (tgt->state == kLEDsDisabled) {
      SetStrip(ORANGE);
    } else if (tgt->state == kLEDsAutonomous) {
      SetStrip(BLUE);
    } else if (tgt->state == kLEDsSequencing) {
      SetStrip(LIGHT_BLUE);
      Flash(RAPID_FLASH);
    } else if (tgt->state == kLEDsTeleop) {
      SetStrip(ORANGE);
      Flash(SLOW_FLASH);
    } else if (tgt->state == kLEDsClimbing) {
      SetStrip(LIGHT_BLUE);
      Flash(SLOW_FLASH);
    } else if (tgt->state == kLEDsHavePiece) {
      SetRainbow();
    }
  } else if (auto* tgt = std::get_if<LEDsCoastingTarget>(&target)) {
    for (int i = 0; i < (int)(kLength * tgt->percent); i++) {
      leds_buffer_[i].SetRGB(LIGHT_BLUE);
    }
    for (int i = (int)(kLength * tgt->percent) + 1; i < kLength; i++) {
      leds_buffer_[i].SetRGB(0, 0, 0);
    }
  }

  loops++;
  loops %= 100;

  leds_.SetData(leds_buffer_);
}
