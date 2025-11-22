#include "subsystems/hardware/leds_logic.h"

#include <iostream>

void LEDsLogic::UpdateLEDs(RobotContainer* container) {
  LEDsTarget target{kLEDsUnready};
  container->leds_.SetTarget(target);
}

void LEDsLogic::SetLEDsState(RobotContainer* container, LEDsState state) {
  LEDsTarget target{state};
  container->leds_.SetTarget(target);
}

void LEDsLogic::CoastingLEDs(RobotContainer* container, double percent) {
  container->leds_.SetTarget(LEDsCoastingTarget{percent});
}