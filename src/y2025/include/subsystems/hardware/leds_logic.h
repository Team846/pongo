#pragma once

#include "subsystems/robot_container.h"

class LEDsLogic {
public:
  static void UpdateLEDs(RobotContainer* container);
  static void CoastingLEDs(RobotContainer* container, double percent);
};