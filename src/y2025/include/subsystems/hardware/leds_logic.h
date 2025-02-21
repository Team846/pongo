#pragma once

#include "frc846/robot/GenericRobot.h"
#include "frc846/robot/swerve/drivetrain.h"
#include "subsystems/robot_container.h"

class LEDsLogic {
public:
  static void UpdateLEDs(RobotContainer* container);

  static LEDsState checkLoc(RobotContainer* container, LEDsTarget target);
  static std::string selected_auto;

  static void SetLEDsState(RobotContainer* container, LEDsState state);
  static void CoastingLEDs(RobotContainer* container, double percent);
};