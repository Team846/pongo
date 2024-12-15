#pragma once

#include <frc2/command/Command.h>

#include <string>
#include <unordered_map>

#include "subsystems/robot_container.h"

class ActionMaker {
 public:
  static std::unique_ptr<frc2::Command> GetAction(std::string name,
                                                  RobotContainer& container);
};