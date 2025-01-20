#pragma once

#include <frc2/command/Command.h>

#include <string_view>
#include <unordered_map>

#include "subsystems/robot_container.h"

class ActionMaker {
public:
  static std::unique_ptr<frc2::Command> GetAction(
      std::string_view name, RobotContainer& container);
};