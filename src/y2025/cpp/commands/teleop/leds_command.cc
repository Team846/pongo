#include "commands/teleop/leds_command.h"

LEDsCommand::LEDsCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, LEDsCommand>{
          container, "leds_command"} {
  AddRequirements({&container_.leds_});
}

void LEDsCommand::OnInit() {}

void LEDsCommand::Periodic() {
  LEDsState lstate{};

  container_.leds_.SetTarget({lstate});
}

void LEDsCommand::OnEnd(bool interrupted) {}

bool LEDsCommand::IsFinished() { return false; }