#include "commands/basic/eject_coral_command.h"

EjectCoralCommand::EjectCoralCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, EjectCoralCommand>{
          container, "eject_coral_command"} {
  AddRequirements({&container_.coral_end_effector});
}

void EjectCoralCommand::OnInit() {}

void EjectCoralCommand::Periodic() {
  container_.coral_end_effector.SetTarget({CoralEndEffectorState::kCoralScore});
}

void EjectCoralCommand::OnEnd(bool interrupted) {}

bool EjectCoralCommand::IsFinished() { return false; }