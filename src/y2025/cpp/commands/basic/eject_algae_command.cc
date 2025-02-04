#include "commands/basic/eject_algae_command.h"

EjectAlgaeCommand::EjectAlgaeCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, EjectAlgaeCommand>{
          container, "eject_algae_command"} {
  AddRequirements({&container_.algae_end_effector});
}

void EjectAlgaeCommand::OnInit() {}

void EjectAlgaeCommand::Periodic() {
  container_.algae_end_effector.SetTarget({AlgaeEndEffectorState::kAlgaeScore});
}

void EjectAlgaeCommand::OnEnd(bool interrupted) {}

bool EjectAlgaeCommand::IsFinished() { return false; }