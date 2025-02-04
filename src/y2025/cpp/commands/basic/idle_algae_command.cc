#include "commands/basic/idle_algae_command.h"

IdleAlgaeCommand::IdleAlgaeCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, IdleAlgaeCommand>{
          container, "idle_algae_command"} {
  AddRequirements({&container_.algae_end_effector});
}

void IdleAlgaeCommand::OnInit() {}

void IdleAlgaeCommand::Periodic() {
  container_.algae_end_effector.SetTarget({AlgaeEndEffectorState::kAlgaeIdle});
}

void IdleAlgaeCommand::OnEnd(bool interrupted) {}

bool IdleAlgaeCommand::IsFinished() { return false; }