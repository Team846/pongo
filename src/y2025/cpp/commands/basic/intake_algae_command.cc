#include "commands/basic/intake_algae_command.h"

IntakeAlgaeCommand::IntakeAlgaeCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, IntakeAlgaeCommand>{
          container, "intake_algae_command"} {
  AddRequirements({&container_.algae_end_effector});
}

void IntakeAlgaeCommand::OnInit() {}

void IntakeAlgaeCommand::Periodic() {
  container_.algae_end_effector.SetTarget({AlgaeEndEffectorState::kAlgaeScore});
}

void IntakeAlgaeCommand::OnEnd(bool interrupted) {}

bool IntakeAlgaeCommand::IsFinished() { return false; }