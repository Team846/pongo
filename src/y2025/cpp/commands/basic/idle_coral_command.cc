#include "commands/basic/idle_coral_command.h"

IdleCoralCommand::IdleCoralCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, IdleCoralCommand>{
          container, "idle_coral_command"} {
  AddRequirements({&container_.coral_end_effector});
}

void IdleCoralCommand::OnInit() {}

void IdleCoralCommand::Periodic() {
  container_.coral_end_effector.SetTarget({CoralEndEffectorState::kCoralIdle});
}

void IdleCoralCommand::OnEnd(bool interrupted) {}

bool IdleCoralCommand::IsFinished() { return false; }