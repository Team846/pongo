#include "commands/basic/intake_coral_command.h"

IntakeCoralCommand::IntakeCoralCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, IntakeCoralCommand>{
          container, "intake_coral_command"} {
  AddRequirements({&container_.coral_end_effector});
}

void IntakeCoralCommand::OnInit() {}

void IntakeCoralCommand::Periodic() {
  container_.coral_end_effector.SetTarget({CoralEndEffectorState::kCoralIntake});
}

void IntakeCoralCommand::OnEnd(bool interrupted) {}

bool IntakeCoralCommand::IsFinished() { return false; }