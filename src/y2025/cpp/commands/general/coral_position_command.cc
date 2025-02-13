#include "commands/general/coral_position_command.h"

CoralPositionCommand::CoralPositionCommand(
    RobotContainer &container, CoralStates where)
    : frc846::robot::GenericCommand<RobotContainer,
          CoralPositionCommand>{container, "coral_position_command"},
      where_{where} {
  AddRequirements({&container_.coral_ss_});
}

void CoralPositionCommand::OnInit() {}

void CoralPositionCommand::Periodic() {
  container_.coral_ss_.SetTarget({where_, false});
}

void CoralPositionCommand::OnEnd(bool interrupted) {}

bool CoralPositionCommand::IsFinished() {
  return !container_.coral_ss_.is_initialized() ||
         false;  // TODO add end condition
}