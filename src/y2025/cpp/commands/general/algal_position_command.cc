#include "commands/general/algal_position_command.h"

AlgalPositionCommand::AlgalPositionCommand(
    RobotContainer &container, AlgalStates where)
    : frc846::robot::GenericCommand<RobotContainer,
          AlgalPositionCommand>{container, "algal_position_command"},
      where_{where} {
  AddRequirements({&container_.algal_ss_});
}

void AlgalPositionCommand::OnInit() {}

void AlgalPositionCommand::Periodic() {
  container_.algal_ss_.SetTarget({where_, false});
}

void AlgalPositionCommand::OnEnd(bool interrupted) {}

bool AlgalPositionCommand::IsFinished() {
  return !container_.algal_ss_.is_initialized() ||
         false;  // TODO add end condition
}