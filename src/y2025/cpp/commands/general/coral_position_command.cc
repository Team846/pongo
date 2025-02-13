#include "commands/general/coral_position_command.h"

CoralPositionCommand::CoralPositionCommand(
    RobotContainer &container, CoralStates where, bool score)
    : frc846::robot::GenericCommand<RobotContainer,
          CoralPositionCommand>{container, "coral_position_command"},
      where_{where},
      score_{score} {
  AddRequirements({&container_.coral_ss_});
}

void CoralPositionCommand::OnInit() {}

void CoralPositionCommand::Periodic() {
  CoralSSTarget coral_target{};
  coral_target.state = where_;
  coral_target.score = score_ && false;  // TODO: check if reached

  container_.coral_ss_.SetTarget(coral_target);
}

void CoralPositionCommand::OnEnd(bool interrupted) {}

bool CoralPositionCommand::IsFinished() {
  return !container_.coral_ss_.is_initialized() ||
         (false && (!container_.algal_ss_.algal_end_effector.GetReadings()
                           .has_piece_ ||
                       !score_));  // TODO add end condition
}