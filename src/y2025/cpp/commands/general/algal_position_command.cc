#include "commands/general/algal_position_command.h"

AlgalPositionCommand::AlgalPositionCommand(
    RobotContainer &container, AlgalStates where, bool score)
    : frc846::robot::GenericCommand<RobotContainer,
          AlgalPositionCommand>{container, "algal_position_command"},
      where_{where},
      score_{score} {
  AddRequirements({&container_.algal_ss_});
}

void AlgalPositionCommand::OnInit() {}

void AlgalPositionCommand::Periodic() {
  AlgalSSTarget algal_target{};
  algal_target.state = where_;
  algal_target.score = container_.algal_ss_.hasReached(where_) && score_;

  container_.algal_ss_.SetTarget(algal_target);
}

void AlgalPositionCommand::OnEnd(bool interrupted) {}

bool AlgalPositionCommand::IsFinished() {
  return !container_.algal_ss_.is_initialized() ||
         (container_.algal_ss_.hasReached(where_) &&
             (!container_.coral_ss_.coral_end_effector.GetReadings()
                     .has_piece_ ||
                 !score_));
}