#include "commands/general/l1_pos.h"

L1PosCommand::L1PosCommand(RobotContainer &container, bool score)
    : frc846::robot::GenericCommand<RobotContainer, L1PosCommand>{container,
          "l1_position_command"},
      score_{score} {
  AddRequirements({&container_.algal_ss_});
}

void L1PosCommand::OnInit() {}

void L1PosCommand::Periodic() {
  AlgalSSTarget algal_target{};
  algal_target.state = kAlgae_L1CoralScore;
  algal_target.score =
      container_.algal_ss_.hasReached(kAlgae_L1CoralScore) && score_;

  container_.algal_ss_.SetTarget(algal_target);
}

void L1PosCommand::OnEnd(bool interrupted) {}

bool L1PosCommand::IsFinished() { return false; }