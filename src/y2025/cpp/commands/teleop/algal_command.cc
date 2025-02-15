#include "commands/teleop/algal_command.h"

AlgalCommand::AlgalCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, AlgalCommand>{
          container, "algal_command"} {
  AddRequirements({&container_.algal_ss_});
}

void AlgalCommand::OnInit() {}

void AlgalCommand::Periodic() {
  AlgalSSTarget algal_target{};
  auto ci_readings = container_.control_input_.GetReadings();

  if (ci_readings.position_algal)
    algal_target.state = ci_readings.algal_state;
  else
    algal_target.state = kAlgae_Stow;

  algal_target.score = ci_readings.score_algae;

  container_.algal_ss_.SetTarget(algal_target);
}

void AlgalCommand::OnEnd(bool interrupted) {}

bool AlgalCommand::IsFinished() { return false; }