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

  if (ci_readings.inc_elevator)
    container_.algal_ss_.adjustElevator(true);
  else if (ci_readings.dec_elevator)
    container_.algal_ss_.adjustElevator(false);

  if (ci_readings.inc_a_wrist)
    container_.algal_ss_.adjustWrist(true);
  else if (ci_readings.dec_a_wrist)
    container_.algal_ss_.adjustWrist(false);

  container_.algal_ss_.SetTarget(algal_target);
}

void AlgalCommand::OnEnd(bool interrupted) {}

bool AlgalCommand::IsFinished() { return false; }