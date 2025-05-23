#include "commands/teleop/algal_command.h"

#include "reef.h"

AlgalCommand::AlgalCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, AlgalCommand>{
          container, "algal_command"} {
  AddRequirements({&container_.algal_ss_});
}

void AlgalCommand::OnInit() {}

void AlgalCommand::Periodic() {
  AlgalSSTarget algal_target{};
  auto ci_readings = container_.control_input_.GetReadings();

  if (ci_readings.lock_left_reef) {
    ci_readings.algal_state =
        (ReefProvider::getClosestReefSide(
             container_.drivetrain_.GetReadings().estimated_pose.position) %
                2 ==
            0)
            ? kAlgae_L2Pick
            : kAlgae_L3Pick;
  }

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

  container_.algal_ss_.elevator.OverrideSoftLimits(
      ci_readings.override_soft_limits);
}

void AlgalCommand::OnEnd(bool interrupted) {}

bool AlgalCommand::IsFinished() { return false; }