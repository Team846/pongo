#include "commands/teleop/coral_command.h"

CoralCommand::CoralCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, CoralCommand>{
          container, "coral_command"} {
  AddRequirements({&container_.coral_ss_});
}

void CoralCommand::OnInit() {}

void CoralCommand::Periodic() {
  CoralSSTarget coral_target{};
  auto ci_readings = container_.control_input_.GetReadings();

  if (container_.algal_ss_.algal_end_effector.GetReadings().has_piece_ &&
      container_.control_input_.GetReadings().coral_state ==
          kCoral_StowNoPiece) {
    if (container_.coral_ss_.coral_end_effector.GetReadings().has_piece_ ||
        (container_.control_input_.GetReadings().algal_state == kAlgae_Net &&
            container_.control_input_.GetReadings().position_algal))
      coral_target.state = kCoral_StowNet;
    else
      coral_target.state = kCoral_StowNoPiece;
  } else if (ci_readings.coral_state != kCoral_StowNoPiece)
    coral_target.state = ci_readings.coral_state;
  else if (container_.coral_ss_.coral_end_effector.GetReadings().has_piece_)
    if (ci_readings.override_autostow)
      coral_target.state = kCoral_StowNoPiece;
    else
      coral_target.state = kCoral_StowWithPiece;
  else
    coral_target.state = kCoral_StowNoPiece;
  coral_target.score = ci_readings.score_coral;

  if (ci_readings.inc_telescope)
    container_.coral_ss_.adjustTelescope(true);
  else if (ci_readings.dec_telescope)
    container_.coral_ss_.adjustTelescope(false);

  if (ci_readings.inc_c_wrist)
    container_.coral_ss_.adjustWrist(true);
  else if (ci_readings.dec_c_wrist)
    container_.coral_ss_.adjustWrist(false);

  container_.coral_ss_.SetTarget(coral_target);

  container_.coral_ss_.telescope.OverrideSoftLimits(
      ci_readings.override_soft_limits);
}

void CoralCommand::OnEnd(bool interrupted) {}

bool CoralCommand::IsFinished() { return false; }