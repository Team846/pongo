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

  if (ci_readings.coral_state != kCoral_StowNoPiece)
    coral_target.state = ci_readings.coral_state;
  else if (container_.coral_ss_.coral_end_effector.GetReadings().has_piece_)
    coral_target.state = kCoral_StowWithPiece;
  else
    coral_target.state = kCoral_StowNoPiece;
  coral_target.score = ci_readings.score_coral;

  if (ci_readings.inc_elevator)
    container_.coral_ss_.adjustTelescope(true);
  else if (ci_readings.dec_elevator)
    container_.coral_ss_.adjustTelescope(false);

  if (ci_readings.inc_c_wrist)
    container_.coral_ss_.adjustWrist(true);
  else if (ci_readings.dec_c_wrist)
    container_.coral_ss_.adjustWrist(false);

  container_.coral_ss_.SetTarget(coral_target);
}

void CoralCommand::OnEnd(bool interrupted) {}

bool CoralCommand::IsFinished() { return false; }