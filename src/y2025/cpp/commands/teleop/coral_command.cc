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

  if (ci_readings.position_algal)
    coral_target.state = ci_readings.coral_state;
  else
    coral_target.state = kCoral_StowNoPiece;  // TODO: fix coral stow

  coral_target.score = ci_readings.score_coral;

  container_.coral_ss_.SetTarget(coral_target);
}

void CoralCommand::OnEnd(bool interrupted) {}

bool CoralCommand::IsFinished() { return false; }