#include "commands/teleop/climber_command.h"

ClimberCommand::ClimberCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, ClimberCommand>{
          container, "climber_command"} {
  AddRequirements({&container_.climber_});
}

void ClimberCommand::OnInit() {}

void ClimberCommand::Periodic() {
  auto ci_readings = container_.control_input_.GetReadings();

  if (ci_readings.extend_climb)
    container_.climber_.SetTarget(
        {container_.climber_.GetPreferenceValue_double("extend_dc")});
  else if (ci_readings.retract_climb)
    container_.climber_.SetTarget(
        {container_.climber_.GetPreferenceValue_double("retract_dc")});
}

void ClimberCommand::OnEnd(bool interrupted) {}

bool ClimberCommand::IsFinished() { return false; }