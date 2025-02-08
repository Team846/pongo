#include "commands/teleop/climber_command.h"

ClimberCommand::ClimberCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, ClimberCommand>{
          container, "climber_command"} {
  AddRequirements({&container_.climber_});
}

void ClimberCommand::OnInit() {}

void ClimberCommand::Periodic() {
  auto ci_readings = container_.control_input_.GetReadings();

  if (ci_readings.climb_state == 1)
    container_.climber_.SetTarget(
        {container_.climber_.GetPreferenceValue_unit_type<units::degree_t>(
            "pre_climb_setpoint")});
  else if (ci_readings.climb_state == 2)
    container_.climber_.SetTarget(
        {container_.climber_.GetPreferenceValue_unit_type<units::degree_t>(
            "climb_setpoint")});
  else
    container_.climber_.SetTarget(
        {container_.climber_.GetPreferenceValue_unit_type<units::degree_t>(
            "stow_setpoint")});
}

void ClimberCommand::OnEnd(bool interrupted) {}

bool ClimberCommand::IsFinished() { return false; }