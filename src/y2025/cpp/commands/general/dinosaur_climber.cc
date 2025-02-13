#include "commands/general/dinosaur_climber.h"

DinosaurClimberCommand::DinosaurClimberCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, DinosaurClimberCommand>{
          container, "dinosaur_climber_command"} {
  AddRequirements({&container_.climber_});
}

void DinosaurClimberCommand::OnInit() {}

void DinosaurClimberCommand::Periodic() {
  units::degree_t target_pos =
      isA ? container_.climber_.GetPreferenceValue_unit_type<units::degree_t>(
                "dinosaur_A_setpoint")
          : container_.climber_.GetPreferenceValue_unit_type<units::degree_t>(
                "dinosaur_B_setpoint");
  container_.climber_.SetTarget({target_pos});

  if (units::math::abs(
          container_.climber_.GetReadings().position - target_pos) < 5_deg) {
    isA = !isA;
  }
}

void DinosaurClimberCommand::OnEnd(bool interrupted) {}

bool DinosaurClimberCommand::IsFinished() { return false; }