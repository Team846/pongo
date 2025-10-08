#include "commands/teleop/gpd_ss_command.h"

GPDSSCommand::GPDSSCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, GPDSSCommand>{
          container, "gpd_ss_command"} {
  AddRequirements({&container_.algal_ss_});
}

void GPDSSCommand::OnInit() {
  container_.algal_ss_.algal_end_effector.ClearHasPiece();
}

void GPDSSCommand::Periodic() {
  AlgalSSTarget algal_target{};

  auto gpd_readings = container_.GPD_.GetReadings();
  const auto [gpd_pos, valid] =
      container_.GPD_.getBestGP(gpd_readings.gamepieces);

  // TODO: determine is_on_top

  algal_target.state = (container_.control_input_.GetReadings().level_one)
                           ? kAlgae_CoralPick
                           : kAlgae_GroundIntake;

  // algal_target.score = true;

  container_.algal_ss_.SetTarget(algal_target);
}

void GPDSSCommand::OnEnd(bool interrupted) {}

bool GPDSSCommand::IsFinished() {
  return container_.algal_ss_.algal_end_effector.GetReadings().has_piece_;
}