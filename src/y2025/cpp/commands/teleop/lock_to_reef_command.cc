#include "commands/teleop/lock_to_reef_command.h"

#include "reef.h"

LockToReefCommand::LockToReefCommand(RobotContainer& container, bool is_left)
    : frc846::robot::GenericCommand<RobotContainer,
          LockToReefCommand>{container, "LockToReefCommand"},
      is_left_{is_left} {
  AddRequirements({&container_.drivetrain_});
}

void LockToReefCommand::OnInit() { Log("LockToReefCommand initialized"); }

void LockToReefCommand::Periodic() {
  auto pos = container_.drivetrain_.GetReadings().estimated_pose.position;
  int reef_target_pos = ReefProvider::getClosestReefSide(pos);

  auto ci_readings_ = container_.control_input_.GetReadings();
  units::inch_t adj_rate =
      container_.drivetrain_.GetPreferenceValue_unit_type<units::inch_t>(
          "lock_adj_rate");
  if (ci_readings_.rc_n_x) {
    base_adj[0] -= adj_rate;
  } else if (ci_readings_.rc_p_x) {
    base_adj[0] += adj_rate;
  } else if (ci_readings_.rc_n_y) {
    base_adj[1] -= adj_rate;
  } else if (ci_readings_.rc_p_y) {
    base_adj[1] += adj_rate;
  }

  auto target_pos =
      ReefProvider::getReefScoringLocations()[2 * reef_target_pos +
                                              (is_left_ ? 0 : 1)];

  target_pos.point += base_adj.rotate(
      container_.drivetrain_.GetReadings().estimated_pose.bearing);

  frc846 ::math::Vector2D r_vec = target_pos.point - pos;
  Graph("lock_to_point/x_err", r_vec[0]);
  Graph("lock_to_point/y_err", r_vec[1]);
  if (r_vec.magnitude() <
      container_.drivetrain_.GetPreferenceValue_unit_type<units::inch_t>(
          "lock_gains/deadband")) {
    container_.drivetrain_.SetTarget(
        frc846::robot::swerve::DrivetrainOLControlTarget{{0_fps, 0_fps},
            container_.drivetrain_.ApplyBearingPID(target_pos.bearing)});
  } else {
    frc846::control::base::MotorGains lock_gains{
        container_.drivetrain_.GetPreferenceValue_double("lock_gains/_kP"),
        container_.drivetrain_.GetPreferenceValue_double("lock_gains/_kI"),
        container_.drivetrain_.GetPreferenceValue_double("lock_gains/_kD"),
        0.0};
    units::feet_per_second_t speed_target =
        1_fps *
        (lock_gains.kP * r_vec.magnitude().to<double>() + lock_gains.kI * 0.0 +
            lock_gains.kD * std::pow(container_.drivetrain_.GetReadings()
                                         .estimated_pose.velocity.magnitude()
                                         .to<double>(),
                                3.5) +
            lock_gains.kFF * 0.0);

    container_.drivetrain_.SetTarget(
        frc846::robot::swerve::DrivetrainOLControlTarget{
            frc846::math::VectorND<units::feet_per_second, 2>{
                speed_target, r_vec.angle(true), true},
            container_.drivetrain_.ApplyBearingPID(target_pos.bearing)});
  }
}

void LockToReefCommand::OnEnd(bool interrupted) {
  container_.drivetrain_.SetTargetZero();
}

bool LockToReefCommand::IsFinished() { return false; }