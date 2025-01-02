#include "frc846/robot/swerve/odometry/swerve_odometry_calculator.h"

namespace frc846::robot::swerve::odometry {

SwerveOdometryCalculator::SwerveOdometryCalculator() {}

SwerveOdometryOutput SwerveOdometryCalculator::calculate(
    SwerveOdometryInputs inputs) {
  auto module_diffs = inputs.drive_pos - previous_module_positions_;
  previous_module_positions_ = inputs.drive_pos;

  frc846::math::Vector2D displacement{0_ft, 0_ft};

  for (int i = 0; i < 4; i++) {
    displacement +=
        frc846::math::Vector2D{inputs.drive_pos[i], inputs.steer_pos[i], true};
  }
  displacement.rotate(inputs.bearing, true);

  last_position_ += displacement;

  return {last_position_};
}

}  // namespace frc846::robot::swerve::odometry