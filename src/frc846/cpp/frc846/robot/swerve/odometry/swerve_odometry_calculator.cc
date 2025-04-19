#include "frc846/robot/swerve/odometry/swerve_odometry_calculator.h"

namespace frc846::robot::swerve::odometry {

SwerveOdometryCalculator::SwerveOdometryCalculator() {}

SwerveOdometryOutput SwerveOdometryCalculator::calculate(
    SwerveOdometryInputs inputs) {
  auto module_diffs = inputs.drive_pos - previous_module_positions_;

  previous_module_positions_ = inputs.drive_pos;

  frc846::math::Vector2D displacement{0_ft, 0_ft};

  std::array<frc846::math::Vector2D, 4> rotation_vecs;
  std::array<frc846::math::Vector2D, 4> trans_vecs;
  frc846::math::Vector2D min_trans_vec{1000_in, 0_in};

  for (int i = 0; i < 4; i++) {
    rotation_vecs[i] = frc846::math::Vector2D{
        units::radian_t(inputs.bearing - last_bearing).to<double>() *
            constants_.center_to_wheel,
        constants_.angle_offsets[i] + 90_deg, true};
  }

  for (int i = 0; i < 4; i++) {
    trans_vecs[i] = frc846::math::Vector2D{module_diffs[i],
                        inputs.steer_pos[i] + inputs.bearing, true} -
                    rotation_vecs[i];
    if (min_trans_vec.magnitude() > trans_vecs[i].magnitude()) {
      min_trans_vec = trans_vecs[i];
    }
  }

  for (int i = 0; i < 4; i++) {
    displacement +=
        (trans_vecs[i].resize(min_trans_vec.magnitude()) + rotation_vecs[i]) /
        4.0;
  }

  displacement *= inputs.odom_ff_;

  last_position_ += displacement;
  last_bearing = inputs.bearing;

  return {last_position_ + position_offset_};
}

}  // namespace frc846::robot::swerve::odometry