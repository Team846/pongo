#include "frc846/robot/swerve/odometry/swerve_odometry_calculator.h"

#include <iostream>

namespace frc846::robot::swerve::odometry {

SwerveOdometryCalculator::SwerveOdometryCalculator() {}

SwerveOdometryOutput SwerveOdometryCalculator::calculate(
    SwerveOdometryInputs inputs) {
  auto module_diffs = inputs.drive_pos - previous_module_positions_;

  previous_module_positions_ = inputs.drive_pos;

  frc846::math::Vector2D displacement{0_ft, 0_ft};

  std::array<frc846::math::Vector2D, 4> wheel_vecs;
  std::array<frc846::math::Vector2D, 4> rotation_vecs;
  std::array<frc846::math::Vector2D, 4> trans_vecs;
  frc846::math::Vector2D min_trans_vec{1000_in, 0_in};

  for (int i = 0; i < 4; i++) {
    wheel_vecs[i] = frc846::math::Vector2D{
        module_diffs[i], inputs.steer_pos[i] + inputs.bearing, true};

    units::degree_t rotation_direction = constants_.angle_offsets[i] + 90_deg;
    std::cout << "Rot dir " << i << ": " << rotation_direction.to<double>()
              << std::endl;

    rotation_vecs[i] = wheel_vecs[i].projectOntoAnother(
        frc846::math::Vector2D{1_in, rotation_direction, true});
  }

  frc846::math::Vector2D avg_rotation_vec{0_in, 0_in};
  for (int i = 0; i < 4; i++) {
    std::cout << i << ">" << wheel_vecs[i].magnitude().to<double>() << ", "
              << wheel_vecs[i].angle(true).to<double>() << std::endl;
    std::cout << i << ":" << rotation_vecs[i].magnitude().to<double>() << ", "
              << rotation_vecs[i].angle(true).to<double>() << std::endl;
    avg_rotation_vec += rotation_vecs[i];
  }
  avg_rotation_vec /= 4.0;
  std::cout << avg_rotation_vec.magnitude().to<double>() << ", "
            << avg_rotation_vec.angle(true).to<double>() << std::endl;
  units::degree_t rot_displacement_odom =
      1_rad * avg_rotation_vec.magnitude() / constants_.center_to_wheel;
  odom_bearing_ += rot_displacement_odom;

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

  return {last_position_ + position_offset_, odom_bearing_};
}

}  // namespace frc846::robot::swerve::odometry