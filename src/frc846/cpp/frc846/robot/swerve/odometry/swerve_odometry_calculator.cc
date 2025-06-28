#include "frc846/robot/swerve/odometry/swerve_odometry_calculator.h"

using Vec2D = frc846::math::Vector2D;

#define w_h constants_.horizontal_wheelbase_dim / 2.0
#define w_f constants_.forward_wheelbase_dim / 2.0

namespace frc846::robot::swerve::odometry {

SwerveOdometryCalculator::SwerveOdometryCalculator() {}

SwerveOdometryOutput SwerveOdometryCalculator::calculate(
    SwerveOdometryInputs inputs) {
  std::array<Vec2D, 4> wheel_positions{
      Vec2D{w_h, w_f}, {-w_h, w_f}, {-w_h, -w_f}, {w_h, -w_f}};

  auto module_diffs = inputs.drive_pos - previous_module_positions_;
  previous_module_positions_ = inputs.drive_pos;

  std::array<frc846::math::Vector2D, 4> wheel_vecs;

  for (int i = 0; i < 4; i++) {
    wheel_vecs[i] =
        frc846::math::Vector2D{module_diffs[i], inputs.steer_pos[i], true};

    // TODO: the true wheel vec depends on the "hidden time" problem
  }

  units::inch_t sum_dx, sum_dy;
  units::unit_t<units::compound_unit<units::inch, units::inch>> sum_dtheta_x,
      sum_dtheta_y, sum_rr;

  for (int i = 0; i < 4; i++) {
    units::inch_t md_x = wheel_vecs[i][0];
    units::inch_t md_y = wheel_vecs[i][1];
    units::inch_t r_x = wheel_positions[i][0];
    units::inch_t r_y = wheel_positions[i][1];

    sum_dx += md_x;
    sum_dy += md_y;

    sum_dtheta_x += md_x * r_y;
    sum_dtheta_y -= md_y * r_x;

    sum_rr += r_x * r_x + r_y * r_y;
  }

  units::inch_t dx = sum_dx / 4.0;
  units::inch_t dy = sum_dy / 4.0;

  units::radian_t dtheta = (sum_dtheta_x + sum_dtheta_y) / sum_rr * 1_rad;

  Vec2D displacement;

  constexpr units::radian_t EPSILON_dtheta = 1e-6_rad;

  if (units::math::abs(dtheta) > EPSILON_dtheta) {  // TODO: fix
    // Arced approximation for displacement
    double dtheta_val = dtheta.to<double>();

    units::scalar_t sin_term = units::math::sin(dtheta) / dtheta_val;
    units::scalar_t cos_term = (1 - units::math::cos(dtheta)) / dtheta_val;

    displacement =
        Vec2D{sin_term * dx - cos_term * dy, cos_term * dx + sin_term * dy};
  } else {
    // Negligible angle change -> straight line approximation

    displacement = Vec2D{dx, dy};
  }

  displacement *= inputs.odom_ff_;

  last_position_ += displacement.rotate(inputs.bearing - dtheta);

  odom_bearing_ += dtheta;

  return {last_position_ + position_offset_, odom_bearing_};
}

}  // namespace frc846::robot::swerve::odometry