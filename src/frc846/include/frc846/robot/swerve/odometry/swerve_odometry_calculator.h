#pragma once

#include <units/angle.h>
#include <units/length.h>

#include <array>

#include "frc846/math/calculator.h"
#include "frc846/math/vectors.h"

namespace frc846::robot::swerve::odometry {

struct SwerveOdometryConstants {
  std::array<units::degree_t, 4> angle_offsets;
  units::inch_t center_to_wheel;
};

struct SwerveOdometryInputs {
  units::degree_t bearing;
  std::array<units::degree_t, 4> steer_pos;
  frc846::math::VectorND<units::inch, 4> drive_pos;
  double odom_ff_;
};

struct SwerveOdometryOutput {
  frc846::math::Vector2D position;
};

class SwerveOdometryCalculator
    : public frc846::math::Calculator<SwerveOdometryInputs,
          SwerveOdometryOutput, SwerveOdometryConstants> {
public:
  SwerveOdometryCalculator();

  SwerveOdometryOutput calculate(SwerveOdometryInputs inputs) override;

  void SetPosition(frc846::math::Vector2D position) {
    position_offset_ = position - last_position_;
  }

private:
  frc846::math::VectorND<units::inch, 4> previous_module_positions_;

  frc846::math::Vector2D last_position_;

  units::degree_t last_bearing;

  frc846::math::Vector2D position_offset_{};
};

}  // namespace frc846::robot::swerve::odometry
