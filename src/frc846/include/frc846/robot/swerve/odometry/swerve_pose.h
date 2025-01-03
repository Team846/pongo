#pragma once

#include "frc846/math/vectors.h"

namespace frc846::robot::swerve::odometry {

/*
SwervePose

A class representing the pose of a swerve drive robot. Contains Vector2D (check
math/vectors.h) position and velocity, and a bearing (degrees).
*/
struct SwervePose {
  frc846::math::Vector2D position;
  units::degree_t bearing;
  frc846::math::VectorND<units::feet_per_second, 2> velocity;

  SwervePose rotate(units::degree_t angle) const;
  SwervePose translate(frc846::math::Vector2D translation) const;

  /*
  Extrapolates the pose of the robot by the given time, knowing the velocity.
  Can be used for latency compensation.

  @note Can be innacurate with large time intervals, especially if the robot is
  moving at a non-constant velocity.
  */
  SwervePose extrapolate(units::second_t time) const;

  SwervePose operator+(const SwervePose& other) const;
  SwervePose operator-(const SwervePose& other) const;
};

}  // namespace frc846::robot::swerve::odometry