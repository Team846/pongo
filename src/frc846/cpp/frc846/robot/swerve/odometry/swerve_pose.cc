#include "frc846/robot/swerve/odometry/swerve_pose.h"

namespace frc846::robot::swerve::odometry {

SwervePose SwervePose::rotate(units::degree_t angle) const {
  return SwervePose{
      position.rotate(angle), bearing + angle, velocity.rotate(angle)};
}

SwervePose SwervePose::translate(frc846::math::Vector2D translation) const {
  return SwervePose{position + translation, bearing, velocity};
}

SwervePose SwervePose::extrapolate(units::second_t time) const {
  return SwervePose{
      position + frc846::math::Vector2D{velocity[0] * time, velocity[1] * time},
      bearing, velocity};
}

SwervePose SwervePose::operator+(const SwervePose& other) const {
  return SwervePose{position + other.position, bearing + other.bearing,
      velocity + other.velocity};
}

SwervePose SwervePose::operator-(const SwervePose& other) const {
  return SwervePose{position - other.position, bearing - other.bearing,
      velocity - other.velocity};
}

}  // namespace frc846::robot::swerve::odometry
