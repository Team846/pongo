#include "frc846/control/calculators/VelocityPositionEstimator.h"

#include <units/angular_acceleration.h>

namespace frc846::control::calculators {

units::radians_per_second_t VelocityPositionEstimator::predict_velocity(
    units::radians_per_second_t current_velocity, units::newton_meter_t torque,
    units::second_t dt, unit_kg_m_sq rot_inertia) {
  units::radians_per_second_squared_t acceleration =
      1_rad * torque / rot_inertia;
  return current_velocity + acceleration * dt;
}

units::radian_t VelocityPositionEstimator::predict_position(
    units::radians_per_second_t avg_velocity, units::radian_t current_position,
    units::second_t dt) {
  return current_position + avg_velocity * dt;
}

}  // namespace frc846::control::calculators