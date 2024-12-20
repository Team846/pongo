#pragma once

#include <units/angular_velocity.h>
#include <units/mass.h>
#include <units/torque.h>

#include "frc846/wpilib/846_units.h"

namespace frc846::control::calculators {

/*
VelocityPositionEstimator.

A class that provides static methods to estimate the velocity and position of a
motorized system, with knowledge of the torque applied by the motor.
*/
class VelocityPositionEstimator {
  using unit_kg_m_sq = frc846::wpilib::unit_kg_m_sq;

public:
  /*
  predict_velocity()

  @param current_velocity: The current velocity of the system.
  @param torque: The torque applied by the motor.
  @param dt: The time step.
  @param rot_inertia: The rotational inertia of the system.

  @return The predicted velocity of the system at the end of the timestep.
  */
  static units::radians_per_second_t predict_velocity(
      units::radians_per_second_t current_velocity,
      units::newton_meter_t torque, units::second_t dt,
      unit_kg_m_sq rot_inertia);

  /*
  predict_position()

  @param avg_velocity: The average velocity of the system over the timestep.
  @param current_position: The current position of the system.
  @param dt: The time step.

  @return The predicted position of the system at the end of the timestep.
  */
  static units::radian_t predict_position(
      units::radians_per_second_t avg_velocity,
      units::radian_t current_position, units::second_t dt);
};

}  // namespace frc846::control::calculators