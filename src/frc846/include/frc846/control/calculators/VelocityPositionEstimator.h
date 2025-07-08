#pragma once

#include <units/angular_velocity.h>
#include <units/mass.h>
#include <units/torque.h>

#include "frc846/control/base/motor_specs.h"
#include "frc846/wpilib/units.h"

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

  - Note: if the motor velocity crosses zero, and effects of friction are
  included, the model will be (slightly) incorrect. This effect may be ignored.

  @param current_velocity: The current velocity of the system.
  @param target_dc: The target duty cycle.
  @param dt: The "hidden time".
  @param I_lim: The current limit.
  @param load: The load on the motor, not including friction.
  @param friction_mag: The magnitude of the friction torque.
  @param rot_inertia: The rotational inertia of the system.
  @param specs: The motor specifications object.
  @param brake_mode: Is brake mode enabled.

  @return The predicted velocity of the system at the end of the timestep.
  */
  static units::radians_per_second_t predict_velocity(
      units::radians_per_second_t current_velocity, double target_dc,
      units::second_t dt, units::ampere_t I_lim, units::newton_meter_t load,
      units::newton_meter_t friction_mag, unit_kg_m_sq rot_inertia,
      frc846::control::base::MotorSpecs specs,
      frc846::wpilib::unit_ohm circuit_res, bool brake_mode = true);

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