#pragma once

#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "frc846/wpilib/846_units.h"

namespace frc846::control::config {

/*
MotorConstructionParameters.

Contains all parameters necessary to construct a motor controller.

@param CANid: The CAN address of the motor controller.
@param inverted: Whether the motor controller is inverted.
@param brake_mode: Whether the motor controller is in brake mode.
@param motor_current_limit: The current limit maintained onboard the ESC.
@param threshold_time: The time threshold for the current limit specified
earlier.
@param smart_current_limit: The smart current limit for the motor controller.
@param voltage_compensation: The voltage compensation of the motor controller.
@param circuit_resistance: The circuit resistance leading upto motor controller.
@param rotational_inertia: The rotational inertia of the system.
@param on_ctre_bus: Whether the motor controller is on the CTRE bus (CANivore).

@note Smart current limit is only applicable with custom control. Does not work
with position or velocity control onboard the ESC.

*/
struct MotorConstructionParameters {
  unsigned int CANid;

  bool inverted;

  bool brake_mode;

  units::ampere_t motor_current_limit;
  units::second_t threshold_time;
  units::ampere_t smart_current_limit;

  units::volt_t voltage_compensation;

  frc846::wpilib::unit_ohm circuit_resistance;

  frc846::wpilib::unit_kg_m_sq rotational_inertia;

  bool on_ctre_bus = false;
};

}  // namespace frc846::control::config