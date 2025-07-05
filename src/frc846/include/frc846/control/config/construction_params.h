#pragma once

#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "frc846/wpilib/units.h"

namespace frc846::control::config {

/*
MotorConstructionParameters.

Contains all parameters necessary to construct a motor controller.

@param can_id: The CAN address of the motor controller.
@param inverted: Whether the motor controller is inverted.
@param brake_mode: Whether the motor controller is in brake mode.
@param motor_current_limit: The current limit maintained onboard the ESC.
@param smart_current_limit: The smart current limit for the motor controller.
@param voltage_compensation: The voltage compensation of the motor controller.
@param circuit_resistance: The circuit resistance leading upto motor controller.
@param rotational_inertia: The rotational inertia of the system.
@param friction: The system friction, as a percentage of the motor stall torque.
@param bus: Use only if the motor controller is on the CTRE bus (CANivore).
@param max_wait_time: The maximum time before a control message times out.

@note Smart current limit is only applicable with custom control. Does not work
with position or velocity control onboard the ESC.

*/
struct MotorConstructionParameters {
  int can_id;

  bool inverted;

  bool brake_mode;

  units::ampere_t motor_current_limit;
  units::ampere_t smart_current_limit;

  units::volt_t voltage_compensation;

  frc846::wpilib::unit_ohm circuit_resistance;

  frc846::wpilib::unit_kg_m_sq rotational_inertia;

  double friction = 0.04;

  std::string_view bus = "";

  units::millisecond_t max_wait_time = 20_ms;
};

}  // namespace frc846::control::config