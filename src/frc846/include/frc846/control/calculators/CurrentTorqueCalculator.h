#pragma once

#include "frc846/control/base/motor_control_base.h"
#include "frc846/control/base/motor_specs.h"
#include "frc846/wpilib/846_units.h"

namespace frc846::control::calculators {

using namespace frc846::control::base;

/*
CurrentTorqueCalculator

This class includes static methods to:
    - predict current draw
    - predict torque
    - convert between current and torque
    - control current
    - control torque
*/
class CurrentTorqueCalculator {
  using unit_ohm = frc846::wpilib::unit_ohm;

public:
  /*
  predict_current_draw()

  @param duty_cycle: The target duty cycle of the motor.
  @param rpm: The current speed of the motor.
  @param v_supply: The motor supply voltage (battery voltage).
  @param circuit_resistance: The resistance of the circuit leading up to the
  motor.
  @param specs: The MotorSpecs of the motor.

  @note rpm is the current speed of the motor, NOT the controlled mechanism.
  */
  static units::ampere_t predict_current_draw(double duty_cycle,
      units::revolutions_per_minute_t rpm, units::volt_t v_supply,
      unit_ohm circuit_resistance, MotorSpecs specs);
  static units::ampere_t predict_current_draw(double duty_cycle,
      units::revolutions_per_minute_t rpm, units::volt_t v_supply,
      unit_ohm circuit_resistance, MotorMonkeyType mmtype) {
    return predict_current_draw(duty_cycle, rpm, v_supply, circuit_resistance,
        MotorSpecificationPresets::get(mmtype));
  }

  /*
  predict_torque()

  @param duty_cycle: The target duty cycle of the motor.
  @param rpm: The current speed of the motor.
  @param v_supply: The motor supply voltage (battery voltage).
  @param circuit_resistance: The resistance of the circuit leading up to the
  motor.
  @param specs: The MotorSpecs of the motor.

  @note rpm is the current speed of the motor, NOT the controlled mechanism.
  */
  static units::newton_meter_t predict_torque(double duty_cycle,
      units::revolutions_per_minute_t rpm, units::volt_t v_supply,
      unit_ohm circuit_resistance, MotorSpecs specs);
  static units::newton_meter_t predict_torque(double duty_cycle,
      units::revolutions_per_minute_t rpm, units::volt_t v_supply,
      unit_ohm circuit_resistance, MotorMonkeyType mmtype) {
    return predict_torque(duty_cycle, rpm, v_supply, circuit_resistance,
        MotorSpecificationPresets::get(mmtype));
  }

  /*
  torque_to_current()

  Converts a torque value to required current draw, given MotorSpecs.
  */
  static units::ampere_t torque_to_current(
      units::newton_meter_t torque, MotorSpecs specs);
  static units::ampere_t torque_to_current(
      units::newton_meter_t torque, MotorMonkeyType mmtype) {
    return torque_to_current(torque, MotorSpecificationPresets::get(mmtype));
  }

  /*
  current_to_torque()

  Converts a current draw to torque output, given MotorSpecs.
  */
  static units::newton_meter_t current_to_torque(
      units::ampere_t current, MotorSpecs specs);
  static units::newton_meter_t current_to_torque(
      units::ampere_t current, MotorMonkeyType mmtype) {
    return current_to_torque(current, MotorSpecificationPresets::get(mmtype));
  }

  /*
  current_control()

  Returns a duty cycle (-1 to 1) such that the motor draws the target current.
  If the target current is greater than possible, it is forced to the maximum
  possible current draw.
  */
  static double current_control(units::ampere_t target_current,
      units::revolutions_per_minute_t rpm, units::volt_t v_supply,
      unit_ohm circuit_resistance, MotorSpecs specs);
  static double current_control(units::ampere_t target_current,
      units::revolutions_per_minute_t rpm, units::volt_t v_supply,
      unit_ohm circuit_resistance, MotorMonkeyType mmtype) {
    return current_control(target_current, rpm, v_supply, circuit_resistance,
        MotorSpecificationPresets::get(mmtype));
  }

  /*
  torque_control()

  Returns a duty cycle (-1 to 1) such that the motor outputs the target
  torque. If the target torque is greater than possible, it is forced to the
  maximum possible torque.
  */
  static double torque_control(units::newton_meter_t target_torque,
      units::revolutions_per_minute_t rpm, units::volt_t v_supply,
      unit_ohm circuit_resistance, MotorSpecs specs);
  static double torque_control(units::newton_meter_t target_torque,
      units::revolutions_per_minute_t rpm, units::volt_t v_supply,
      unit_ohm circuit_resistance, MotorMonkeyType mmtype) {
    return torque_control(target_torque, rpm, v_supply, circuit_resistance,
        MotorSpecificationPresets::get(mmtype));
  }
};

}  // namespace frc846::control::calculators