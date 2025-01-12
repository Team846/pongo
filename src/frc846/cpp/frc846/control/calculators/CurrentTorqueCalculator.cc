#include "frc846/control/calculators/CurrentTorqueCalculator.h"

#include <units/math.h>

#include <algorithm>

namespace frc846::control::calculators {

units::ampere_t CurrentTorqueCalculator::predict_current_draw(double duty_cycle,
    units::revolutions_per_minute_t rpm, units::volt_t v_supply,
    unit_ohm circuit_resistance, MotorSpecs specs) {
  double pct_speed = rpm / specs.free_speed;

  unit_ohm winding_resistance =
      12_V / (specs.stall_current - specs.free_current);
  unit_ohm total_resistance = winding_resistance + circuit_resistance;

  units::volt_t back_emf = pct_speed * v_supply;
  units::volt_t voltage_difference = duty_cycle * v_supply - back_emf;

  units::ampere_t current_draw = voltage_difference / total_resistance;

  return current_draw;
}

units::newton_meter_t CurrentTorqueCalculator::predict_torque(double duty_cycle,
    units::revolutions_per_minute_t rpm, units::volt_t v_supply,
    unit_ohm circuit_resistance, MotorSpecs specs) {
  units::ampere_t current_draw = predict_current_draw(
      duty_cycle, rpm, v_supply, circuit_resistance, specs);

  return current_to_torque(current_draw, specs);
}

double CurrentTorqueCalculator::scale_current_draw(double scale_factor,
    double duty_cycle, units::revolutions_per_minute_t rpm,
    units::volt_t v_supply, MotorMonkeyType mmtype) {
  MotorSpecs specs = MotorSpecificationPresets::get(mmtype);
  double pct_speed = rpm / specs.free_speed;
  unit_ohm winding_resistance =
      12_V / (specs.stall_current - specs.free_current);
  units::volt_t back_emf = pct_speed * v_supply;
  units::volt_t voltage_difference = duty_cycle * v_supply - back_emf;
  units::volt_t output = voltage_difference * scale_factor + back_emf;
  return output / v_supply;
}

double CurrentTorqueCalculator::limit_current_draw(double duty_cycle,
    units::ampere_t current_limit, units::revolutions_per_minute_t rpm,
    units::volt_t v_supply, unit_ohm circuit_resistance, MotorSpecs specs) {
  units::ampere_t current_draw = units::math::abs(predict_current_draw(
      duty_cycle, rpm, v_supply, circuit_resistance, specs));
  if (current_draw > current_limit) {
    return current_control(
        current_limit, rpm, v_supply, circuit_resistance, specs);
  }
  return duty_cycle;
}

units::ampere_t CurrentTorqueCalculator::torque_to_current(
    units::newton_meter_t torque, MotorSpecs specs) {
  return (torque / specs.stall_torque) * specs.stall_current;
}
units::newton_meter_t CurrentTorqueCalculator::current_to_torque(
    units::ampere_t current, MotorSpecs specs) {
  return (current / specs.stall_current) * specs.stall_torque;
}

double CurrentTorqueCalculator::current_control(units::ampere_t target_current,
    units::revolutions_per_minute_t rpm, units::volt_t v_supply,
    unit_ohm circuit_resistance, MotorSpecs specs) {
  double pct_speed = rpm / specs.free_speed;

  unit_ohm winding_resistance =
      12_V / (specs.stall_current - specs.free_current);
  unit_ohm total_resistance = winding_resistance + circuit_resistance;

  double duty_cycle_added = (target_current / specs.stall_current) *
                            (total_resistance / winding_resistance);

  double DC_target = std::clamp(pct_speed + duty_cycle_added, -1.0, 1.0);

  return DC_target;
}

double CurrentTorqueCalculator::torque_control(
    units::newton_meter_t target_torque, units::revolutions_per_minute_t rpm,
    units::volt_t v_supply, unit_ohm circuit_resistance, MotorSpecs specs) {
  return current_control(torque_to_current(target_torque, specs), rpm, v_supply,
      circuit_resistance, specs);
}

}  // namespace frc846::control::calculators