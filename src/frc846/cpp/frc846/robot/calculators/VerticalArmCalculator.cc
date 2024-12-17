#include "frc846/robot/calculators/VerticalArmCalculator.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/math.h>

namespace frc846::robot::calculators {

VerticalArmCalculator::VerticalArmCalculator(VerticalArmConfigs& configs)
    : configs_(configs) {
  setConstants(configs);
}

double VerticalArmCalculator::calculate(VerticalArmInputs inputs) {
  units::newton_meter_t gravity_torque =
      configs_.arm_mass * configs_.center_of_mass *
      frc846::math::constants::physics::g *
      units::math::cos(inputs.arm_position + configs_.offset_angle);

  units::degree_t position_error =
      inputs.target_arm_position - inputs.arm_position;

  double duty_cycle = inputs.motor_gains.calculate(
      position_error.to<double>(), 0.0, inputs.current_velocity.to<double>(),
      gravity_torque.to<double>());

  return std::max(std::min(duty_cycle, configs_.peak_output_forward),
                  configs_.peak_output_reverse);
}

}  // namespace frc846::robot::calculators