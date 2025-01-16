#include "frc846/robot/calculators/VerticalArmCalculator.h"

#include <units/math.h>

namespace frc846::robot::calculators {

VerticalArmCalculator::VerticalArmCalculator() {}

double VerticalArmCalculator::calculate(VerticalArmInputs inputs) {
  units::newton_meter_t gravity_torque =
      constants_.arm_mass * constants_.center_of_mass *
      frc846::math::constants::physics::g *
      units::math::cos(inputs.arm_position + constants_.offset_angle);

  units::degree_t position_error =
      inputs.target_arm_position - inputs.arm_position;

  double duty_cycle = inputs.motor_gains.calculate(position_error.to<double>(),
      0.0, inputs.current_velocity.to<double>(), gravity_torque.to<double>());

  return std::clamp(duty_cycle, constants_.peak_output_reverse,
      constants_.peak_output_forward);
}

}  // namespace frc846::robot::calculators