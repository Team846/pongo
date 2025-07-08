#include "frc846/control/calculators/VelocityPositionEstimator.h"

#include <units/angular_acceleration.h>
#include <units/math.h>

#include <iostream>

#include "frc846/math/constants.h"

namespace frc846::control::calculators {

units::radians_per_second_t VelocityPositionEstimator::predict_velocity(
    units::radians_per_second_t current_velocity, double target_dc,
    units::second_t dt, units::ampere_t I_lim, units::newton_meter_t load,
    units::newton_meter_t friction_mag, unit_kg_m_sq rot_inertia,
    frc846::control::base::MotorSpecs specs,
    frc846::wpilib::unit_ohm circuit_res, bool brake_mode) {
  double DC_delta_limit = (I_lim / specs.stall_current).to<double>();
  double current_vel_percentage =
      (current_velocity / specs.free_speed).to<double>();
  if (target_dc > DC_delta_limit + current_vel_percentage) {
    target_dc = DC_delta_limit + current_vel_percentage;
  } else if (target_dc < -DC_delta_limit + current_vel_percentage) {
    target_dc = -DC_delta_limit + current_vel_percentage;
  }

  if (std::abs(target_dc) < 0.03 &&
      units::math::abs(specs.free_speed * target_dc) <
          units::math::abs(current_velocity) &&
      !brake_mode) {
    bool og_vel_positive = current_velocity > 0_rad_per_s;
    current_velocity += (og_vel_positive ? -friction_mag : friction_mag) /
                        rot_inertia * dt * 1_rad;
    if ((current_velocity < 0_rad_per_s && og_vel_positive) ||
        (current_velocity > 0_rad_per_s && !og_vel_positive))
      current_velocity = 0_rad_per_s;
    return current_velocity;
  }

  frc846::wpilib::unit_ohm winding_res = 12_V / specs.stall_current;
  units::newton_meter_t effective_torque =
      specs.stall_torque * winding_res / (winding_res + circuit_res);

  units::newton_meter_t pred_torque_output =
      (target_dc - current_velocity / specs.free_speed) * effective_torque;
  if (current_velocity > 0.5_rad_per_s) {
    load += friction_mag;
  } else if (current_velocity < -0.5_rad_per_s) {
    load -= friction_mag;
  } else if (pred_torque_output - load > 0_Nm) {
    load += friction_mag;
    load = units::math::min(load, pred_torque_output);
  } else {
    load -= friction_mag;
    if (pred_torque_output - load > 0_Nm) load = pred_torque_output;
  }

  units::radians_per_second_t w_conv =
      specs.free_speed * (target_dc - load / effective_torque);
  auto conv_rate =
      effective_torque /
      (rot_inertia * specs.free_speed.convert<units::radians_per_second>());

  return w_conv + (current_velocity - w_conv) *
                      std::pow(frc846::math::constants::e,
                          -conv_rate.to<double>() * dt.to<double>());
}

units::radian_t VelocityPositionEstimator::predict_position(
    units::radians_per_second_t avg_velocity, units::radian_t current_position,
    units::second_t dt) {
  return current_position + avg_velocity * dt;
}

}  // namespace frc846::control::calculators