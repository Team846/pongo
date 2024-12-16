#pragma once

#include "frc846/robot/GenericSubsystem.h"
#include "frc846/math/constants.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include "frc846/control/base/motor_control_base.h"
#include "frc846/control/base/motor_gains.h"
#include "frc846/control/base/motor_specs.h"
#include "frc846/control/calculators/CurrentTorqueCalculator.h"
#include "frc846/wpilib/846_units.h"

namespace frc846::robot {

// TODO: create limits class
struct VerticalArmConfigs {
  units::kilogram_t arm_mass;
  units::inch_t center_of_mass;
  units::degree_t max_pos;
  units::degree_t min_pos;
  double peak_output_forward = 1.0;
  double peak_output_reverse = -1.0;
};

template <class Readings, class Target>
class GenericVerticalArmSubsystem : public frc846::robot::GenericSubsystem<Readings, Target> {
 public:
  GenericVerticalArmSubsystem(VerticalArmConfigs& configs,
                             frc846::control::base::MotorMonkeyType& mmtype,
                             frc846::control::config::MotorConstructionParameters& constr_params)
     : configs_(configs),
       mmtype_(mmtype),
       constr_params_(constr_params) {}

  double CalculateDutyCycle(units::degree_t arm_position, units::degree_t target_arm_position, units::degrees_per_second_t current_velocity) {
    units::newton_meter_t gravity_torque = configs_.arm_mass_ * configs_.center_of_mass_ * frc846::math::physics.g
                                           * std::abs(units::math::cos(arm_position).to<double>());
    units::newton_meter_t acceleration_torque = constr_params_.rotational_inertia * (current_velocity - previous_velocity) / 20.0_ms;
    units::degree_t position_error = target_arm_position - arm_position;
    previous_velocity = current_velocity;
    // torque feedforward
    units::newton_meter_t total_torque = gravity_torque + acceleration_torque;
    double duty_cycle = frc846::control::base::MotorGains::calculate(position_error, 0, current_velocity.to<double(), total_torque.to<double>());
    return std::max(std::min(duty_cycle, configs.peak_output_forward), configs.peak_output_reverse);
  }

 private:
  VerticalArmConfigs& configs_;
  frc846::control::base::MotorMonkeyType& mmtype_;
  frc846::control::config::MotorConstructionParameters& constr_params_;
  frc846::control::HardLimitsConfigHelper<units::degree_t>& hard_limits_;
  units::degrees_per_second_t previous_velocity 0.0_deg_per_s;
};

} // namespace frc846::robot