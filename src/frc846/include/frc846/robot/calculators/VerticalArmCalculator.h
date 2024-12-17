#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/mass.h>

#include "frc846/control/base/motor_gains.h"
#include "frc846/math/calculator.h"
#include "frc846/math/constants.h"
#include "frc846/wpilib/846_units.h"

namespace frc846::robot::calculators {

/*
Vertical Arm Configurations.

Contains all parameters necessary to construct a new Vertical Arm Calculator.

@param arm_mass: The mass of the arm.
@param center_of_mass: The distance from the pivot point to center of mass.
@param offset_angle: The fixed angle offset relative to the horizontal axis.
@param max_pos: The maximum allowed position.
@param min_pos: The minimum allowed position.
@param peak_output_forward: The maximum duty cycle for forward motion [0, 1]
@param peak_output_reverse: The maximum duty cycle for reverse motion [-1, 0]
*/
struct VerticalArmConfigs {
  units::kilogram_t arm_mass;
  units::inch_t center_of_mass;
  units::degree_t offset_angle = 0.0_deg;
  units::degree_t max_pos;
  units::degree_t min_pos;
  double peak_output_forward = 1.0;
  double peak_output_reverse = -1.0;
};

/*
Vertical Arm Inputs.

Contains all parameters necessary to calculate the duty cycle for the motors to
achieve the target position.

@param arm_position: The current position.
@param target_arm_position: The desired position.
@param current_velocity: The current angular velocity.
@param motor_gains: PIDF gains used to control the motor.
*/
struct VerticalArmInputs {
  units::degree_t arm_position;
  units::degree_t target_arm_position;
  units::degrees_per_second_t current_velocity;
  frc846::control::base::MotorGains motor_gains;
};

/*
VerticalArmCalculator

A class that calculates the duty cycle for achiving a target postion using
characteristics of an arm.
*/
class VerticalArmCalculator
    : public frc846::math::Calculator<VerticalArmInputs, double,
                                      VerticalArmConfigs> {
 public:
  VerticalArmCalculator(VerticalArmConfigs& configs);

  /*
  calculate()

  This method takes into account the current arm position, target position,
  velocity, and compensates for gravity, to compute the required duty cycle to
  achieve the desired position.

  @param inputs: Current data of the arm.

  @return A duty cycle
  */
  double calculate(VerticalArmInputs inputs) override;

 private:
  VerticalArmConfigs& configs_;
};

}  // namespace frc846::robot::calculators
