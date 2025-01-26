#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <units/voltage.h>

#include "frc846/math/calculator.h"

namespace frc846::robot::calculators {

/*
Potentiometer Configurations.

Contains all parameters necessary to construct a new Potentiometer Calculator.

@param zero_voltage The voltage reading which corresponds to the zero position
of the potentiometer.
@param max_voltage The maximum voltage it can output.
@param range The max number of turns.
@param gear_reduction The gear reduction between the potentiometer and the
mechanism.
@param inverted Make readings inverted to match motor direction.
*/
struct PotentiometerConfigs {
  units::volt_t zero_voltage;
  units::volt_t max_voltage;
  units::turn_t range;
  units::unit_t<units::compound_unit<units::inch, units::inverse<units::turn>>>
      gear_reduction;
  bool inverted;
};

/*
Potentiometer Inputs.

Contains all parameters necessary to calculate the position which the

@param current_voltage The voltage reading from the potentiometer.
*/
struct PotentiometerInputs {
  units::volt_t current_voltage;
};

/*
PotentiometerCalculator

A class that calculates the position of a mechanism from potentiometer readings.
*/
class PotentiometerCalculator
    : public frc846::math::Calculator<PotentiometerInputs, units::inch_t,
          PotentiometerConfigs> {
public:
  PotentiometerCalculator();

  /*
  calculate()

  Calculates the current position of the mechanism.

  @param inputs: Current state of the potentiometer
  @return The position of the mechanism.
  */
  units::inch_t calculate(PotentiometerInputs inputs) override;
};

}  // namespace frc846::robot::calculators
