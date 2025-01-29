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
@param inverted Make readings inverted to match motor direction.
*/
struct PotentiometerConfigs {
  units::volt_t zero_voltage;
  units::volt_t max_voltage;
  units::turn_t range;
  bool inverted;
};

/*
PotentiometerCalculator

A class that calculates the position of a potentiometer.
*/
class PotentiometerCalculator : public frc846::math::Calculator<units::volt_t,
                                    units::turn_t, PotentiometerConfigs> {
public:
  PotentiometerCalculator();

  /*
  calculate()

  Calculates the current position of the potentiometer.

  @param voltage: Current state of the potentiometer
  @return The position of the potentiometer.
  */
  units::turn_t calculate(units::volt_t current_voltage) override;
};

}  // namespace frc846::robot::calculators
