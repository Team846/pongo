#include "frc846/robot/calculators/PotentiometerCalculator.h"

namespace frc846::robot::calculators {

PotentiometerCalculator::PotentiometerCalculator() {}

units::inch_t PotentiometerCalculator::calculate(PotentiometerInputs inputs) {
  units::volt_t normalized_voltage = inputs.current_voltage;
  if (constants_.inverted) {
    normalized_voltage =
        constants_.max_voltage - normalized_voltage + constants_.zero_voltage;
  } else {
    normalized_voltage = normalized_voltage - constants_.zero_voltage;
  }
  units::turn_t potentiometer_turns =
      normalized_voltage / (constants_.max_voltage - constants_.zero_voltage) *
      constants_.range;
  units::inch_t actual_position =
      potentiometer_turns * constants_.gear_reduction;
  return actual_position;
}

}  // namespace frc846::robot::calculators