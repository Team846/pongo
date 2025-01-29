#include "frc846/robot/calculators/PotentiometerCalculator.h"

namespace frc846::robot::calculators {

PotentiometerCalculator::PotentiometerCalculator() {}

units::turn_t PotentiometerCalculator::calculate(
    units::volt_t current_voltage) {
  units::volt_t normalized_voltage = current_voltage;
  if (constants_.inverted) {
    normalized_voltage =
        constants_.max_voltage - normalized_voltage + constants_.zero_voltage;
  } else {
    normalized_voltage = normalized_voltage - constants_.zero_voltage;
  }
  units::turn_t potentiometer_turns =
      normalized_voltage / (constants_.max_voltage - constants_.zero_voltage) *
      constants_.range;
  return potentiometer_turns;
}

}  // namespace frc846::robot::calculators