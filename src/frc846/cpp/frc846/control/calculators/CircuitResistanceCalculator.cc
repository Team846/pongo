#include "frc846/control/calculators/CircuitResistanceCalculator.h"

namespace frc846::control::calculators {

frc846::wpilib::unit_ohm CircuitResistanceCalculator::calculate(
    units::foot_t wire_length, WireGauge gauge, unsigned int num_connectors) {
  frc846::wpilib::unit_ohms_per_meter resistance_per_meter;
  switch (gauge) {
    case twelve_gauge:
      resistance_per_meter = KnownResistances::kTwelveGaugeResistance;
      break;
    case fourteen_gauge:
      resistance_per_meter = KnownResistances::kFourteenGaugeResistance;
      break;
    case sixteen_gauge:
      resistance_per_meter = KnownResistances::kSixteenGaugeResistance;
      break;
    case eighteen_gauge:
      resistance_per_meter = KnownResistances::kEighteenGaugeResistance;
      break;
  }

  frc846::wpilib::unit_ohm total_resistance =
      resistance_per_meter * wire_length + KnownResistances::kPDPResistance +
      KnownResistances::kBatteryResistance +
      KnownResistances::kConnectorResistance * num_connectors;

  return total_resistance;
}

}  // namespace frc846::control::calculators