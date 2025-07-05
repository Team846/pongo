#pragma once

#include <units/length.h>

#include "frc846/wpilib/units.h"

namespace frc846::control::calculators {

/*
KnownResistances

Contains known resistances for various components, such as:
    - The Battery
    - The PDP
    - Anderson connectors
    - Wires of various thicknesses
*/
struct KnownResistances {
  static constexpr frc846::wpilib::unit_ohm kBatteryResistance{0.02};
  static constexpr frc846::wpilib::unit_ohm kPDPResistance{0.001};

  static constexpr frc846::wpilib::unit_ohm kConnectorResistance{0.0006};

  static constexpr frc846::wpilib::unit_ohms_per_meter kTwelveGaugeResistance{
      0.00521};
  static constexpr frc846::wpilib::unit_ohms_per_meter kFourteenGaugeResistance{
      0.00829};
  static constexpr frc846::wpilib::unit_ohms_per_meter kSixteenGaugeResistance{
      0.0132};
  static constexpr frc846::wpilib::unit_ohms_per_meter kEighteenGaugeResistance{
      0.0209};
};

enum WireGauge { twelve_gauge, fourteen_gauge, sixteen_gauge, eighteen_gauge };

/*
CircuitResistanceCalculator

This class includes static methods that can be used to estimate the resistance
of a motor controller circuit.
*/

class CircuitResistanceCalculator {
  using unit_ohm = frc846::wpilib::unit_ohm;

public:
  /*
  calculate()

  @param wire_length: The length of the wire between the PDP and the motor
  controller.
  @param gauge: The wire gauge between the PDP and the motor controller.
  @param num_connectors: The number of connection points in the circuit.

  @return The estimated resistance of the circuit.
  */
  static constexpr unit_ohm calculate(
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
    default:
      resistance_per_meter = KnownResistances::kEighteenGaugeResistance;
      break;
    }

    frc846::wpilib::unit_ohm total_resistance =
        resistance_per_meter * wire_length + KnownResistances::kPDPResistance +
        KnownResistances::kBatteryResistance +
        KnownResistances::kConnectorResistance * num_connectors;

    return total_resistance;
  }
};

}  // namespace frc846::control::calculators