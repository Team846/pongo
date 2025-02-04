#pragma once

#include <units/length.h>

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "frc846/math/vectors.h"

struct robot_constants {
  struct base {
    static constexpr units::inch_t wheelbase_x = 28_in;
    static constexpr units::inch_t wheelbase_y = 28_in;
    static constexpr units::pound_t weight = 75_lb;

    static constexpr units::inch_t height = 6_in;
  };

  struct elevator {
    static constexpr units::pound_t elevator_weight = 20_lb;
    static constexpr units::inch_t min_height_off_base = 38_in;

    static constexpr units::pound_t end_effector_weight = 10_lb;

    static constexpr units::pound_t total_weight =
        elevator_weight + end_effector_weight;

    static constexpr units::inch_t pos_x = -5_in;
    static constexpr units::inch_t pos_y = -7_in;
  };

  struct telescope {
    static constexpr units::pound_t total_weight = 20_lb;

    static constexpr units::inch_t pos_x = 5_in;
    static constexpr units::inch_t pos_y = -7_in;
  };

  struct algae_ss_ {
    static constexpr units::inch_t wire_length_base = 20_in;
    static constexpr frc846::control::calculators::WireGauge wire_gauge_base =
        frc846::control::calculators::twelve_gauge;
    static constexpr unsigned int num_connectors_base = 3U;
    static constexpr frc846::wpilib::unit_ohm wire_resistance_base =
        frc846::control::calculators::CircuitResistanceCalculator::calculate(
            wire_length_base, wire_gauge_base, num_connectors_base);

    static constexpr units::inch_t wire_length = 120_in;
    static constexpr frc846::control::calculators::WireGauge wire_gauge =
        frc846::control::calculators::fourteen_gauge;
    static constexpr unsigned int num_connectors = 5U;

    static constexpr frc846::wpilib::unit_ohm wire_resistance =
        frc846::control::calculators::CircuitResistanceCalculator::calculate(
            wire_length, wire_gauge, num_connectors);
  };

  struct coral_ss_ {
    static constexpr units::inch_t wire_length_base = 20_in;
    static constexpr frc846::control::calculators::WireGauge wire_gauge_base =
        frc846::control::calculators::twelve_gauge;
    static constexpr unsigned int num_connectors_base = 3U;
    static constexpr frc846::wpilib::unit_ohm wire_resistance_base =
        frc846::control::calculators::CircuitResistanceCalculator::calculate(
            wire_length_base, wire_gauge_base, num_connectors_base);

    static constexpr units::inch_t wire_length = 90_in;
    static constexpr frc846::control::calculators::WireGauge wire_gauge =
        frc846::control::calculators::fourteen_gauge;
    static constexpr unsigned int num_connectors = 5U;

    static constexpr frc846::wpilib::unit_ohm wire_resistance =
        frc846::control::calculators::CircuitResistanceCalculator::calculate(
            wire_length, wire_gauge, num_connectors);
  };

  struct climber_ {
    static constexpr units::inch_t wire_length = 20_in;
    static constexpr frc846::control::calculators::WireGauge wire_gauge =
        frc846::control::calculators::twelve_gauge;
    static constexpr unsigned int num_connectors = 3U;

    static constexpr frc846::wpilib::unit_ohm wire_resistance =
        frc846::control::calculators::CircuitResistanceCalculator::calculate(
            wire_length, wire_gauge, num_connectors);
  };

  static constexpr units::pound_t total_weight =
      base::weight + elevator::elevator_weight + elevator::end_effector_weight +
      telescope::total_weight;
};