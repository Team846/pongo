#pragma once

#include <units/conductance.h>
#include <units/torque.h>

namespace frc846::wpilib {

/* Unit of resistance */
typedef units::unit_t<units::inverse<units::conductance::siemens>> unit_ohm;

/* Unit of resistance per unit length */
typedef units::unit_t<units::compound_unit<units::inverse<units::meter>,
    units::inverse<units::conductance::siemens>>>
    unit_ohms_per_meter;

/* Unit for the rotational inertia of a system */
typedef units::unit_t<
    units::compound_unit<units::kilogram, units::squared<units::meter>>>
    unit_kg_m_sq;

/* Defining Literals */
constexpr unit_ohm operator""_ohms(long double);
constexpr unit_ohms_per_meter operator""_ohms_per_meter(long double);
constexpr unit_kg_m_sq operator""_kg_m_sq(long double);

}  // namespace frc846::wpilib

using namespace frc846::wpilib;