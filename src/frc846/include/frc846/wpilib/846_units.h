#pragma once

#include <units/conductance.h>
#include <units/torque.h>

namespace frc846::wpilib {

/* Unit of resistance */
typedef units::unit_t<units::inverse<units::conductance::siemens>> unit_ohm;

/* Unit of resistance per unit length */
typedef units::unit_t<units::compound_unit<
    units::inverse<units::meter>, units::inverse<units::conductance::siemens>>>
    unit_ohms_per_meter;

/* Unit for the rotational inertia of a system */
typedef units::unit_t<
    units::compound_unit<units::meter_kilogram, units::kilogram>>
    unit_kg_m_sq;

}  // namespace frc846::wpilib