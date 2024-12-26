#pragma once

#include <units/angular_velocity.h>
#include <units/conductance.h>
#include <units/current.h>
#include <units/torque.h>
#include <units/voltage.h>

#include "frc846/control/base/motor_control_base.h"

namespace frc846::control::base {

/*
MotorSpecs
  - free_speed: rpm
  - stall_current: amps
  - free_current: amps
  - stall_torque: Nm
  - winding_resistance: ohms

Values can be found on specification sheets for the respective motors.
*/
struct MotorSpecs {
  units::revolutions_per_minute_t free_speed;
  units::ampere_t stall_current;
  units::ampere_t free_current;
  units::newton_meter_t stall_torque;
};

/*
MotorSpecificationPresets

Includes MotorSpecs for:
  - NEO 550 (https://www.revrobotics.com/rev-21-1651/)
  - NEO (https://www.revrobotics.com/rev-21-1650/)
  - NEO Vortex (https://www.revrobotics.com/rev-21-1652/)
  - Kraken X60
(https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance)
  - Kraken X44
(https://docs.wcproducts.com/kraken-x44/kraken-x44-motor/overview-and-features/motor-performance)

*/
struct MotorSpecificationPresets {
  static constexpr MotorSpecs kNeo550 = {
      .free_speed = 11000_rpm,
      .stall_current = 100_A,
      .free_current = 1.4_A,
      .stall_torque = 0.97_Nm,
  };
  static constexpr MotorSpecs kNeo = {
      .free_speed = 5676_rpm,
      .stall_current = 105_A,
      .free_current = 1.8_A,
      .stall_torque = 2.6_Nm,
  };
  static constexpr MotorSpecs kNeoVortex = {
      .free_speed = 6784_rpm,
      .stall_current = 211_A,
      .free_current = 3.6_A,
      .stall_torque = 3.6_Nm,
  };
  static constexpr MotorSpecs kKrakenX60 = {
      .free_speed = 6000_rpm,
      .stall_current = 366_A,
      .free_current = 2_A,
      .stall_torque = 7.09_Nm,
  };
  static constexpr MotorSpecs kKrakenX44 = {
      .free_speed = 7530_rpm,
      .stall_current = 275_A,
      .free_current = 1.4_A,
      .stall_torque = 4.05_Nm,
  };

  static MotorSpecs get(frc846::control::base::MotorMonkeyType type) {
    switch (type) {
    case frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO550:
      return kNeo550;
    case frc846::control::base::MotorMonkeyType::SPARK_MAX_NEO: return kNeo;
    case frc846::control::base::MotorMonkeyType::SPARK_FLEX_VORTEX:
    case frc846::control::base::MotorMonkeyType::SPARK_MAX_VORTEX:
      return kNeoVortex;
    case frc846::control::base::MotorMonkeyType::TALON_FX_KRAKENX60:
      return kKrakenX60;
    case frc846::control::base::MotorMonkeyType::TALON_FX_KRAKENX44:
      return kKrakenX44;
    default: return kNeo;
    }
  }
};

}  // namespace frc846::control::base
