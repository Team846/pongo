#pragma once

namespace frc846::control::base {

/*
MotorMonkeyType (enum)

Contains all motor and speed controller combinations compatible with the
frc846 architecture.
*/
enum MotorMonkeyType {
  TALON_FX_KRAKENX60,
  TALON_FX_KRAKENX44,
  SPARK_FLEX_VORTEX,
  SPARK_MAX_VORTEX,
  SPARK_MAX_NEO,
  SPARK_MAX_NEO550,
  TALON_FX_KRAKENX60_SIM,
  TALON_FX_KRAKENX44_SIM,
  SPARK_FLEX_VORTEX_SIM,
  SPARK_MAX_VORTEX_SIM,
  SPARK_MAX_NEO_SIM,
  SPARK_MAX_NEO550_SIM,
};

/*
MotorMonkeyTypeHelper

Provides static methods to help determine the type of motor controller.
*/
class MotorMonkeyTypeHelper {
 public:
  static bool is_simulated_motor(MotorMonkeyType mmtype) {
    switch (mmtype) {
      case TALON_FX_KRAKENX60_SIM:
      case TALON_FX_KRAKENX44_SIM:
      case SPARK_FLEX_VORTEX_SIM:
      case SPARK_MAX_VORTEX_SIM:
      case SPARK_MAX_NEO_SIM:
      case SPARK_MAX_NEO550_SIM:
        return true;
      default:
        return false;
    }
  }

  static bool is_talon_fx(MotorMonkeyType mmtype) {
    switch (mmtype) {
      case TALON_FX_KRAKENX60:
      case TALON_FX_KRAKENX44:
      case TALON_FX_KRAKENX60_SIM:
      case TALON_FX_KRAKENX44_SIM:
        return true;
      default:
        return false;
    }
  }

  static bool is_spark_max(MotorMonkeyType mmtype) {
    switch (mmtype) {
      case SPARK_MAX_NEO:
      case SPARK_MAX_NEO550:
      case SPARK_MAX_VORTEX:
      case SPARK_MAX_NEO_SIM:
      case SPARK_MAX_NEO550_SIM:
      case SPARK_MAX_VORTEX_SIM:
        return true;
      default:
        return false;
    }
  }

  static bool is_spark_flex(MotorMonkeyType mmtype) {
    switch (mmtype) {
      case SPARK_FLEX_VORTEX:
      case SPARK_FLEX_VORTEX_SIM:
        return true;
      default:
        return false;
    }
  }
};

}  // namespace frc846::control::base