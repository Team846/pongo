#pragma once

#include <units/current.h>
#include <units/voltage.h>

#include "frc846/base/Loggable.h"

inline std::string MakeKey(std::string base, std::string suffix) {
  return std::string(frc846::base::Loggable::Join(base, suffix));
}

// TODO: remove motor configs

#define REGISTER_MOTOR_CONFIG(name, inverted, brake_mode, current_limit, \
    smart_current_limit, voltage_compensation)                           \
  RegisterPreference(MakeKey(name, "inverted"), inverted);               \
  RegisterPreference(MakeKey(name, "brake_mode"), brake_mode);           \
  RegisterPreference(MakeKey(name, "current_limit"), current_limit);     \
  RegisterPreference(                                                    \
      MakeKey(name, "smart_current_limit"), smart_current_limit);        \
  RegisterPreference(                                                    \
      MakeKey(name, "voltage_compensation"), voltage_compensation)

#define GET_MOTOR_CONFIG(name, can_id, circuit_resistance, rotational_inertia) \
  {can_id, GetPreferenceValue_bool(MakeKey(name, "inverted")),                 \
      GetPreferenceValue_bool(MakeKey(name, "brake_mode")),                    \
      GetPreferenceValue_unit_type<units::ampere_t>(                           \
          MakeKey(name, "current_limit")),                                     \
      GetPreferenceValue_unit_type<units::ampere_t>(                           \
          MakeKey(name, "smart_current_limit")),                               \
      GetPreferenceValue_unit_type<units::volt_t>(                             \
          MakeKey(name, "voltage_compensation")),                              \
      circuit_resistance, rotational_inertia}

#define REGISTER_PIDF_CONFIG(name, p, i, d, f) \
  RegisterPreference(MakeKey(name, "_kP"), p); \
  RegisterPreference(MakeKey(name, "_kI"), i); \
  RegisterPreference(MakeKey(name, "_kD"), d); \
  RegisterPreference(MakeKey(name, "_kF"), f)

#define GET_PIDF_GAINS(name)                           \
  {GetPreferenceValue_double(MakeKey(name, "_kP")),    \
      GetPreferenceValue_double(MakeKey(name, "_kI")), \
      GetPreferenceValue_double(MakeKey(name, "_kD")), \
      GetPreferenceValue_double(MakeKey(name, "_kF"))}

#define REGISTER_SOFTLIMIT_CONFIG(                                       \
    use_limits, upper, lower, reduce_upper, reduce_lower, reduce_max_dc) \
  RegisterPreference("limits/use_limits", use_limits);                   \
  RegisterPreference("limits/upper_limit", upper);                       \
  RegisterPreference("limits/lower_limit", lower);                       \
  RegisterPreference("limits/reduce_upper", reduce_upper);               \
  RegisterPreference("limits/reduce_lower", reduce_lower);               \
  RegisterPreference("limits/reduce_max_dc", reduce_max_dc)

#define GET_SOFTLIMITS(typename)                                     \
  {GetPreferenceValue_bool("limits/use_limits"),                     \
      GetPreferenceValue_unit_type<typename>("limits/upper_limit"),  \
      GetPreferenceValue_unit_type<typename>("limits/lower_limit"),  \
      GetPreferenceValue_unit_type<typename>("limits/reduce_upper"), \
      GetPreferenceValue_unit_type<typename>("limits/reduce_lower"), \
      GetPreferenceValue_double("limits/reduce_max_dc")}
