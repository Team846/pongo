#pragma once

#include <units/current.h>
#include <units/voltage.h>

#include "frc846/control/base/motor_gains.h"
#include "frc846/control/config/construction_params.h"

#define REGISTER_MOTOR_CONFIG(subsystem_path, inverted, brake_mode,    \
    current_limit, smart_current_limit, voltage_compensation)          \
  RegisterPreference(#subsystem_path "/inverted", inverted);           \
  RegisterPreference(#subsystem_path "/brake_mode", brake_mode);       \
  RegisterPreference(#subsystem_path "/current_limit", current_limit); \
  RegisterPreference(                                                  \
      #subsystem_path "/smart_current_limit", smart_current_limit);    \
  RegisterPreference(                                                  \
      #subsystem_path "/voltage_compensation", voltage_compensation)

#define GET_MOTOR_CONFIG(                                           \
    subsystem_path, can_id, circuit_resistance, rotational_inertia) \
  {can_id, GetPreferenceValue_bool(#subsystem_path "/inverted"),    \
      GetPreferenceValue_bool(#subsystem_path "/brake_mode"),       \
      GetPreferenceValue_unit_type<units::ampere_t>(                \
          #subsystem_path "/current_limit"),                        \
      GetPreferenceValue_unit_type<units::ampere_t>(                \
          #subsystem_path "/smart_current_limit"),                  \
      GetPreferenceValue_unit_type<units::volt_t>(                  \
          #subsystem_path "/voltage_compensation"),                 \
      circuit_resistance, rotational_inertia}

#define REGISTER_PIDF_CONFIG(subsystem_path, p, i, d, f) \
  RegisterPreference(#subsystem_path "/kP", p);          \
  RegisterPreference(#subsystem_path "/kI", i);          \
  RegisterPreference(#subsystem_path "/kD", d);          \
  RegisterPreference(#subsystem_path "/kF", f)

#define GET_PIDF_GAINS(subsystem_path)                   \
  {GetPreferenceValue_double(#subsystem_path "/_kP"),    \
      GetPreferenceValue_double(#subsystem_path "/_kI"), \
      GetPreferenceValue_double(#subsystem_path "/_kD"), \
      GetPreferenceValue_double(#subsystem_path "/_kF")}

#define REGISTER_SOFTLIMIT_CONFIG(subsystem_path, use_limits, reduce_max_dc) \
  RegisterPreference(#subsystem_path "/using_limits", use_limits);           \
  RegisterPreference(#subsystem_path "/reduce_max_dc", reduce_max_dc)
