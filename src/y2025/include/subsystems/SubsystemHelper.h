#pragma once

#include "frc846/base/Loggable.h"

#define REGISTER_MOTOR_CONFIG(current_limit, smart_current_limit)   \
  RegisterPreference("motor_configs/current_limit", current_limit); \
  RegisterPreference("motor_configs/smart_current_limit", smart_current_limit)

#define REGISTER_PIDF_CONFIG(p, i, d, f) \
  RegisterPreference("gains/_kP", p);    \
  RegisterPreference("gains/_kI", i);    \
  RegisterPreference("gains/_kD", d);    \
  RegisterPreference("gains/_kF", f)

#define GET_PIDF_GAINS()                      \
  {GetPreferenceValue_double("gains/_kP"),    \
      GetPreferenceValue_double("gains/_kI"), \
      GetPreferenceValue_double("gains/_kD"), \
      GetPreferenceValue_double("gains/_kF")}

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
