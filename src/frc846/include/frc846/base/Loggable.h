#pragma once

#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <units/base.h>

#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <type_traits>
#include <unordered_set>
#include <variant>

#include "frc846/base/FunkyLogSystem.h"

namespace frc846::base {

/*
Loggable

A class that provides logging functionality to any class that inherits from it.
*/
class Loggable {
public:
  Loggable(const Loggable& parent_, std::string name)
      : name_{parent_.name() + "/" + name}, logger(name_) {
    Loggable(parent_.name() + "/" + name);
  }

  Loggable(std::string name) : name_{name}, logger(name_) {}

  std::string name() const { return name_; }

  template <typename... T>
  void Log(fmt::format_string<T...> fmt, T&&... args) const {
    logger.Log(fmt, std::forward<T>(args)...);
  }
  template <typename... T>
  void Warn(fmt::format_string<T...> fmt, T&&... args) {
    logger.Warn(fmt, std::forward<T>(args)...);
    warn_count_++;
  }
  template <typename... T>
  void Error(fmt::format_string<T...> fmt, T&&... args) {
    logger.Error(fmt, std::forward<T>(args)...);
    error_count_++;
  }

  // Puts a double entry on the smart dashboard.
  void Graph(std::string key, double value) const;

  // Puts an integer entry on the smart dashboard.
  void Graph(std::string key, int value) const;

  // Puts a boolean entry on the smart dashboard.
  void Graph(std::string key, bool value) const;

  // Puts a string entry on the smart dashboard.
  void Graph(std::string key, std::string value) const;

  // Puts a unit entry on the smart dashboard.
  template <typename U> void Graph(std::string key, U value) const {
    static_assert(units::traits::is_unit_t<U>(), "must be a unit type");

    std::string modkey = fmt::format(
        "{} ({})", key, units::abbreviation(units::make_unit<U>(0)));
    Graph(modkey, value.template to<double>());
  }

  // Creates a unit-type preference.
  template <typename U> void RegisterPreference(std::string key, U fallback) {
    std::string modkey = fmt::format(
        "{} ({})", key, units::abbreviation(units::make_unit<U>(0)));
    RegisterPreference(modkey, fallback.template to<double>());
  }

  // Creates a double preference.
  void RegisterPreference(std::string key, double fallback);

  // Creates a boolean preference.
  void RegisterPreference(std::string key, bool fallback);

  // Creates an integer preference.
  void RegisterPreference(std::string key, int fallback);

  // Creates a string preference.
  void RegisterPreference(std::string key, std::string fallback);

  // Returns the value of the preference for a unit-type.
  template <typename U> U GetPreferenceValue_unit_type(std::string key) {
    std::string modkey = fmt::format(
        "{} ({})", key, units::abbreviation(units::make_unit<U>(0)));
    return units::make_unit<U>(GetPreferenceValue_double(modkey));
  }

  // Returns the value of the preference for a double.
  double GetPreferenceValue_double(std::string key);

  // Returns the value of the preference for a boolean.
  bool GetPreferenceValue_bool(std::string key);

  // Returns the value of the preference for an integer.
  int GetPreferenceValue_int(std::string key);

  // Returns the value of the preference for a string.
  std::string GetPreferenceValue_string(std::string key);

  // Sets but does NOT initialize the value of the preference for a unit-type.
  template <typename U> void SetPreferenceValue(std::string key, U value) {
    static_assert(units::traits::is_unit_t<U>(), "must be a unit type");

    std::string fullkey = fmt::format("{} ({})", name_ + "/" + key,
        units::abbreviation(units::make_unit<U>(0)));
    SetPreferenceValue(fullkey, value.template to<double>());
  }

  // Sets but does NOT initialize the value of the preference for a double.
  void SetPreferenceValue(std::string key, double value);

  // Sets but does NOT initialize the value of the preference for a boolean.
  void SetPreferenceValue(std::string key, bool value);

  // Sets but does NOT initialize the value of the preference for an integer.
  void SetPreferenceValue(std::string key, int value);

  // Sets but does NOT initialize the value of the preference for a string.
  void SetPreferenceValue(std::string key, std::string value);

  static unsigned int GetWarnCount();
  static unsigned int GetErrorCount();

  static std::string Join(std::string p, std::string n);

  static std::vector<std::string> ListKeysToPrune();

  static void PrunePreferences(const Loggable* caller);

private:
  bool CheckPreferenceKeyExists(std::string key);

  const std::string name_;

  static std::unordered_set<std::string> used_preferences_;

  static unsigned int warn_count_;
  static unsigned int error_count_;

  frc846::base::FunkyLogger logger;
};

}  // namespace frc846::base
