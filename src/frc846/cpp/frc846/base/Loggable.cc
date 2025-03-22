#include "frc846/base/Loggable.h"

#include <iostream>
#include <sstream>

namespace frc846::base {

std::string_view Loggable::Join(std::string p, std::string n) {
  return p + "/" + n;
}

unsigned int Loggable::GetWarnCount() { return warn_count_; }

unsigned int Loggable::GetErrorCount() { return error_count_; }

std::unordered_set<std::string_view> Loggable::used_preferences_{};

unsigned int Loggable::warn_count_ = 0;
unsigned int Loggable::error_count_ = 0;

void Loggable::Graph(std::string key, double value) const {
  frc::SmartDashboard::PutNumber(name_ + "/" + key, value);
}

void Loggable::Graph(std::string key, int value) const {
  frc::SmartDashboard::PutNumber(name_ + "/" + key, value);
}

void Loggable::Graph(std::string key, bool value) const {
  frc::SmartDashboard::PutBoolean(name_ + "/" + key, value);
}

void Loggable::Graph(std::string key, std::string value) const {
  frc::SmartDashboard::PutString(name_ + "/" + key, value);
}

void Loggable::RegisterPreference(std::string key, double fallback) {
  std::string fullkey = name_ + "/" + key;
  frc::Preferences::InitDouble(fullkey, fallback);
  if (frc::Preferences::GetDouble(fullkey, 0.0) != fallback) {
    Log("Preference [{}] does not match fallback", fullkey);
  }
  used_preferences_.insert(fullkey);
}

void Loggable::RegisterPreference(std::string key, bool fallback) {
  std::string fullkey = name_ + "/" + key;
  frc::Preferences::InitBoolean(fullkey, fallback);
  if (frc::Preferences::GetBoolean(fullkey, false) != fallback) {
    Log("Preference [{}] does not match fallback", fullkey);
  }
  used_preferences_.insert(fullkey);
}

void Loggable::RegisterPreference(std::string key, int fallback) {
  std::string fullkey = name_ + "/" + key;
  frc::Preferences::InitInt(fullkey, fallback);
  if (frc::Preferences::GetInt(fullkey, 0) != fallback) {
    Log("Preference [{}] does not match fallback", fullkey);
  }
  used_preferences_.insert(fullkey);
}

void Loggable::RegisterPreference(std::string key, std::string fallback) {
  std::string fullkey = name_ + "/" + key;
  frc::Preferences::InitString(fullkey, fallback);
  if (frc::Preferences::GetString(fullkey, "") != fallback) {
    Log("Preference [{}] does not match fallback", fullkey);
  }
  used_preferences_.insert(fullkey);
}

bool Loggable::CheckPreferenceKeyExists(std::string key) {
  if (!frc::Preferences::ContainsKey(name_ + "/" + key)) {
    Warn("Attempted to access uninitialized preference {}", key);
    return false;
  }
  return true;
}

double Loggable::GetPreferenceValue_double(std::string key) {
  if (!CheckPreferenceKeyExists(key)) { return 0; }
  return frc::Preferences::GetDouble(name_ + "/" + key);
}

bool Loggable::GetPreferenceValue_bool(std::string key) {
  if (!CheckPreferenceKeyExists(key)) { return false; }
  return frc::Preferences::GetBoolean(name_ + "/" + key);
}

int Loggable::GetPreferenceValue_int(std::string key) {
  if (!CheckPreferenceKeyExists(key)) { return 0; }
  return frc::Preferences::GetInt(name_ + "/" + key);
}

std::string Loggable::GetPreferenceValue_string(std::string key) {
  if (!CheckPreferenceKeyExists(key)) { return ""; }
  return frc::Preferences::GetString(name_ + "/" + key);
}

void Loggable::SetPreferenceValue(std::string key, double value) {
  frc::Preferences::SetDouble(name_ + "/" + key, value);
}

void Loggable::SetPreferenceValue(std::string key, bool value) {
  frc::Preferences::SetBoolean(name_ + "/" + key, value);
}

void Loggable::SetPreferenceValue(std::string key, int value) {
  frc::Preferences::SetInt(name_ + "/" + key, value);
}

void Loggable::SetPreferenceValue(std::string key, std::string value) {
  frc::Preferences::SetString(name_ + "/" + key, value);
}

std::vector<std::string> Loggable::ListKeysToPrune() {
  std::vector<std::string> keys_to_prune{};
  for (auto& key : frc::Preferences::GetKeys()) {
    if (used_preferences_.find(key) == used_preferences_.end()) {
      keys_to_prune.push_back(key);
    }
  }
  return keys_to_prune;
}

void Loggable::PrunePreferences(const Loggable* caller) {
  for (auto& key : ListKeysToPrune()) {
    caller->Log("Pruning unused preference {}", key);
    frc::Preferences::Remove(key);
  }
}

}  // namespace frc846::base