#pragma once

#include <chrono>

namespace frc846::math {

/*
Differentiator class

A class that differentiates a given value with respect to time
*/

class Differentiator {
public:
  Differentiator();

  void Reset();

  double Calculate(double value);

  double GetRate() const;

private:
  double last_value_;
  double last_timestamp_;
  double rate_;
};
}  // namespace frc846::math
