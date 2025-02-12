#pragma once

#include <chrono>

namespace frc846::math {

/*
Smoother

Exponentially smooths a given value over time
*/

class Smoother {
public:
  Smoother(double K);

  double Calculate(double value);

  double Get() const;

private:
  double K_;
  double last_value_;

  bool first_loop_;
};
}  // namespace frc846::math