#pragma once

#include <chrono>

namespace frc846::math {

/*
RampRateLimiter

A class that limits the rate of change of a value.
*/
class RampRateLimiter {
public:
  RampRateLimiter();

  double limit(double value, double rateLimit);

  double findRate(double value);

private:
  double m_lastValue = 0.0;
  std::chrono::milliseconds m_lastTime;
};

}