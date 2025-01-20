#include "frc846/math/RampRateLimiter.h"

namespace frc846::math {

RampRateLimiter::RampRateLimiter() {
  m_lastTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
}

double RampRateLimiter::limit(double value, double rateLimit) {
  auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  auto elapsedTime = currentTime - m_lastTime;
  m_lastTime = currentTime;

  double maxChange = std::abs(rateLimit * elapsedTime.count() / 1000.0);
  double change = value - m_lastValue;

  if (change > maxChange) {
    value = m_lastValue + maxChange;
  } else if (change < -maxChange) {
    value = m_lastValue - maxChange;
  }

  m_lastValue = value;
  return value;
}

}  // namespace frc846::math