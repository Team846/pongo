#include "frc846/math/Differentiator.h"

namespace frc846::math {

Differentiator::Differentiator()
    : last_value_(0.0), last_timestamp_(0.0), rate_(0.0) {}

void Differentiator::Reset() {
  last_value_ = 0.0;
  last_timestamp_ = 0.0;
  rate_ = 0.0;
  first_loop_ = true;
}

double Differentiator::Calculate(double value) {
  double current_timestamp =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count() /
      1000.0;

  if (first_loop_) {
    last_value_ = value;
    last_timestamp_ = current_timestamp;
    first_loop_ = false;
    return 0.0;
  }

  if (last_timestamp_ == 0.0) {
    last_value_ = value;
    last_timestamp_ = current_timestamp;
    return 0.0;
  }

  double dt = current_timestamp - last_timestamp_;

  if (dt <= 0.0) { return rate_; }

  rate_ = (value - last_value_) / dt;

  last_value_ = value;
  last_timestamp_ = current_timestamp;

  return rate_;
}

double Differentiator::GetRate() const { return rate_; }

}  // namespace frc846::math