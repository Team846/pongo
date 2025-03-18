#include "frc846/math/Smoother.h"

namespace frc846::math {

Smoother::Smoother(double K) : K_(K), last_value_(0.0), first_loop_(true) {}

double Smoother::Calculate(double value) {
  if (first_loop_) {
    last_value_ = value;
    first_loop_ = false;
    return last_value_;
  }

  last_value_ = K_ * value + (1 - K_) * last_value_;

  return last_value_;
}

double Smoother::Get() const { return last_value_; }

}  // namespace frc846::math