#pragma once

namespace frc846::math {

template <typename I, typename O, typename C>
class Calculator {
 public:
  void setConstants(C constants) { constants_ = constants; }

  virtual O calculate(I input) = 0;

 protected:
  C constants_;
};

template <typename I, typename O, typename C>
class IterativeCalculator : public Calculator<I, O, C> {
 protected:
  virtual O calculateIteration(I input, O prev_output) = 0;

 public:
  O calculate(I input) override final {
    O output = O{};
    for (int i = 0; i < max_iterations; i++) {
      output = calculateIteration(input, output);
    }
    return output;
  }

  void setMaxIterations(int max_iterations) {
    this->max_iterations = max_iterations;
  }

 protected:
  int max_iterations = 7;
};

}  // namespace frc846::math