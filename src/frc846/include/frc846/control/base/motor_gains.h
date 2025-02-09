#pragma once

namespace frc846::control::base {

/*
MotorGains

Represents the FPID gains for a motor controller.
*/
struct MotorGains {
  double kP;
  double kI;
  double kD;
  double kFF;

  /*
  calculate()

  @param error: The error.
  @param integral: The accumulated error.
  @param derivative: The rate of change of error.
  @param feedforward_multiplier: The feedforward multiplier. This may vary as
  function of the system state, for example, in a vertical arm.

  @return The calculated output.
  */
  double calculate(double error, double integral, double derivative,
      double feedforward_multiplier) const {
    return 0.0 /*kP * error + kI * integral + kD * derivative +
           kFF * feedforward_multiplier*/
        ;
  }
};

}  // namespace frc846::control::base