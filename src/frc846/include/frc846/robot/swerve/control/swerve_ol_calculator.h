#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <array>

#include "frc846/math/calculator.h"
#include "frc846/math/vectors.h"

namespace frc846::robot::swerve::control {

struct SwerveOpenLoopCalculatorConstants {
  units::inch_t wheelbase_horizontal_dim;
  units::inch_t wheelbase_forward_dim;
};

struct SwerveOpenLoopCalculatorInputs {
  frc846::math::VectorND<units::feet_per_second, 2> translation_target;
  units::degrees_per_second_t rotation_target;
  units::degree_t bearing;
  units::feet_per_second_t max_speed;
};

struct SwerveOpenLoopCalculatorOutput {
  std::array<units::feet_per_second_t, 4> drive_outputs;
  std::array<units::degree_t, 4> steer_outputs;
};

/*
SwerveOpenLoopCalculator

Calculates the open-loop control targets for each swerve module, given target
translational and rotational velocities.
*/
class SwerveOpenLoopCalculator
    : public frc846::math::Calculator<SwerveOpenLoopCalculatorInputs,
          SwerveOpenLoopCalculatorOutput, SwerveOpenLoopCalculatorConstants> {
public:
  SwerveOpenLoopCalculatorOutput calculate(
      SwerveOpenLoopCalculatorInputs inputs) override;
};

}  // namespace frc846::robot::swerve::odometry
