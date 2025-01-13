#pragma once

#include <time.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/velocity.h>

#include "frc846/math/calculator.h"
#include "frc846/math/constants.h"
#include "frc846/math/vectors.h"

namespace y2025 {

/*
RobotCGCalculator

This class will be used to calculate the robot's center of gravity, later used
to prevent tipping
*/
class RobotCGCalculator {
public:
  struct SubsystemState {
    units::pound_t weight;
    frc846::math::Vector3D position;
    frc846::math::VectorND<units::feet_per_second, 3> velocity;
  };

  RobotCGCalculator();

  void SetElevatorHeight(units::inch_t height);
  void SetElevatorVelocity(
      frc846::math::VectorND<units::feet_per_second, 3> vel);

  void SetTelescopeHeight(units::inch_t height);
  void SetTelescopeVelocity(
      frc846::math::VectorND<units::feet_per_second, 3> vel);

  void SetBaseHeight(units::inch_t height);
  void SetBaseVelocity(frc846::math::VectorND<units::feet_per_second, 3> vel);

  units::feet_per_second_t CalculateMaxAcceleration(units::inch_t distance_from_wheels);

  /*
  CalculateRobotCG()

  This is the function that calculates the robots center of gravity

  */
  frc846::math::Vector3D CalculateRobotCG() const;

private:
  SubsystemState elevator_state_;
  SubsystemState telescope_state_;
  SubsystemState base_state_;

  SubsystemState elevator_velocity_;
  SubsystemState telescope_velocity_;
  SubsystemState base_velocity_;
};

}