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

  SubsystemState GetElevatorState() const;
  SubsystemState GetTelescopeState() const;
  SubsystemState GetBaseState() const;

  void SetElevatorPosition(frc846::math::Vector3D pos);
  void SetElevatorVelocity(
      frc846::math::VectorND<units::feet_per_second, 3> vel);

  void SetTelescopePosition(frc846::math::Vector3D pos);
  void SetTelescopeVelocity(
      frc846::math::VectorND<units::feet_per_second, 3> vel);

  void SetBasePosition(frc846::math::Vector3D pos);
  void SetBaseVelocity(frc846::math::VectorND<units::feet_per_second, 3> vel);

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