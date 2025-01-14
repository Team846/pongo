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

/*
AntiTippingCalculator

Class that provides static methods to aid in preventing the robot from tipping.
*/
class AntiTippingCalculator {
public:
  /*
  SetElevatorHeight()

  Sets the position of the end effector from the ground. Location of the CG
  of the elevator is heuristically determined.
  */
  static void SetElevatorHeight(units::inch_t height);

  /*
  SetElevatorHeight()

  Sets the position of the end effector from the ground. Location of the CG
  of the elevator is heuristically determined.
  */
  static void SetTelescopeHeight(units::inch_t height);

  /*
  CalculateRobotCG()

  Finds the location of the center of gravity of the robot.
  */
  static frc846::math::Vector3D CalculateRobotCG();

  /*
  LimitAcceleration()

  Limits the acceleration of the robot to prevent tipping.
  */
  static frc846::math::VectorND<units::feet_per_second_squared, 2>
  LimitAcceleration(
      frc846::math::VectorND<units::feet_per_second_squared, 2> accel,
      units::degree_t bearing);

private:
  static frc846::math::Vector3D elev_cg_position_;
  static frc846::math::Vector3D tele_cg_position_;
};