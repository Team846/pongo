#include "robot_cg_calculator.h"

namespace y2025 {

RobotCGCalculator::RobotCGCalculator() {
  elevator_state_.weight = 20.0_lb;
  telescope_state_.weight = 10.0_lb;
  base_state_.weight = 80.0_lb;

  elevator_state_.position = {0_in, 0_in, 0_in};
  telescope_state_.position = {0_in, 0_in, 0_in};
  base_state_.position = {0_in, 0_in, 0_in};

  elevator_state_.velocity = {0_fps, 0_fps, 0_fps};
  telescope_state_.velocity = {0_fps, 0_fps, 0_fps};
  base_state_.velocity = {0_fps, 0_fps, 0_fps};
}

RobotCGCalculator::SubsystemState RobotCGCalculator::GetElevatorState() const {
  return elevator_state_;
}

RobotCGCalculator::SubsystemState RobotCGCalculator::GetTelescopeState() const {
  return telescope_state_;
}

RobotCGCalculator::SubsystemState RobotCGCalculator::GetBaseState() const {
  return base_state_;
}

void RobotCGCalculator::SetElevatorPosition(frc846::math::Vector3D pos) {
  elevator_state_.position = pos;
}

void RobotCGCalculator::SetTelescopePosition(frc846::math::Vector3D pos) {
  telescope_state_.position = pos;
}

void RobotCGCalculator::SetBasePosition(frc846::math::Vector3D pos) {
  base_state_.position = pos;
}

void RobotCGCalculator::SetElevatorVelocity(
    frc846::math::VectorND<units::feet_per_second, 3> vel) {
  elevator_state_.velocity = vel;
}

void RobotCGCalculator::SetTelescopeVelocity(
    frc846::math::VectorND<units::feet_per_second, 3> vel) {
  telescope_state_.velocity = vel;
}

void RobotCGCalculator::SetBaseVelocity(
    frc846::math::VectorND<units::feet_per_second, 3> vel) {
  base_state_.velocity = vel;
}

frc846::math::VectorND<units::inch, 3>
RobotCGCalculator::CalculateRobotCG() const {
  units::pound_t total_weight =
      elevator_state_.weight + telescope_state_.weight + base_state_.weight;

  double total_weight_scalar = total_weight.value();

  // weighted sum
  frc846::math::VectorND<units::inch, 3> weighted_sum =
      (elevator_state_.position * elevator_state_.weight.value()) +
      (telescope_state_.position * telescope_state_.weight.value()) +
      (base_state_.position * base_state_.weight.value());

  return weighted_sum / total_weight_scalar;
}

}