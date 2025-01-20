#pragma once

#include <frc/EigenCore.h>
#include <units/length.h>

#include <array>

#include "frc846/math/collection.h"
#include "frc846/math/filter.h"
#include "frc846/math/vectors.h"
#include "units/base.h"

namespace frc846::robot::swerve::odometry {

class PoseEstimator {
private:
  static constexpr int kModuleCount = 4;

public:
  PoseEstimator(frc846::math::VectorND<units::foot, 2> initial_position,
      frc846::math::VectorND<units::feet_per_second, 2> initial_vel,
      frc846::math::VectorND<units::feet_per_second_squared, 2> initial_accl);

  PoseEstimator() {
    PoseEstimator({0_ft, 0_ft}, {0_fps, 0_fps}, {0_fps_sq, 0_fps_sq});
  };

  frc846::math::VectorND<units::foot, 2> position() {
    return {units::foot_t(state_[0]), units::foot_t(state_[1])};
  }

  void Update(double pVar, double vVar, double aVar);

  void AddAccelerationMeasurement(
      frc846::math::VectorND<units::feet_per_second_squared, 2> accl);

  void AddVisionMeasurement(
      frc846::math::VectorND<units::foot, 2> pos, double variance);

  void AddOdometryMeasurement(frc846::math::VectorND<units::foot, 2> difPos);

  void SetPoint(frc846::math::VectorND<units::foot, 2> point);

  double getVariance();

  void Zero();

private:
  Eigen::Matrix<double, 6, 1> state_;
  frc846::math::LinearKalmanFilter<6> filter;

  Eigen::Matrix<double, 2, 6> Hv;

  Eigen::Matrix<double, 2, 6> Ha;
  Eigen::Matrix<double, 2, 1> Vara;

  Eigen::Matrix<double, 2, 6> Ho;
  Eigen::Matrix<double, 2, 1> Varo;
};

}  // namespace frc846
