#include "frc846/robot/swerve/odometry/pose_estimator.h"

#include <frc/DriverStation.h>

#include "frc846/math/vectors.h"
#include "frc846/robot/GenericRobot.h"

namespace frc846::robot::swerve::odometry {

PoseEstimator::PoseEstimator(
    frc846::math::VectorND<units::foot, 2> initial_position,
    frc846::math::VectorND<units::feet_per_second, 2> initial_vel,
    frc846::math::VectorND<units::feet_per_second_squared, 2> initial_accl) {
  state_ = {initial_position[0].to<double>(), initial_position[1].to<double>(),
      initial_vel[0].to<double>(), initial_vel[1].to<double>(),
      initial_accl[0].to<double>(), initial_accl[1].to<double>()};
  double dt = units::second_t(frc846::robot::GenericRobot::kPeriod).to<double>();
  filter = frc846::math::LinearKalmanFilter<6>(state_,
      Eigen::Matrix<double, 6, 6>(
          {{1, 0, dt, 0, dt * dt / 2, 0}, {0, 1, 0, dt, 0, dt * dt / 2},
              {0, 0, 1, 0, dt, 0}, {0, 0, 0, 1, 0, dt}, {0, 0, 0, 0, 1, 0},
              {0, 0, 0, 0, 0, 1}}));  // TODO: how to
                                      // fix this to
                                      // be prefs?
  Hv = Eigen::Matrix<double, 2, 6>({{1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}});

  Ha = Eigen::Matrix<double, 2, 6>({{0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 1}});
  Vara = Eigen::Matrix<double, 2, 1>({{1.0}, {1.0}});

  Ho = Eigen::Matrix<double, 2, 6>({{0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0}});
  Varo = Eigen::Matrix<double, 2, 1>({{1.0}, {1.0}});
}

void PoseEstimator::Update(double pVar, double vVar, double aVar) {
  filter.Predict(Eigen::Matrix<double, 6, 1>(
      {{pVar}, {pVar}, {vVar}, {vVar}, {aVar}, {aVar}})
          .asDiagonal());
  state_ = filter.getEstimate();
}

void PoseEstimator::AddVisionMeasurement(
    frc846::math::VectorND<units::foot, 2> pos, double variance) {
  filter.Update(Hv,
      Eigen::Matrix<double, 2, 1>(
          {{pos[0].to<double>()}, {pos[1].to<double>()}}),
      Eigen::Matrix<double, 2, 1>({{variance, variance}}));
  state_ = filter.getEstimate();
}

void PoseEstimator::AddAccelerationMeasurement(
    frc846::math::VectorND<units::feet_per_second_squared, 2> accl) {
  filter.Update(Ha,
      Eigen::Matrix<double, 2, 1>(
          {{accl[0].to<double>()}, {accl[1].to<double>()}}),
      Vara);
  state_ = filter.getEstimate();
}

void PoseEstimator::AddOdometryMeasurement(
    frc846::math::VectorND<units::foot, 2> difPos) {
  double dt = units::second_t(frc846::robot::GenericRobot::kPeriod).to<double>();
  filter.Update(Ho,
      Eigen::Matrix<double, 2, 1>(
          {{(difPos[0] / dt).to<double>()}, {(difPos[1] / dt).to<double>()}}),
      Varo);
  state_ = filter.getEstimate();
}

void PoseEstimator::SetPoint(frc846::math::VectorND<units::foot, 2> point) {
  state_ = Eigen::Matrix<double, 6, 1>(
      {{point[0].to<double>()}, {point[1].to<double>()}, {state_.coeff(2, 0)},
          {state_.coeff(3, 0)}, {state_.coeff(4, 0)}, {state_.coeff(5, 0)}});
  filter.setSureEstimate(state_);
}

void PoseEstimator::Zero() { SetPoint({0_ft, 0_ft}); }

double PoseEstimator::getVariance() {
  Eigen::Matrix<double, 6, 6> cov = filter.getCoVar();
  return (cov.coeff(0, 0) + cov.coeff(1, 1)) / 2;
}

}  // namespace frc846