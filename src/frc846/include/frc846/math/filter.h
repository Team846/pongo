#pragma once

// #include <frc/EigenCore.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>

namespace frc846::math {

using mat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

template <int N> class LinearKalmanFilter {
public:
  LinearKalmanFilter(mat starterState, mat transformerMatrix) {
    pTransformer = transformerMatrix;
    state = starterState;
    covar = Eigen::MatrixXd::Identity(N, N);
  }

  LinearKalmanFilter() {}

  void Predict(mat pCovariance) {
    state = pTransformer * state;
    covar = pTransformer * covar * pTransformer.transpose() + pCovariance;
  }

  void Update(mat H, mat z, mat var) {
    mat upCov = var.asDiagonal();
    mat pre_fit_resid = z - H * state;
    mat pre_fit_cov = H * covar * H.transpose() + upCov;
    mat kalm_gain = covar * H.transpose() * pre_fit_cov.inverse();
    state = state + kalm_gain * pre_fit_resid;
    mat bel_factor = kalm_gain * H;
    covar = (Eigen::MatrixXd::Identity(bel_factor.rows(), bel_factor.cols()) -
                bel_factor) *
            covar;
  }

  mat getEstimate() { return state; }

  void setEstimate(mat setState, mat var) {
    state = setState;
    covar = var.asDiagonal();
  }

  void setSureEstimate(mat setState) {
    state = setState;
    covar = Eigen::MatrixXd::Identity(N, N);
  }

  mat getCoVar() { return covar; };

protected:
  mat state{N, 1};
  mat covar{N, N};
  mat pTransformer{N, N};

  // MatrixXd x;
};

};  // namespace frc846::math