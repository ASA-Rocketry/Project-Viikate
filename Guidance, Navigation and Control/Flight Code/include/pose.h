// KalmanFilter.h
#pragma once
#include <ArduinoEigen.h>
using namespace Eigen;

// ─────────────────────────────────────────────────────────────
//  KalmanFilter1D
//  assumes that the observation matrix is I
// ─────────────────────────────────────────────────────────────
class KalmanFilter {
public:

  KalmanFilter(double dt,
               const Eigen::MatrixXd& A, // state transition matrix
               const Eigen::MatrixXd& Q, // process noise matrix
               const Eigen::MatrixXd& R); // measurement noise matrix

  void init(double t0, const Eigen::VectorXd& x0);

  void init();

  void update(const Eigen::VectorXd& z);

  void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

  Eigen::VectorXd state() { return x_hat; };
  double time() { return t; };

private:
  Eigen::MatrixXd A, Q, R, P, K, P0;

  int m, n;

  double t0, t, dt;

  bool initialised;

  Eigen::MatrixXd I;

  Eigen::MatrixXd x_hat, x_hat_1;
};
