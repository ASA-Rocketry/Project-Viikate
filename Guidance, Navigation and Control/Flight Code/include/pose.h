// KalmanFilter.h
#pragma once
#include <ArduinoEigen.h>
using namespace Eigen;

// ─────────────────────────────────────────────────────────────
//  KalmanFilter1D
//  assumes that the observation matrix is I
// A: state transition matrix
// Q: process noise covariance
// R: measurement noise covariance
// P: covariance of the state estimate
// K: Kalman gain
// ─────────────────────────────────────────────────────────────
class KalmanFilter {
public:

  KalmanFilter() {};

  KalmanFilter(double dt,
               const Eigen::MatrixXd& A, // state transition matrix
               const Eigen::MatrixXd& Q, // process noise matrix
               const Eigen::MatrixXd& R,
               const Eigen::MatrixXd& H, // observation matrix
               const Eigen::MatrixXd& P0); // initial state covariance matrix

  void init(const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R,
            const Eigen::MatrixXd& H,
            const Eigen::MatrixXd& P0, double t0, const Eigen::VectorXd& x0);

  void init(double t0, const Eigen::VectorXd& x0);

  void init();

  void update(const Eigen::VectorXd& z);

  void update(const Eigen::VectorXd& z, double dt, const Eigen::MatrixXd A);

  Eigen::VectorXd state() { return x_hat; };
  double time() { return t; };

private:

  Eigen::MatrixXd A, Q, R, P, K, P0, H;

  int m, n;

  double t0, t, dt;

  bool initialised;

  Eigen::MatrixXd I;

  Eigen::MatrixXd x_hat, x_hat_1;
};
