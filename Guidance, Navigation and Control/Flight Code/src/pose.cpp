// KalmanFilter.cpp
#include "pose.h"
#include <Arduino.h>
#include <ArduinoEigen.h>

KalmanFilter::KalmanFilter(double dt,
                             const Eigen::MatrixXd& A,
                             const Eigen::MatrixXd& Q,
                             const Eigen::MatrixXd& R,
                             const Eigen::MatrixXd& H, 
                             const Eigen::MatrixXd& P0)
    : A(A), Q(Q), R(R), P0(P0), I(I), m(R.rows()), n(A.rows()), dt(dt), initialised(false) {
        
        I.setIdentity(m, n);

    }

void KalmanFilter::init(const Eigen::MatrixXd& A,
                             const Eigen::MatrixXd& Q,
                             const Eigen::MatrixXd& R, 
                             const Eigen::MatrixXd& H,
                             const Eigen::MatrixXd& P0, double t0, const Eigen::VectorXd& x0) {
    this->A = A;
    this->Q = Q;
    this->R = R;
    this->H = H;
    this->P0 = P0;
    x_hat = x0;
    P = P0;
    this->t0 = t0;
    t = t0;
    initialised = true;
}
  
void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
    x_hat = x0;
    P = P0;
    this->t0 = t0;
    t = t0;
    initialised = true;
}

void KalmanFilter::init() {
    x_hat.setZero();
    P = P0;
    t0 = 0.0;
    t = 0.0;
    I = Eigen::MatrixXd::Identity(m, n);
    initialised = true;
}

void KalmanFilter::update(const Eigen::VectorXd& z) {
    if (!initialised) {
        Serial.println("KalmanFilter not initialized!");
        return;
    }

    x_hat_1 = A * x_hat;
    P = A * P * A.transpose() + Q;
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x_hat_1 += K * (z - H * x_hat_1);
    P = (I - K * H) * P;
    x_hat = x_hat_1;

    t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {

    this->A = A;
    this->dt = dt;
    update(y);
}
