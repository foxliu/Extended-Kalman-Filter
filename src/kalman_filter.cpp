#include "kalman_filter.h"

#include <iostream>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    /**
    TODO:
      * predict the state
    */
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Kalman Filter equations
    */
    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::VectorXd y_ = z - H_ * x_;
    Eigen::MatrixXd S_ = H_ * P_ * Ht + R_;
    Eigen::MatrixXd Si = S_.inverse();
    Eigen::MatrixXd K_ = P_ * Ht * Si;
    x_ = x_ + (K_ * y_);

    long x_size = x_.size();
    Eigen::MatrixXd I_ = Eigen::MatrixXd::Identity(x_size, x_size);
    P_ = (I_ - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Extended Kalman Filter equations
    */
    Eigen::VectorXd hx(3);
    double px, py, vx, vy;
    px = x_[0];
    py = x_[1];
    vx = x_[2];
    vy = x_[3];

    double c = sqrt(px * px + py * py);
    if (c < 0.0000001) c = 0.0000001;
    hx << c, atan2(py, px), (px * vx + py * vy) / c;
    Eigen::VectorXd y_ = z - hx;

    if (y_[1] > M_PI) y_[1] -= 2.0f * M_PI;
    else if (y_[1] < -M_PI) y_[1] += 2.0f * M_PI;

    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S_ = H_ * P_ * Ht + R_;
    Eigen::MatrixXd K_ = P_ * Ht * S_.inverse();
    x_ = x_ + (K_ * y_);

    long x_size = x_.size();
    Eigen::MatrixXd I_ = Eigen::MatrixXd::Identity(x_size, x_size);

    P_ = (I_ - K_ * H_) * P_;
}
