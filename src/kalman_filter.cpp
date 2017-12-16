#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

#define PI 3.14159265

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  //predict the state
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  //update the state by using Kalman Filter equations
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  DoUpdateStep(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //update the state by using Extended Kalman Filter equations
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float hip = sqrt(px*px+py*py);
  //map from cartesian to polar coordinates (3x1 matrix)
  VectorXd z_pred = VectorXd(3);
  z_pred << hip, atan2(py, px), (px * vx + py * vy) / hip;

  VectorXd y = z - z_pred;

  //normalizing angles (phi)

  while (y[1] > PI || y[1] < -PI) {
    if (y[1] > PI) {
      y[1] -= 2*PI;
    } else if (y[1] < -PI) {
      y[1] += 2*PI;
    }
  }
  
  DoUpdateStep(y);
}

void KalmanFilter::DoUpdateStep(const VectorXd &y) {  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}