#include "kalman_filter.h"
#include <iostream>
#include <cstdlib>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  cout << "Initialize KalmanFilter" << endl;
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // calculate Kalman gain
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // convert prediction to polar - avoid dividing by zero
  VectorXd z_pred = VectorXd(3);
  if(x_[0] == 0)
  {
    z_pred << 0,0,0;
  }
  else
  {
    double rho = pow(x_[0] * x_[0] + x_[1] * x_[1], 0.5);
    double phi = atan2(x_[1], x_[0]);
    double rho_dot = (x_[0] * x_[2] + x_[1] * x_[3]) / rho;
    z_pred << rho, phi, rho_dot;
  }

  VectorXd y = z - z_pred;

  // make sure y_phi between -pi and pi
  // my logic assumes we are never off by more than one factor of 2pi
  if(y[1] > M_PI )
  {
      y[1] = y[1] - 2*M_PI;
  }
  if(y[1] < -M_PI)
  {
      y[1] = y[1] + 2*M_PI;
  }

  // calculate Kalman gain
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
