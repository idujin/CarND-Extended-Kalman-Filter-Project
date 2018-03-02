#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

const double PI = 3.14159265;
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

  long x_size = x_.rows();
  I_ = Eigen::MatrixXd::Identity(x_size,x_size);
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
  VectorXd y = z - H_* x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_* P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  x_ = x_ + K * y;
  P_ = (I_ - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float rho, phi, rho_dot;
  // not to divide by zero
  if(py == 0.f && px == 0.f)
  {
    rho =0.f;
    phi =0.f;
    rho_dot =0.f;
  }
  else
  {
      rho = sqrtf(px*px + py*py);
      phi = atan2(py , px);
      rho_dot = (px*vx + py*vy)/(rho);
  }

  
  VectorXd hx(3);
  hx << rho, phi, rho_dot;

  VectorXd y = z - hx;

  while (y[1]>PI) {
    y[1] -= 2.f * PI;
  }
  while (y(1)< -PI) {
    y[1] += 2.f * PI;
  }

  MatrixXd Ht = H_.transpose();


  MatrixXd S = H_* P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  x_ = x_ + K * y;
  P_ = (I_ - K * H_) * P_;


}
