#include <iostream>
#include "kalman_filter.h"
#include <math.h>
#include <iomanip> 
#include <ios> 

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
   x_ = F_ * x_;
   MatrixXd Ft = F_.transpose();
   P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  UpdateCommon(y);

  /*
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
  */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  double px = x_[0]; //from prdict result
  double py = x_[1]; //from prdict result
  double vx = x_[2]; //from prdict result
  double vy = x_[3]; //from prdict result

  double rho = sqrt(px*px+py*py);
  double phi = atan2(py, px);
  double rho_dot = (px*vx+py*vy)/rho;

  /*
  while(phi < -M_PI)
  {
    phi += 2*M_PI;
  }
  while(phi > +M_PI)
  {
    phi -= 2*M_PI;
  }
  */

  VectorXd z_pred(3,1);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  NormalizeAngle(y(1));
  UpdateCommon(y);

  /*
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
  */

}

void KalmanFilter::NormalizeAngle(double& angle){
  while(angle < -M_PI){
    angle += 2*M_PI;
  }
  while(angle > +M_PI){
    angle -= 2*M_PI;
  }
}

void KalmanFilter::UpdateCommon(Eigen::VectorXd& y){
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
