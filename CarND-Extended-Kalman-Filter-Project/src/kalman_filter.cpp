#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  Tools tools;
  
  // First convert the px, py, vx, vy to radar measurements
  double range = sqrt( pow(x_[0],2) + pow(x_[1],2) );
  double bearing = atan2(x_[1], x_[0]);
  
  if (fabs(x_[0]) > 0.001)
  {
      bearing = atan2(x_[1], x_[0]);
  }
  else
  {
      bearing = atan2(0.0001, 0.001);
      
  }
//  std::cout << "Bearing is: "<< bearing << std::endl;
  
  double range_rate =  ((x_[0]*x_[2]+x_[1]*x_[3])/(sqrt( pow(x_[0],2) + pow(x_[1],2) )));
  if (std::abs(sqrt(pow(x_[0],2) + pow(x_[1], 2))) < 0.0001) {
    range_rate = 0.0001;
  }

  MatrixXd z_pred(3, 1);
  z_pred << range, bearing, range_rate;
  
  // Now see the difference in our prediction to our measured and get the linearized K
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  // New estimate
  // x = x'+Ky=x'+K(z-h(x'))
  // This is the equation that fits Taylor expansion, where Df(a) = K (which is the Jacobian)
  // and (x-a) is z-h(x') here
  
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
