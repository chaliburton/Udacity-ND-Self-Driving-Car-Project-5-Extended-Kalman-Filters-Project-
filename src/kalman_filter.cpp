#include "kalman_filter.h"
#include <cmath>
#include<algorithm> 



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
  VectorXd z_pred = H_ * x_;				//predicted measurement based on existing position x and measurement matrix
  VectorXd y = z - z_pred;						//difference in actual measurement and predicted measurement  MatrixXd Ht  = H_.transpose()
  UpdateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  // transfer x' to polar coordinates
  float  eps = 0.0001;
  float  rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  if (std::abs(x_(0)) < 0.0001){
    x_(0) = eps;
  }
     
  float  theta = atan2(x_(1) , x_(0));
  float  rho_dot = (x_(0)*x_(2)+x_(1)*x_(3)) / std::max(eps, rho);
  VectorXd h_Of_X = VectorXd(3); 
  h_Of_X << rho, theta, rho_dot;
  VectorXd y = z - h_Of_X;
  //make sure transformation vector is between -pi and pi
  NormalizeAngle(y(1));  
  UpdateCommon(y);
}
void KalmanFilter::UpdateCommon(const VectorXd& y){
  const MatrixXd PHt = P_ * H_.transpose();
  const MatrixXd S = H_ * PHt + R_;
  const MatrixXd K = PHt * S.inverse();

  x_ += K * y;
  P_ -= K * H_ * P_;
}
void KalmanFilter::NormalizeAngle(double& phi){
  //phi = atan2(sin(phi), cos(phi));
  while (phi>M_PI || phi < -M_PI) {
    if (phi > M_PI) {
      phi -= 2 * M_PI;
    }
    if (phi < -M_PI) {
      phi += 2 * M_PI;
    }
  }
}