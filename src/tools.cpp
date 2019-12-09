#include "tools.h"
#include <iostream>
#include <cmath>
//#include "Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  
  //CONFIRM that this is fed as a vector of vectors//////////////////////////////////////////////////////
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if (estimations.size() == 0 ||ground_truth.size() ==0)
  {
    std::cout<<"Error, vector size = 0"<< std::endl;
    rmse << -1,-1,-1,-1;
    return rmse;
  }  
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size())
  {
    std::cout<<"Error, vector sizes do not match"<< std::endl;
    rmse << 99,99,99,99;
    return rmse;
  }
  // TODO: accumulate squared residuals
  VectorXd mean(4);
  mean << 0,0,0,0;
//  VectorXd err(4);
//  err << 0,0,0,0;
  for (unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd err = estimations[i]-ground_truth[i];
    err = err.array()*err.array();
    mean += err;
   // cout<<mean<<endl<<endl;
  }
  
  // TODO: calculate the mean
  mean = mean/estimations.size();
  rmse = mean.array().sqrt();
  // TODO: calculate the squared root

  // return the result

  return rmse;
  ////////////////////////////////////////////////////////
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  ///////////////////////////////////////////////////////
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 
  float px2py2 = px*px+py*py;
  float sqrtpx2py2 = sqrt(px2py2);
  float J11 = px/sqrtpx2py2;
  float J12 = py/sqrtpx2py2;
  
  // check division by zero
  if (fabs(px2py2) < 0.0001) {
    float  eps = 0.0001;
    px2py2 = eps;
  }
  
  float J31 = py*(vx*py-vy*px)/pow(px2py2,(3.0/2.0));
  float J32 = px*(vy*px-vx*py)/pow(px2py2,(3.0/2.0));

  // compute the Jacobian matrix
  Hj << J11, J12, 0, 0,
        (-py)/px2py2, px/px2py2, 0, 0,
        J31, J32, J11, J12;
  return Hj;
  ///////////////////////////////////////////////////////
  
}
