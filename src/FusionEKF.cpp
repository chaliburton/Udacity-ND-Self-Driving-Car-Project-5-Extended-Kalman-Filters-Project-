#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;


  previous_timestamp_ = 0;

  // initializing matrices
  /*
  //TODO: Finish initializing the FusionEKF.
  //TODO: Set the process and measurement noises
  */
  
  //measurement covariance matrix - laser * to @param R_in Measurement covariance matrix for laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  
  //measurement covariance matrix - radar * to @param R_in Measurement covariance matrix for radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // laser measurement matrix * to param H_in Measurement matrix for laser
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
    
  // radar jacobian linearization measurement matrix  * to param H_in Measurement matrix for radar
  Hj_ = MatrixXd(3, 4);
//  Hj_ << 1, 1, 0, 0,
//         1, 1, 0, 0,
//         1, 1, 1, 1;
  
  // state covariance matrix P @param P_in Initial state covariance
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
    //implement control flow to deactivate Radar or Lidar
  int use_Lidar = 1;
  int use_Radar = 0;
  
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_Radar ==1) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_[0]; // range
      float phi = measurement_pack.raw_measurements_[1]; // bearing
      float rho_dot = measurement_pack.raw_measurements_[2]; // range rate (speed along bearing line from Doppler effect)
      // Normalize phi to [-pi, pi]
      if (phi > M_PI)  phi -= 2.0 * M_PI;
      if (phi < -M_PI) phi += 2.0 * M_PI;
      
      float raw_px = rho * cos(phi);
      float raw_py = rho * sin(phi);
      ekf_.x_ << raw_px, 
                 raw_py, 
                 0, 
                 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_Lidar == 1) {
      // TODO: Initialize state.
      float raw_x = measurement_pack.raw_measurements_[0];
      float raw_y = measurement_pack.raw_measurements_[1];
      ekf_.x_ << raw_x, 
                 raw_y, 
                 0, 
                 0;
    }
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // set measurement noises
  float noise_ax_ = 9.0;
  float noise_ay_ = 9.0;
  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
//  ekf_.F_ << 1, 0, dt, 0,
//             0, 1, 0, dt,
//             0, 0, 1, 0,
//             0, 0, 0, 1;
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  
  float dt3b2 = dt_3/2.0;
  float dt4b4 = dt_4/4.0;
  
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4b4*noise_ax_, 0, dt3b2*noise_ax_, 0,
             0, dt4b4*noise_ay_, 0, dt3b2*noise_ay_,
             dt3b2*noise_ax_, 0, dt_2*noise_ax_, 0,
             0, dt3b2*noise_ay_, 0, dt_2*noise_ay_;
  
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices. 
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_Radar == 1) {
    // TODO: Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
  } else if ( use_Lidar == 1) {
    // TODO: Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
