#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
#define PI 3.14159265

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  ekf_.x_ = VectorXd(4);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
        
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      // Convert radar from polar to cartesian coordinates and initialize state.
      float px = rho * cos(phi);
      float py = rho * sin(phi);

      ekf_.x_ << px, py, 0, 0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;      
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
   */

  //convert from microsencod to second
  float delta = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  ekf_.F_(0, 2) = delta;
  ekf_.F_(1, 3) = delta;
    
  //Uses noise_ax = 9 and noise_ay = 9 for your Q matrix.
  int noise_ax = 9; 
  int noise_ay = 9;

  float delta2 = delta * delta;
  float delta3 = delta2 * delta;
  float delta4 = delta3 * delta;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << delta4/4*noise_ax, 0, delta3/2*noise_ax, 0,
         0, delta4/4*noise_ay, 0, delta3/2*noise_ay,
         delta3/2*noise_ax, 0, delta2*noise_ax, 0,
         0, delta3/2*noise_ay, 0, delta2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    MatrixXd Hj = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates    
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
