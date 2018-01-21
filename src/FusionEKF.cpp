#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cstdlib>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  H_laser_ << 1,0,0,0,
                0,1,0,0;

  Hj_ << 0,0,0,0,
        0,0,0,0,
        0,0,0,0;

  noise_ax = 9;
  noise_ay = 9;
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
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double rho_dot = measurement_pack.raw_measurements_[2];
      ekf_.x_[0] = cos(phi) * rho;
      ekf_.x_[1] = sin(phi) * rho;
      // not sure about this since rho_dot is the "projection" of velocity
      // vector on rho vector, not the actual velocity vector itself, so
      // sin/cos of phi * rho_dot does not actually give us vx an vy!
      //ekf_.x_[2] = cos(phi) * rho_dot;
      //ekf_.x_[3] = sin(phi) * rho_dot;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_[0] = measurement_pack.raw_measurements_[0];
      ekf_.x_[1] = measurement_pack.raw_measurements_[1];
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // initialize P_
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // update time change and current timestamp
  dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  // update transition state matrix F
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1,0,dt,0,
            0,1,0,dt,
            0,0,1,0,
            0,0,0,1;

  // Set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_(0,0) = pow(dt,4) * noise_ax / 4.0;
  ekf_.Q_(0,1) = 0.0;
  ekf_.Q_(0,2) = pow(dt,3) * noise_ax / 2.0;
  ekf_.Q_(0,3) = 0.0;
  ekf_.Q_(1,0) = 0.0;
  ekf_.Q_(1,1) = pow(dt,4) * noise_ay / 4.0;
  ekf_.Q_(1,2) = 0.0;
  ekf_.Q_(1,3) = pow(dt,3) * noise_ay / 2.0;
  ekf_.Q_(2,0) = pow(dt,3) * noise_ax / 2.0;
  ekf_.Q_(2,1) = 0.0;
  ekf_.Q_(2,2) = pow(dt,2) * noise_ax;
  ekf_.Q_(2,3) = 0.0;
  ekf_.Q_(3,0) = 0.0;
  ekf_.Q_(3,1) = pow(dt,3) * noise_ay / 2.0;
  ekf_.Q_(3,2) = 0.0;
  ekf_.Q_(3,3) = pow(dt,2) * noise_ay;

  //predict new state
  ekf_.Predict();

  // DEBUG ONLY: show predicted value
  //cout << "PRD: " << ekf_.x_[0] << ", " << ekf_.x_[1] << ", " << ekf_.x_[2] << ", " << ekf_.x_[3] << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // get state parameters
	double px = ekf_.x_(0);
	double py = ekf_.x_(1);
	double vx = ekf_.x_(2);
	double vy = ekf_.x_(3);

	// as long as px_py_sqd not too close to zero, calculate new
	// jacobian matrix, otherwise, default to previous jacobian matrix
	float px_py_sqd = px*px + py*py;
	if(px_py_sqd > 0.0001)
	{
	    float px_py_mag = sqrt(px_py_sqd);
	    Hj_(0,0) = px / px_py_mag;
        Hj_(0,1) = py / px_py_mag;
        Hj_(1,0) = -py / px_py_sqd;
        Hj_(1,1) = px / px_py_sqd;
        Hj_(2,0) = py*(vx*py - vy*px) / (px_py_sqd * px_py_mag);
        Hj_(2,1) = px*(vy*px - vx*py) / (px_py_sqd * px_py_mag);
        Hj_(2,2) = px / px_py_mag;
        Hj_(2,3) = py / px_py_mag;
	}

    ekf_.H_ = Hj_;
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
