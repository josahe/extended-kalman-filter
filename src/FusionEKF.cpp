#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

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
  R_laser_in = MatrixXd(2, 2);
  R_radar_in = MatrixXd(3, 3);
  H_laser_in = MatrixXd(2, 4);
  H_radar_in = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_in << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_in << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  //measurement matrix - laser
  H_laser_in << 1, 0, 0, 0,
  		  0, 1, 0, 0;

  //measurement matrix - radar
  H_radar_in << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;

  //initial state
  x_in = VectorXd(4);

  P_in = MatrixXd(4, 4);
  P_in << 1, 0, 0, 0,
  		  0, 1, 0, 0,
  		  0, 0, 1000, 0,
  		  0, 0, 0, 1000;

  F_in = MatrixXd(4, 4);
  F_in << 1, 0, 1, 0,
  		  0, 1, 0, 1,
  		  0, 0, 1, 0,
  		  0, 0, 0, 1;

  Q_in = MatrixXd(4, 4);
  Q_in << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

  ekf_.Init(x_in, P_in, F_in, Q_in, R_laser_in, R_radar_in, H_laser_in, H_radar_in);

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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], //px
                 measurement_pack.raw_measurements_[1], //py
                 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      ekf_.x_ << rho * cos(phi), //px
                 rho * sin(phi), //py
                 0, 0;
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
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

   //compute the time elapsed between the current and previous measurements
 	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
 	previous_timestamp_ = measurement_pack.timestamp_;

	//1. Modify the F matrix so that the time is integrated
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	//2. Set the process covariance matrix Q
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << (pow(dt,4) / 4) * noise_ax, 0, (pow(dt,3) / 2) * noise_ax, 0,
			0, (pow(dt,4) / 4) * noise_ay, 0, (pow(dt,3) / 2) * noise_ay,
			(pow(dt,3) / 2) * noise_ax, 0, pow(dt,2) * noise_ax, 0,
			0, (pow(dt,3) / 2) * noise_ay, 0, pow(dt,2) * noise_ay;


  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
