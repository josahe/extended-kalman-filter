#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include <cmath>
#include "tools.h"

class KalmanFilter {
public:
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix - LIDAR
  Eigen::MatrixXd H_laser_;

  // measurement (Jacobian) matrix - RADAR
  Eigen::MatrixXd H_radar_;

  // measurement covariance matrix - LIDAR
  Eigen::MatrixXd R_laser_;

  // measurement covariance matrix - RADAR
  Eigen::MatrixXd R_radar_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param Q_in Process covariance matrix
   * @param R_laser_in Measurement covariance matrix
   * @param R_radar_in Measurement covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &Q_in, Eigen::MatrixXd &R_laser_in, Eigen::MatrixXd &R_radar_in,
      Eigen::MatrixXd &H_laser_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

private:
  const float Pi_ = atan(1)*4;
  const float twoPi_ = 2*Pi_;
  Tools tools_;

};

#endif /* KALMAN_FILTER_H_ */
