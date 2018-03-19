#include "kalman_filter.h"
#include <cmath>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
												MatrixXd &Q_in, MatrixXd &R_laser_in,
												MatrixXd &R_radar_in, MatrixXd &H_laser_in) {
	/**
		* initialise the state
	*/
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	Q_ = Q_in;
	R_laser_ = R_laser_in;
	R_radar_ = R_radar_in;
	H_laser_ = H_laser_in;
}

void KalmanFilter::Predict() {
	/**
		* predict the state
	*/
	x_ = F_ * x_;

	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
		* update the state by using Kalman Filter equations
	*/
	MatrixXd H = H_laser_;
	MatrixXd R = R_laser_;

	VectorXd z_pred = H * x_;
	VectorXd y = z.head(2) - z_pred;
	MatrixXd Ht = H.transpose();
	MatrixXd S = H * P_ * Ht + R;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
		* update the state by using Extended Kalman Filter equations
	*/
	MatrixXd H = H_radar_;
	MatrixXd R = R_radar_;

	// Linearise the non-linear measurement function using Jacobian matrix
	try {
		H_radar_ = tools_.CalculateJacobian(x_);
	} catch (std::overflow_error e) {
		Predict();
		return;
	}

	float h0 = sqrt(pow(x_[0], 2) + pow(x_[1], 2));
	float h1 = atan2(x_[1], x_[0]);
	float h2 = (x_[0]*x_[2] + x_[1]*x_[3]) / h0;

	while (h1 > Pi_) { h1 -= twoPi_; }
	while (h1 < -Pi_) {	h1 += twoPi_;	}

	VectorXd z_pred(3);
	z_pred << h0, h1, h2;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H.transpose();
	MatrixXd S = H * P_ * Ht + R;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H) * P_;
}
