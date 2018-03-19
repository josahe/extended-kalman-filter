#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
															const vector<VectorXd> &ground_truth) {
	/**
		* Calculate the RMSE here.
	*/
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() == 0) {
		cout << "Estimation vector is empty" << endl;
	} else if (estimations.size() != ground_truth.size()) {
		cout << "Vecor size does not match" << endl;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i)  {
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array() * residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse.array() / estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	assert (~rmse.isZero());
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	/**
		* Calculate a Jacobian here.
	*/
	MatrixXd Hj(3,4);

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute some common calculations
	float c1 = pow(px,2) + pow(py,2);
	float c2 = sqrt(c1);
	float c3 = vx*py - vy*px;
	float c4 = pow(c1, 3/2);

	//check division by zero or close to zero
	if (c1 <= 0.001) {
		throw std::overflow_error("Attempted divide by zero\n");
	}

	//compute the Jacobian matrix elements
	float dpdpx = px / c2;
	float dpdpy = py / c2;
	float dpdvx = 0;
	float dpdvy = 0;

	float drdpx = -(py / c1);
	float drdpy = px / c1;
	float drdvx = 0;
	float drdvy = 0;

	float dPdpx = (py * c3) / c4;
	float dPdpy = (px * c3) / c4;
	float dPdvx = px / c2;
	float dPdvy = py / c2;

	//populate the Jacobian matrix
	Hj << dpdpx, dpdpy, dpdvx, dpdvy,
				drdpx, drdpy, drdvx, drdvy,
				dPdpx, dPdpy, dPdvx, dPdvy;

	return Hj;
}
