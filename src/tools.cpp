#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() == 0 || estimations.size() != ground_truth.size()) {
	    return rmse;
	}

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
	    VectorXd diff = estimations[i] - ground_truth[i];
	    VectorXd sqd = diff.array() * diff.array();
	    rmse = rmse + sqd;
	}

	//calculate the mean
	rmse = rmse / estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
	if(px == 0 && py == 0)
	{
	    return Hj;
	}

	//compute the Jacobian matrix
	float px_py_sqd = pow(px,2) + pow(py,2);
	float px_py_mag = pow(px_py_sqd, 0.5);
	Hj(0,0) = px / px_py_mag;
	Hj(0,1) = py / px_py_mag;
	Hj(1,0) = -py / px_py_sqd;
	Hj(1,1) = px / px_py_sqd;
	Hj(2,0) = py*(vx*py - vy*px) / pow(px_py_mag, 1.5);
	Hj(2,1) = px*(vy*px - vx*py) / pow(px_py_mag, 1.5);
	Hj(2,2) = px / px_py_mag;
	Hj(2,3) = py / px_py_mag;

	return Hj;
}
