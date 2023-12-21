// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#include "../include/kalman_filter/kalman_filter.hpp"

#include <random>
#include <eigen3/Eigen/Core>


int main(int argc, char **argv)
{
	KalmanFilter kf;

	// State transition matrix with constant values
	Eigen::MatrixXd A(2, 2);
	// State transition matrix multiplied by time step
	Eigen::MatrixXd A_dt(2, 2);
	// Measurement matrix
	Eigen::MatrixXd C(2, 2);
	// Process noise covariance matrix
	Eigen::MatrixXd Q(2, 2);
	// Measurement noise covariance matrix
	Eigen::MatrixXd R(2, 2);

	A << 1, 0, 0, 1;
	A_dt << 0, 1, 0, 0;
	C << 1, 0, 0, 1;
	Q << 1.0, 0, 0, 1.0;
	R << 0.5, 0, 0, 1.0;

	kf.set_matrices(A, A_dt, C, Q, R);

	// State vector
	Eigen::VectorXd x(2);
	// State covariance matrix
	Eigen::MatrixXd P(2, 2);
	// Measurement vector
	Eigen::VectorXd y(2);

	x << 0, 0;
	P << 0.3, 0, 0, 100.0;

	kf.set_states(x, P);

	// Time step
	double dt = 0.1;

	// Normal distribution
	std::random_device rd;
	std::mt19937 mt(rd());
	std::normal_distribution<> nd(0.0, 1.0/3.0);

	for (int i = 0; i < 30; i++)
	{
		printf("-------------------------\n");
		printf("Iteration %d\n", i);

		// Predict next state.
		x = kf.predict(dt);
		printf("x = %10.5f %10.5f\n", x[0], x[1]);

		y << (i+1)*dt+0.1*nd(mt), 1.0+0.3*nd(mt);

		// Update state vector.
		x = kf.update(y);
		printf("y = %10.5f %10.5f\n", y[0], y[1]);
		printf("x = %10.5f %10.5f\n", x[0], x[1]);
	}

	return 0;
}