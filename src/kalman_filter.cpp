// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#include <kalman_filter/kalman_filter.hpp>

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>


KalmanFilter::KalmanFilter()
{
	matrices_initialized_ = false;
	states_initialized_ = false;
}

KalmanFilter::KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &A_dt, const Eigen::MatrixXd &C, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R)
{
	set_matrices(A, A_dt, C, Q, R);
	matrices_initialized_ = true;
	states_initialized_ = false;
}

void KalmanFilter::set_matrices(const Eigen::MatrixXd &A, const Eigen::MatrixXd &A_dt, const Eigen::MatrixXd &C, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R)
{
	A_const_ = A;
	A_dt_ = A_dt;
	C_ = C;
	Q_ = Q;
	R_ = R;
	I_ = Eigen::MatrixXd::Identity(A_const_.rows(), A_const_.cols());
	matrices_initialized_ = true;
}

void KalmanFilter::set_states(const Eigen::VectorXd &x, const Eigen::MatrixXd &P)
{
	x_ = x;
	P_ = P;
	states_initialized_ = true;
}

void KalmanFilter::check_initialized()
{
	if (!matrices_initialized_)
		throw std::runtime_error("Matrices are not initialized.");

	if (!states_initialized_)
	{
		std::cerr << "States are not initialized. Using zeros for state vector and state covariance matrix." << std::endl;
		x_ = x_pred_ = Eigen::VectorXd::Zero(A_const_.rows());
		P_ = P_pred_ = Eigen::MatrixXd::Zero(A_const_.rows(), A_const_.cols());
		states_initialized_ = true;
	}
}

Eigen::VectorXd KalmanFilter::get_predicted_state() const
{
	return x_pred_;
}

Eigen::VectorXd KalmanFilter::get_updated_state() const
{
	return x_;
}

Eigen::VectorXd KalmanFilter::predict(const double dt)
{
	check_initialized();

	// Calculate state transition matrix.
	auto A = A_const_ + A_dt_ * dt;

	// Predict state vector.
	x_pred_ = A * x_;

	// Predict state covariance matrix.
	P_pred_ = A * P_ * A.transpose() + Q_;

	return x_pred_;
}

Eigen::VectorXd KalmanFilter::update(const Eigen::VectorXd &y)
{
	check_initialized();

	// Residual
	Eigen::VectorXd e = y - C_ * x_pred_;

	// Observation residual covariance
	Eigen::MatrixXd S = R_ + C_ * P_pred_ * C_.transpose();

	// Kalman gain
	Eigen::MatrixXd K = P_pred_ * C_.transpose() * S.inverse();

	// Update state vector.
	x_ = x_pred_ + K * e;

	// Update state covariance matrix.
	P_ = (I_ - K * C_) * P_pred_;

	return x_;
}

