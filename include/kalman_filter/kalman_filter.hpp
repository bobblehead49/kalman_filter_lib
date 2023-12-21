// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <eigen3/Eigen/Core>


class KalmanFilter
{
public:
    /**
     * @brief Kalman filter
    */
    KalmanFilter();

    /**
     * @brief Kalman filter
     * @param A State transition matrix with constant values.
     * @param A_dt State transition matrix multiplied by time step.
     * @param C Measurement matrix.
     * @param Q Process noise covariance matrix.
     * @param R Measurement noise covariance matrix.
    */
    KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &A_dt, const Eigen::MatrixXd &C, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R);

    /**
     * @brief Set matrices for Kalman filter.
     * @param A State transition matrix.
     * @param A_dt State transition matrix multiplied by time step.
     * @param C Measurement matrix.
     * @param Q Process noise covariance matrix.
     * @param R Measurement noise covariance matrix.
    */
    void set_matrices(const Eigen::MatrixXd &A, const Eigen::MatrixXd &A_dt, const Eigen::MatrixXd &C, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R);

    /**
     * @brief Set initial states.
     * @param x State vector.
     * @param P State covariance matrix.
    */
    void set_states(const Eigen::VectorXd &x, const Eigen::MatrixXd &P);

    /**
     * @brief Get predicted state vector.
     * @return Predicted state vector.
    */
    Eigen::VectorXd get_predicted_state() const;

    /**
     * @brief Get updated state vector.
     * @return Updated state vector.
    */
    Eigen::VectorXd get_updated_state() const;

    /**
     * @brief Predict next state.
     * @param dt Time step.
     * return Predicted state vector.
    */
    Eigen::VectorXd predict(double dt);

    /**
     * @brief Update state vector.
     * @param y Measurement vector.
     * @return Updated state vector.
    */
    Eigen::VectorXd update(const Eigen::VectorXd &y);

private:
    bool matrices_initialized_;
    bool states_initialized_;

    /**
     * @brief State vector
    */
    Eigen::VectorXd x_;

    /**
     * @brief Predicted state vector
    */
    Eigen::VectorXd x_pred_;

    /**
     * @brief Control vector
    */
    Eigen::VectorXd u_;

    /**
     * @brief Measurement vector
    */
    Eigen::VectorXd y_;

    /**
     * @brief State transition matrix with constant values
    */
    Eigen::MatrixXd A_const_;

    /**
     * @brief State transition matrix multiplied by time step
    */
    Eigen::MatrixXd A_dt_;

    /**
     * @brief Measurement matrix
    */
    Eigen::MatrixXd C_;

    /**
     * @brief State covariance matrix
    */
    Eigen::MatrixXd P_;

    /**
     * @brief Predicted state covariance matrix
    */
    Eigen::MatrixXd P_pred_;

    /**
     * @brief Process noise covariance matrix
    */
    Eigen::MatrixXd Q_;

    /**
     * @brief Measurement noise covariance matrix
    */
    Eigen::MatrixXd R_;

    /**
     * @brief Kalman gain
    */
    Eigen::MatrixXd K_;

    /**
     * @brief Identity matrix
    */
    Eigen::MatrixXd I_;

    /**
     * @brief Check if initialized.
    */
    void check_initialized();
};


#endif // KALMAN_FILTER_HPP_
