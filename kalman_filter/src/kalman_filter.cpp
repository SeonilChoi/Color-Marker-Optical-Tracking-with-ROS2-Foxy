#include "kalman_filter/kalman_filter.hpp"

Eigen::MatrixXd init_matrix(int rows, int cols, double noise)
{
    return Eigen::MatrixXd::Identity(rows, cols) * noise;
}

KalmanFilter::KalmanFilter(double dt, int state_size, int observation_size, double error_covariance, double process_noise, double observation_noise) :
    H_(init_matrix(observation_size, state_size, 1.0)),
    Q_(init_matrix(state_size, state_size, process_noise)),
    R_(init_matrix(observation_size, observation_size, observation_noise)),
    dt_(dt)
{
    A_ = Eigen::MatrixXd::Zero(state_size, state_size);
    P_ = Eigen::MatrixXd::Identity(state_size, state_size) * error_covariance;
    K_ = Eigen::MatrixXd::Zero(state_size, observation_size);

    x_ = Eigen::VectorXd::Zero(state_size);
    z_ = Eigen::VectorXd::Zero(observation_size);
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::set_kalman_gain()
{
    predict();
    compute();
}

void KalmanFilter::set_covariance_matrix(const Eigen::VectorXd & observation)
{
    measure(observation);
    update();
}

void KalmanFilter::set_transition_matrix(const Eigen::MatrixXd & A)
{
    A_ = A;
}

void KalmanFilter::set_state_vector(const Eigen::VectorXd & x)
{
    x_ = x;
}

void KalmanFilter::get_state_vector(Eigen::VectorXd & x)
{
    x = x_;
}

void KalmanFilter::predict()
{
    x_ = A_ * x_;
    P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::compute()
{
    K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
}

void KalmanFilter::measure(const Eigen::VectorXd & observation)
{
    z_ = observation;
}

void KalmanFilter::update()
{
    x_ = x_ + K_ * (z_ - H_ * x_);
    P_ = P_ - K_ * H_ * P_;
}
