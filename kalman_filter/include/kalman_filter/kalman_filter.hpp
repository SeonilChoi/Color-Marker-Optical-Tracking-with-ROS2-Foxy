#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <Eigen/Dense>

class KalmanFilter
{
public:
    KalmanFilter(double dt, int state_size, int observation_size, double error_covariance, double process_noise, double observation_noise);
    virtual ~KalmanFilter();

    void set_kalman_gain();
    void set_covariance_matrix(const Eigen::VectorXd & observation);
    void set_transition_matrix(const Eigen::MatrixXd & A);
    void set_state_vector(const Eigen::VectorXd & x);
    void get_state_vector(Eigen::VectorXd & x);
    
private:
    void predict();
    void compute();
    void measure(const Eigen::VectorXd & observation);
    void update();

    Eigen::MatrixXd A_;
    Eigen::MatrixXd P_;
    const Eigen::MatrixXd H_;
    const Eigen::MatrixXd Q_;
    const Eigen::MatrixXd R_;
    Eigen::MatrixXd K_;

    Eigen::VectorXd x_;
    Eigen::VectorXd z_;
    
    const double dt_;
};

#endif
