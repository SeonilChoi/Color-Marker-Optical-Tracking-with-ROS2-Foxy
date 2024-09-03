#ifndef MARKER_TRIANGULATOR_HPP_
#define MARKER_TRIANGULATOR_HPP_

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "optical_tracking_msgs/msg/point_multi_array.hpp"

class MarkerTriangulator : public rclcpp::Node
{
public:
    using PointMultiArray = optical_tracking_msgs::msg::PointMultiArray;

    MarkerTriangulator();
    virtual ~MarkerTriangulator();

private:
    void triangulate_points(
        const std::vector<double> & pt1,
        const std::vector<double> & pt2,
        Eigen::Vector3d & Pw
    );
    
    void marker_callback(const PointMultiArray::SharedPtr msg);

    rclcpp::Subscription<PointMultiArray>::SharedPtr marker_subscriber_;
    rclcpp::Publisher<PointMultiArray>::SharedPtr marker_publisher_;

    const Eigen::Matrix<double, 3, 4> pmtx0_;
    const Eigen::Matrix<double, 3, 4> pmtx2_;
    const Eigen::Matrix3d rmtx0_;
    const Eigen::Vector3d tvec0_;

    Eigen::Matrix<double, 3, 4> initialize_pmtx(const int & number);
};

#endif
