#ifndef MARKER_TRACKER_HPP_
#define MARKER_TRACKER_HPP_

#include <memory>
#include <cstdint>
#include <vector>
#include "Eigen/Dense"

#include "rclcpp/rclcpp.hpp"
#include "optical_tracking_msgs/msg/point_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
    
class MarkerTracker : public rclcpp::Node
{
public:
    using PointMultiArray = optical_tracking_msgs::msg::PointMultiArray;
    using PoseStamped = geometry_msgs::msg::PoseStamped;

    MarkerTracker();
    virtual ~MarkerTracker();

private:
    Eigen::Matrix3d initialize_parameter();
    
    void find_marker_geometry(
        const std::vector<Eigen::Vector3d> & in_pts,
        uint8_t & idx,
        std::vector<Eigen::Vector3d> & out_pts
    );
    
    void permutation(
        std::vector<uint8_t> data,
        const uint8_t & depth,
        const uint8_t & n,
        const uint8_t & r,
        const std::vector<Eigen::Vector3d> & in_pts,
        uint8_t & idx,
        double & min_error,
        std::vector<uint8_t> & min_error_index
    );
    
    void reconstruct_pose(
        const uint8_t & idx,
        const std::vector<Eigen::Vector3d> & in_pts
    );
    
    void convert_rotation_matrix_to_quaternion(
        const Eigen::Matrix3d & R,
        Eigen::Vector4d & q
    );
    
    void publish_point_msg(
        const Eigen::Vector3d & position,
        const Eigen::Vector4d & orientation        
    );

    void marker_callback(const PointMultiArray::SharedPtr msg);
    
    rclcpp::Subscription<PointMultiArray>::SharedPtr marker_subscriber_;
    rclcpp::Publisher<PoseStamped>::SharedPtr pose_publisher_;

    const Eigen::Matrix3d Rw_;
    const Eigen::Vector3d tw_;
    
    const std::vector<Eigen::Matrix3d> Ro_;
    const std::vector<Eigen::Vector3d> to_;
    const std::vector<Eigen::Vector3d> marker_geometry_;
    const std::vector<Eigen::Vector3d> marker_geometry_angle_;

    Eigen::Vector4d pre_q_;
};

#endif
