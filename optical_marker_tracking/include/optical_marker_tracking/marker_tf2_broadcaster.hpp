#ifndef MARKER_TF2_BROADCASTER_HPP_
#define MARKER_TF2_BROADCASTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <memory>

class MarkerTF2Broadcaster : public rclcpp::Node
{
public:
    using PoseStamped = geometry_msgs::msg::PoseStamped;

    MarkerTF2Broadcaster();
    virtual ~MarkerTF2Broadcaster();

private:
    void marker_callback(const PoseStamped::SharedPtr msg);

    rclcpp::Subscription<PoseStamped>::SharedPtr marker_subscriber_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif
