#ifndef MARKER_TF2_BROADCASTER_HPP_
#define MARKER_TF2_BROADCASTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "optical_tracking_msgs/msg/point_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <memory>

class MarkerTF2Broadcaster : public rclcpp::Node
{
public:
    using PointMultiArray = optical_tracking_msgs::msg::PointMultiArray;

    MarkerTF2Broadcaster();
    virtual ~MarkerTF2Broadcaster();

private:
    void marker_callback(const PointMultiArray::SharedPtr msg);

    rclcpp::Subscription<PointMultiArray>::SharedPtr marker_subscriber_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif
