#ifndef STATIC_CAMERA_TF2_BROADCASTER_HPP_
#define STATIC_CAMERA_TF2_BROADCASTER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticCameraTF2Broadcaster : public rclcpp::Node
{
public:
    StaticCameraTF2Broadcaster();
    virtual ~StaticCameraTF2Broadcaster();

private:
    void send_static_transform(const std::string & parent_link, const uint8_t & camera_number);

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

#endif
