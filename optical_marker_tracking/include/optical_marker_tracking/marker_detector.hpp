#ifndef MARKER_DETECTOR_HPP_
#define MARKER_DETECTOR_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "optical_tracking_msgs/msg/point_multi_array.hpp"

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

class MarkerDetector : public rclcpp::Node
{
public:
    using Image = sensor_msgs::msg::Image;
    using PointMultiArray = optical_tracking_msgs::msg::PointMultiArray;

    MarkerDetector();
    virtual ~MarkerDetector();

private:
    void publish_point_msg(const cv::Vec2f pt);
    void publish_point_msg(const std::vector<cv::Vec3f> & pts);
    void segment_markers(const cv::Mat & frame, std::vector<cv::Vec3f> & pts);

    void image_callback(const Image::SharedPtr msg);
    
    rclcpp::Subscription<Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<PointMultiArray>::SharedPtr marker_publisher_;
};

#endif
