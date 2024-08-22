#ifndef IMAGE_PUBLISHER_HPP_
#define IMAGE_PUBLISHER_HPP_

#include <memory>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImagePublisher : public rclcpp::Node
{
public:
    using Image = sensor_msgs::msg::Image;
    
    ImagePublisher();
    virtual ~ImagePublisher();
    
private:
    void timer_callback();

    rclcpp::Publisher<Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    cv::VideoCapture cap_;
    
    cv_bridge::CvImage bridge_;
};

#endif
