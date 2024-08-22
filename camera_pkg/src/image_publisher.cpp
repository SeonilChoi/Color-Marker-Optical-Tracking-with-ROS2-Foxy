#include "camera_pkg/image_publisher.hpp"

#include <chrono>
#include <cstdint>
#include <string>

using namespace std::chrono_literals;

ImagePublisher::ImagePublisher() : Node("image_publisher")
{
    this->declare_parameter("camera_number", 0);
    uint8_t camera_number = this->get_parameter("camera_number").get_value<uint8_t>();
    
    this->declare_parameter("frame_width", 1280);
    uint16_t frame_width = this->get_parameter("frame_width").get_value<uint16_t>();
    
    this->declare_parameter("frame_height", 720);
    uint16_t frame_height = this->get_parameter("frame_height").get_value<uint16_t>();
    
    this->declare_parameter("encoding", "bgr8");
    std::string encoding = this->get_parameter("encoding").get_value<std::string>();
    
    const rclcpp::QoS QOS_BEKL5V = rclcpp::QoS(rclcpp::KeepLast(5))
        .best_effort()
        .durability_volatile();
    
    image_publisher_ = this->create_publisher<Image>("camera/data_raw", QOS_BEKL5V);
    timer_ = this->create_wall_timer(5ms, [this](){ timer_callback(); });
        
    if (!cap_.open(camera_number)){
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera %d.", camera_number);
        return;
    }
    cap_.set(cv::CAP_PROP_AUTOFOCUS, 0);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
    
    bridge_ = cv_bridge::CvImage();
    bridge_.header.frame_id = "camera" + std::to_string(camera_number);
    bridge_.encoding = encoding;
}

ImagePublisher::~ImagePublisher()
{
    cap_.release();
}

void ImagePublisher::timer_callback()
{
    cv::Mat frame;
    
    if (!cap_.read(frame)){
        return;
    }
    
    bridge_.header.stamp = this->get_clock()->now();
    bridge_.image = frame;
    
    Image msg;
    bridge_.toImageMsg(msg);
    
    image_publisher_->publish(msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto image_publisher = std::make_shared<ImagePublisher>();
    rclcpp::spin(image_publisher);
    rclcpp::shutdown();
    
    return 0;
}
