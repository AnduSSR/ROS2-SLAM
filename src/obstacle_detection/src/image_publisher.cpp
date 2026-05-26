#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("image_publisher"), cap_(0) {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ImagePublisher::timer_callback, this));

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video stream. Check your camera index!");
        }
    }

private:
    void timer_callback() {
        cv::Mat frame;
        
        cap_ >> frame; 

        if (!frame.empty()) {
            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = "camera_frame";

            sensor_msgs::msg::Image::SharedPtr msg = 
                cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

            // 7. Publish the message
            publisher_->publish(*msg);
            RCLCPP_INFO(this->get_logger(), "Publishing video frame");
        } else {
            RCLCPP_WARN(this->get_logger(), "Captured frame is empty");
        }
    }

    // Node member variables
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char * argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Spin the node
    rclcpp::spin(std::make_shared<ImagePublisher>());
    
    // Clean up
    rclcpp::shutdown();
    return 0;
}