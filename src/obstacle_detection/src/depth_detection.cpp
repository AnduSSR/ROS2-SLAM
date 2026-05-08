#include "rclcpp/rclcpp.hpp"
#include "dependencies/msg/imu.hpp"
#include <chrono>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

using namespace std::chrono_literals;


using std::placeholders::_1;

const float focal_length[2] = {1.0f, 1.0f}; //focal length (x,y) of the camera in pixels, this is a placeholder value and should be replaced with the actual focal length of the camera being used
const float optical_center[2] = {0.0f, 0.0f}; // optical center (x,y) of the camera in pixels, this is a placeholder value and should be replaced with the actual optical center of the camera being used


class imu_data : public rclcpp::Node{
    public:
        imu_data() : Node("imu"){

            publisher_ = this->create_publisher<dependencies::msg::Imu>(
                "imu_data_topic",
                10
            );
            RCLCPP_INFO(this->get_logger(), "Lidar listener node has started.");

            timer_ = this->create_wall_timer(1s, std::bind(&imu_data::topic_callback, this));
            
        }
    
    private:
    void topic_callback(){
        
    }

        rclcpp::Publisher<dependencies::msg::Imu>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imu_data>());
    rclcpp::shutdown();

    return 0;
}