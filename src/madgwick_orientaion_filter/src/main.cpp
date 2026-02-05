#include "rclcpp/rclcpp.hpp"
#include "dependencies/msg/imu.hpp"

using std::placeholders::_1;

float delta_t = 0.2;

void hmailton_product(float a[4], float b[4],float result[4]){

    float temp[4];
    temp[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    temp[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    temp[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    temp[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    result = temp;
}

void orientation_gyro(float gyro[4], float orientation[4]){
    float temp[4] = {0.5f*orientation[0],0.5f*orientation[1],0.5f*orientation[2],0.5f*orientation[3]};
    hmailton_product(temp,gyro,gyro);
    temp[0] = gyro[0]*delta_t; temp[1] = gyro[1]*delta_t; temp[2] = gyro[2]*delta_t; temp[3] = gyro[3]*delta_t;
    orientation[0] = orientation[0]+temp[0];orientation[1] = orientation[1]+temp[1];orientation[2] = orientation[2]+temp[2];orientation[3] = orientation[3]+temp[3];
}

class recive_lidar_data : public rclcpp::Node{
    public:
        recive_lidar_data() : Node("lidar_subscriber"){
            subscription_ = this->create_subscription<dependencies::msg::Imu>(
                "lidar_data_topic",
                10,
                std::bind(&recive_lidar_data::topic_callback, this, _1)
            );
            
            RCLCPP_INFO(this->get_logger(), "Lidar listener node has started.");

        }
    
    private:
    void topic_callback(const dependencies::msg::Imu::SharedPtr msg) const{
        
    }

    rclcpp::Subscription<dependencies::msg::Imu>::SharedPtr subscription_;
};