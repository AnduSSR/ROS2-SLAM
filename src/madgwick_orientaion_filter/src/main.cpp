#include "rclcpp/rclcpp.hpp"
#include "dependencies/msg/imu.hpp"

using std::placeholders::_1;

float delta_t = 0.2;

void normilize_quaternion(float q[4]){
    float norm = sqrt(q[0]^2 + q[1]^2 + q[2]^2 + q[3]^2);
    q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
}

void hmailton_product(float a[4], float b[4],float result[4]){

    float temp[4];
    temp[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    temp[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    temp[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    temp[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    result = temp;
}

void orientation_gyro(float gyro[4] /*the measurement quaternion from the gyro*/, float orientation[4]/* this is the previous estimated orientation !!!I don't change this value while I update the estimation variables */, float return_q[4] /* This is the returned orientation from the gyro*/){
    float temp[4] = {0.5f*orientation[0],0.5f*orientation[1],0.5f*orientation[2],0.5f*orientation[3]};
    hmailton_product(temp,gyro,gyro);
    temp[0] = gyro[0]*delta_t; temp[1] = gyro[1]*delta_t; temp[2] = gyro[2]*delta_t; temp[3] = gyro[3]*delta_t;
    return_q[0] = orientation[0]+temp[0];return_q[1] = orientation[1]+temp[1];return_q[2] = orientation[2]+temp[2];return_q[3] = orientation[3]+temp[3];
}

void calculate_magnetic_field(float magneto[4] /*the measurement quaternion from the magnetometer*/, float orientation[4] /*this is the previous estimated orientation !!!I don't change this value while I update the estimation variables*/, float return_q[4] /*This is the returned quaternion for the magnetic field quaternion*/){ //Madgwick paper Figure 3 Group 1
    float conj_orientation[4] = {orientation[0],-orientation[1],-orientation[2],-orientation[3]};
    float temp[4];
    hamilton_product(orientation,magneto,temp);
    hamilton_product(temp,conj_orientation,temp);
    return_q[0] = 0; return_q[1] = sqrt(temp[1]*temp[1]+temp[2]*temp[2]); return_q[2] = 0; return_q[3] = temp[3];
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