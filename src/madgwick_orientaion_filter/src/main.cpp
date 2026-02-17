#include "rclcpp/rclcpp.hpp"
#include "dependencies/msg/imu.hpp"

using std::placeholders::_1;

float delta_t = 0.2;
float b[2] = {1,0};

void normilize_quaternion(float q[4]){
    float norm = sqrt(q[0]*q[0] + q[1]*q[0] + q[2]*q[0] + q[3]*q[0]);
    q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
}

void hamilton_product(float a[4], float b[4],float result[4]){

    float temp[4];
    temp[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    temp[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    temp[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    temp[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    result = temp;
}

void orientation_gyro(float gyro[4] /*the measurement quaternion from the gyro*/, float orientation[4]/* this is the previous estimated orientation !!!I don't change this value while I update the estimation variables */, float return_q[4] /* This is the returned orientation from the gyro*/){
    float temp[4] = {0.5f*orientation[0],0.5f*orientation[1],0.5f*orientation[2],0.5f*orientation[3]};
    hamilton_product(temp,gyro,gyro);
    temp[0] = gyro[0]*delta_t; temp[1] = gyro[1]*delta_t; temp[2] = gyro[2]*delta_t; temp[3] = gyro[3]*delta_t;
    return_q[0] = orientation[0]+temp[0];return_q[1] = orientation[1]+temp[1];return_q[2] = orientation[2]+temp[2];return_q[3] = orientation[3]+temp[3];
}

void calculate_magnetic_field(float magneto[4] /*the measurement quaternion from the magnetometer*/, float orientation[4] /*this is the previous estimated orientation !!!I don't change this value while I update the estimation variables*/, float return_q[2] /*This is the returned quaternion for the magnetic field quaternion*/){ //Madgwick paper Figure 3 Group 1
    float conj_orientation[4] = {orientation[0],-orientation[1],-orientation[2],-orientation[3]};
    float temp[4];
    hamilton_product(orientation,magneto,temp);
    hamilton_product(temp,conj_orientation,temp);
    return_q[0] = sqrt(temp[1]*temp[1]+temp[2]*temp[2]); return_q[1] = temp[3];
}

void objective_function(float accel[4], float mag[4], float orientation[4], float obj_func[6]){
    //accelerometer objective function
    obj_func[0] = 2*(orientation[1]*orientation[3]-orientation[0]*orientation[2]) - accel[1];
    obj_func[1] = 2*(orientation[0]*orientation[1]+orientation[3]*orientation[4]) - accel[2];
    obj_func[2] = 2*(0.5f-orientation[1]*orientation[1]-orientation[2]*orientation[2]) - accel[3];

    //magnetometer objective function
    obj_func[3] = 2*b[0]*(0.5f-orientation[2]*orientation[2]-orientation[3]*orientation[3]) + 2*b[1]*(orientation[1]*orientation[3]-orientation[0]*orientation[2]) - mag[1];
    obj_func[4] = 2*b[0]*(orientation[1]*orientation[2]-orientation[0]*orientation[3]) + 2*b[1]*(orientation[0]*orientation[1]+orientation[2]*orientation[3]) - mag[2];
    obj_func[5] = 2*b[0]*(orientation[0]*orientation[2]+orientation[1]*orientation[3]) + 2*b[1]*(0.5f-orientation[1]*orientation[1]-orientation[2]*orientation[2]) - mag[3];
}

void jacobian(float orientation[4], float jac[6][4]){
    //accelerometer jacobian
    jac[0][0] = -2*orientation[2]; jac[0][1] = 2*orientation[3]; jac[0][2] = -2*orientation[0]; jac[0][3] = 2*orientation[1];
    jac[1][0] = 2*orientation[1]; jac[1][1] = 2*orientation[0]; jac[1][2] = 2*orientation[3]; jac[1][3] = 2*orientation[2];
    jac[2][0] = 0; jac[2][1] = -4*orientation[1]; jac[2][2] = -4*orientation[2]; jac[2][3] = 0;

    //magnetometer jacobian
    jac[3][0] = -2*b[1]*orientation[2]; jac[3][1] = 2*b[1]*orientation[3]; jac[3][2] = -4*b[0]*orientation[2]-2*b[1]*orientation[0]; jac[3][3] = -4*b[0]*orientation[3]+2*b[1]*orientation[1];
    jac[4][0] = -2*b[0]*orientation[3]+2*b[1]*orientation[1]; jac[4][1] = 2*b[0]*orientation[2]+2*b[1]*orientation[0]; jac[4][2] = 2*b[0]*orientation[1]+2*b[1]*orientation[3]; jac[4][3] = -2*b[0]*orientation[0]+2*b[1]*orientation[2];
    jac[5][0] = 2*b[0]*orientation[2]; jac[5][1] = 2*b[0]*orientation[3] - 4*b[1]*orientation[1]; jac[5][2] = 2*b[0]*orientation[0] - 4*b[1]*orientation[2]; jac[5][3] = 2*b[0]*orientation[1];

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