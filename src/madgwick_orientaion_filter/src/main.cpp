#include "rclcpp/rclcpp.hpp"
#include "dependencies/msg/imu.hpp"
#include "dependencies/msg/vector.hpp"
#include <cmath>

using std::placeholders::_1;

float delta_t = 0.2;
float b[2] = {1,0}; 
float jac[6][4];
float obj_func[6];
float orientation[4] = {1,0,0,0};
float gyro_ori[4], accel_ori[4];
float beta = sqrt(3.0f / 4.0f) * (3.14159265358979 * (5.0f / 180.0f));
float zeta = sqrt(3.0f / 4.0f) * (3.14159265358979 * (0.2f / 180.0f));
float yaw, pitch, roll;

void normilize_quaternion(float q[4]){
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
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

void orientation_gyro(float gyro[4] /*the measurement quaternion from the gyro*/, float return_q[4] /* This is the returned orientation from the gyro*/){
    float temp[4] = {0.5f*orientation[0],0.5f*orientation[1],0.5f*orientation[2],0.5f*orientation[3]};
    hamilton_product(temp,gyro,gyro);
    temp[0] = gyro[0]*delta_t; temp[1] = gyro[1]*delta_t; temp[2] = gyro[2]*delta_t; temp[3] = gyro[3]*delta_t;
    return_q[0] = orientation[0]+temp[0];return_q[1] = orientation[1]+temp[1];return_q[2] = orientation[2]+temp[2];return_q[3] = orientation[3]+temp[3];
}

void calculate_magnetic_field(float magneto[4] /*the measurement quaternion from the magnetometer*/, float return_q[2] /*This is the returned quaternion for the magnetic field quaternion*/){ //Madgwick paper Figure 3 Group 1
    float conj_orientation[4] = {orientation[0],-orientation[1],-orientation[2],-orientation[3]};
    float temp[4];
    hamilton_product(orientation,magneto,temp);
    hamilton_product(temp,conj_orientation,temp);
    return_q[0] = sqrt(temp[1]*temp[1]+temp[2]*temp[2]); return_q[1] = temp[3];
}

void objective_function(float accel[4], float mag[4]){
    //accelerometer objective function
    obj_func[0] = 2*(orientation[1]*orientation[3]-orientation[0]*orientation[2]) - accel[1];
    obj_func[1] = 2*(orientation[0]*orientation[1]+orientation[3]*orientation[4]) - accel[2];
    obj_func[2] = 2*(0.5f-orientation[1]*orientation[1]-orientation[2]*orientation[2]) - accel[3];

    //magnetometer objective function
    obj_func[3] = 2*b[0]*(0.5f-orientation[2]*orientation[2]-orientation[3]*orientation[3]) + 2*b[1]*(orientation[1]*orientation[3]-orientation[0]*orientation[2]) - mag[1];
    obj_func[4] = 2*b[0]*(orientation[1]*orientation[2]-orientation[0]*orientation[3]) + 2*b[1]*(orientation[0]*orientation[1]+orientation[2]*orientation[3]) - mag[2];
    obj_func[5] = 2*b[0]*(orientation[0]*orientation[2]+orientation[1]*orientation[3]) + 2*b[1]*(0.5f-orientation[1]*orientation[1]-orientation[2]*orientation[2]) - mag[3];
}

void jacobian(){
    //accelerometer jacobian
    jac[0][0] = -2*orientation[2]; jac[0][1] = 2*orientation[3]; jac[0][2] = -2*orientation[0]; jac[0][3] = 2*orientation[1];
    jac[1][0] = 2*orientation[1]; jac[1][1] = 2*orientation[0]; jac[1][2] = 2*orientation[3]; jac[1][3] = 2*orientation[2];
    jac[2][0] = 0; jac[2][1] = -4*orientation[1]; jac[2][2] = -4*orientation[2]; jac[2][3] = 0;

    //magnetometer jacobian
    jac[3][0] = -2*b[1]*orientation[2]; jac[3][1] = 2*b[1]*orientation[3]; jac[3][2] = -4*b[0]*orientation[2]-2*b[1]*orientation[0]; jac[3][3] = -4*b[0]*orientation[3]+2*b[1]*orientation[1];
    jac[4][0] = -2*b[0]*orientation[3]+2*b[1]*orientation[1]; jac[4][1] = 2*b[0]*orientation[2]+2*b[1]*orientation[0]; jac[4][2] = 2*b[0]*orientation[1]+2*b[1]*orientation[3]; jac[4][3] = -2*b[0]*orientation[0]+2*b[1]*orientation[2];
    jac[5][0] = 2*b[0]*orientation[2]; jac[5][1] = 2*b[0]*orientation[3] - 4*b[1]*orientation[1]; jac[5][2] = 2*b[0]*orientation[0] - 4*b[1]*orientation[2]; jac[5][3] = 2*b[0]*orientation[1];

}



class madgwick_orientation : public rclcpp::Node{
    public:
        madgwick_orientation() : Node("madgwick"){
            subscription_ = this->create_subscription<dependencies::msg::Imu>(
                "imu_data_topic",
                10,
                std::bind(&madgwick_orientation::topic_callback, this, _1)
            );
            
            publisher_ = this->create_publisher<dependencies::msg::Vector>(
                "orientation_topic",
                10
            );
            RCLCPP_INFO(this->get_logger(), "Lidar listener node has started.");

        }
    
    private:
    void topic_callback(const dependencies::msg::Imu::SharedPtr msg) const{
        float gyro[4] = {0,msg -> gyro[0], msg -> gyro[1], msg -> gyro[2]};
        float accel[4] = {0,msg -> acceleration[0],msg -> acceleration[1],msg -> acceleration[2]};
        float magnet[4] = {0,msg -> magnet[0],msg -> magnet[1],msg -> magnet[2]};

       

        normilize_quaternion(gyro);
        normilize_quaternion(accel);
        normilize_quaternion(magnet);

        objective_function(accel, magnet);
        jacobian();

        accel_ori[0] = jac[1][0] * obj_func[1] + jac[0][0] * obj_func[0] + jac[3][0] * obj_func[3] + jac[4][0] * obj_func[4] + jac[5][0] * obj_func[5];
        accel_ori[1] = jac[0][1] * obj_func[0] + jac[1][1] * obj_func[1] + jac[2][1] * obj_func[2] + jac[3][1] * obj_func[3] + jac[4][1] * obj_func[4] + jac[5][1] * obj_func[5];
        accel_ori[2] = jac[1][2] * obj_func[1] + jac[2][2] * obj_func[2] + jac[0][2] * obj_func[0] + jac[3][2] * obj_func[3] + jac[4][2] * obj_func[4] + jac[5][2] * obj_func[5];
        accel_ori[3] = jac[0][3] * obj_func[0] + jac[1][3] * obj_func[1] + jac[3][3] * obj_func[3] + jac[4][3] * obj_func[4] + jac[5][3] * obj_func[5];

        normilize_quaternion(accel_ori);

        float temp[4] = {2*orientation[0],2*orientation[1],2*orientation[2],2*orientation[3]};

        float gyro_error[4];
        hamilton_product(temp, accel_ori, gyro_error);
        float gyro_bias[3] = {gyro_error[1] * delta_t * zeta, gyro_error[2] * delta_t * zeta, gyro_error[3] * delta_t * zeta};
        gyro[1] -= gyro_bias[0];
        gyro[2] -= gyro_bias[1];
        gyro[3] -= gyro_bias[2];

        orientation_gyro(gyro,gyro_ori);

        orientation[0] += (gyro_ori[0] - (beta* accel_ori[0])) * delta_t;
        orientation[1] += (gyro_ori[1] - (beta* accel_ori[1])) * delta_t;
        orientation[2] += (gyro_ori[2] - (beta* accel_ori[2])) * delta_t;
        orientation[3] += (gyro_ori[3] - (beta* accel_ori[3])) * delta_t;


        normilize_quaternion(orientation);

        calculate_magnetic_field(magnet, b);

        dependencies::msg::Vector orientation_vector;

        orientation_vector.vector[0] = atan2(2*orientation[1]*orientation[2] - 2*orientation[0]*orientation[3], 2*(orientation[0]*orientation[0]+orientation[1]*orientation[1])-1); // Yaw
        orientation_vector.vector[1] = -asin(2*orientation[1]*orientation[3] + 2*orientation[0]*orientation[2]); // Pitch
        orientation_vector.vector[2] = atan2(2*orientation[2]*orientation[3] - 2*orientation[0]*orientation[1], 2*(orientation[0]*orientation[0]+orientation[3]*orientation[3])-1); //Roll
        
        std::cout<<orientation_vector.vector[0]<<" "<<orientation_vector.vector[1]<<" "<<orientation_vector.vector[2]<<" "<<std::endl;

        publisher_ -> publish(orientation_vector);
    }

    rclcpp::Subscription<dependencies::msg::Imu>::SharedPtr subscription_;
    rclcpp::Publisher<dependencies::msg::Vector>::SharedPtr publisher_;

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<madgwick_orientation>());
    rclcpp::shutdown();

    return 0;
}