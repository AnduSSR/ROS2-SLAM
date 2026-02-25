#include "rclcpp/rclcpp.hpp"
#include "dependencies/msg/imu.hpp"
#include <chrono>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

using namespace std::chrono_literals;


using std::placeholders::_1;

class imu_data : public rclcpp::Node{
    public:
        imu_data() : Node("imu"){
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("madgwick_orientaion_filter");
            std::string file_path = package_share_directory + "/include/IMU_Simulation.txt";

            inFile.open(file_path);


            publisher_ = this->create_publisher<dependencies::msg::Imu>(
                "imu_data_topic",
                10
            );
            RCLCPP_INFO(this->get_logger(), "Lidar listener node has started.");

            timer_ = this->create_wall_timer(1s, std::bind(&imu_data::topic_callback, this));
            
        }
    
    private:
    void topic_callback(){
        //std::cout<<"Started Reading" <<std::endl;
        if (inFile.eof()) {
            RCLCPP_INFO(this->get_logger(), "End of file reached. Stopping publisher...");
            inFile.close();
            timer_->cancel();
            rclcpp::shutdown();
            return;
        }
        
        std::string msg, data[11];
        std::getline(inFile, msg);
        std::cout<<"Found line "<< msg.c_str() <<std::endl;

        int count = 0;
        while(msg != ""){
            int pos = msg.rfind(',');
            if(pos != -1){
                data[count] = msg.substr(pos+1,msg.length());
                msg.erase(pos,msg.length()-pos);
            }else{
                std::cout<<count<<std::endl;
                data[count] = msg;
                msg.erase(0,msg.length()-pos);
            }
            count++;
        }

        std::cout<<data[0]<<std::endl;

        dependencies::msg::Imu imu_data;
        imu_data.magnet = {std::stof(data[2]),std::stof(data[1]),std::stof(data[0])};
        imu_data.gyro = {std::stof(data[5]),std::stof(data[4]),std::stof(data[3])};
        imu_data.acceleration = {std::stof(data[8]),std::stof(data[7]),std::stof(data[6])};

        //std::cout<<imu_data.acceleration[0]<<" "<<imu_data.acceleration[1]<<" "<<imu_data.acceleration[2]<<std::endl;
       // std::cout<<imu_data.gyro[0]<<" "<<imu_data.gyro[1]<<" "<<imu_data.gyro[2]<<std::endl;
        //std::cout<<imu_data.magnet[0]<<" "<<imu_data.magnet[1]<<" "<<imu_data.magnet[2]<<std::endl;
        publisher_ -> publish(imu_data);
    }

        std::ifstream inFile;

        rclcpp::Publisher<dependencies::msg::Imu>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imu_data>());
    rclcpp::shutdown();

    return 0;
}