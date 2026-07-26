#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "shm_cpp/srv/set_spring.hpp"


using namespace std::chrono_literals;

class Position_Publisher : public rclcpp::Node{
public:
    Position_Publisher() : Node("position_publisher"){
        k = this->declare_parameter<double>("k", 10.0);
        b = this->declare_parameter<double>("b", 0.1);
        mass = this->declare_parameter<double>("mass", 1.0);
        pos = 50.0;
        velocity = 20.0;
        dt = 0.1;
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("mass_position", 10);
        publisher_KE = this->create_publisher<std_msgs::msg::Float64>("Kinetic_Energy", 10);
        publisher_PE = this->create_publisher<std_msgs::msg::Float64>("Potential_Energy", 10);
        RCLCPP_INFO(this->get_logger(), "Publishing the position of the sphere");

        timer_ = this->create_wall_timer(
        100ms, std::bind(&Position_Publisher::timer_callback, this));
    }

private:
    void timer_callback(){
        spring_force = -k * pos;
        damp_force = -b * velocity;
        acc = (spring_force + damp_force) / mass;

        velocity += dt * acc;
        pos += dt * velocity;

        ke = (mass*velocity*velocity)/2;
        pe = (k*pos*pos)/2;
        
        std_msgs::msg::Float64 msg;
        msg.data = pos;
        publisher_->publish(msg);
        msg.data = ke;
        publisher_KE->publish(msg);
        msg.data = pe;
        publisher_PE->publish(msg);
    }
    void change_param_service(){
        
    }
    double k, b, mass;
    double pos, velocity;
    double spring_force, damp_force, acc;
    double dt;
    double ke,pe;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_KE;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_PE;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Position_Publisher>());
    rclcpp::shutdown();
    return 0;
}
