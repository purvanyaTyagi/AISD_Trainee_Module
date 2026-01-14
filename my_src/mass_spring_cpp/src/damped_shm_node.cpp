#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>

using namespace std::chrono_literals;

class DampedSHMNode : public rclcpp::Node {
public:
    DampedSHMNode() : Node("damped_shm_node") {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("mass_position", 10);
        
        // Physics Parameters (With Damping)
        mass_ = 1.0;
        k_ = 10.0;
        b_ = 0.5;   // Damping coefficient
        dt_ = 0.01;

        position_ = 1.0;
        velocity_ = 0.0;

        timer_ = this->create_wall_timer(10ms, std::bind(&DampedSHMNode::update_physics, this));
        RCLCPP_INFO(this->get_logger(), "Damped SHM Node Started (F = -kx - bv)");
    }

private:
    void update_physics() {
        // Force calculation: F_net = -kx - bv
        double f_spring = -k_ * position_;
        double f_damping = -b_ * velocity_;
        double acceleration = (f_spring + f_damping) / mass_;

        // Euler Integration
        velocity_ += acceleration * dt_;
        position_ += velocity_ * dt_;

        // Publish
        std_msgs::msg::Float64 msg;
        msg.data = position_;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double mass_, k_, b_, position_, velocity_, dt_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DampedSHMNode>());
    rclcpp::shutdown();
    return 0;
}
