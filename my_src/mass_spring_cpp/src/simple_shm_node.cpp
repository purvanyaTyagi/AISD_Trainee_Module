#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>

using namespace std::chrono_literals;

class SimpleSHMNode : public rclcpp::Node {
public:
    SimpleSHMNode() : Node("simple_shm_node") {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("mass_position", 10);
        
        // Physics Parameters (No Damping)
        mass_ = 1.0;
        k_ = 10.0;
        dt_ = 0.01; // 10ms

        position_ = 1.0; // Start at 1m displacement
        velocity_ = 0.0;

        timer_ = this->create_wall_timer(10ms, std::bind(&SimpleSHMNode::update_physics, this));
        RCLCPP_INFO(this->get_logger(), "Simple SHM Node Started (F = -kx)");
    }

private:
    void update_physics() {
        // Force calculation: F_spring = -k * x
        double force = -k_ * position_;
        double acceleration = force / mass_;

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
    double mass_, k_, position_, velocity_, dt_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSHMNode>());
    rclcpp::shutdown();
    return 0;
}
