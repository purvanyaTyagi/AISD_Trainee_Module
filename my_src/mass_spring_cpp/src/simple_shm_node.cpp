#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>

using namespace std::chrono_literals;

class SimpleSHMNode : public rclcpp::Node {
public:
    SimpleSHMNode() : Node("simple_shm_node") {
        // --- Publishers ---
        // 1. Position (Existing)
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("mass_position", 10);
        
        // 2. Energy Publishers (NEW for plotting)
        pub_ke_ = this->create_publisher<std_msgs::msg::Float64>("kinetic_energy", 10);
        pub_pe_ = this->create_publisher<std_msgs::msg::Float64>("potential_energy", 10);
        pub_total_e_ = this->create_publisher<std_msgs::msg::Float64>("total_energy", 10); // Optional: to see conservation

        // --- Physics Parameters (No Damping) ---
        mass_ = 1.0;
        k_ = 10.0;
        dt_ = 0.01; // 10ms

        position_ = 1.0; // Start at 1m displacement
        velocity_ = 0.0;

        timer_ = this->create_wall_timer(10ms, std::bind(&SimpleSHMNode::update_physics, this));
        RCLCPP_INFO(this->get_logger(), "Simple SHM Node Started (F = -kx) with Energy Publishing");
    }

private:
    void update_physics() {
        // --- 1. Physics Calculations ---
        // Force calculation: F_spring = -k * x
        double force = -k_ * position_;
        double acceleration = force / mass_;

        // Euler Integration
        velocity_ += acceleration * dt_;
        position_ += velocity_ * dt_;

        // --- 2. Energy Calculations (NEW) ---
        // Kinetic Energy: KE = 1/2 * m * v^2
        double ke = 0.5 * mass_ * (velocity_ * velocity_);

        // Potential Energy: PE = 1/2 * k * x^2
        double pe = 0.5 * k_ * (position_ * position_);

        // Total Energy (Should remain mostly constant in Simple SHM)
        double total_e = ke + pe;

        // --- 3. Publish Data ---
        std_msgs::msg::Float64 msg;
        
        // Publish Position
        msg.data = position_;
        publisher_->publish(msg);

        // Publish Kinetic Energy
        msg.data = ke;
        pub_ke_->publish(msg);

        // Publish Potential Energy
        msg.data = pe;
        pub_pe_->publish(msg);

        // Publish Total Energy
        msg.data = total_e;
        pub_total_e_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    
    // New Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ke_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pe_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_total_e_;

    rclcpp::TimerBase::SharedPtr timer_;
    double mass_, k_, position_, velocity_, dt_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSHMNode>());
    rclcpp::shutdown();
    return 0;
}
