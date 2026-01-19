#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class MassSpringSim : public rclcpp::Node
{
public:
    MassSpringSim() : Node("aisd_mass_spring")
    {
        // --- 1. Physics Parameters ---
        mass_ = 1.0;
        k_ = 5.0;
        b_ = 0.2; // Low damping so we can see oscillations
        dt_ = 0.02;

        // Checkpoint 2: Start at REST (0.0). We wait for the service to kick it.
        position_ = 0.0; 
        velocity_ = 0.0; 

        // --- 2. Publishers ---
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);
        
        // Energy Publishers (Checkpoint 2)
        ke_pub_ = this->create_publisher<std_msgs::msg::Float32>("kinetic_energy", 10);
        pe_pub_ = this->create_publisher<std_msgs::msg::Float32>("potential_energy", 10);
        total_e_pub_ = this->create_publisher<std_msgs::msg::Float32>("total_energy", 10);

        // --- 3. Service (Checkpoint 2) ---
        // Service to "kick" the mass
        force_service_ = this->create_service<std_srvs::srv::Trigger>(
            "apply_force",
            std::bind(&MassSpringSim::apply_force_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // --- 4. Timer ---
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_), 
            std::bind(&MassSpringSim::update_physics, this));

        RCLCPP_INFO(this->get_logger(), "Checkpoint 2 Ready. System is at rest. Call /apply_force to start!");
    }

private:
    // Service Callback
    void apply_force_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // Unused
        
        // Apply Impulse: Add instant velocity
        velocity_ += 3.0; 

        response->success = true;
        response->message = "Kick applied! Velocity += 3.0 m/s";
        RCLCPP_INFO(this->get_logger(), "Force Applied! System perturbed.");
    }

    void update_physics()
    {
        // --- Physics Engine ---
        double f_spring = -k_ * position_;
        double f_damping = -b_ * velocity_;
        double f_net = f_spring + f_damping;

        double acceleration = f_net / mass_;
        velocity_ += acceleration * dt_;
        position_ += velocity_ * dt_;

        // --- Energy Calculation (Checkpoint 2) ---
        // KE = 1/2 * m * v^2
        double ke = 0.5 * mass_ * std::pow(velocity_, 2);
        
        // PE = 1/2 * k * x^2
        double pe = 0.5 * k_ * std::pow(position_, 2);
        
        // Total = KE + PE
        double total_e = ke + pe;

        // Publish Energy Data
        std_msgs::msg::Float32 msg;
        
        msg.data = ke;
        ke_pub_->publish(msg);
        
        msg.data = pe;
        pe_pub_->publish(msg);
        
        msg.data = total_e;
        total_e_pub_->publish(msg);

        // Update Visuals
        publish_markers();
    }

    void publish_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        auto time = this->get_clock()->now();

        // 1. Wall
        visualization_msgs::msg::Marker wall;
        wall.header.frame_id = "map"; wall.header.stamp = time; wall.id = 0;
        wall.type = visualization_msgs::msg::Marker::CUBE; wall.action = 0;
        wall.pose.position.x = -3.0; 
        wall.scale.x = 0.2; wall.scale.y = 2.0; wall.scale.z = 2.0;
        wall.color.a = 1.0; wall.color.r = 0.5; wall.color.g = 0.5; wall.color.b = 0.5;
        marker_array.markers.push_back(wall);

        // 2. Mass
        visualization_msgs::msg::Marker mass;
        mass.header.frame_id = "map"; mass.header.stamp = time; mass.id = 1;
        mass.type = visualization_msgs::msg::Marker::SPHERE; mass.action = 0;
        mass.pose.position.x = position_; 
        mass.scale.x = 0.5; mass.scale.y = 0.5; mass.scale.z = 0.5;
        mass.color.a = 1.0; mass.color.r = 1.0; // Red
        marker_array.markers.push_back(mass);

        // 3. Spring
        visualization_msgs::msg::Marker spring;
        spring.header.frame_id = "map"; spring.header.stamp = time; spring.id = 2;
        spring.type = visualization_msgs::msg::Marker::LINE_STRIP; spring.action = 0;
        spring.scale.x = 0.05; spring.color.a = 1.0; spring.color.b = 1.0; // Blue
        geometry_msgs::msg::Point p1, p2;
        p1.x = -3.0; p2.x = position_;
        spring.points.push_back(p1); spring.points.push_back(p2);
        marker_array.markers.push_back(spring);

        marker_pub_->publish(marker_array);
    }

    double mass_, k_, b_, dt_;
    double position_, velocity_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // New Pointers for Checkpoint 2
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ke_pub_, pe_pub_, total_e_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr force_service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MassSpringSim>());
    rclcpp::shutdown();
    return 0;
}
