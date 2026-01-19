#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class MassSpringSim : public rclcpp::Node
{
public:
    MassSpringSim() : Node("aisd_mass_spring")
    {
        // --- 1. Initialize Parameters ---
        // Hardcoded values for Checkpoint 1
        mass_ = 1.0;   // Mass (kg)
        k_ = 10.0;     // Spring Constant
        b_ = 0.5;      // Damping Coefficient
        dt_ = 0.02;    // Time step (50Hz)

        // Initial State:
        // We start at 2.0 so the system moves immediately (since we have no kick service yet)
        position_ = 2.0; 
        velocity_ = 0.0;

        // --- 2. Create Publishers ---
        // Publisher for RViz markers (Wall, Spring, Mass)
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);
        
        // --- 3. Create Timer ---
        // Runs the physics loop at 50Hz
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_), 
            std::bind(&MassSpringSim::update_physics, this));

        RCLCPP_INFO(this->get_logger(), "Checkpoint 1 Node Started. Mass Initial Position: %.2f", position_);
    }

private:
    void update_physics()
    {
        // --- A. Physics Calculations ---
        // F_spring = -k * x
        double f_spring = -k_ * position_;
        
        // F_damping = -b * v
        double f_damping = -b_ * velocity_;
        
        // F_net = F_spring + F_damping
        double f_net = f_spring + f_damping;

        // Newton's 2nd Law: F = ma  ->  a = F / m
        double acceleration = f_net / mass_;

        // Euler Integration
        velocity_ += acceleration * dt_;
        position_ += velocity_ * dt_;

        // --- B. Visualization ---
        publish_markers();
    }

    void publish_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        auto time = this->get_clock()->now();

        // 1. The Wall (Static cube at x = -2.0)
        visualization_msgs::msg::Marker wall;
        wall.header.frame_id = "map";
        wall.header.stamp = time;
        wall.id = 0;
        wall.type = visualization_msgs::msg::Marker::CUBE;
        wall.action = visualization_msgs::msg::Marker::ADD;
        wall.pose.position.x = -2.0;
        wall.pose.position.y = 0.0;
        wall.pose.position.z = 0.0;
        wall.scale.x = 0.2; 
        wall.scale.y = 2.0; 
        wall.scale.z = 2.0;
        wall.color.a = 1.0; 
        wall.color.r = 0.5; wall.color.g = 0.5; wall.color.b = 0.5; // Grey
        marker_array.markers.push_back(wall);

        // 2. The Mass (Sphere moving at position_)
        visualization_msgs::msg::Marker mass;
        mass.header.frame_id = "map";
        mass.header.stamp = time;
        mass.id = 1;
        mass.type = visualization_msgs::msg::Marker::SPHERE;
        mass.action = visualization_msgs::msg::Marker::ADD;
        mass.pose.position.x = position_;
        mass.pose.position.y = 0.0;
        mass.pose.position.z = 0.0;
        mass.scale.x = 0.5; 
        mass.scale.y = 0.5; 
        mass.scale.z = 0.5;
        mass.color.a = 1.0; 
        mass.color.r = 1.0; wall.color.g = 0.0; wall.color.b = 0.0; // Red
        marker_array.markers.push_back(mass);

        // 3. The Spring (Line connecting Wall to Mass)
        visualization_msgs::msg::Marker spring;
        spring.header.frame_id = "map";
        spring.header.stamp = time;
        spring.id = 2;
        spring.type = visualization_msgs::msg::Marker::LINE_STRIP;
        spring.action = visualization_msgs::msg::Marker::ADD;
        spring.scale.x = 0.05; // Line width
        spring.color.a = 1.0; 
        spring.color.r = 1.0; wall.color.g = 1.0; wall.color.b = 1.0; // White
        
        geometry_msgs::msg::Point p1, p2;
        p1.x = -2.0; p1.y = 0.0; p1.z = 0.0; // Wall position
        p2.x = position_; p2.y = 0.0; p2.z = 0.0; // Mass position
        
        spring.points.push_back(p1);
        spring.points.push_back(p2);
        marker_array.markers.push_back(spring);

        // Publish all markers
        marker_pub_->publish(marker_array);
    }

    // Member Variables
    double mass_, k_, b_, dt_;
    double position_, velocity_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MassSpringSim>());
    rclcpp::shutdown();
    return 0;
}
