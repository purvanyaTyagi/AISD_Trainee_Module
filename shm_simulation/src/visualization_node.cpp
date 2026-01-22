#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

class SHM_Visualizer : public rclcpp::Node
{
public:
    SHM_Visualizer() : Node("sim_visualizer")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mass_position", 10, 
            std::bind(&SHM_Visualizer::topic_callback, this, _1));
        
        marker_pub_mass = this->create_publisher<visualization_msgs::msg::Marker>(
            "/visualization_marker_mass", 10);
        
        marker_pub_spring = this->create_publisher<visualization_msgs::msg::Marker>(
            "/visualization_marker_spring", 10);
        
        marker_pub_cube = this->create_publisher<visualization_msgs::msg::Marker>(
            "/visualization_marker_cube", 10);
        
        current_x = 0.0;
        
        publish_cube();
        
        RCLCPP_INFO(this->get_logger(), "Visualizer Node Started");
    }

private:
    void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        current_x = msg->data;
        publish_mass(current_x);
        publish_spring(current_x);
    }
    
    void publish_mass(double x)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "shm";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = x;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        marker_pub_mass->publish(marker);
    }
    
    void publish_spring(double x)
    {
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = this->now();
        line_strip.ns = "shm";
        line_strip.id = 1;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        
        line_strip.scale.x = 0.1;
        
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;
        
        geometry_msgs::msg::Point p1, p2;
        p1.x = 0.0; p1.y = 0.0 ; p1.z = 0.0;
        p2.x = current_x; p2.y = 0.0; p2.z = 0.0;
        
        line_strip.points.push_back(p1);
        line_strip.points.push_back(p2);
        
        marker_pub_spring->publish(line_strip);
    }
    
    void publish_cube()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "shm";
        marker.id = 2;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        
        marker.lifetime = rclcpp::Duration(0, 0);
        
        marker_pub_cube->publish(marker);
    }
    
    double current_x;
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_mass;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_spring;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_cube;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SHM_Visualizer>());
    rclcpp::shutdown();
    return 0;
}