#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

class SHM_Visualizer : public rclcpp::Node
{
public:
  SHM_Visualizer() : Node("shm_visualizer")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/mass_position", 10, std::bind(&SHM_Visualizer::topic_callback, this, _1));

    marker_pub_origin_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/visualization_marker_origin", 10);

    marker_pub_spring_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/visualization_marker_spring", 10);


    marker_pub_ball_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/visualization_marker_ball", 10);
      
    RCLCPP_INFO(this->get_logger(), "Visualizer Node Started");
  }

private:
  void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    double current_x = msg->data;

    publish_marker(visualization_msgs::msg::Marker::SPHERE, 1, 
                   current_x, 0.0, 0.0,  
                   1.0, 0.0, 0.0,         
                   1.0);                  

    //publish_marker(visualization_msgs::msg::Marker::CUBE, 2, 
                   // 0.0, 0.0, 0.0,       
                   // 0.0, 0.0, 1.0,       
                    //1.0);

    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = this->now();
    line_strip.ns = "shm_shapes";
    line_strip.id = 3;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.scale.x = 0.1; 
    line_strip.color.a = 1.0;
    line_strip.color.b = 0.0; 

    geometry_msgs::msg::Point p1, p2;
    p1.x = 0.0; p1.y = 0.0; p1.z = 0.0; 
    p2.x = current_x; p2.y = 0.0; p2.z = 0.0; 
    
    line_strip.points.push_back(p1);
    line_strip.points.push_back(p2);
    
    marker_pub_spring_->publish(line_strip);
  }

  void publish_marker(int type, int id, double x, double y, double z, double r, double g, double b, double scale)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map"; 
    marker.header.stamp = this->now();
    marker.ns = "shm_shapes";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    
    marker.color.a = 1.0; 
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    
    marker_pub_ball_->publish(marker);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_origin_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_spring_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_ball_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SHM_Visualizer>());
  rclcpp::shutdown();
  return 0;
}