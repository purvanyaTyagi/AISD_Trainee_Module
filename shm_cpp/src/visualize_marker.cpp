#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"


using namespace std::chrono_literals;

class Marker_Publisher : public rclcpp::Node
{
  public:
    Marker_Publisher() : Node("visualize_marker"){
        
      publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_data",10);
      subscriber_ = this->create_subscription<std_msgs::msg::Float64>("mass_position",10, std::bind(&Marker_Publisher::timer_callback, this, std::placeholders::_1));
    }

  private:
    void timer_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
      visualization_msgs::msg::Marker marker1, marker2, marker3;

      marker1.header.frame_id = "map";
      marker1.header.stamp = this->get_clock()->now();
      marker1.ns = "basic_shapes";
      marker1.id = 0;
      marker1.type = visualization_msgs::msg::Marker::CUBE;
      marker1.action = visualization_msgs::msg::Marker::ADD;
      marker1.pose.orientation.w = 1.0;
      marker1.scale.x = 0.2;
      marker1.scale.y = 0.2;
      marker1.scale.z = 0.2;
      marker1.color.b = 1.0;
      marker1.color.a = 1.0;

      publisher_->publish(marker1);

      marker2 = marker1;
      marker2.id = 1;
      marker2.type = visualization_msgs::msg::Marker::SPHERE;
      marker2.pose.position.x = msg->data;
      marker2.scale.x = 0.5;
      marker2.scale.y = 0.5;
      marker2.scale.z = 0.5;
      marker2.color.r = 1.0;
      marker2.color.b = 0.0;

      publisher_->publish(marker2);

      marker3.header.frame_id = "map";
      marker3.header.stamp = this->get_clock()->now();
      marker3.ns = "basic_shapes";
      marker3.id = 2;
      marker3.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker3.action = visualization_msgs::msg::Marker::ADD;
      marker3.pose.orientation.w = 1.0;
      marker3.scale.x = 0.1;
      marker3.color.g = 1.0;
      marker3.color.a = 1.0;

      geometry_msgs::msg::Point p1, p2;
      p1.x = 0.0; p1.y = 0.0;
      p2.x = msg->data; p2.y = 0.0;

      marker3.points.push_back(p1);
      marker3.points.push_back(p2);

      publisher_->publish(marker3);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Marker_Publisher>());
  rclcpp::shutdown();
  return 0;
}