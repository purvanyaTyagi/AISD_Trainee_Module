#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // New message type for velocity

using namespace std::chrono_literals;

class RobotDriver : public rclcpp::Node
{
public:
  RobotDriver()
  : Node("robot_driver")
  {
    // Create a publisher for velocity commands on topic 'cmd_vel'
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Set a timer to run every 500ms (2 times per second)
    timer_ = this->create_wall_timer(
      500ms, std::bind(&RobotDriver::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "Driver Node Started: Driving in circles!");
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    
    // Set Linear Velocity (Forward speed)
    message.linear.x = 0.5;  // 0.5 meters per second
    
    // Set Angular Velocity (Turning speed)
    message.angular.z = 0.5; // 0.5 radians per second
    
    // Publish the message
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDriver>());
  rclcpp::shutdown();
  return 0;
}