#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class DynamicJointPublisher : public rclcpp::Node
{
public:
  DynamicJointPublisher()
  : Node("dynamic_joint_publisher")
  {
    // Create a publisher on the topic /joint_states
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Create a timer that fires every 33ms (approx 30 Hz)
    timer_ = this->create_wall_timer(
      33ms, std::bind(&DynamicJointPublisher::timer_callback, this));
      
    angle_ = 0.0;
    RCLCPP_INFO(this->get_logger(), "Dynamic Joint Publisher Started! Watch RViz.");
  }

private:
  void timer_callback()
  {
    auto msg = sensor_msgs::msg::JointState();
    
    // 1. Header Stamp (Current Time)
    msg.header.stamp = this->get_clock()->now();

    // 2. Joint Names (MUST match your URDF exactly)
    msg.name = {
        "front_left_wheel_joint", 
        "front_right_wheel_joint", 
        "rear_left_wheel_joint", 
        "rear_right_wheel_joint"
    };

    // 3. Update Position (Spin the wheels)
    // We update the angle continuously to simulate rotation
    msg.position = {angle_, angle_, angle_, angle_};

    // Publish the message
    publisher_->publish(msg);

    // Increment angle for the next frame (Controls speed)
    angle_ += 0.1; 
    
    // Reset to keep numbers small (optional, but good practice)
    if (angle_ > 2 * M_PI) {
        angle_ -= 2 * M_PI;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  double angle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicJointPublisher>());
  rclcpp::shutdown();
  return 0;
}
