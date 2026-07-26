#include <cmath>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStateMover : public rclcpp::Node
{
public:
  JointStateMover() : Node("joint_state_mover"), t_(0.0)
  {
    // Change these joint names to match your URDF joint names
    joint_names_ = {
      "left_front_wheel_joint",
      "right_front_wheel_joint",
      "left_behind_wheel_joint",
      "right_behind_wheel_joint"
    };

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&JointStateMover::tick, this)
    );

    RCLCPP_INFO(this->get_logger(), "Publishing /joint_states");
  }

private:
  void tick()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->get_clock()->now();

    msg.name = joint_names_;

    // Example motion: wheels spin using a sine wave
    double angle = std::sin(t_);

    msg.position.resize(joint_names_.size());
    msg.position[0] =  angle;
    msg.position[1] = -angle;
    msg.position[2] =  angle;
    msg.position[3] = -angle;

    pub_->publish(msg);
    t_ += 0.05;
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> joint_names_;
  double t_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateMover>());
  rclcpp::shutdown();
  return 0;
}