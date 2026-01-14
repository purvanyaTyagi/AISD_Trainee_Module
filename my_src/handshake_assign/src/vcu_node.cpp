#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono_literals;

class VCUNode : public rclcpp::Node {
public:
  VCUNode() : Node("vcu_node") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "AI2VCU", 10, std::bind(&VCUNode::heartbeat_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<std_msgs::msg::String>("VCU2AI", 10);
    last_heartbeat_time_ = this->now();
    health_check_timer_ = this->create_wall_timer(
      100ms, std::bind(&VCUNode::check_ai_health, this));
  }

private:
  void heartbeat_callback(const std_msgs::msg::String::SharedPtr msg) {
    last_heartbeat_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Heartbeat Received: '%s'", msg->data.c_str());
    
    auto ack_msg = std_msgs::msg::String();
    ack_msg.data = "ACKNOWLEDGE";
    publisher_->publish(ack_msg);
  }

  void check_ai_health() {
    auto current_time = this->now();
    auto time_diff = current_time - last_heartbeat_time_;

    if (time_diff.seconds() > 1.5) {
        RCLCPP_ERROR(this->get_logger(), "CRITICAL WARNING: AI Node timeout! Stopping Vehicle.");
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr health_check_timer_;
  rclcpp::Time last_heartbeat_time_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VCUNode>());
  rclcpp::shutdown();
  return 0;
}
