#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class AINode : public rclcpp::Node {
	public :
		AINode() : Node("ai_node") {

			publisher_ = this->create_publisher<std_msgs::msg::String>("AI2VCU",10);
			subscription_ = this->create_subscription<std_msgs::msg::String>("VCU2AI" , 10 , std::bind(&AINode::command_callback, this, std::placeholders::_1));
		       timer_ = this->create_wall_timer(500ms, std::bind(&AINode::send_heartbeat, this));
	       RCLCPP_INFO(this->get_logger(), "AI Node Started. Sending heartbeats...");
  }	
	 private:
  void send_heartbeat() {
    auto message = std_msgs::msg::String();
    message.data = "ALIVE";
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Sent Heartbeat: '%s'", message.data.c_str());
  }

  void command_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received from VCU: '%s'", msg->data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AINode>());
  rclcpp::shutdown();
  return 0;
}

