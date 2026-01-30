#include "rclcpp/rclcpp.hpp"
#include "my_robot_description/srv/set_velocity.hpp" 
#include "geometry_msgs/msg/twist.hpp" // Required for velocity commands
#include <memory>

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("robot_controller") {
        // 1. Create a Publisher to send velocity to the robot motors/simulation
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 2. Create the Service Server
        service_ = this->create_service<my_robot_description::srv::SetVelocity>(
            "set_robot_velocity",
            std::bind(&RobotController::handle_velocity_request, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Robot Controller Service is ready and publishing to /cmd_vel.");
    }

private:
    void handle_velocity_request(
        const std::shared_ptr<my_robot_description::srv::SetVelocity::Request> request,
        std::shared_ptr<my_robot_description::srv::SetVelocity::Response> response) 
    {
        // Log the received values to the terminal
        RCLCPP_INFO(this->get_logger(), "Executing Command: Linear=%.2f, Angular=%.2f", 
                    request->linear_x, request->angular_z);

        // 3. Create a Twist message and fill it with the service request data
        auto move_msg = geometry_msgs::msg::Twist();
        move_msg.linear.x = request->linear_x;
        move_msg.angular.z = request->angular_z;

        // 4. Publish the message to actually move the robot
        publisher_->publish(move_msg);

        // Send confirmation back to the teleop_node
        response->success = true;
        response->message = "Velocity command published to /cmd_vel.";
    }

    // Member variables
    rclcpp::Service<my_robot_description::srv::SetVelocity>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}