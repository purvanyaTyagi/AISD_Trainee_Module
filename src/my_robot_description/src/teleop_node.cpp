#include "rclcpp/rclcpp.hpp"
#include "my_robot_description/srv/set_velocity.hpp"
#include <iostream>
#include <memory>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("teleop_node");
    
    // Updated with correct package namespace
    auto client = node->create_client<my_robot_description::srv::SetVelocity>("set_robot_velocity");

    while (rclcpp::ok()) {
        auto request = std::make_shared<my_robot_description::srv::SetVelocity::Request>();
        
        std::cout << "\n--- Enter Robot Commands ---" << std::endl;
        std::cout << "Linear Velocity: ";
        if (!(std::cin >> request->linear_x)) break;
        std::cout << "Angular Velocity: ";
        if (!(std::cin >> request->angular_z)) break;

        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(node->get_logger(), "Service not available. Is the controller running?");
            continue;
        }

        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Status: %s", result.get()->message.c_str());
        }
    }
    rclcpp::shutdown();
    return 0;
}