// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("physics_node")
  {
    x_ = 5.0;    // initial displacement
    v_ = 0.0;

    m_ = 1.0;
    k_ = 5.0;
    b_ = 0.05;

    dt_ = 0.01;

    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("mass_position", 10);
    kineticpub_ = this->create_publisher<std_msgs::msg::Float64>("kinetic_energy", 10);
    potentialpub_ = this->create_publisher<std_msgs::msg::Float64>("potential_energy", 10);
    cube_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "cube_marker", 10);
    sphere_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "sphere_marker", 10);
    line_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "line_marker", 10);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
    void timer_callback()
    {
    // 1. Forces
    double F_spring = -k_ * (-3 + x_);
    double F_damping = -b_ * v_;
    double F = F_spring + F_damping;

    // 2. Acceleration
    double a = F / m_;

    // 3. Euler integration
    v_ = v_ + a * dt_;
    x_ = x_ + v_ * dt_;

    // 4. Energy calculation
    ke_ = 0.5*m_*v_*v_;
    pe_ = 0.5*k_*(-3 + x_)*(-3 + x_);

    // 5. Publish position and energies
    geometry_msgs::msg::Point message;
    std_msgs::msg::Float64 kinetic;
    std_msgs::msg::Float64 potential;
    message.x = x_;
    message.y = 0.0;
    message.z = 0.0;
    kinetic.data = ke_;
    potential.data = pe_;

    publisher_->publish(message);
    kineticpub_->publish(kinetic);
    potentialpub_->publish(potential);

    RCLCPP_INFO(this->get_logger(), "x = %f", x_);
    RCLCPP_INFO(this->get_logger(), "\nKE = %f", ke_);
    RCLCPP_INFO(this->get_logger(), "\nPE = %f", pe_);


    // cube:
    visualization_msgs::msg::Marker fixed;
    fixed.header.frame_id = "world";
    fixed.header.stamp = this->now();
    fixed.ns = "fixed";
    fixed.id = 0;
    fixed.type = visualization_msgs::msg::Marker::CUBE;
    fixed.action = visualization_msgs::msg::Marker::ADD;

    fixed.scale.x = 0.2;
    fixed.scale.y = 0.2;
    fixed.scale.z = 0.2;

    fixed.color.r = 1.0;
    fixed.color.g = 0.0;
    fixed.color.b = 0.0;
    fixed.color.a = 1.0;

    fixed.pose.position.x = 0.0;
    fixed.pose.position.y = 0.0;
    fixed.pose.position.z = 0.0;

    cube_pub_->publish(fixed);

    //sphere:
    visualization_msgs::msg::Marker mass;
    mass.header.frame_id = "world";
    mass.header.stamp = this->now();
    mass.ns = "mass";
    mass.id = 1;
    mass.type = visualization_msgs::msg::Marker::SPHERE;
    mass.action = visualization_msgs::msg::Marker::ADD;

    mass.scale.x = 0.2;
    mass.scale.y = 0.2;
    mass.scale.z = 0.2;

    mass.color.r = 0.0;
    mass.color.g = 1.0;
    mass.color.b = 0.0;
    mass.color.a = 1.0;

    mass.pose.position.x = message.x;
    mass.pose.position.y = message.y;
    mass.pose.position.z = message.z;

    sphere_pub_->publish(mass);

    //line
    visualization_msgs::msg::Marker spring;
    spring.header.frame_id = "world";
    spring.header.stamp = this->now();
    spring.ns = "spring";
    spring.id = 2;
    spring.type = visualization_msgs::msg::Marker::LINE_STRIP;
    spring.action = visualization_msgs::msg::Marker::ADD;

    spring.scale.x = 0.05;

    spring.color.r = 0.0;
    spring.color.g = 0.0;
    spring.color.b = 1.0;
    spring.color.a = 1.0;

    geometry_msgs::msg::Point p0;
    p0.x = 0.0; p0.y = 0.0; p0.z = 0.0;

    spring.points.push_back(p0);
    spring.points.push_back(message);

    line_pub_->publish(spring);

    }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kineticpub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr potentialpub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sphere_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cube_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub_;
    double x_;   // position
    double v_;   // velocity
    double ke_;  // KE
    double pe_;   // PE

    double m_;   // mass
    double k_;   // spring constant
    double b_;   // damping coefficient

    double dt_;  // timestep

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
