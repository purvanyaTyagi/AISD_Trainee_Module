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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("visualisation_node")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "mass_position", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    cube_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "cube_marker", 10);
    sphere_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "sphere_marker", 10);
    line_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "line_marker", 10);
  }

private:
  void topic_callback(const geometry_msgs::msg::Point & msg)
  {
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

    mass.pose.position.x = msg.x;
    mass.pose.position.y = msg.y;
    mass.pose.position.z = msg.z;

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
    spring.points.push_back(msg);

    line_pub_->publish(spring);
  }
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sphere_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cube_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
