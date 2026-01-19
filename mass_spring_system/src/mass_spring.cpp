#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class MassSpringNode : public rclcpp::Node
{
public:
  MassSpringNode() : Node("mass_spring_node")
  {
    publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "visualization_marker", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&MassSpringNode::update, this));
  }

private:
  double x = 4.0;     // position
  double v = 0.0;     // velocity
  double k = 10.0;     // spring constant
  double b = 0.05;     // damping
  double m = 1.0;     // mass
  double dt = 0.05;   // time step

  void update()
  {
    double F_spring = -k * x;
    double F_damp = -b * v;
    double a = (F_spring + F_damp) / m;

    v = v + a * dt;
    x = x + v * dt;

    visualization_msgs::msg::Marker marker;
marker.header.frame_id = "map";
marker.header.stamp = this->now();

marker.ns = "mass";
marker.id = 0;
marker.type = visualization_msgs::msg::Marker::SPHERE;
marker.action = visualization_msgs::msg::Marker::ADD;

marker.pose.position.x = x;
marker.pose.position.y = 0.0;
marker.pose.position.z = 0.0;

marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;

marker.scale.x = 0.3;
marker.scale.y = 0.3;
marker.scale.z = 0.3;

marker.color.r = 1.0;
marker.color.g = 0.0;
marker.color.b = 0.0;
marker.color.a = 1.0;

publisher_->publish(marker);


  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MassSpringNode>());
  rclcpp::shutdown();
  return 0;
}

