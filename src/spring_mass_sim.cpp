#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

class SpringMassSim : public rclcpp::Node
{
public:
  SpringMassSim() : Node("spring_mass_sim")
  {
    marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);

    ke_pub_ = create_publisher<std_msgs::msg::Float64>("/energy/kinetic", 10);
    pe_pub_ = create_publisher<std_msgs::msg::Float64>("/energy/potential", 10);

    timer_ = create_wall_timer(10ms, std::bind(&SpringMassSim::update, this));
  }

private:
  // -------- Physical parameters --------
  double m = 2.0;
  double k = 6.0;
  double b = 0.6;

  // -------- State --------
  double x = -5.0;   // initial displacement (negative extreme)
  double v = 0.0;
  double dt = 0.01;

  // -------- Geometry --------
  double ball_radius = 0.08;

  // -------- ROS --------
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ke_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pe_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void update()
  {
    // -------- Physics (spring-mass-damper) --------
    double a = (-k * x - b * v) / m;
    v += a * dt;
    x += v * dt;

    // -------- Energy (Checkpoint 2) --------
    std_msgs::msg::Float64 ke, pe;
    ke.data = 0.5 * m * v * v;
    pe.data = 0.5 * k * x * x;
    ke_pub_->publish(ke);
    pe_pub_->publish(pe);

    visualization_msgs::msg::MarkerArray markers;

    // ================= MEAN POSITION =================
    visualization_msgs::msg::Marker mean;
    mean.header.frame_id = "world";
    mean.header.stamp = now();
    mean.ns = "mean";
    mean.id = 1;
    mean.type = visualization_msgs::msg::Marker::CUBE;
    mean.action = visualization_msgs::msg::Marker::ADD;
    mean.pose.position.x = 0.0;
    mean.pose.position.y = 0.0;
    mean.pose.position.z = ball_radius;
    mean.scale.x = mean.scale.y = mean.scale.z = 0.025;
    mean.color.r = 0.0;
    mean.color.g = 1.0;
    mean.color.b = 0.0;
    mean.color.a = 1.0;
    markers.markers.push_back(mean);

    // ================= BALL =================
    visualization_msgs::msg::Marker mass;
    mass.header.frame_id = "world";
    mass.header.stamp = now();
    mass.ns = "mass";
    mass.id = 2;
    mass.type = visualization_msgs::msg::Marker::SPHERE;
    mass.action = visualization_msgs::msg::Marker::ADD;
    mass.pose.position.x = x;
    mass.pose.position.y = 0.0;
    mass.pose.position.z = ball_radius;
    mass.scale.x = mass.scale.y = mass.scale.z = 0.16;
    mass.color.r = 1.0;
    mass.color.g = 0.0;
    mass.color.b = 0.0;
    mass.color.a = 1.0;
    markers.markers.push_back(mass);

    // ================= SPRING =================
    visualization_msgs::msg::Marker spring;
    spring.header.frame_id = "world";
    spring.header.stamp = now();
    spring.ns = "spring";
    spring.id = 3;
    spring.type = visualization_msgs::msg::Marker::LINE_STRIP;
    spring.action = visualization_msgs::msg::Marker::ADD;
    spring.scale.x = 0.015;
    spring.color.r = 1.0;
    spring.color.g = 1.0;
    spring.color.b = 0.0;
    spring.color.a = 1.0;

    geometry_msgs::msg::Point p1, p2;
    p1.x = 0.0;
    p1.y = 0.0;
    p1.z = ball_radius;

    p2.x = x;
    p2.y = 0.0;
    p2.z = ball_radius;

    spring.points.push_back(p1);
    spring.points.push_back(p2);
    markers.markers.push_back(spring);

    marker_pub_->publish(markers);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpringMassSim>());
  rclcpp::shutdown();
  return 0;
}
