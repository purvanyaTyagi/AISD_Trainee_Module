#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

using std::placeholders::_1;

class MassSpringNode : public rclcpp::Node
{
public:
    MassSpringNode() : Node("mass_spring_node")
    {
        // Publisher for RViz markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "visualization_marker", 10);

        // Physics parameters
        k_ = 5.0;   // spring constant
        b_ = 0.05;   // damping
        m_ = 1.0;   // mass

        // Initial conditions
        x_ = 2.0;   // position
        v_ = 4;   // velocity

        dt_ = 0.1; // time step

        // Timer to update system
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&MassSpringNode::update_system, this));
    }

private:
    void update_system()
    {
        x_ += 0.05;
        // Calculate forces
        double spring_force = -k_ * x_;
        double damping_force = -b_ * v_;

        // Net force and acceleration
        double force = spring_force + damping_force;
        double acceleration = force / m_;

        // Euler integration
        v_ = v_ + acceleration * dt_;
        x_ = x_ + v_ * dt_;

        // Publish markers
        publish_fixed_point();
        publish_mass();
        publish_spring();
    }

    void publish_fixed_point()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();

        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        marker.id = 0;

        marker_pub_->publish(marker);
    }

    void publish_mass()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();

        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.pose.position.x = x_;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        marker.id = 1;

        marker_pub_->publish(marker);
    }

    void publish_spring()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();

        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.1;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        geometry_msgs::msg::Point start;
        start.x = 0.0;
        start.y = 0.0;
        start.z = 0.0;

        geometry_msgs::msg::Point end;
        end.x = x_;
        end.y = 0.0;
        end.z = 0.0;

        marker.points.push_back(start);
        marker.points.push_back(end);

        marker.id = 2;

        marker_pub_->publish(marker);
    }

    // ROS objects
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Physics variables
    double k_, b_, m_;
    double x_, v_, dt_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MassSpringNode>());
    rclcpp::shutdown();
    return 0;
}
