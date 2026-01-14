#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class VisualizerNode : public rclcpp::Node {
public:
    VisualizerNode() : Node("visualizer_node"), displacement_(0.0) {

        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "mass_position",
            10,
            std::bind(&VisualizerNode::topic_callback, this, std::placeholders::_1)
        );

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "visualization_marker", 10
        );

        // Publish markers at fixed rate (20 Hz)
        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&VisualizerNode::publish_markers, this)
        );

        RCLCPP_INFO(this->get_logger(), "Visualizer running with stable marker updates");
    }

private:
    void topic_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        displacement_ = msg->data;
    }

    void publish_markers() {
        auto t = this->get_clock()->now();

        // ================= Anchor Cube =================
        publish_marker(
            0,
            visualization_msgs::msg::Marker::CUBE,
            0.0, 0.0, 2.0,
            0.2,
            0.0, 1.0, 0.0,
            t
        );

        // ================= Mass Sphere =================
        double z_mass = 1.0 + displacement_;
        publish_marker(
            1,
            visualization_msgs::msg::Marker::SPHERE,
            0.0, 0.0, z_mass,
            0.3,
            1.0, 0.0, 0.0,
            t
        );

        // ================= Spring Line =================
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = t;
        line.ns = "spring_sim";
        line.id = 2;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;

        line.pose.orientation.w = 1.0;

        line.scale.x = 0.05;
        line.scale.y = 0.05;
        line.scale.z = 0.05;

        line.color.r = 0.0;
        line.color.g = 0.0;
        line.color.b = 1.0;
        line.color.a = 1.0;

        line.lifetime = rclcpp::Duration::from_seconds(0.0);

        geometry_msgs::msg::Point p1, p2;
        p1.x = 0.0; p1.y = 0.0; p1.z = 2.0;
        p2.x = 0.0; p2.y = 0.0; p2.z = z_mass;

        line.points.clear();
        line.points.push_back(p1);
        line.points.push_back(p2);

        marker_pub_->publish(line);
    }

    void publish_marker(
        int id,
        int type,
        double x, double y, double z,
        double scale,
        float r, float g, float b,
        rclcpp::Time t
    ) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = t;
        m.ns = "spring_sim";
        m.id = id;
        m.type = type;
        m.action = visualization_msgs::msg::Marker::ADD;

        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;

        m.pose.orientation.w = 1.0;

        m.scale.x = scale;
        m.scale.y = scale;
        m.scale.z = scale;

        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0;

        m.lifetime = rclcpp::Duration::from_seconds(0.0);

        marker_pub_->publish(m);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double displacement_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizerNode>());
    rclcpp::shutdown();
    return 0;
}

