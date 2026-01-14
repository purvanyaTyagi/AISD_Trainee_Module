#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>

using namespace std::chrono_literals;

class MassSpringNode : public rclcpp::Node
{
public:
    MassSpringNode() : Node("mass_spring_node")
    {
        // Publisher
        pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        // Timer for updates (smooth animation)
        timer_ = this->create_wall_timer(20ms, std::bind(&MassSpringNode::update, this));

        // Physics parameters
        x_ = 2.0; // Start at +2 (right extreme)
        v_ = 0.0; // Start at rest
        k_ = 15.0; // Spring constant → fast oscillation
        b_ = 0.2; // Damping → gradual decay
        m_ = 1.0; // Mass
        dt_ = 0.02; // Time step for integration

        cube_x_ = 0.0; // Fixed cube at origin
    }

    private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_, v_, k_, b_, m_, dt_;
    double cube_x_;

    void update()
    {
        // --- Damped SHM physics ---
        double a = (-k_ * x_ - b_ * v_) / m_; // Acceleration
        v_ += a * dt_;
        x_ += v_ * dt_;

        publish_markers();
    }

    void publish_markers()
    {
        auto now = this->now();

        // ----- Cube (fixed) -----
        visualization_msgs::msg::Marker cube;
        cube.header.frame_id = "map";
        cube.header.stamp = now;
        cube.ns = "cube";
        cube.id = 0;
        cube.type = visualization_msgs::msg::Marker::CUBE;
        cube.action = visualization_msgs::msg::Marker::ADD;
        cube.pose.position.x = cube_x_;
        cube.pose.position.y = 0.0;
        cube.pose.position.z = 0.0;
        cube.scale.x = 0.3;
        cube.scale.y = 0.3;
        cube.scale.z = 0.3;
        cube.color.b = 1.0;
        cube.color.a = 1.0;

         // ----- Mass (moving sphere) -----
        visualization_msgs::msg::Marker mass;
        mass.header.frame_id = "map";
        mass.header.stamp = now;
        mass.ns = "mass";
        mass.id = 1;
        mass.type = visualization_msgs::msg::Marker::SPHERE;
        mass.action = visualization_msgs::msg::Marker::ADD;
        mass.pose.position.x = x_;
        mass.pose.position.y = 0.0;
        mass.pose.position.z = 0.0;
        mass.scale.x = 0.25;
        mass.scale.y = 0.25;
        mass.scale.z = 0.25;
        mass.color.r = 1.0;
        mass.color.a = 1.0;

        // ----- Spring (line connecting cube ↔ mass) -----
        visualization_msgs::msg::Marker spring;
        spring.header.frame_id = "map";
        spring.header.stamp = now;
        spring.ns = "spring";
        spring.id = 2;
        spring.type = visualization_msgs::msg::Marker::LINE_STRIP;
        spring.action = visualization_msgs::msg::Marker::ADD;
        spring.scale.x = 0.3;
        
        spring.color.r = 0.0;
        spring.color.g = 1.0;
        spring.color.b = 0.0;
        spring.color.a = 1.0;

        spring.points.clear();
int num_zags = 10; // number of zig-zags
double start = cube_x_;
double end = x_;
for (int i = 0; i <= num_zags; ++i) {
    geometry_msgs::msg::Point p;
    p.x = start + (end - start) * i / num_zags;
    p.y = ((i % 2 == 0) ? 0.1 : -0.1); // zig-zag in y
    p.z = 0.0;
    spring.points.push_back(p);
}


        /*geometry_msgs::msg::Point p1, p2;
        p1.x = cube_x_;
        p1.y = 0.0;
        p1.z = 0.0;

        p2.x = x_;
        p2.y = 0.0;
        p2.z = 0.0;

         spring.points.clear();
        spring.points.push_back(p1);
        spring.points.push_back(p2);*/

        // Publish markers
        pub_->publish(cube);
        pub_->publish(mass);
        pub_->publish(spring);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MassSpringNode>());
    rclcpp::shutdown();
    return 0;
}