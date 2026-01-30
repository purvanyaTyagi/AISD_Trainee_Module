#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

class MassSpringNode : public rclcpp::Node
{
public:
    MassSpringNode() : Node("mass_spring_node")
    {
        // Marker publisher
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "visualization_marker", 10);

        // Energy publishers
        ke_pub_ = this->create_publisher<std_msgs::msg::Float64>("kinetic_energy", 10);
        pe_pub_ = this->create_publisher<std_msgs::msg::Float64>("potential_energy", 10);

        // Perturbation service
        perturb_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "apply_perturbation",
            std::bind(&MassSpringNode::apply_perturbation, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Physics parameters
        k_ = 5.0;
        b_ = 0.05;
        m_ = 1.0;

        // Initial conditions
        x_ = 2.0;
        v_ = 4.0;

        dt_ = 0.1;

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&MassSpringNode::update_system, this));
    }

private:
    void update_system()
    {
        // Forces
        double spring_force = -k_ * x_;
        double damping_force = -b_ * v_;
        double force = spring_force + damping_force;

        double acceleration = force / m_;

        // Euler integration
        v_ += acceleration * dt_;
        x_ += v_ * dt_;

        // Publish visualization
        publish_fixed_point();
        publish_mass();
        publish_spring();

        // Publish energy
        publish_energy();
    }

    void publish_energy()
    {
        std_msgs::msg::Float64 ke_msg;
        std_msgs::msg::Float64 pe_msg;

        ke_msg.data = 0.5 * m_ * v_ * v_;
        pe_msg.data = 0.5 * k_ * x_ * x_;

        ke_pub_->publish(ke_msg);
        pe_pub_->publish(pe_msg);
    }

    void apply_perturbation(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // Sudden velocity kick
        v_ += 5.0;

        response->success = true;
        response->message = "Perturbation applied: velocity increased";
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
        marker.color.a = 1.0;

        marker.pose.position.x = 0.0;
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

        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.pose.position.x = x_;
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
        marker.color.g = 1.0;
        marker.color.a = 1.0;

        geometry_msgs::msg::Point start, end;
        start.x = 0.0;
        end.x = x_;

        marker.points.push_back(start);
        marker.points.push_back(end);

        marker.id = 2;
        marker_pub_->publish(marker);
    }

    // ROS entities
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ke_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pe_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr perturb_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Physics
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
