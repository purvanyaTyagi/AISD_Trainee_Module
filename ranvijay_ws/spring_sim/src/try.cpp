#include <chrono>
#include<functional>
#include <memory>
#include <string>
using std::placeholders::_1;
using std::placeholders::_2;
#include "spring_sim/srv/set_spring_params.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"
using namespace std::chrono_literals;
class tryrviz : public rclcpp :: Node
{
    public :
      double x=1;
      double k=11;
      double acc=0.0;
      double v=100.0;
      double mass=0.5;
      double dt=0.01;
      double d=0.1;
      double ke=0.0;
      double pe=0.0;

      tryrviz() : Node("tryrviz")
        {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("bob_marker", 10);
            origin_pub_=this->create_publisher<visualization_msgs::msg::Marker>("origin_marker",10);
            ke_pub_=this->create_publisher<std_msgs::msg::Float64>("/ke",10);
            pe_pub_=this->create_publisher<std_msgs::msg::Float64>("/pe",10);
            timer_=this->create_wall_timer(10ms,std::bind(&tryrviz::publish_marker,this));
            service_=this->create_service<spring_sim::srv::SetSpringParams>("set_spring_params",std::bind(&tryrviz::set_params_callback,this,_1,_2));
        }
    private:
void set_params_callback(
      const std::shared_ptr<spring_sim::srv::SetSpringParams::Request> request,
      std::shared_ptr<spring_sim::srv::SetSpringParams::Response> response)
    {
      k = request->stiffness;
      d = request->damping;

      response->success = true;
      response->message = "Spring parameters updated";

      RCLCPP_WARN(this->get_logger(),
        "Updated params: k=%.2f  damping=%.2f", k, d);
    }
    void publish_marker(){
      visualization_msgs::msg::Marker origin;
      origin.header.frame_id="map";
      origin.header.stamp=this->now();
      origin.ns="basic_shapes0";
      origin.id= 1;
      origin.type=visualization_msgs::msg::Marker::CUBE;
      origin.action=visualization_msgs::msg::Marker::ADD;
      origin.pose.position.x =0;
      origin.pose.position.y = 0;
      origin.pose.position.z = 0;

      origin.pose.orientation.x = 0.0;
      origin.pose.orientation.y = 0.0;
      origin.pose.orientation.z = 0.0;
      origin.pose.orientation.w = 0.0;

      origin.scale.x = 0.2;
      origin.scale.y = 0.2;
      origin.scale.z = 0.2;

      origin.color.r = 0.0f;
      origin.color.g = 1.0f;
      origin.color.b = 0.0f;
      origin.color.a = 1.0;
      origin_pub_->publish(origin);
      visualization_msgs::msg::Marker marker;
      // visualization_msgs::msg::Marker marker1;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();

      marker.ns = "basic_shapes";
      marker.id = 0;

      marker.type = visualization_msgs::msg::Marker::SPHERE;

      marker.action = visualization_msgs::msg::Marker::ADD;
      acc=-(k*x+d*v)/mass;
      v=v + acc*dt;
      x=x+ v*dt;
      ke=0.5*mass*v*v;
      pe=0.5*k*x*x; 
      auto ke_message=std_msgs::msg::Float64();
      ke_message.data=ke;
      ke_pub_->publish(ke_message);
      auto pe_message=std_msgs::msg::Float64();
      pe_message.data=pe;
      pe_pub_->publish(pe_message);

      marker.pose.position.x =x;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;

      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker_pub_->publish(marker);
    }
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr origin_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ke_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pe_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<spring_sim::srv::SetSpringParams>::SharedPtr service_;
};
 int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tryrviz>());
  rclcpp::shutdown();
  return 0;
}