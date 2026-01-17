#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"   
#include "std_msgs/msg/float64.hpp"
#include "spring_system/srv/parm_set.hpp"

using namespace std::chrono_literals;

class springsys: public rclcpp::Node
{
    public:
      springsys():Node("springsystem"), x_(5.0) ,v_(0.0)
      {

        pub_=this->create_publisher<visualization_msgs::msg::MarkerArray>("visualize_markers",1);

        ke_pub_ = this->create_publisher<std_msgs::msg::Float64>("/energy/kinetic", 10);
        pe_pub_ = this->create_publisher<std_msgs::msg::Float64>("/energy/potential", 10);
        te_pub_ = this->create_publisher<std_msgs::msg::Float64>("/energy/total", 10);

        srv_ = this->create_service<spring_system::srv::ParmSet>(
          "/set_parameters",
          std::bind(&springsys::parm_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        timer_=this->create_wall_timer(33ms, std::bind(&springsys::features, this));
        RCLCPP_INFO(this->get_logger(), "custom_service_with_parameter_tuning");
      }
    private:
      void parm_callback(const std::shared_ptr<spring_system::srv::ParmSet::Request>req,
        std::shared_ptr<spring_system::srv::ParmSet::Response> res){
        if(req->k<=0.0){
            res->success = false;
            res->message = "Invalid k. Must be > 0.";
            RCLCPP_WARN(this->get_logger(), "Rejected update: k=%.3f (must be > 0)", req->k);
            return;
        }
        if (req->b < 0.0) {
          res->success = false;
          res->message = "Invalid b. Must be >= 0.";
          RCLCPP_WARN(this->get_logger(), "Rejected update: b=%.3f (must be >= 0)", req->b);
          return;
        }
        k_ = req->k;
        b_ = req->b;

        res->success = true;
        res->message = "Updated k and b successfully.";
        RCLCPP_WARN(this->get_logger(), "UPDATED PARAMS: k=%.3f, b=%.3f", k_, b_);
      }

      void features(){
       const double k = k_;
       const double b  = b_;
       const double dt = 0.033;
       const double m = 1.0;
       const double y = 0.0;
       const double z = 0.0;

       const double dx = (x_ -3.5);
       const double F_spr = -k*dx;
       const double F_damp = -b*v_;
       const double F_net = F_spr + F_damp ;
       const double a = F_net/m;
       v_ += a*dt;
       x_ += (v_)*dt;

       const double KE = 0.5 * m * v_ * v_;
       const double PE = 0.5 * k * dx * dx;
       const double TE = KE + PE;

       std_msgs::msg::Float64 msg;
       msg.data = KE;
       ke_pub_->publish(msg);
       msg.data = PE;
       pe_pub_->publish(msg);
       msg.data = TE;
       te_pub_->publish(msg);

       RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "k=%.2f b=%.2f | x=%.3f v=%.3f | KE=%.3f PE=%.3f TE=%.3f",
      k, b, x_, v_, KE, PE, TE);

       visualization_msgs::msg::MarkerArray arr;

       visualization_msgs::msg::Marker ball;
       ball.header.frame_id = "map";
       ball.header.stamp = this->now();;
       ball.ns = "ball";
       ball.id =0;
       ball.type = visualization_msgs::msg::Marker::SPHERE;
       ball.action = visualization_msgs::msg::Marker::ADD;

       ball.scale.x =0.3;
       ball.scale.y =0.3;
       ball.scale.z =0.3;

       ball.color.r =1.0f;
       ball.color.g =0.0f;
       ball.color.b =0.0f;
       ball.color.a =1.0f;

       ball.pose.position.x = x_;
       ball.pose.position.y = y;
       ball.pose.position.z = z;
       ball.pose.orientation.w = 1.0;

       ball.lifetime = rclcpp::Duration::from_seconds(0.0);

       arr.markers.push_back(ball);  

       visualization_msgs::msg::Marker center ;
       center.header.frame_id = "map";
       center.header.stamp = this->now();;
       center.ns = "center";
       center.id =0;
       center.type = visualization_msgs::msg::Marker::CUBE;
       center.action = visualization_msgs::msg::Marker::ADD;

       center.scale.x = 0.15;
       center.scale.y = 0.15;
       center.scale.z = 0.15;

       center.color.g = 1.0f;
       center.color.a = 1.0f;

       center.pose.position.x = center.pose.position.y = center.pose.position.z =0;
       center.pose.orientation.w = 1.0;

       center.lifetime = rclcpp::Duration::from_seconds(0.0);

       arr.markers.push_back(center); 

       visualization_msgs::msg::Marker spring;
       spring.header.frame_id = "map";
       spring.header.stamp = this->now();;
       spring.ns ="spring";
       spring.id =0;
       spring.type = visualization_msgs::msg::Marker::LINE_STRIP;
       spring.action = visualization_msgs::msg::Marker::ADD;

       spring.color.b = 1.0f;
       spring.color.a = 1.0f;

       spring.scale.x = 0.05;

       spring.points.resize(2);
       spring.points[0].x = 0.0; spring.points[0].y = 0.0; spring.points[0].z = 0.0; 
       spring.points[1].x = x_;  spring.points[1].y = y;   spring.points[1].z = z;

       spring.pose.orientation.w = 1.0;
       spring.lifetime = rclcpp::Duration::from_seconds(0.0);

       arr.markers.push_back(spring); 

       pub_->publish(arr);
      }


       rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
       rclcpp::Service<spring_system::srv::ParmSet>::SharedPtr srv_;

       rclcpp::TimerBase::SharedPtr timer_;
       rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ke_pub_;
       rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pe_pub_;
       rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr te_pub_;

       double x_,v_;

       double k_ = 4.0;
       double b_ = 0.13;
};

int main(int argc,char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<springsys>());
    rclcpp::shutdown();
    return 0;
}
