#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class SHM_physics : public rclcpp::Node
{
public:
  SHM_physics() : Node("SHM_Constants")
  {
    k  = 10; 
    b  = 0.5;   
    m  = 1.0;   
    dt = 0.01;  

    x = 0.0;    
    v = 15.0;     

    position_pub_ =
      this->create_publisher<std_msgs::msg::Float64>(
        "/mass_position", 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&SHM_physics::Equations, this));

    RCLCPP_INFO(this->get_logger(),
                "Mass-Spring-Damper physics node started");
  }

private:

  void Equations()
  {
    double F_spring = -k * x;
    double F_damping = -b * v;
    double a = (F_spring + F_damping) / m;
    v = v + a * dt;
    x = x + v * dt;

    std_msgs::msg::Float64 msg;
    msg.data = x;
    position_pub_->publish(msg);
  }

  double x;  
  double v;  

  double k;  
  double b;  
  double m;  
  double dt;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SHM_physics>());
  rclcpp::shutdown();
  return 0;
}