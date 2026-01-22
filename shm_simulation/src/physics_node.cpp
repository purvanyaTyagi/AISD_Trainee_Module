#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "shm_simulation/msg/energy.hpp"
#include "shm_simulation/srv/apply_force.hpp"
#include "shm_simulation/srv/update_parameters.hpp"

using namespace std::chrono_literals;

class SHM_physics : public rclcpp::Node
{
public:
    SHM_physics() : Node("SHM_Constants")
    {
        k = 10;
        b = 0.0;
        m = 1.0;
        dt = 0.0001;
        
        x = 0.0;
        v = 15.0;
        
        counter = 0;
        
        position_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/mass_position", 10);
        
        energy_pub = this->create_publisher<shm_simulation::msg::Energy>(
            "/energy_data", 10);
        
        ke_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/kinetic_energy", 10);

        pe_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/potential_energy", 10);

        total_energy_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/total_energy", 10);

        
        force_service = this->create_service<shm_simulation::srv::ApplyForce>(
            "apply_force",
            std::bind(&SHM_physics::apply_force_callback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        param_service = this->create_service<shm_simulation::srv::UpdateParameters>(
            "update_parameters",
            std::bind(&SHM_physics::update_parameters_callback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt),
            std::bind(&SHM_physics::Equations, this));
        
        RCLCPP_INFO(this->get_logger(), 
                   "Mass-Spring-Damper physics node started");
        RCLCPP_INFO(this->get_logger(), 
                   "Initial parameters: k=%.2f, b=%.2f, m=%.2f", k, b, m);
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
        position_pub->publish(msg);
        
        double KE = 0.5 * m * v * v;
        double PE = 0.5 * k * x * x;

        std_msgs::msg::Float64 ke_msg;
        ke_msg.data = KE;
        ke_pub->publish(ke_msg);

        std_msgs::msg::Float64 pe_msg;
        pe_msg.data = PE;
        pe_pub->publish(pe_msg);

        std_msgs::msg::Float64 total_msg;
        total_msg.data = KE + PE;
        total_energy_pub->publish(total_msg);

        
        shm_simulation::msg::Energy energy_msg;
        energy_msg.kinetic_energy = KE;
        energy_msg.potential_energy = PE;
        energy_msg.total_energy = KE + PE;
        energy_msg.timestamp = this->get_clock()->now().seconds();
        
        energy_pub->publish(energy_msg);
        
        if (counter % 100 == 0) {
            RCLCPP_INFO(this->get_logger(),
                       "x=%.3f, v=%.3f, KE=%.3f, PE=%.3f, Total=%.3f",
                       x, v, KE, PE, KE + PE);
        }
        counter++;
    }
    
   void apply_force_callback(
    const std::shared_ptr<shm_simulation::srv::ApplyForce::Request> request,
    std::shared_ptr<shm_simulation::srv::ApplyForce::Response> response)
{
    double impulse_duration = 0.1;
    double velocity_change = (request->force_magnitude * impulse_duration) / m;
    
    v += velocity_change; 
    
    response->success = true;
    response->message = "Applied impulse: " + 
                       std::to_string(request->force_magnitude) + 
                       " N for " + std::to_string(impulse_duration) + " s";
    
    RCLCPP_INFO(this->get_logger(), 
               "Impulse applied: %.2f N, velocity change: %.2f m/s", 
               request->force_magnitude, velocity_change);
}
    
    void update_parameters_callback(
        const std::shared_ptr<shm_simulation::srv::UpdateParameters::Request> request,
        std::shared_ptr<shm_simulation::srv::UpdateParameters::Response> response)
    {
        double old_k = k;
        double old_b = b;
        
        k = request->spring_stiffness;
        b = request->damping_coefficient;
        
        response->success = true;
        response->message = "Updated parameters: k=" + std::to_string(k) + 
                           ", b=" + std::to_string(b);
        
        RCLCPP_INFO(this->get_logger(),
                   "Parameters updated: k=%.2f->%.2f, b=%.2f->%.2f",
                   old_k, k, old_b, b);
    }

    double x, v;
    double k, b, m, dt;
    int counter;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub;
    rclcpp::Publisher<shm_simulation::msg::Energy>::SharedPtr energy_pub;
    rclcpp::Service<shm_simulation::srv::ApplyForce>::SharedPtr force_service;
    rclcpp::Service<shm_simulation::srv::UpdateParameters>::SharedPtr param_service;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ke_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pe_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_energy_pub;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SHM_physics>());
    rclcpp::shutdown();
    return 0;
}