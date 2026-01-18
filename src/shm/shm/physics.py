import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64

class MassSpringPhysics(Node):
    def __init__(self):
        super().__init__('mass_spring_physics')

        self.declare_parameter('k',10.0)
        self.declare_parameter('b',0.0)
        self.declare_parameter('F',0.0)

        self.k = self.get_parameter('k').value
        self.b = self.get_parameter('b').value
        self.external_force = self.get_parameter('F').value

        self.add_on_set_parameters_callback(self.on_param_change)

        self.m = 1.0
        self.x = 1.0
        self.v = 0.0
        self.dt = 0.02
        self.kE = 0
        self.pE = 0

        self.position_pub = self.create_publisher(Float64,'mass_position',10)
        self.velocity_pub = self.create_publisher(Float64,'velocity',10)
        self.ke_pub = self.create_publisher(Float64,'kinetic_energy',10)
        self.pe_pub = self.create_publisher(Float64,'potential_energy',10)
        self.timer = self.create_timer(self.dt,self.update_physics)
        self.get_logger().info('Mass-spring phyics node started')
    
    def update_physics(self):
        spring_force = -self.k * self.x
        damping_force = -self.b*self.v
        net_force = spring_force+damping_force+self.external_force
        acceleration = net_force/self.m
        self.v = self.v +acceleration*self.dt
        self.x = self.x + self.v*self.dt
        self.kE = 0.5*self.m*self.v*self.v
        self.pE = 0.5*self.k*self.x*self.x
        self.external_force = 0.0
        msg = Float64()
        
        msg.data = self.x
        self.position_pub.publish(msg)
        
        msg.data = self.v
        self.velocity_pub.publish(msg)

        msg.data = self.kE
        self.ke_pub.publish(msg)

        msg.data = self.pE
        self.pe_pub.publish(msg)

    def on_param_change(self,params):
        for param in params:
            if param.name == 'k':
                if param.value <= 0.0:
                    return SetParametersResult(successful=False)
                self.k = param.value
                self.get_logger().info(f"Updated k = {self.k}")
            elif param.name == 'b':
                if param.value <0.0:
                    return SetParametersResult(successful=False)
                self.b = param.value
                self.get_logger().info(f"Updated b = {self.b}")
            elif param.name == 'F':
                self.external_force = param.value
                self.get_logger().info(f"Applied external force F = {param.value}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    node = MassSpringPhysics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()