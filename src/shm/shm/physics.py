import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from std_msgs.msg import Float64

class MassSpringPhysics(Node):
    def __init__(self):
        super().__init__('mass_spring_physics')
        self.k = 10.0
        self.b = 0.1
        self.m = 1.0
        self.x = 1.0
        self.v = 0.0
        self.dt = 0.02
        self.kE = 0
        self.pE = 0
        self.external_force = 0.0

        self.position_pub = self.create_publisher(Float64,'mass_position',10)
        self.velocity_pub = self.create_publisher(Float64,'velocity',10)
        self.ke_pub = self.create_publisher(Float64,'kinetic_energy',10)
        self.pe_pub = self.create_publisher(Float64,'potential_energy',10)
        self.force_service = self.create_service(SetBool,'apply_force',self.apply_force_callback)
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

    def apply_force_callback(self,request,response):
        if request.data:
            self.external_force += 1000
            response.success = True
            response.message = "Applied external force"
        else:
            response.success = False
            response.message = "Force not applied"
        return response

def main(args=None):
    rclpy.init(args=args)

    node = MassSpringPhysics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()