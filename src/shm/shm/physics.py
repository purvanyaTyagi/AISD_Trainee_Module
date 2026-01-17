import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

class MassSpringPhysics(Node):
    def __init__(self):
        super().__init__('mass_spring_physics')
        self.k = 10.0
        self.b = 0.0
        self.m = 1.0
        self.x = 1.0
        self.v = 0.0
        self.dt = 0.02
        self.position_pub = self.create_publisher(Float64,'mass_position',10)
        self.timer = self.create_timer(self.dt,self.update_physics)
        self.get_logger().info('Mass-spring phyics node started')
    
    def update_physics(self):
        spring_force = -self.k * self.x
        damping_force = -self.b*self.v
        net_force = spring_force+damping_force
        acceleration = net_force/self.m
        self.v = self.v +acceleration*self.dt
        self.x = self.x + self.v*self.dt
        msg = Float64()
        msg.data = self.x
        self.position_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = MassSpringPhysics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()