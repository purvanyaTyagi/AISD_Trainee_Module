import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class MassSpringNode(Node):
    def __init__(self):
        super().__init__('mass_spring_node')

        self.m = 1.0
        self.k = 10.0
        self.b = 0.01

        self.x = 10.0
        self.v = 0.0

        self.dt = 0.05

        self.publisher = self.create_publisher(
            Marker,
            'visualization_marker',
            10
        )

        self.service = self.create_service(
            ApplyDisplacement,
            '/apply_displacement',
            self.apply_displacement_callback
        )

        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info('Executing goal...')

    def apply_displacement_callback(self, request, response):
        self.x += request.displacement
        response.success = True
        return response

    def update(self):
        spring_force = -self.k * self.x
        damping_force = -self.b * self.v
        net_force = spring_force + damping_force

        a = net_force / self.m

        self.v = self.v + a * self.dt
        self.x = self.x + self.v * self.dt

        fixed = Marker()
        fixed.header.frame_id = "world"
        fixed.header.stamp = self.get_clock().now().to_msg()
        fixed.ns = "11"
        fixed.id = 0
        fixed.type = Marker.CUBE
        fixed.action = Marker.ADD
        fixed.pose.position.x = 0.0
        fixed.pose.position.y = 0.0
        fixed.pose.position.z = 0.1
        fixed.scale.x = 0.3
        fixed.scale.y = 0.3
        fixed.scale.z = 0.3
        fixed.color.r = 0.0
        fixed.color.g = 0.0
        fixed.color.b = 1.0
        fixed.color.a = 1.0

        mass = Marker()
        mass.header.frame_id = "world"
        mass.header.stamp = self.get_clock().now().to_msg()
        mass.ns = "12"
        mass.id = 1
        mass.type = Marker.SPHERE
        mass.action = Marker.ADD
        mass.pose.position.x = self.x
        mass.pose.position.y = 0.0
        mass.pose.position.z = 0.1
        mass.scale.x = 0.2
        mass.scale.y = 0.2
        mass.scale.z = 0.2
        mass.color.r = 1.0
        mass.color.g = 0.0
        mass.color.b = 0.0
        mass.color.a = 1.0

        spring = Marker()
        spring.header.frame_id = "world"
        spring.header.stamp = self.get_clock().now().to_msg()
        spring.ns = "13"
        spring.id = 2
        spring.type = Marker.LINE_STRIP
        spring.action = Marker.ADD
        spring.scale.x = 0.15
        spring.scale.y = 0.0
        spring.scale.z = 0.1
        spring.color.r = 0.0
        spring.color.g = 1.0
        spring.color.b = 0.0
        spring.color.a = 1.0
        
        spring.points.clear()

        p1 = Point()
        p1.x = 0.0
        p1.y = 0.0
        p1.z = 0.2

        p2 = Point()
        p2.x = self.x
        p2.y = 0.0
        p2.z = 0.2
        
        spring.points.append(p1)
        spring.points.append(p2)

        self.publisher.publish(fixed)
        self.publisher.publish(mass)
        self.publisher.publish(spring)


def main(args=None):
    rclpy.init(args=args)
    node = MassSpringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
