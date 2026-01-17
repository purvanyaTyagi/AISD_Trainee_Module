import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MassSpringMarkers(Node):
    def __init__(self):
        super().__init__('mass_spring_markers')
        self.fixed_x = 0.0
        self.mass_x = 0.0
        self.sub = self.create_subscription(Float64,'mass_position',self.position_callback,10)
        self.marker_pub = self.create_publisher(Marker,'visualization_marker',10)
        self.timer = self.create_timer(0.05,self.publish_markers)

        self.fixed_marker = Marker()
        self.fixed_marker.header.frame_id = 'world'
        self.fixed_marker.ns = 'mass_spring'
        self.fixed_marker.id =0
        self.fixed_marker.type = Marker.CUBE
        self.fixed_marker.action = Marker.ADD
        self.fixed_marker.scale.x = 0.1
        self.fixed_marker.scale.y = 0.1
        self.fixed_marker.scale.z = 0.1
        self.fixed_marker.color.r = 1.0
        self.fixed_marker.color.a = 1.0
        self.fixed_marker.pose.position.x = self.fixed_x
        self.fixed_marker.pose.position.y = 0.0
        self.fixed_marker.pose.position.z = 0.0
        self.fixed_marker.pose.orientation.w = 1.0

        self.mass_marker = Marker()
        self.mass_marker.header.frame_id = 'world'
        self.mass_marker.ns = 'mass_spring'
        self.mass_marker.id = 1
        self.mass_marker.type = Marker.SPHERE
        self.mass_marker.action = Marker.ADD
        self.mass_marker.scale.x = 0.1
        self.mass_marker.scale.y = 0.1
        self.mass_marker.scale.z = 0.1
        self.mass_marker.color.b = 1.0
        self.mass_marker.color.a = 1.0
        self.mass_marker.pose.position.y = 0.0
        self.mass_marker.pose.position.z = 0.0
        self.mass_marker.pose.orientation.w = 1.0


        self.spring_marker = Marker()
        self.spring_marker.header.frame_id = 'world'
        self.spring_marker.ns = 'mass_spring'
        self.spring_marker.id = 2
        self.spring_marker.type = Marker.LINE_STRIP
        self.spring_marker.action = Marker.ADD
        self.spring_marker.scale.x = 0.02
        self.spring_marker.color.g = 1.0
        self.spring_marker.color.a = 1.0
        self.spring_marker.pose.position.z = 0.0
        self.spring_marker.pose.position.y = 0.0
        self.spring_marker.pose.orientation.w = 1.0
        self.get_logger().info('Marker node started')

    def position_callback(self,msg):
        self.mass_x = msg.data
    
    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        self.fixed_marker.header.stamp = now
        self.mass_marker.header.stamp = now
        self.spring_marker.header.stamp = now
        self.mass_marker.pose.position.x = self.mass_x
        self.spring_marker.points.clear()
        p1 = Point()
        p1.x = self.fixed_x
        p2 = Point()
        p2.x = self.mass_x
        self.spring_marker.points.append(p1)
        self.spring_marker.points.append(p2)

        self.marker_pub.publish(self.fixed_marker)
        self.marker_pub.publish(self.mass_marker)
        self.marker_pub.publish(self.spring_marker)

def main(args=None):
    rclpy.init(args=args)
    node = MassSpringMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown 

if __name__ == '__main__':
    main()