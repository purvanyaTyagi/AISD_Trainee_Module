import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateNOde(Node):
    def __init__(self):
        super().__init__('custom_joint_state_node')

        self.pub = self.create_publisher(JointState,'/joint_states',10)

        self.timer = self.create_timer(0.02,self.publish_joint_states)

        self.time=0.0

        self.joint_names = [
            'left_arm_joint',
            'right_arm_joint',
            'head_and_torso',
            'left_leg_front_wheel_joint',
            'left_leg_back_wheel_joint',
            'right_leg_front_wheel_joint',
            'right_leg_back_wheel_joint'
        ]

        self.get_logger().info('Custom joint state node started')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = self.joint_names

        self.time +=0.02

        msg.position = [
            0.8*math.sin(self.time),
            -0.8*math.sin(self.time),
            0.5*math.sin(self.time),
            -self.time*2.0,
            -self.time*2.0,
            -self.time*2.0,
            -self.time*2.0,
        ]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = JointStateNOde()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == 'main':
    main()