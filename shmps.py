import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from interfaces.srv import Shm

class phy(Node):
    def __init__(self):
        super().__init__('phy_node')
        self.phy_spring = self.create_publisher(Marker,'shm',10)
        self.publisher1 = self.create_publisher(Float64,'PE',10)
        self.publisher2 = self.create_publisher(Float64,'KE',10)
        self.trigger= self.create_service(Trigger,'impulse',self.impulse)
        self.input= self.create_service(Shm,'update',self.change)
        clear = Marker()
        clear.action = Marker.DELETEALL
        self.phy_spring.publish(clear)
        self.pos=4.0
        self.m=1.0
        self.v=0.0
        self.b=0.05
        self.k=2.0
        self.fs=0.0
        self.fd=0.0
        self.a=0.0
        self.ke=0.0
        self.pe=0.0
        self.t=0.1
        self.repeat=self.create_timer(self.t,self.call)

    def change(self,request,response):      
        self.update_k=request.k
        self.update_b=request.b
        self.k=self.update_k
        self.b=self.update_b
        response.success=True
        response.message='updated'
        return (response)
   
    def impulse(self,request,response): # for trigger request is not needed
        self.v+=2.0                     #For std_srvs/Trigger, the service definition is:
        response.success= True          #bool success , string message
        response.message='done'
        return(response)                

    def call(self):
        x=self.pos - 1.0
        self.fs=-self.k*x
        self.fd=-self.b*self.v
        self.a=(self.fs+self.fd)/self.m
        self.v += self.a*self.t
        self.pos += self.v*self.t

        if self.pos<=(-2.0) :
            self.pos=(-2.0)
            self.v=0.0

        x=self.pos - 1.0    
        self.ke=0.5*self.m*self.v*self.v
        self.pe=0.5*self.k*x*x
        msg1=Float64()
        msg1.data=self.pe
        msg2=Float64()
        msg2.data=self.ke
        self.publisher1.publish(msg1)
        self.publisher2.publish(msg2)

        marker_mass = Marker()
        marker_mass.header.frame_id = 'map' #define axis
        marker_mass.header.stamp = self.get_clock().now().to_msg() #set time frame
        marker_mass.ns = 'mass'
        marker_mass.id = 1
        marker_mass.type = Marker.SPHERE
        marker_mass.action = Marker.ADD
        marker_mass.pose.position.x = self.pos
        marker_mass.pose.position.y = 0.0
        marker_mass.pose.position.z = 0.0
        marker_mass.pose.orientation.w = 1.0
        marker_mass.scale.x = 0.4
        marker_mass.scale.y = 0.4
        marker_mass.scale.z = 0.4
        marker_mass.color.r = 0.0
        marker_mass.color.g = 0.0
        marker_mass.color.b = 1.0
        marker_mass.color.a = 1.0 #opacity
        marker_mass.lifetime.sec = 0 # =1 would mean if marker not updated for 1sec then delete
        marker_mass.lifetime.nanosec = 0 # time stored as sec+nanosec  no float values

        self.phy_spring.publish(marker_mass)

        marker_fixed = Marker()
        marker_fixed.header.frame_id = 'map' #define axis
        marker_fixed.header.stamp = self.get_clock().now().to_msg() #set time frame
        marker_fixed.ns = 'fixed'
        marker_fixed.id = 2
        marker_fixed.type = Marker.CUBE
        marker_fixed.action = Marker.ADD
        marker_fixed.pose.position.x = -2.0
        marker_fixed.pose.position.y = 0.0
        marker_fixed.pose.position.z = 0.0
        marker_fixed.pose.orientation.w = 1.0
        marker_fixed.scale.x = 0.05
        marker_fixed.scale.y = 1.0
        marker_fixed.scale.z = 1.0
        marker_fixed.color.r = 1.0
        marker_fixed.color.g = 0.0
        marker_fixed.color.b = 0.0
        marker_fixed.color.a = 1.0 #opacity
        marker_fixed.lifetime.sec = 0 # =1 would mean if marker not updated for 1sec then delete
        marker_fixed.lifetime.nanosec = 0 # time stored as sec+nanosec  no float values

        self.phy_spring.publish(marker_fixed)

        marker_spring = Marker()
        marker_spring.header.frame_id = 'map' #define axis
        marker_spring.header.stamp = self.get_clock().now().to_msg() #set time frame
        marker_spring.ns = 'spring'
        marker_spring.id = 3
        marker_spring.type = Marker.LINE_STRIP
        marker_spring.action = Marker.ADD
        start=Point()
        start.x=-2.0
        start.y=0.0
        start.z=0.0
        end=Point()
        end.x=self.pos
        end.y=0.0
        end.z=0.0
        marker_spring.points = [start, end]
        marker_spring.scale.x = 0.02
        marker_spring.color.r = 0.0
        marker_spring.color.g = 1.0
        marker_spring.color.b = 0.0
        marker_spring.color.a = 1.0 #opacity
        marker_spring.lifetime.sec = 0 # =1 would mean if marker not updated for 1sec then delete
        marker_spring.lifetime.nanosec = 0 # time stored as sec+nanosec  no float values

        self.phy_spring.publish(marker_spring)


def main(args=None):
    rclpy.init(args=args)
    phy_node=phy()
    rclpy.spin(phy_node)
    phy_node.destroy_node()
    rclpy.shutdown()

