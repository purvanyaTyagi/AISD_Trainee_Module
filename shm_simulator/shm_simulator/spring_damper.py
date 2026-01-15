import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Vector3

class DampedSHM(Node):
    def __init__(self):
        super().__init__('damped_shm_node')
        
        # Physics Parameters
        self.position = 2.0   # 2 meters door start karenge with 0 velocity.
        self.velocity = 0.0
        self.mass = 1.0       # in kg
        self.k = 5.0          # Spring constant
        self.c = 0.5          # Damping coefficient (dont take c/2m here)
        # Time step (sec) - basically time real life mein continuos flow karta hai, but computer ko ek dt chahiye, har dt baad vo positions and all calculate kare. 
        self.dt = 0.08

        # --- ROS Setup ---
        self.publisher_ = self.create_publisher(Marker, 'shm_marker', 10)  #shm marker is node name.
        
        self.energy_publisher_ = self.create_publisher(Vector3, 'shm_energy', 10) 

        self.timer = self.create_timer(self.dt, self.update_physics)
        self.get_logger().info("SHM Simulator Started") #shuru hone se pehle ka msg

    def update_physics(self):
        # Forces
        f_spring = -self.k * self.position
        f_damping = -self.c * self.velocity
        f_total = f_spring + f_damping

        #Energies
        ke = 0.5 * self.mass * (self.velocity**2)
        pe = 0.5 * self.k * (self.position**2)
        tot_e = ke + pe  
        energy_msg = Vector3()
        energy_msg.x = ke
        energy_msg.y = pe
        energy_msg.z = tot_e 

        # define acceleration
        acc = f_total / self.mass

        # Update Velocity and Position
        self.velocity += acc * self.dt
        self.position += self.velocity * self.dt

        # Publish to RViz
        self.get_logger().info(f'Position: {self.position:.2f}')
        self.publish_marker()
        self.energy_publisher_.publish(energy_msg)

    def publish_marker(self):

        # ---1. The Red Ball ---
        r_ball = Marker()  # A class in ros
        r_ball.header.frame_id = "map"
        r_ball.header.stamp = self.get_clock().now().to_msg()
        
        # Set shape (SPHERE)
        r_ball.type = Marker.SPHERE
        r_ball.id = 0
        
        # Set Scale (meters)
        r_ball.scale.x = 0.5
        r_ball.scale.y = 0.5
        r_ball.scale.z = 0.5

        # Set Color (Red color, non-transparent)
        r_ball.color.r = 1.0
        r_ball.color.g = 0.0
        r_ball.color.b = 0.0
        r_ball.color.a = 1.0

        # Set Position (Moving along X axis) - we want to restrict the motion.
        r_ball.pose.position.x = self.position
        r_ball.pose.position.y = 0.0
        r_ball.pose.position.z = 0.0

        self.publisher_.publish(r_ball)



        # --- 2. The Green Center Point ---
        wall = Marker()
        wall.header.frame_id = "map"
        wall.header.stamp = self.get_clock().now().to_msg()
        wall.type = Marker.CUBE  # trying new things
        wall.id = 1              # Unique ID hona chahiye of every object
        
        # Size (Make it a tall, thin wall)
        wall.scale.x = 0.1
        wall.scale.y = 0.1
        wall.scale.z = 0.1
        
        # Color (Green)
        wall.color.g = 1.0
        wall.color.a = 1.0
        
        # Position (Fixed at 0,0,0)
        wall.pose.position.x = 0.0
        wall.pose.position.y = 0.0
        wall.pose.position.z = 0.0
        
        self.publisher_.publish(wall)



        # --- 3. The Yellow String (The Spring) ---
        string = Marker()
        string.header.frame_id = "map"
        string.header.stamp = self.get_clock().now().to_msg()
        string.type = Marker.LINE_STRIP
        string.id = 2            # IMPORTANT: Unique ID!
        
        # Line Thickness (sirf x mein hi dimension dene ka fayda hai)
        string.scale.x = 0.05
        
        # Color (Yellow)
        string.color.r = 1.0
        string.color.g = 1.0
        string.color.a = 1.0
        
        # Define the two points of the line
        start_point = Point()  # Start at Wall
        start_point.x = 0.0
        start_point.y = 0.0
        start_point.z = 0.0
        
        end_point = Point()    # End at Ball
        end_point.x = self.position
        end_point.y = 0.0
        end_point.z = 0.0
        
        string.points = [start_point, end_point] # ek array bnata hai of points aur unko color kardeta hai.
        
        self.publisher_.publish(string)

def main(args=None):
    rclpy.init(args=args)
    node = DampedSHM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
