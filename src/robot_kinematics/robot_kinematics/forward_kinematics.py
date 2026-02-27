import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import math

class ForwardKinematics(Node):
    def __init__(self):
        super().__init__('forward_kinematics')
        self.subscription = self.create_subscription(Float32MultiArray, 'joint_states', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Point, 'robot_position', 10)
        
        # Parámetros de la Tabla DH
        self.l1 = 0.5  # Longitud eslabón 1 (ajustar según tu robot real)
        self.l2 = 0.3  # Longitud eslabón 2

    def listener_callback(self, msg):
        q1 = msg.data[0]
        q2 = msg.data[1]

        # Ecuaciones de Cinemática Directa (Robot Planar RR)
        x = self.l1 * math.cos(q1) + self.l2 * math.cos(q1 + q2)
        y = self.l1 * math.sin(q1) + self.l2 * math.sin(q1 + q2)

        point_msg = Point()
        point_msg.x = float(x)
        point_msg.y = float(y)
        point_msg.z = 0.0 # Es un robot planar
        
        self.publisher_.publish(point_msg)
        self.get_logger().info(f'Extremo en -> X: {x:.3f}, Y: {y:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematics()
    rclpy.spin(node)
    rclpy.shutdown()