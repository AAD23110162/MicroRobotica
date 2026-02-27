import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joint_states', 10)
        # Timer más rápido para un movimiento fluido (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Variables de control de movimiento
        self.q1 = 0.0
        self.q2 = 0.0
        self.step = 0.02  # Velocidad del incremento
        self.dir_q1 = 1   # 1 para subir, -1 para bajar
        self.dir_q2 = 1

    def timer_callback(self):
        # Lógica de incremento/decremento para q1 (0 a 180 grados -> 0 a pi rad)
        if self.q1 >= math.pi:
            self.dir_q1 = -1
        elif self.q1 <= 0:
            self.dir_q1 = 1
        
        # Lógica para q2 (un poco más lenta o desfasada para que se vea natural)
        if self.q2 >= math.pi:
            self.dir_q2 = -1
        elif self.q2 <= 0:
            self.dir_q2 = 1

        self.q1 += self.step * self.dir_q1
        self.q2 += (self.step * 0.7) * self.dir_q2 # q2 se mueve a otra velocidad

        # Publicar en Radianes
        msg = Float32MultiArray()
        msg.data = [self.q1, self.q2]
        self.publisher_.publish(msg)

        # Convertir a grados para el log extra
        deg_q1 = math.degrees(self.q1)
        deg_q2 = math.degrees(self.q2)

        self.get_logger().info(
            f'Ángulos -> q1: {deg_q1:>6.2f}° ({self.q1:.2f} rad) | '
            f'q2: {deg_q2:>6.2f}° ({self.q2:.2f} rad)'
        )

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()