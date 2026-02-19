import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PubFahrenheit(Node):
    def __init__(self):
        super().__init__('pub_fahrenheit')
        self.publisher_ = self.create_publisher(Float32, 'temp_f', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.f_val = 70.0

    def timer_callback(self):
        msg = Float32()
        msg.data = self.f_val
        self.publisher_.publish(msg)
        self.get_logger().info(f'Enviando: {msg.data} °F')
        self.f_val += 1.0

def main(args=None): # <--- Asegúrate que diga 'main'
    rclpy.init(args=args)
    node = PubFahrenheit()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()