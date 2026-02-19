import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ConvCelsius(Node):
    def __init__(self):
        super().__init__('conv_celsius')
        self.subscription = self.create_subscription(Float32, 'temp_f', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Float32, 'temp_c', 10)

    def listener_callback(self, msg):
        f = msg.data
        c = (f - 32) * 5 / 9
        msg_c = Float32()
        msg_c.data = float(c)
        self.publisher_.publish(msg_c)
        self.get_logger().info(f'Recibido: {f}°F -> Convertido: {c:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = ConvCelsius()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()