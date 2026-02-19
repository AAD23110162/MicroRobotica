import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubsSensor (Node):
    def __init__(self):
        super().__init__('subs_sensor')
        self.subscriber = self.create_subscription(String, 'sensor', self.callback, 10)
    
    def callback(self,msg):
        print(msg.data)

def main(args=None):
    rclpy.init(args=args)
    subs_sensor=SubsSensor()
    rclpy.spin(subs_sensor)
    subs_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
