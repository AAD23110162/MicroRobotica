import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PubSensor(Node):
    def __init__(self):
        super().__init__('pub_tem')
        self.publisher=self.create_publisher(String, 'sensor',10)
        period=0.5
        self.timer=self.create_timer(period,self.callback)
        self.i=0
    def callback(self):
        msg=String()
        msg.data="temperatura"+str(self.i)
        self.publisher.publish(msg)
        self.i+=1
        print(msg)

def main(arg=None):
    rclpy.init(args=arg)
    pub_sensor=PubSensor()
    rclpy.spin(pub_sensor)
    pub_sensor.destroy_node()
    rclpy.shutdown()
    print('Hi from sensor_pkg.')


if __name__ == '__main__':
    main()
