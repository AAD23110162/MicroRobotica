import sys
import rclpy
from rclpy.node import Node
from interface_robotics.srv import MoveArm

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.cli = self.create_client(MoveArm, 'solve_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, esperando...')
        self.req = MoveArm.Request()

    def send_request(self, x, y):
        self.req.x = float(x)
        self.req.y = float(y)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        print("Uso: ros2 run robot_kinematics ik_client <X> <Y>")
        return
    
    client = IKClient()
    x_val = sys.argv[1]
    y_val = sys.argv[2]
    client.get_logger().info(f'Enviando coordenadas: X={x_val}, Y={y_val}')
    
    response = client.send_request(x_val, y_val)
    client.get_logger().info(f'Respuesta recibida: q1={response.q1:.4f} rad, q2={response.q2:.4f} rad')
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
