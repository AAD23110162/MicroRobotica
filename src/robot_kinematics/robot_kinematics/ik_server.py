import rclpy
from rclpy.node import Node
from interface_robotics.srv import MoveArm
import math

class IKServer(Node):
    def __init__(self):
        super().__init__('ik_server')
        self.srv = self.create_service(MoveArm, 'solve_ik', self.ik_callback)
        self.l1 = 1.0
        self.l2 = 1.0
        self.get_logger().info('Servidor de Cinematica Inversa listo para recibir coordenadas (X, Y)...')

    def ik_callback(self, request, response):
        x = request.x
        y = request.y
        self.get_logger().info(f'Recibido objetivo: X={x}, Y={y}')
        try:
            cos_q2 = (x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
            if cos_q2 > 1.0 or cos_q2 < -1.0:
                self.get_logger().error('Punto fuera del espacio de trabajo')
                response.q1 = 0.0
                response.q2 = 0.0
                return response
            sin_q2 = math.sqrt(1 - cos_q2**2) 
            q2 = math.atan2(sin_q2, cos_q2)
            k1 = self.l1 + self.l2 * cos_q2
            k2 = self.l2 * sin_q2
            q1 = math.atan2(y, x) - math.atan2(k2, k1)
            response.q1 = q1
            response.q2 = q2
            self.get_logger().info(f'Resultado IK: q1={math.degrees(q1):.2f}°, q2={math.degrees(q2):.2f}°')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            response.q1 = 0.0
            response.q2 = 0.0
        return response

def main(args=None):
    rclpy.init(args=args)
    node = IKServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
