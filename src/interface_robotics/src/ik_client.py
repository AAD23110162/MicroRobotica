#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from interface_robotics.srv import MoveArm


class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.client = self.create_client(MoveArm, 'move_arm')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servidor move_arm...')

    def send_request(self, x: float, y: float):
        request = MoveArm.Request()
        request.x = x
        request.y = y

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is not None:
            import math
            self.get_logger().info(
                f'Respuesta IK: q1={math.degrees(result.q1):.2f}°, '
                f'q2={math.degrees(result.q2):.2f}°'
            )
        else:
            self.get_logger().error('Fallo al obtener respuesta del servidor.')
        return result


def main(args=None):
    rclpy.init(args=args)

    # Leer coordenadas desde argumentos de línea de comandos, o usar valores por defecto
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    else:
        x = 0.4
        y = 0.3

    node = IKClient()
    node.send_request(x, y)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
