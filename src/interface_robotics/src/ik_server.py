#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interface_robotics.srv import MoveArm
import math


class IKServer(Node):
    def __init__(self):
        super().__init__('ik_server')
        self.srv = self.create_service(MoveArm, 'move_arm', self.handle_move_arm)

        # Parámetros del robot planar RR (deben coincidir con forward_kinematics.py)
        self.l1 = 0.5  # Longitud eslabón 1
        self.l2 = 0.3  # Longitud eslabón 2

        self.get_logger().info('Servidor de Cinemática Inversa listo.')

    def handle_move_arm(self, request, response):
        x = request.x
        y = request.y

        # Cinemática Inversa para robot planar RR
        # cos(q2) = (x² + y² - l1² - l2²) / (2 * l1 * l2)
        cos_q2 = (x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)

        if abs(cos_q2) > 1.0:
            self.get_logger().warn(
                f'Posición ({x:.3f}, {y:.3f}) fuera del espacio de trabajo del robot.'
            )
            response.q1 = float('nan')
            response.q2 = float('nan')
            return response

        # Se toma la solución "codo arriba" (sin_q2 positivo)
        sin_q2 = math.sqrt(1.0 - cos_q2**2)
        q2 = math.atan2(sin_q2, cos_q2)

        # q1 = atan2(y, x) - atan2(l2*sin(q2), l1 + l2*cos(q2))
        k1 = self.l1 + self.l2 * cos_q2
        k2 = self.l2 * sin_q2
        q1 = math.atan2(y, x) - math.atan2(k2, k1)

        response.q1 = q1
        response.q2 = q2

        self.get_logger().info(
            f'IK: ({x:.3f}, {y:.3f}) -> q1={math.degrees(q1):.2f}°, q2={math.degrees(q2):.2f}°'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IKServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
