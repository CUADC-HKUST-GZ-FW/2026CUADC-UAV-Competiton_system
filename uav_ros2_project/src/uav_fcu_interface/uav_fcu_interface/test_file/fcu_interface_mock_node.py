# 模拟

import rclpy
from rclpy.node import Node

from uav_interfaces.srv import GoToGlobal


class FcuInterfaceMockNode(Node):
    def __init__(self):
        super().__init__('fcu_interface_mock_node')

        self.goto_srv = self.create_service(
            GoToGlobal,
            '/fcu/goto_global',
            self.handle_goto_global
        )

        self.get_logger().info(
            'FCU mock interface started. No real MAVROS command will be sent.'
        )

    def handle_goto_global(self, request, response):
        self.get_logger().info(
            'MOCK GOTO | '
            f'lat={request.latitude:.7f}, '
            f'lon={request.longitude:.7f}, '
            f'alt={request.altitude:.2f}, '
            f'radius={request.acceptance_radius:.2f}'
        )

        response.success = True
        response.message = 'Mock goto accepted. No real flight command was sent.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FcuInterfaceMockNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
