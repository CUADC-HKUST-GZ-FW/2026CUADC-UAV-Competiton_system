import rclpy
from rclpy.node import Node

from uav_interfaces.srv import SimulatedRelease


class PayloadSimNode(Node):
    def __init__(self):
        super().__init__('payload_sim_node')

        self.srv = self.create_service(
            SimulatedRelease,
            '/payload/simulated_release',
            self.handle_simulated_release
        )

        self.get_logger().info(
            '[PAYLOAD][task=none][state=IDLE] simulator started '
            'controls_servo=false'
        )

    def handle_simulated_release(self, request, response):
        self.get_logger().warning(
            f'[PAYLOAD][task={request.target_id or "unknown"}]'
            '[state=SIMULATED_RELEASE] simulated release requested '
            f'lat={request.latitude:.7f} '
            f'lon={request.longitude:.7f}'
        )

        response.success = True
        response.message = 'Simulated release event recorded. No actuator command was sent.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PayloadSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
