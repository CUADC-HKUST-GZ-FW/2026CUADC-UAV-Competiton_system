import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped


class FcuStatusNode(Node):
    def __init__(self):
        super().__init__('fcu_status_node')

        self.state = None
        self.gps = None
        self.local_pose = None

        self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pose_callback,
            10
        )

        self.create_timer(1.0, self.print_status)

    def state_callback(self, msg):
        self.state = msg

    def gps_callback(self, msg):
        self.gps = msg

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def print_status(self):
        if self.state is None:
            self.get_logger().warn('No MAVROS state received yet.')
            return

        state_text = (
            f'connected={self.state.connected}, '
            f'armed={self.state.armed}, '
            f'mode={self.state.mode}'
        )

        gps_text = 'GPS: no data'
        if self.gps is not None:
            gps_text = (
                f'GPS: lat={self.gps.latitude:.7f}, '
                f'lon={self.gps.longitude:.7f}, '
                f'alt={self.gps.altitude:.2f}'
            )

        local_text = 'Local pose: no data'
        if self.local_pose is not None:
            p = self.local_pose.pose.position
            local_text = f'Local: x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}'

        self.get_logger().info(f'{state_text} | {gps_text} | {local_text}')


def main(args=None):
    rclpy.init(args=args)
    node = FcuStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()