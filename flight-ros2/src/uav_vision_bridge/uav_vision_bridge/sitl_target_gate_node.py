import json
import math
import os
import time

import rclpy
from mavros_msgs.msg import WaypointList
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool
from std_msgs.msg import String

from uav_interfaces.msg import TargetCommand


class SitlTargetGate(Node):
    """SITL-only gate that publishes a JSON target after MissionManager STANDBY."""

    STATE_WAIT_STANDBY = 'WAIT_STANDBY'
    STATE_WAIT_TARGET_FILE = 'WAIT_TARGET_FILE'
    STATE_PUBLISH_TARGET = 'PUBLISH_TARGET'
    STATE_DONE = 'DONE'
    STATE_ERROR = 'ERROR'

    def __init__(self):
        super().__init__('sitl_target_gate_node')

        self.declare_parameter('result_root', '/tmp/recon_results/latest')
        self.declare_parameter('target_id', 'target_001')
        self.declare_parameter('heading_deg', 0.0)
        self.declare_parameter('manager_state_topic', '/mission_manager/state')
        self.declare_parameter('target_valid_topic', '/mission_manager/target_valid')
        self.declare_parameter('required_manager_state', 'STANDBY')
        self.declare_parameter('standby_stable_sec', 0.5)
        self.declare_parameter('standby_timeout_sec', 30.0)
        self.declare_parameter('publish_period_sec', 0.3)
        self.declare_parameter('max_publish_attempts', 3)
        self.declare_parameter('timer_period_sec', 0.1)

        self.result_root = str(self.get_parameter('result_root').value)
        self.target_id = str(self.get_parameter('target_id').value)
        self.heading_deg = float(self.get_parameter('heading_deg').value)
        self.required_manager_state = str(
            self.get_parameter('required_manager_state').value
        )
        self.standby_stable_sec = float(
            self.get_parameter('standby_stable_sec').value
        )
        self.standby_timeout_sec = float(
            self.get_parameter('standby_timeout_sec').value
        )
        self.publish_period_sec = float(
            self.get_parameter('publish_period_sec').value
        )
        self.max_publish_attempts = int(
            self.get_parameter('max_publish_attempts').value
        )
        self.timer_period_sec = float(
            self.get_parameter('timer_period_sec').value
        )

        self.result_file = os.path.join(
            self.result_root,
            self.target_id,
            'result.json',
        )

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        target_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            String,
            str(self.get_parameter('manager_state_topic').value),
            self.manager_state_callback,
            state_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('target_valid_topic').value),
            self.target_valid_callback,
            state_qos,
        )
        self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.mission_waypoints_callback,
            10,
        )
        self.publisher = self.create_publisher(
            TargetCommand,
            '/vision/target_command',
            target_qos,
        )

        self.state = self.STATE_WAIT_STANDBY
        self.started_at = time.monotonic()
        self.current_manager_state = None
        self.standby_since = None
        self.publish_attempts = 0
        self.last_publish_time = None
        self.target_accepted = False
        self.last_wait_file_log_time = 0.0
        self.mission_ready = False
        self.mission_count = 0
        self.mission_ready_logged = False
        self.wait_mission_sync_logged = False

        self.create_timer(self.timer_period_sec, self.timer_callback)

        self.get_logger().info(
            '[SITL_TARGET_GATE] started '
            f'result={self.result_file} '
            f'heading_deg={self.heading_deg:.2f} '
            f'waiting_for={self.required_manager_state}'
        )

    @staticmethod
    def _valid_lat_lon(latitude, longitude):
        return (
            math.isfinite(latitude)
            and math.isfinite(longitude)
            and -90.0 <= latitude <= 90.0
            and -180.0 <= longitude <= 180.0
        )

    @staticmethod
    def _valid_heading(heading_deg):
        return math.isfinite(heading_deg)

    def manager_state_callback(self, msg):
        manager_state = str(msg.data)
        previous = self.current_manager_state
        self.current_manager_state = manager_state

        if manager_state == self.required_manager_state:
            if self.standby_since is None:
                self.standby_since = time.monotonic()
                self.get_logger().info(
                    '[SITL_TARGET_GATE] '
                    f'mission_manager state={manager_state}'
                )
        else:
            if previous == self.required_manager_state:
                self.get_logger().info(
                    '[SITL_TARGET_GATE] standby interrupted '
                    f'new_state={manager_state}'
                )
            self.standby_since = None
            self.wait_mission_sync_logged = False

    def mission_waypoints_callback(self, msg):
        self.mission_count = len(msg.waypoints)
        self.mission_ready = self.mission_count > 0
        if self.mission_ready and not self.mission_ready_logged:
            self.get_logger().info(
                '[SITL_TARGET_GATE] mavros mission ready '
                f'count={self.mission_count}'
            )
            self.mission_ready_logged = True
        elif not self.mission_ready:
            self.mission_ready_logged = False

    def standby_ready(self, now):
        if self.current_manager_state != self.required_manager_state:
            return False
        if self.standby_since is None:
            return False
        return now - self.standby_since >= self.standby_stable_sec

    def target_valid_callback(self, msg):
        if bool(msg.data) and self.state == self.STATE_PUBLISH_TARGET:
            self.target_accepted = True
            self.state = self.STATE_DONE
            self.get_logger().info(
                '[SITL_TARGET_GATE] target accepted by mission_manager '
                f'target_id={self.target_id}'
            )

    def load_target_command(self):
        if not os.path.exists(self.result_file):
            return None, 'target_json_missing'

        try:
            with open(self.result_file, 'r', encoding='utf-8') as file:
                result = json.load(file)
        except Exception as error:
            return None, f'target_json_read_failed:{error}'

        if not result.get('valid', False):
            return None, 'target_json_valid_false'
        if result.get('status') != 'confirmed':
            return None, 'target_json_status_not_confirmed'

        try:
            coordinate = result['coordinate']
            latitude = float(coordinate['latitude'])
            longitude = float(coordinate['longitude'])
        except (KeyError, TypeError, ValueError) as error:
            return None, f'target_json_coordinate_invalid:{error}'

        heading = result.get(
            'heading_deg',
            result.get('heading', self.heading_deg),
        )
        try:
            heading_deg = float(heading)
        except (TypeError, ValueError) as error:
            return None, f'target_json_heading_invalid:{error}'

        if not self._valid_lat_lon(latitude, longitude):
            return None, 'target_json_lat_lon_out_of_range'
        if not self._valid_heading(heading_deg):
            return None, 'target_json_heading_not_finite'

        msg = TargetCommand()
        msg.latitude = latitude
        msg.longitude = longitude
        msg.heading_deg = heading_deg
        return msg, 'ok'

    def timer_callback(self):
        if self.state in {self.STATE_DONE, self.STATE_ERROR}:
            return

        now = time.monotonic()
        if (
            self.state == self.STATE_WAIT_STANDBY
            and now - self.started_at > self.standby_timeout_sec
        ):
            self.state = self.STATE_ERROR
            self.get_logger().error(
                '[SITL_TARGET_GATE][ERROR] standby timeout '
                f'timeout_sec={self.standby_timeout_sec:.1f} '
                'target NOT published'
            )
            return

        if self.state == self.STATE_WAIT_STANDBY:
            if self.standby_since is None:
                return
            stable_sec = now - self.standby_since
            if stable_sec < self.standby_stable_sec:
                return
            self.state = self.STATE_WAIT_TARGET_FILE
            self.get_logger().info(
                '[SITL_TARGET_GATE] standby confirmed '
                f'stable_sec={stable_sec:.2f}'
            )

        if self.state == self.STATE_WAIT_TARGET_FILE:
            if not self.standby_ready(now):
                self.state = self.STATE_WAIT_STANDBY
                self.wait_mission_sync_logged = False
                return
            if not self.mission_ready:
                if not self.wait_mission_sync_logged:
                    self.get_logger().info(
                        '[SITL_TARGET_GATE] standby confirmed but mission not ready '
                        'waiting_for=mavros_mission_sync'
                    )
                    self.wait_mission_sync_logged = True
                return
            msg, reason = self.load_target_command()
            if msg is None:
                if now - self.last_wait_file_log_time >= 2.0:
                    self.get_logger().warning(
                        '[SITL_TARGET_GATE] waiting_for=target_json '
                        f'reason={reason} file={self.result_file}'
                    )
                    self.last_wait_file_log_time = now
                return
            self.state = self.STATE_PUBLISH_TARGET
            self.cached_target = msg
            self.get_logger().info(
                '[SITL_TARGET_GATE] target json valid '
                f'target_id={self.target_id} '
                f'lat={msg.latitude:.7f} lon={msg.longitude:.7f} '
                f'heading={msg.heading_deg:.2f}'
            )

        if self.state == self.STATE_PUBLISH_TARGET:
            if not self.standby_ready(now):
                self.state = self.STATE_WAIT_STANDBY
                self.wait_mission_sync_logged = False
                return
            if not self.mission_ready:
                self.state = self.STATE_WAIT_TARGET_FILE
                self.wait_mission_sync_logged = False
                return
            if self.target_accepted:
                self.state = self.STATE_DONE
                return
            if self.publish_attempts >= self.max_publish_attempts:
                self.state = self.STATE_DONE
                self.get_logger().warning(
                    '[SITL_TARGET_GATE][WARN] target not confirmed '
                    f'attempts={self.publish_attempts}'
                )
                return
            if (
                self.last_publish_time is not None
                and now - self.last_publish_time < self.publish_period_sec
            ):
                return

            self.publish_attempts += 1
            self.last_publish_time = now
            self.publisher.publish(self.cached_target)
            self.get_logger().info(
                '[SITL_TARGET_GATE] target publish '
                f'attempt={self.publish_attempts}/{self.max_publish_attempts} '
                f'target_id={self.target_id}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SitlTargetGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
