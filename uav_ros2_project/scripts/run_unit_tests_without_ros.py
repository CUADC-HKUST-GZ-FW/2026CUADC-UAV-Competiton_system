"""Explicit non-ROS unit runner; message stand-ins only, no ROS integration.

Run with a Python environment containing pytest. On ROS hosts use pytest directly
with the sourced workspace instead. This runner never connects to a flight stack.
"""
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace


def module(name, **attributes):
    value = ModuleType(name)
    value.__dict__.update(attributes)
    sys.modules[name] = value
    return value


class Message(SimpleNamespace):
    pass


class Waypoint(Message):
    def __init__(self):
        super().__init__(frame=0, command=0, is_current=False, autocontinue=False,
                         param1=0., param2=0., param3=0., param4=0.,
                         x_lat=0., y_long=0., z_alt=0.)


def main():
    root = Path(__file__).resolve().parents[1]
    for package in ('uav_fcu_interface', 'uav_payload', 'uav_bringup', 'uav_mission_manager'):
        sys.path.insert(0, str(root / 'src' / package))
    module('rclpy', ok=lambda: True)
    module('rclpy.node', Node=type('Node', (), {}))
    module('rclpy.callback_groups', ReentrantCallbackGroup=Message)
    module('rclpy.executors', MultiThreadedExecutor=Message)
    policy = SimpleNamespace(RELIABLE=1, BEST_EFFORT=2, KEEP_LAST=1,
                             TRANSIENT_LOCAL=1, VOLATILE=2)
    module('rclpy.qos', QoSProfile=Message, ReliabilityPolicy=policy,
           HistoryPolicy=policy, DurabilityPolicy=policy)
    for package, classes in {
        'sensor_msgs': ['NavSatFix'],
        'std_msgs': ['String', 'Float64', 'Bool'],
        'diagnostic_msgs': ['DiagnosticArray'],
        'mavros_msgs': ['State', 'VfrHud', 'WaypointList', 'WaypointReached',
                        'Mavlink', 'EstimatorStatus', 'GPSRAW', 'SysStatus'],
        'uav_interfaces': ['TargetCommand'],
    }.items():
        module(package)
        module(package + '.msg', **{name: Message for name in classes})
    sys.modules['mavros_msgs.msg'].Waypoint = Waypoint
    service = SimpleNamespace(Request=Message)
    module('mavros_msgs.srv', **{name: service for name in (
        'WaypointClear', 'WaypointPush', 'WaypointPull', 'WaypointSetCurrent',
        'VehicleInfoGet')})
    module('std_srvs')
    module('std_srvs.srv', Trigger=service)
    module('uav_interfaces.srv', GoToGlobal=service)
    import pytest
    selected = [
        'src/uav_fcu_interface/uav_fcu_interface/test_abcdr_mission.py',
        'src/uav_fcu_interface/uav_fcu_interface/test_full_mission_update.py',
        'src/uav_fcu_interface/test/test_log_format.py',
        'src/uav_payload/test/test_payload_monitor.py',
    ]
    return pytest.main([str(root / path) for path in selected] + sys.argv[1:])


if __name__ == '__main__':
    raise SystemExit(main())
