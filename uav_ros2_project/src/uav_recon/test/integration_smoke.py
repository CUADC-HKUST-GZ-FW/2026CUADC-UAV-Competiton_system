"""Jetson-only smoke test for the complete ROS join and output path."""

import json
from pathlib import Path
import tempfile
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GPSRAW
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from uav_recon.recon_node import ReconGeolocatorNode


def main():
    with tempfile.TemporaryDirectory(prefix='uav_recon_smoke_') as directory:
        root = Path(directory)
        crops = root / 'crops'
        output = root / 'output'
        crops.mkdir()
        frame_path = root / 'frame.jpg'
        crop_path = crops / 'crop_00.jpg'
        frame_path.write_bytes(b'\xff\xd8\xff\xd9')
        crop_path.write_bytes(b'\xff\xd8\xff\xd9')
        manifest_path = crops / 'manifest.json'
        event_log_path = root / 'recognition_events.jsonl'
        event_log_path.write_text('', encoding='utf-8')

        rclpy.init(args=[
            '--ros-args',
            '-p', f'manifest_path:={manifest_path}',
            '-p', f'recognition_event_log_path:={event_log_path}',
            '-p', f'frame_source_path:={frame_path}',
            '-p', f'output_root:={output}',
            '-p', 'calibration_valid:=true',
            '-p', 'position_solution_mode:=global_interpolation',
            '-p', 'ground_altitude_mode:=fixed_msl',
            '-p', 'fixed_ground_altitude_msl_m:=0.0',
        ])
        recon = ReconGeolocatorNode()
        source = Node('recon_smoke_source')
        position_pub = source.create_publisher(NavSatFix, '/mavros/global_position/global', 10)
        pose_pub = source.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)
        gps_pub = source.create_publisher(GPSRAW, '/mavros/gpsstatus/gps1/raw', 10)
        executor = SingleThreadedExecutor()
        executor.add_node(recon)
        executor.add_node(source)

        try:
            base_time = time.time() + 0.2
            for sequence in range(-2, 17):
                stamp_value = base_time + sequence / 60.0
                stamp = source.get_clock().now().to_msg()
                stamp.sec = int(stamp_value)
                stamp.nanosec = int((stamp_value - stamp.sec) * 1e9)
                position = NavSatFix()
                position.header.stamp = stamp
                position.status.status = NavSatStatus.STATUS_FIX
                position.latitude = 22.88480535
                position.longitude = 113.49559465
                position.altitude = 35.0
                pose = PoseStamped()
                pose.header.stamp = stamp
                pose.pose.orientation.w = 1.0
                gps = GPSRAW()
                gps.header.stamp = stamp
                gps.fix_type = GPSRAW.GPS_FIX_TYPE_RTK_FIXED
                position_pub.publish(position)
                pose_pub.publish(pose)
                gps_pub.publish(gps)
                executor.spin_once(timeout_sec=0.01)

            with event_log_path.open('a', encoding='utf-8') as stream:
                for sequence in range(14):
                    capture_time = base_time + sequence / 60.0
                    event = {
                        'record_type': 'recognition',
                        'det_found': True,
                        'frame': sequence,
                        'capture_timestamp_unix_ns': int(capture_time * 1e9),
                        'capture_monotonic_ns': time.monotonic_ns(),
                        'capture_clock_source': 'smoke_test',
                        'source_sequence': sequence,
                        'frame_width': 1440,
                        'frame_height': 1080,
                        'mode': 'digit',
                        'detections': [{
                            'detection_index': 0,
                            'score': 0.96,
                            'roi': [670 + sequence, 490, 100, 100],
                            'center': [719.5, 539.5],
                            'class_id': 79,
                            'class_label': '79',
                            'class_prob': 0.982,
                        }],
                    }
                    stream.write(json.dumps(event) + '\n')
                stream.flush()

            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.02)

            result_path = output / 'target_001' / 'result.json'
            result = json.loads(result_path.read_text(encoding='utf-8'))
            assert result['recognition']['label'] == '79'
            assert result['observation_count'] == 14
            assert result['frame_packet_fusion']['tracking_observation_count'] == 14
            assert result['frame_packet_fusion']['geolocated_observation_count'] == 14
            assert result['rtk_fixed'] is True
            assert result['valid'] is True
            assert (output / result['frame_path']).is_file()
            assert (output / result['crop_path']).is_file()
            print(json.dumps(result, indent=2))
        finally:
            executor.remove_node(source)
            executor.remove_node(recon)
            source.destroy_node()
            recon.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
