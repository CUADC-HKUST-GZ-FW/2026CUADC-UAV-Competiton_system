#!/usr/bin/env python3
"""Record at most 300 seconds from the Hikrobot camera directly to disk."""

import argparse
import fcntl
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from ctypes import POINTER, byref, c_ubyte, cast, memset, sizeof
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
GEO_ROOT = ROOT / "geo_bridge"
if str(GEO_ROOT) not in sys.path:
    sys.path.insert(0, str(GEO_ROOT))

os.environ.setdefault("MVCAM_COMMON_RUNENV", "/opt/MVS/lib")
sys.path.insert(0, "/opt/MVS/Samples/aarch64/Python")

import cv2
import numpy as np
import yaml

from MvImport.MvCameraControl_class import *
from MvImport.PixelType_header import (
    PixelType_Gvsp_BGR8_Packed,
    PixelType_Gvsp_BayerBG8,
    PixelType_Gvsp_BayerGB8,
    PixelType_Gvsp_BayerGR8,
    PixelType_Gvsp_BayerRG8,
    PixelType_Gvsp_Mono8,
    PixelType_Gvsp_RGB8_Packed,
)


EXPECTED_WIDTH = 1440
EXPECTED_HEIGHT = 1080
MAX_DURATION_SECONDS = 300.0
MIN_FREE_BYTES = 2 * 1024**3
LOCK_PATH = "/tmp/youth_camera_recording.lock"
DEFAULT_CAMERA_CONFIG = ROOT / "configs" / "camera_capture.yaml"
DEFAULT_PIPELINE_CONFIG = ROOT / "configs" / "youth_pipeline.yaml"
stop_requested = False


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record the 1440x1080 Hikrobot stream for no more than 300 seconds."
    )
    parser.add_argument("--duration", type=float, default=MAX_DURATION_SECONDS)
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--bitrate-kbps", type=int, default=12000)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--camera-config",
        type=Path,
        default=Path(os.environ.get("YOUTH_CAMERA_CONFIG", DEFAULT_CAMERA_CONFIG)),
        help="Shared camera parameter config file.",
    )
    parser.add_argument(
        "--pipeline-config",
        type=Path,
        default=Path(os.environ.get("YOUTH_PIPELINE_CONFIG", DEFAULT_PIPELINE_CONFIG)),
        help="Vision pipeline config used to select the same MVS camera.",
    )
    parser.add_argument(
        "--serial",
        default=os.environ.get("YOUTH_CAMERA_SERIAL", ""),
        help="MVS camera serial; defaults to mvs_serial in the pipeline config.",
    )
    parser.add_argument(
        "--sidecar",
        type=Path,
        help="If set, write one lat/lon/alt JSON object per captured frame.",
    )
    parser.add_argument(
        "--rel-alt-topic",
        default="/mavros/global_position/rel_alt",
    )
    parser.add_argument(
        "--global-topic",
        default="/mavros/global_position/global",
    )
    args = parser.parse_args()
    if not 1.0 <= args.duration <= MAX_DURATION_SECONDS:
        parser.error("--duration must be between 1 and 300 seconds")
    if not 1.0 <= args.fps <= 60.0:
        parser.error("--fps must be between 1 and 60")
    if not 128 <= args.bitrate_kbps <= 16384:
        parser.error("--bitrate-kbps must be between 128 and 16384")
    return args


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def request_stop(_signum, _frame):
    global stop_requested
    stop_requested = True


def require_ok(result, operation):
    if result != 0:
        raise RuntimeError(f"{operation} failed: 0x{result:08x}")


def _parse_scalar(value):
    value = value.strip().strip("\"'")
    lowered = value.lower()
    if lowered in {"true", "yes", "on"}:
        return True
    if lowered in {"false", "no", "off"}:
        return False
    try:
        if any(ch in value for ch in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def load_camera_config(path):
    config = {
        "exposure_auto": "continuous",
        "exposure_time_us": 1000.0,
        "auto_exposure_lower_us": 15,
        "auto_exposure_upper_us": 1500,
    }
    if not path:
        return config
    path = Path(path).expanduser()
    if not path.exists():
        print(f"WARNING camera config does not exist: {path}; using defaults", flush=True)
        return config
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        config[key.strip().lower()] = _parse_scalar(value)
    config["exposure_auto"] = str(config.get("exposure_auto", "continuous")).lower()
    config["exposure_time_us"] = float(config.get("exposure_time_us", 1000.0))
    config["auto_exposure_lower_us"] = int(config.get("auto_exposure_lower_us", 15))
    config["auto_exposure_upper_us"] = int(config.get("auto_exposure_upper_us", 1500))
    return config


def set_if_supported(camera, method, name, value):
    try:
        result = getattr(camera, method)(name, value)
    except Exception as exc:
        print(f"WARNING {name} was not changed: {exc}", flush=True)
        return
    if result != 0:
        print(f"WARNING {name} was not changed: 0x{result:08x}", flush=True)


def set_auto_exposure_limit(camera, which, value):
    node_names = [
        f"AutoExposureTime{which}Limit",
        f"AutoExposureTime{which}",
        f"ExposureTime{which}Limit",
        f"ExposureTime{which}",
    ]
    for node_name in node_names:
        for method_name, cast_value in (
            ("MV_CC_SetIntValue", int(value)),
            ("MV_CC_SetFloatValue", float(value)),
        ):
            method = getattr(camera, method_name, None)
            if method is None:
                continue
            try:
                result = method(node_name, cast_value)
            except Exception:
                continue
            if result == 0:
                return node_name
    print(f"WARNING auto exposure {which.lower()} limit was not changed", flush=True)
    return None


def exposure_auto_enum(mode):
    mapping = {
        "off": MV_EXPOSURE_AUTO_MODE_OFF,
        "once": MV_EXPOSURE_AUTO_MODE_ONCE,
        "continuous": MV_EXPOSURE_AUTO_MODE_CONTINUOUS,
    }
    if mode not in mapping:
        raise RuntimeError(f"unsupported exposure_auto: {mode}")
    return mapping[mode]


def apply_camera_config(camera, config):
    mode = str(config["exposure_auto"]).lower()
    exposure_time_us = float(config["exposure_time_us"])
    lower_us = int(config["auto_exposure_lower_us"])
    upper_us = int(config["auto_exposure_upper_us"])
    if lower_us > upper_us:
        raise RuntimeError("auto_exposure_lower_us must not exceed auto_exposure_upper_us")

    set_if_supported(camera, "MV_CC_SetEnumValue", "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF)
    set_if_supported(camera, "MV_CC_SetFloatValue", "ExposureTime", exposure_time_us)
    lower_node = set_auto_exposure_limit(camera, "Lower", lower_us)
    upper_node = set_auto_exposure_limit(camera, "Upper", upper_us)
    set_if_supported(camera, "MV_CC_SetEnumValue", "ExposureAuto", exposure_auto_enum(mode))

    print(
        "CAMERA_CONFIG_APPLIED "
        f"exposure_auto={mode} exposure_time_us={exposure_time_us:.1f} "
        f"auto_exposure_lower_us={lower_us} auto_exposure_upper_us={upper_us} "
        f"lower_node={lower_node or 'unapplied'} upper_node={upper_node or 'unapplied'}",
        flush=True,
    )
    return {
        "exposure_auto": mode,
        "exposure_time_us": exposure_time_us,
        "auto_exposure_lower_us": lower_us,
        "auto_exposure_upper_us": upper_us,
        "auto_exposure_lower_node": lower_node,
        "auto_exposure_upper_node": upper_node,
    }


def camera_text(value):
    raw = bytes(value)
    return raw.split(b"\0", 1)[0].decode("utf-8", errors="replace")


def load_pipeline_serial(path):
    path = Path(path).expanduser()
    if not path.exists():
        raise RuntimeError(f"pipeline config does not exist: {path}")
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    serial = str(payload.get("mvs_serial", "")).strip()
    if not serial:
        raise RuntimeError(f"mvs_serial is missing from pipeline config: {path}")
    return serial


def mvs_device_serial(device):
    if device.nTLayerType == MV_USB_DEVICE:
        return camera_text(device.SpecialInfo.stUsb3VInfo.chSerialNumber)
    if device.nTLayerType == MV_GIGE_DEVICE:
        return camera_text(device.SpecialInfo.stGigEInfo.chSerialNumber)
    return ""


def mvs_device_model(device):
    if device.nTLayerType == MV_USB_DEVICE:
        return camera_text(device.SpecialInfo.stUsb3VInfo.chModelName)
    if device.nTLayerType == MV_GIGE_DEVICE:
        return camera_text(device.SpecialInfo.stGigEInfo.chModelName)
    return "unknown"


def mvs_device_transport(device):
    if device.nTLayerType == MV_USB_DEVICE:
        return "USB3Vision"
    if device.nTLayerType == MV_GIGE_DEVICE:
        return "GigE"
    return "unknown"


def choose_mvs_camera(device_list, preferred_serial):
    devices = []
    for index in range(device_list.nDeviceNum):
        device = cast(
            device_list.pDeviceInfo[index], POINTER(MV_CC_DEVICE_INFO)
        ).contents
        if device.nTLayerType not in (MV_USB_DEVICE, MV_GIGE_DEVICE):
            continue
        serial = mvs_device_serial(device)
        devices.append((device, serial))
        print(
            "CAMERA_DISCOVERED "
            f"index={index} transport={mvs_device_transport(device)} "
            f"model={mvs_device_model(device)} serial={serial or 'unknown'}",
            flush=True,
        )
        if serial == preferred_serial:
            return device, serial
    available = ", ".join(serial or "unknown" for _, serial in devices)
    if not devices:
        raise RuntimeError("no GigE/USB MVS camera was found")
    if preferred_serial:
        raise RuntimeError(
            f"MVS camera serial {preferred_serial} was not found; "
            f"available serials: [{available}]"
        )
    return devices[0]


def read_int(camera, name):
    value = MVCC_INTVALUE()
    memset(byref(value), 0, sizeof(value))
    require_ok(camera.MV_CC_GetIntValue(name, value), f"read {name}")
    return int(value.nCurValue)


def read_enum(camera, name):
    value = MVCC_ENUMVALUE()
    memset(byref(value), 0, sizeof(value))
    require_ok(camera.MV_CC_GetEnumValue(name, value), f"read {name}")
    return int(value.nCurValue)


def read_float(camera, name, fallback):
    value = MVCC_FLOATVALUE()
    memset(byref(value), 0, sizeof(value))
    result = camera.MV_CC_GetFloatValue(name, value)
    return float(value.fCurValue) if result == 0 else float(fallback)


def default_output_path():
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.home() / "camera_recordings" / (
        f"camera_{stamp}_{EXPECTED_WIDTH}x{EXPECTED_HEIGHT}_300s.mp4"
    )


def update_latest_link(output_path):
    link_path = output_path.parent / "latest_1080p.mp4"
    temporary_link = output_path.parent / ".latest_1080p.mp4.tmp"
    temporary_link.unlink(missing_ok=True)
    temporary_link.symlink_to(output_path.name)
    os.replace(temporary_link, link_path)


def start_telemetry_node(rel_alt_topic, global_topic):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import Float64

    from geo_bridge.telemetry import TelemetryBuffer

    rclpy.init()
    buffer = TelemetryBuffer()

    class CameraGeoNode(Node):
        def __init__(self):
            super().__init__("camera_geo_sidecar")
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.create_subscription(Float64, rel_alt_topic, self._rel_alt_cb, qos)
            self.create_subscription(NavSatFix, global_topic, self._global_cb, qos)

        def _rel_alt_cb(self, msg):
            buffer.update_rel_alt(time.time(), msg.data)

        def _global_cb(self, msg):
            buffer.update_global(time.time(), msg.latitude, msg.longitude, msg.altitude)

    node = CameraGeoNode()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return buffer, node


def stop_telemetry_node(node):
    if node is None:
        return
    import rclpy

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def build_gstreamer_command(output_path, width, height, fps, bitrate_kbps):
    max_input_bytes = width * height * 3 * 2
    fps_integer = int(round(fps))
    return [
        "gst-launch-1.0",
        "-q",
        "-e",
        "fdsrc",
        "fd=0",
        f"blocksize={max_input_bytes}",
        "!",
        "rawvideoparse",
        "format=bgr",
        f"width={width}",
        f"height={height}",
        f"framerate={fps_integer}/1",
        "!",
        "queue",
        "max-size-buffers=2",
        "max-size-bytes=0",
        "max-size-time=0",
        "!",
        "videoconvert",
        "n-threads=4",
        "!",
        "video/x-raw,format=BGRx",
        "!",
        "nvvidconv",
        "!",
        "video/x-raw(memory:NVMM),format=NV12",
        "!",
        "nvv4l2h264enc",
        "maxperf-enable=true",
        f"bitrate={bitrate_kbps * 1000}",
        "control-rate=1",
        "preset-level=1",
        f"iframeinterval={fps_integer}",
        "!",
        "h264parse",
        "!",
        "qtmux",
        "!",
        "filesink",
        f"location={output_path}",
        "sync=false",
    ]


class GStreamerWriter:
    def __init__(self, output_path, width, height, fps, bitrate_kbps):
        command = build_gstreamer_command(
            output_path, width, height, fps, bitrate_kbps
        )
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
        )
        time.sleep(0.2)
        if self.process.poll() is not None:
            raise RuntimeError(
                f"GStreamer H.264 encoder exited during startup: {self.process.returncode}"
            )

    def write(self, frame):
        if self.process.poll() is not None:
            raise RuntimeError(
                f"GStreamer H.264 encoder exited: {self.process.returncode}"
            )
        try:
            self.process.stdin.write(memoryview(frame).cast("B"))
        except BrokenPipeError as exc:
            raise RuntimeError("GStreamer H.264 encoder closed its input") from exc

    def release(self):
        if self.process.stdin is not None and not self.process.stdin.closed:
            self.process.stdin.close()
        try:
            return_code = self.process.wait(timeout=20)
        except subprocess.TimeoutExpired as exc:
            self.process.terminate()
            self.process.wait(timeout=5)
            raise RuntimeError("GStreamer H.264 encoder did not finalize") from exc
        if return_code != 0:
            raise RuntimeError(f"GStreamer H.264 encoder failed: {return_code}")


BAYER_CONVERSIONS = {
    PixelType_Gvsp_BayerBG8: cv2.COLOR_BayerBG2BGR,
    PixelType_Gvsp_BayerGB8: cv2.COLOR_BayerGB2BGR,
    PixelType_Gvsp_BayerGR8: cv2.COLOR_BayerGR2BGR,
    PixelType_Gvsp_BayerRG8: cv2.COLOR_BayerRG2BGR,
}


def convert_frame(raw_view, frame_length, pixel_type, width, height, bgr):
    pixels = width * height
    if pixel_type in BAYER_CONVERSIONS:
        if frame_length < pixels:
            raise RuntimeError(f"short Bayer frame: {frame_length} bytes")
        raw = raw_view[:pixels].reshape(height, width)
        cv2.cvtColor(raw, BAYER_CONVERSIONS[pixel_type], bgr)
        return
    if pixel_type == PixelType_Gvsp_Mono8:
        if frame_length < pixels:
            raise RuntimeError(f"short Mono8 frame: {frame_length} bytes")
        raw = raw_view[:pixels].reshape(height, width)
        cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR, bgr)
        return
    if pixel_type in (PixelType_Gvsp_RGB8_Packed, PixelType_Gvsp_BGR8_Packed):
        if frame_length < pixels * 3:
            raise RuntimeError(f"short packed color frame: {frame_length} bytes")
        raw = raw_view[: pixels * 3].reshape(height, width, 3)
        if pixel_type == PixelType_Gvsp_RGB8_Packed:
            cv2.cvtColor(raw, cv2.COLOR_RGB2BGR, bgr)
        else:
            np.copyto(bgr, raw)
        return
    raise RuntimeError(f"unsupported camera pixel type: {pixel_type}")


def main():
    args = parse_args()
    preferred_serial = args.serial.strip() or load_pipeline_serial(args.pipeline_config)
    output_path = (args.output or default_output_path()).expanduser().resolve()
    if output_path.suffix.lower() != ".mp4":
        raise RuntimeError("hardware recording output must use the .mp4 extension")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        raise RuntimeError(f"refusing to overwrite {output_path}")
    if shutil.disk_usage(output_path.parent).free < MIN_FREE_BYTES:
        raise RuntimeError("less than 2 GiB of free disk space remains")

    partial_path = output_path.with_name(f"{output_path.stem}.partial.mp4")
    partial_path.unlink(missing_ok=True)
    metadata_path = output_path.with_suffix(".json")
    lock_file = open(LOCK_PATH, "w", encoding="ascii")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError("another camera recording is already running") from exc
    lock_file.write(str(os.getpid()))
    lock_file.flush()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    camera = MvCamera()
    sdk_initialized = False
    handle_created = False
    camera_opened = False
    grabbing_started = False
    writer = None
    sidecar = None
    telemetry = None
    ros_node = None
    completed = False
    frame_count = 0
    capture_timeouts = 0
    started_monotonic = None
    started_utc = None
    width = 0
    height = 0
    serial = "unknown"
    resulting_fps = args.fps
    camera_config = load_camera_config(args.camera_config)
    applied_camera_config = {}
    final_exposure_time_us = 0.0

    try:
        require_ok(MvCamera.MV_CC_Initialize(), "initialize MVS SDK")
        sdk_initialized = True

        devices = MV_CC_DEVICE_INFO_LIST()
        require_ok(
            MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, devices),
            "enumerate GigE/USB MVS cameras",
        )
        if devices.nDeviceNum == 0:
            raise RuntimeError("no GigE/USB MVS camera was found")
        device, serial = choose_mvs_camera(devices, preferred_serial)

        require_ok(camera.MV_CC_CreateHandle(device), "create camera handle")
        handle_created = True
        require_ok(camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0), "open camera")
        camera_opened = True

        set_if_supported(camera, "MV_CC_SetEnumValue", "TriggerMode", MV_TRIGGER_MODE_OFF)
        set_if_supported(camera, "MV_CC_SetBoolValue", "AcquisitionFrameRateEnable", True)
        set_if_supported(camera, "MV_CC_SetFloatValue", "AcquisitionFrameRate", args.fps)
        applied_camera_config = apply_camera_config(camera, camera_config)
        try:
            result = camera.MV_CC_SetImageNodeNum(2)
            if result != 0:
                print(f"WARNING image buffer count was not changed: 0x{result:08x}", flush=True)
        except Exception as exc:
            print(f"WARNING image buffer count was not changed: {exc}", flush=True)

        width = read_int(camera, "Width")
        height = read_int(camera, "Height")
        if (width, height) != (EXPECTED_WIDTH, EXPECTED_HEIGHT):
            raise RuntimeError(
                f"camera is {width}x{height}; expected {EXPECTED_WIDTH}x{EXPECTED_HEIGHT}"
            )
        pixel_type = read_enum(camera, "PixelFormat")
        resulting_fps = read_float(camera, "ResultingFrameRate", args.fps)
        writer = GStreamerWriter(
            partial_path, width, height, args.fps, args.bitrate_kbps
        )

        if args.sidecar is not None:
            from geo_bridge.recorder import FrameSidecar

            sidecar_path = args.sidecar.expanduser().resolve()
            if sidecar_path.suffix.lower() != ".jsonl":
                raise RuntimeError("sidecar output must use the .jsonl extension")
            sidecar_path.parent.mkdir(parents=True, exist_ok=True)
            telemetry, ros_node = start_telemetry_node(
                args.rel_alt_topic, args.global_topic
            )
            sidecar = FrameSidecar(str(sidecar_path))
            print(f"SIDECAR_ENABLED path={sidecar_path}", flush=True)

        payload = read_int(camera, "PayloadSize")
        raw_buffer = (c_ubyte * payload)()
        raw_view = np.ctypeslib.as_array(raw_buffer)
        bgr = np.empty((height, width, 3), dtype=np.uint8)
        frame_info = MV_FRAME_OUT_INFO_EX()

        require_ok(camera.MV_CC_StartGrabbing(), "start camera acquisition")
        grabbing_started = True

        started_utc = utc_now()
        started_monotonic = time.monotonic()
        deadline = started_monotonic + args.duration
        next_progress = started_monotonic + 10.0
        print(
            "RECORDING_STARTED "
            f"output={output_path} resolution={width}x{height} "
            f"camera_fps={resulting_fps:.2f} video_fps={args.fps:.2f} "
            f"encoder=nvv4l2h264enc max_seconds={args.duration:.1f}"
            + (
                ""
                if sidecar is None
                else f" sidecar={sidecar.path}"
            ),
            flush=True,
        )

        while not stop_requested and time.monotonic() < deadline:
            memset(byref(frame_info), 0, sizeof(frame_info))
            result = camera.MV_CC_GetOneFrameTimeout(
                raw_buffer, payload, frame_info, 200
            )
            if result != 0:
                capture_timeouts += 1
                continue
            if (frame_info.nWidth, frame_info.nHeight) != (width, height):
                raise RuntimeError(
                    f"camera frame changed to {frame_info.nWidth}x{frame_info.nHeight}"
                )
            convert_frame(
                raw_view,
                frame_info.nFrameLen,
                int(frame_info.enPixelType),
                width,
                height,
                bgr,
            )
            writer.write(bgr)
            if sidecar is not None:
                sidecar.append(frame_count, telemetry.latest, now=time.time())
            frame_count += 1

            now = time.monotonic()
            if now >= next_progress:
                elapsed = now - started_monotonic
                print(
                    f"RECORDING_PROGRESS seconds={elapsed:.1f} "
                    f"frames={frame_count} capture_fps={frame_count / elapsed:.2f}",
                    flush=True,
                )
                next_progress += 10.0
                if shutil.disk_usage(output_path.parent).free < MIN_FREE_BYTES:
                    print("WARNING stopping because free disk space is below 2 GiB", flush=True)
                    break

        completed = not stop_requested and time.monotonic() >= deadline
    finally:
        if camera_opened:
            final_exposure_time_us = read_float(
                camera,
                "ExposureTime",
                applied_camera_config.get("exposure_time_us", 0.0),
            )
        if grabbing_started:
            camera.MV_CC_StopGrabbing()
        if writer is not None:
            writer.release()
        if sidecar is not None:
            sidecar.close()
        stop_telemetry_node(ros_node)
        if camera_opened:
            camera.MV_CC_CloseDevice()
        if handle_created:
            camera.MV_CC_DestroyHandle()
        if sdk_initialized:
            MvCamera.MV_CC_Finalize()

    elapsed = time.monotonic() - started_monotonic if started_monotonic else 0.0
    if not partial_path.exists() or partial_path.stat().st_size == 0:
        raise RuntimeError("recorder produced no video data")
    os.replace(partial_path, output_path)
    metadata = {
        "status": "complete" if completed else "stopped_early",
        "started_utc": started_utc,
        "finished_utc": utc_now(),
        "duration_seconds": round(elapsed, 3),
        "frame_count": frame_count,
        "measured_capture_fps": round(frame_count / elapsed, 3) if elapsed else 0.0,
        "recording_fps": round(resulting_fps, 3),
        "video_fps": round(args.fps, 3),
        "resolution": [width, height],
        "camera_serial": serial,
        "pixel_type": pixel_type,
        "camera_config_path": str(args.camera_config),
        "camera_config": applied_camera_config,
        "final_exposure_time_us": final_exposure_time_us,
        "encoder": "nvv4l2h264enc",
        "bitrate_kbps": args.bitrate_kbps,
        "capture_timeouts": capture_timeouts,
        "video_path": str(output_path),
        "video_bytes": output_path.stat().st_size,
        "sidecar_path": None if sidecar is None else sidecar.path,
        "sidecar_rows": 0 if sidecar is None else sidecar.rows,
    }
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    update_latest_link(output_path)
    print(
        "RECORDING_COMPLETE "
        f"output={output_path} metadata={metadata_path} seconds={elapsed:.2f} "
        f"frames={frame_count} capture_fps={metadata['measured_capture_fps']:.2f} "
        f"size_mb={output_path.stat().st_size / 1024**2:.1f}"
        + (
            ""
            if sidecar is None
            else f" sidecar={sidecar.path} sidecar_rows={sidecar.rows}"
        ),
        flush=True,
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"RECORDING_FAILED {exc}", file=sys.stderr, flush=True)
        sys.exit(1)
