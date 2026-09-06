#!/usr/bin/env python3
"""Live Hikrobot MVS preview with continuous auto exposure.

This script is intended for the Jetson / NX163 side that has the MVS SDK
installed. It:

* opens the USB3Vision Hikrobot camera
* loads shared exposure settings from configs/camera_capture.yaml
* streams a live MJPEG preview over HTTP for viewing on a laptop browser
* overlays latency, FPS, resolution and exposure info on the frame

Default usage on the Jetson:

    python3 -u live_mvs_preview.py

Then open on the laptop:

    http://192.168.55.1:8000/

If your network path is unstable, you can also port-forward the HTTP port.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from collections import deque
from ctypes import POINTER, byref, c_ubyte, cast, memset, sizeof
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

os.environ.setdefault("MVCAM_COMMON_RUNENV", "/opt/MVS/lib")
sys.path.insert(0, "/opt/MVS/Samples/aarch64/Python")

import cv2
import numpy as np

from MvImport.MvCameraControl_class import *  # noqa: F401,F403
from MvImport.CameraParams_header import *  # noqa: F401,F403


DEFAULT_SERIAL = os.environ.get("YOUTH_CAMERA_SERIAL", "DA4824869")
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_CAMERA_CONFIG = os.path.join(ROOT, "configs", "camera_capture.yaml")
DEFAULT_EXPOSURE_TIME_US = 1000.0
DEFAULT_AUTO_EXPOSURE_LOWER_US = 15
DEFAULT_AUTO_EXPOSURE_UPPER_US = 1500
DEFAULT_GAIN = 0.0
DEFAULT_FPS = 60.0
DEFAULT_PORT = 8000
DEFAULT_PREVIEW_WIDTH = 960
DEFAULT_JPEG_QUALITY = 80

stop_requested = False


def require_ok(result, operation):
    if result != 0:
        raise RuntimeError(f"{operation} failed: 0x{result:08x}")


def camera_text(value):
    raw = bytes(value)
    return raw.split(b"\0", 1)[0].decode("utf-8", errors="replace")


def parse_scalar(value):
    value = value.strip().strip("\"'")
    lowered = value.lower()
    if lowered in ("true", "yes", "on"):
        return True
    if lowered in ("false", "no", "off"):
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
        "exposure_time_us": DEFAULT_EXPOSURE_TIME_US,
        "auto_exposure_lower_us": DEFAULT_AUTO_EXPOSURE_LOWER_US,
        "auto_exposure_upper_us": DEFAULT_AUTO_EXPOSURE_UPPER_US,
    }
    if not path or not os.path.exists(path):
        return config
    with open(path, "r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            config[key.strip().lower()] = parse_scalar(value)
    return config


def choose_usb_camera(device_list, preferred_serial):
    fallback = None
    for index in range(device_list.nDeviceNum):
        device = cast(
            device_list.pDeviceInfo[index], POINTER(MV_CC_DEVICE_INFO)
        ).contents
        if device.nTLayerType != MV_USB_DEVICE:
            continue
        if fallback is None:
            fallback = device
        serial = camera_text(device.SpecialInfo.stUsb3VInfo.chSerialNumber)
        if preferred_serial and serial == preferred_serial:
            return device, serial
    if fallback is None:
        raise RuntimeError("no USB3Vision camera was found")
    serial = camera_text(fallback.SpecialInfo.stUsb3VInfo.chSerialNumber)
    return fallback, serial


def set_if_supported(camera, method_name, node_name, value):
    method = getattr(camera, method_name, None)
    if method is None:
        return False
    try:
        result = method(node_name, value)
    except Exception as exc:
        raise RuntimeError(f"set {node_name} via {method_name} failed: {exc}") from exc
    if result != 0:
        raise RuntimeError(f"set {node_name} via {method_name} failed: 0x{result:08x}")
    return True


def set_enum(camera, node_name, value):
    require_ok(camera.MV_CC_SetEnumValue(node_name, int(value)), f"set {node_name}")


def set_float(camera, node_name, value):
    require_ok(camera.MV_CC_SetFloatValue(node_name, float(value)), f"set {node_name}")


def set_bool(camera, node_name, value):
    require_ok(camera.MV_CC_SetBoolValue(node_name, bool(value)), f"set {node_name}")


def set_auto_exposure_limit(camera, which, value):
    node_names = [
        f"AutoExposureTime{which}Limit",
        f"AutoExposureTime{which}",
        f"ExposureTime{which}Limit",
        f"ExposureTime{which}",
    ]
    for node_name in node_names:
        for method_name in ("MV_CC_SetIntValue", "MV_CC_SetFloatValue", f"MV_CC_SetAutoExposureTime{which}"):
            method = getattr(camera, method_name, None)
            if method is None:
                continue
            try:
                result = method(node_name, int(value)) if method_name != "MV_CC_SetFloatValue" else method(node_name, float(value))
            except Exception:
                continue
            if result == 0:
                return True
    return False


def read_float(camera, node_name):
    value = MVCC_FLOATVALUE()
    memset(byref(value), 0, sizeof(value))
    require_ok(camera.MV_CC_GetFloatValue(node_name, value), f"read {node_name}")
    return float(value.fCurValue), float(value.fMin), float(value.fMax)


def read_int(camera, node_name):
    value = MVCC_INTVALUE()
    memset(byref(value), 0, sizeof(value))
    require_ok(camera.MV_CC_GetIntValue(node_name, value), f"read {node_name}")
    return int(value.nCurValue), int(value.nMin), int(value.nMax), int(value.nInc)


def read_enum(camera, node_name):
    value = MVCC_ENUMVALUE()
    memset(byref(value), 0, sizeof(value))
    require_ok(camera.MV_CC_GetEnumValue(node_name, value), f"read {node_name}")
    return int(value.nCurValue)


def exposure_auto_name(value):
    mapping = {
        MV_EXPOSURE_AUTO_MODE_OFF: "off",
        MV_EXPOSURE_AUTO_MODE_ONCE: "once",
        MV_EXPOSURE_AUTO_MODE_CONTINUOUS: "continuous",
    }
    return mapping.get(value, str(value))


def gain_auto_name(value):
    mapping = {
        MV_GAIN_MODE_OFF: "off",
        MV_GAIN_MODE_ONCE: "once",
        MV_GAIN_MODE_CONTINUOUS: "continuous",
    }
    return mapping.get(value, str(value))


def parse_args():
    parser = argparse.ArgumentParser(description="Live Hikrobot MVS preview with auto exposure.")
    parser.add_argument("--serial", default=DEFAULT_SERIAL, help="Preferred USB camera serial number.")
    parser.add_argument("--index", type=int, default=0, help="Fallback USB camera index.")
    parser.add_argument("--use-index", action="store_true", help="Select camera by index instead of serial.")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP listen address.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="HTTP listen port.")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS, help="Target acquisition frame rate.")
    parser.add_argument("--preview-width", type=int, default=DEFAULT_PREVIEW_WIDTH, help="Downscale preview width; 0 keeps native width.")
    parser.add_argument("--jpeg-quality", type=int, default=DEFAULT_JPEG_QUALITY, help="MJPEG quality (1-100).")
    parser.add_argument("--camera-config", default=os.environ.get("YOUTH_CAMERA_CONFIG", DEFAULT_CAMERA_CONFIG), help="Shared camera parameter config file.")
    parser.add_argument("--exposure-time", type=float, default=DEFAULT_EXPOSURE_TIME_US, help="Initial exposure time in microseconds.")
    parser.add_argument("--ae-lower", type=int, default=DEFAULT_AUTO_EXPOSURE_LOWER_US, help="Auto-exposure lower bound in microseconds.")
    parser.add_argument("--ae-upper", type=int, default=DEFAULT_AUTO_EXPOSURE_UPPER_US, help="Auto-exposure upper bound in microseconds.")
    parser.add_argument("--gain", type=float, default=DEFAULT_GAIN, help="Manual gain value.")
    parser.add_argument("--gain-auto", choices=["off", "once", "continuous"], default="off", help="Gain auto mode.")
    parser.add_argument("--keep-window", action="store_true", help="Also show a local OpenCV window if a GUI is available.")
    args = parser.parse_args()
    camera_config = load_camera_config(args.camera_config)
    args.exposure_auto = str(camera_config.get("exposure_auto", "continuous")).lower()
    args.exposure_time = float(camera_config.get("exposure_time_us", args.exposure_time))
    args.ae_lower = int(camera_config.get("auto_exposure_lower_us", args.ae_lower))
    args.ae_upper = int(camera_config.get("auto_exposure_upper_us", args.ae_upper))
    if args.exposure_auto not in ("off", "once", "continuous"):
        parser.error("--camera-config exposure_auto must be off, once, or continuous")
    if args.ae_lower > args.ae_upper:
        parser.error("--camera-config auto_exposure_lower_us must not exceed auto_exposure_upper_us")
    return args


class PreviewState:
    def __init__(self):
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.latest_jpeg: Optional[bytes] = None
        self.latest_stats = {}
        self.latest_seq = 0
        self.running = True


class MvsPreviewApp:
    def __init__(self, args):
        self.args = args
        self.camera = MvCamera()
        self.sdk_initialized = False
        self.handle_created = False
        self.opened = False
        self.grabbing = False
        self.state = PreviewState()
        self.camera_serial = "unknown"
        self.frame_window = deque(maxlen=60)
        self.start_mono = time.monotonic()
        self.frame_info = None

    def setup_camera(self):
        require_ok(MvCamera.MV_CC_Initialize(), "initialize MVS SDK")
        self.sdk_initialized = True

        devices = MV_CC_DEVICE_INFO_LIST()
        require_ok(
            MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, devices),
            "enumerate USB cameras",
        )
        if devices.nDeviceNum == 0:
            raise RuntimeError("no USB3Vision camera was found")

        if self.args.use_index:
            if self.args.index < 0 or self.args.index >= devices.nDeviceNum:
                raise RuntimeError(
                    f"camera index {self.args.index} out of range; found {devices.nDeviceNum}"
                )
            device = cast(devices.pDeviceInfo[self.args.index], POINTER(MV_CC_DEVICE_INFO)).contents
        else:
            device, self.camera_serial = choose_usb_camera(devices, self.args.serial)
        if not self.camera_serial:
            self.camera_serial = camera_text(device.SpecialInfo.stUsb3VInfo.chSerialNumber)

        require_ok(self.camera.MV_CC_CreateHandle(device), "create camera handle")
        self.handle_created = True
        require_ok(self.camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0), "open camera")
        self.opened = True

        # Keep the camera in free-run mode and let exposure control work continuously.
        set_enum(self.camera, "TriggerMode", MV_TRIGGER_MODE_OFF)

        try:
            set_bool(self.camera, "AcquisitionFrameRateEnable", True)
            set_float(self.camera, "AcquisitionFrameRate", self.args.fps)
        except Exception:
            pass

        # Some cameras reject ExposureTime writes once auto exposure is already active.
        # Program manual nodes first, then enable the configured auto mode.
        set_enum(self.camera, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF)
        try:
            set_float(self.camera, "ExposureTime", self.args.exposure_time)
        except Exception as exc:
            print(f"WARNING initial ExposureTime write failed: {exc}", flush=True)

        if not set_auto_exposure_limit(self.camera, "Lower", self.args.ae_lower):
            print("WARNING auto exposure lower limit node not applied", flush=True)
        if not set_auto_exposure_limit(self.camera, "Upper", self.args.ae_upper):
            print("WARNING auto exposure upper limit node not applied", flush=True)
        exposure_mode = {
            "off": MV_EXPOSURE_AUTO_MODE_OFF,
            "once": MV_EXPOSURE_AUTO_MODE_ONCE,
            "continuous": MV_EXPOSURE_AUTO_MODE_CONTINUOUS,
        }[self.args.exposure_auto]
        set_enum(self.camera, "ExposureAuto", exposure_mode)

        gain_mode = {
            "off": MV_GAIN_MODE_OFF,
            "once": MV_GAIN_MODE_ONCE,
            "continuous": MV_GAIN_MODE_CONTINUOUS,
        }[self.args.gain_auto]
        set_enum(self.camera, "GainAuto", gain_mode)
        set_float(self.camera, "Gain", self.args.gain)

        # The width/height nodes are integer nodes, so read them separately.
        cur_w, min_w, max_w, inc_w = read_int(self.camera, "Width")
        cur_h, min_h, max_h, inc_h = read_int(self.camera, "Height")
        self.frame_info = {
            "width": cur_w,
            "height": cur_h,
            "width_min": min_w,
            "width_max": max_w,
            "width_inc": inc_w,
            "height_min": min_h,
            "height_max": max_h,
            "height_inc": inc_h,
        }

        pixel_format = read_enum(self.camera, "PixelFormat")
        exp_cur, exp_min, exp_max = read_float(self.camera, "ExposureTime")
        gain_cur, gain_min, gain_max = read_float(self.camera, "Gain")
        try:
            ae_cur = read_enum(self.camera, "ExposureAuto")
        except Exception:
            ae_cur = None
        try:
            ga_cur = read_enum(self.camera, "GainAuto")
        except Exception:
            ga_cur = None

        print(
            json.dumps(
                {
                    "camera_serial": self.camera_serial,
                    "width": cur_w,
                    "height": cur_h,
                    "pixel_format": pixel_format,
                    "exposure_time": {
                        "cur": exp_cur,
                        "min": exp_min,
                        "max": exp_max,
                    },
                    "gain": {
                        "cur": gain_cur,
                        "min": gain_min,
                        "max": gain_max,
                    },
                    "exposure_auto": ae_cur,
                    "gain_auto": ga_cur,
                    "auto_exposure_limit": {
                        "lower": self.args.ae_lower,
                        "upper": self.args.ae_upper,
                    },
                },
                ensure_ascii=False,
            ),
            flush=True,
        )

        require_ok(self.camera.MV_CC_StartGrabbing(), "start camera acquisition")
        self.grabbing = True

    def make_overlay(self, frame, recv_mono, live_exposure_time, live_exposure_auto, live_gain, live_gain_auto):
        now = time.monotonic()
        latency_ms = (now - recv_mono) * 1000.0
        self.frame_window.append(now)
        fps = 0.0
        if len(self.frame_window) >= 2:
            span = self.frame_window[-1] - self.frame_window[0]
            if span > 0:
                fps = (len(self.frame_window) - 1) / span

        h, w = frame.shape[:2]
        overlay_lines = [
            f"serial={self.camera_serial}  res={w}x{h}  fps={fps:.1f}  latency={latency_ms:.1f} ms",
            f"ExposureAuto={live_exposure_auto}  ExposureTime={live_exposure_time:.1f} us  AE=[{self.args.ae_lower}, {self.args.ae_upper}] us",
            f"GainAuto={live_gain_auto}  Gain={live_gain:.2f}",
        ]

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.62
        thickness = 1
        margin = 10
        line_gap = 8
        text_sizes = [cv2.getTextSize(line, font, font_scale, thickness)[0] for line in overlay_lines]
        box_w = max(size[0] for size in text_sizes) + margin * 2
        box_h = sum(size[1] for size in text_sizes) + line_gap * (len(overlay_lines) - 1) + margin * 2
        cv2.rectangle(frame, (8, 8), (8 + box_w, 8 + box_h), (0, 0, 0), -1)
        y = 8 + margin + text_sizes[0][1]
        for idx, line in enumerate(overlay_lines):
            cv2.putText(frame, line, (8 + margin, y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
            if idx + 1 < len(overlay_lines):
                y += text_sizes[idx][1] + line_gap

        return {
            "latency_ms": round(latency_ms, 3),
            "fps": round(fps, 3),
            "width": w,
            "height": h,
            "serial": self.camera_serial,
        }

    def capture_loop(self):
        payload = read_int(self.camera, "PayloadSize")[0]
        raw_buffer = (c_ubyte * payload)()
        raw_view = np.ctypeslib.as_array(raw_buffer)
        frame_info = MV_FRAME_OUT_INFO_EX()
        frame_count = 0

        while self.state.running and not stop_requested:
            memset(byref(frame_info), 0, sizeof(frame_info))
            result = self.camera.MV_CC_GetOneFrameTimeout(raw_buffer, payload, frame_info, 200)
            if result != 0:
                continue

            width = int(frame_info.nWidth)
            height = int(frame_info.nHeight)
            pixel_type = int(frame_info.enPixelType)

            bgr = np.empty((height, width, 3), dtype=np.uint8)
            convert_frame(
                raw_view,
                int(frame_info.nFrameLen),
                pixel_type,
                width,
                height,
                bgr,
            )

            if self.args.preview_width and self.args.preview_width > 0 and width > self.args.preview_width:
                preview_height = max(1, int(round(height * self.args.preview_width / width)))
                bgr = cv2.resize(bgr, (self.args.preview_width, preview_height), interpolation=cv2.INTER_AREA)

            recv_mono = time.monotonic()
            try:
                live_exposure_time = read_float(self.camera, "ExposureTime")[0]
            except Exception:
                live_exposure_time = float(self.args.exposure_time)
            try:
                live_gain = read_float(self.camera, "Gain")[0]
            except Exception:
                live_gain = float(self.args.gain)
            try:
                live_exposure_auto = read_enum(self.camera, "ExposureAuto")
            except Exception:
                live_exposure_auto = None
            try:
                live_gain_auto = read_enum(self.camera, "GainAuto")
            except Exception:
                live_gain_auto = None
            live_exposure_auto_name = (
                exposure_auto_name(live_exposure_auto) if live_exposure_auto is not None else "n/a"
            )
            live_gain_auto_name = (
                gain_auto_name(live_gain_auto) if live_gain_auto is not None else "n/a"
            )

            stats = self.make_overlay(
                bgr,
                recv_mono,
                live_exposure_time,
                live_exposure_auto_name,
                live_gain,
                live_gain_auto_name,
            )

            ok, jpeg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.args.jpeg_quality)])
            if not ok:
                continue

            with self.state.condition:
                self.state.latest_jpeg = jpeg.tobytes()
                self.state.latest_stats = {
                    **stats,
                    "exposure_time_us": float(live_exposure_time),
                    "gain_us": float(live_gain),
                    "exposure_auto": live_exposure_auto_name,
                    "gain_auto": live_gain_auto_name,
                    "ae_lower_us": int(self.args.ae_lower),
                    "ae_upper_us": int(self.args.ae_upper),
                }
                self.state.latest_stats["frame_count"] = frame_count
                self.state.latest_seq += 1
                self.state.condition.notify_all()
            frame_count += 1

    def close(self):
        self.state.running = False
        if self.grabbing:
            self.camera.MV_CC_StopGrabbing()
        if self.opened:
            self.camera.MV_CC_CloseDevice()
        if self.handle_created:
            self.camera.MV_CC_DestroyHandle()
        if self.sdk_initialized:
            MvCamera.MV_CC_Finalize()


def convert_frame(raw_view, frame_length, pixel_type, width, height, bgr):
    pixels = width * height
    if pixel_type in (
        PixelType_Gvsp_BayerBG8,
        PixelType_Gvsp_BayerGB8,
        PixelType_Gvsp_BayerGR8,
        PixelType_Gvsp_BayerRG8,
    ):
        if frame_length < pixels:
            raise RuntimeError(f"short Bayer frame: {frame_length} bytes")
        raw = raw_view[:pixels].reshape(height, width)
        conversion = {
            PixelType_Gvsp_BayerBG8: cv2.COLOR_BayerBG2BGR,
            PixelType_Gvsp_BayerGB8: cv2.COLOR_BayerGB2BGR,
            PixelType_Gvsp_BayerGR8: cv2.COLOR_BayerGR2BGR,
            PixelType_Gvsp_BayerRG8: cv2.COLOR_BayerRG2BGR,
        }[pixel_type]
        cv2.cvtColor(raw, conversion, bgr)
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


class PreviewHandler(BaseHTTPRequestHandler):
    app: MvsPreviewApp = None  # type: ignore[assignment]

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(self.render_html().encode("utf-8"))
            return
        if self.path == "/stats.json":
            with self.app.state.condition:
                payload = dict(self.app.state.latest_stats)
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(json.dumps(payload, ensure_ascii=False).encode("utf-8"))
            return
        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            self.stream_loop()
            return
        self.send_error(404)

    def log_message(self, format, *args):  # noqa: A003
        return

    def render_html(self):
        return """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Hikrobot MVS Preview</title>
  <style>
    html, body { margin: 0; height: 100%; background: #111; color: #eee; font-family: system-ui, sans-serif; }
    .wrap { display: grid; grid-template-columns: minmax(0, 1fr) 320px; gap: 0; height: 100%; }
    .main { position: relative; background: #000; overflow: auto; }
    img { display: block; width: 100%; height: auto; max-width: 100%; }
    .panel { padding: 16px; border-left: 1px solid #2a2a2a; background: #151515; font-size: 14px; line-height: 1.5; }
    .label { color: #8ec7ff; font-weight: 600; margin-top: 12px; }
    pre { white-space: pre-wrap; word-break: break-word; margin: 8px 0 0 0; color: #ddd; }
    @media (max-width: 980px) { .wrap { grid-template-columns: 1fr; } .panel { border-left: 0; border-top: 1px solid #2a2a2a; } }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="main"><img src="/stream.mjpg" alt="live stream"></div>
    <div class="panel">
      <div class="label">Live Stats</div>
      <pre id="stats">waiting for frames...</pre>
      <div class="label">Mode</div>
      <pre>Camera parameters are loaded from configs/camera_capture.yaml</pre>
    </div>
  </div>
  <script>
    async function refresh() {
      try {
        const r = await fetch('/stats.json', { cache: 'no-store' });
        const j = await r.json();
        document.getElementById('stats').textContent = JSON.stringify(j, null, 2);
      } catch (e) {
        document.getElementById('stats').textContent = 'waiting for frames...';
      }
    }
    setInterval(refresh, 500);
    refresh();
  </script>
</body>
</html>
        """

    def stream_loop(self):
        last_seq = -1
        while self.app.state.running and not stop_requested:
            with self.app.state.condition:
                self.app.state.condition.wait_for(
                    lambda: not self.app.state.running or self.app.state.latest_seq != last_seq,
                    timeout=1.0,
                )
                if not self.app.state.running or self.app.state.latest_jpeg is None:
                    break
                if self.app.state.latest_seq == last_seq:
                    continue
                last_seq = self.app.state.latest_seq
                jpeg = self.app.state.latest_jpeg
            try:
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
            except BrokenPipeError:
                break


def request_stop(_signum, _frame):
    global stop_requested
    stop_requested = True


def main():
    args = parse_args()
    app = MvsPreviewApp(args)
    PreviewHandler.app = app

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    app.setup_camera()
    server = ThreadingHTTPServer((args.host, args.port), PreviewHandler)
    print(f"preview_url=http://{args.host}:{args.port}/", flush=True)

    try:
        capture_thread = threading.Thread(target=app.capture_loop, daemon=True)
        capture_thread.start()

        if args.keep_window:
            print("OpenCV local window enabled. Press q in the window or Ctrl-C in the terminal to stop.", flush=True)

        if args.keep_window:
            def window_loop():
                while not stop_requested:
                    with app.state.condition:
                        app.state.condition.wait(timeout=0.2)
                        jpeg = app.state.latest_jpeg
                    if jpeg is None:
                        continue
                    arr = np.frombuffer(jpeg, dtype=np.uint8)
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if frame is None:
                        continue
                    cv2.imshow("Hikrobot MVS Preview", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        request_stop(None, None)
                        break

            window_thread = threading.Thread(target=window_loop, daemon=True)
            window_thread.start()

        server_thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.5}, daemon=True)
        server_thread.start()

        while not stop_requested:
            time.sleep(0.2)
    finally:
        try:
            server.shutdown()
            server.server_close()
        except Exception:
            pass
        app.close()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"ERROR {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
