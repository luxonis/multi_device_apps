#!/usr/bin/env python3
"""Multi-device app: pick OAK 4 devices on the network, calibrate them against each other
(dai.beta.node.MultiDeviceCalibration), then run a merged point cloud, calibrated panorama or detections.

A single Python process serves a small control page (device list, Check devices, Calibrate, Stop).
"Check devices" connects to every selected device and reads which cameras it has and whether its
factory calibration contains a stereo pair. Devices with a stereo pair register both cameras (the
pair fixes the metric scale); devices without one register a single camera, and when no device
provides a stereo pair the page asks for measured camera-to-camera distances (setKnownDistance).
"Calibrate" streams the cameras into the node, shows live previews so the overlap can be checked,
stores the result as JSON and renders the device coordinate systems in a 3D view together with the
calibration metrics (data confidence, Sampson error, pairwise distances, change against the previous
calibration of the same devices).

"Point cloud" runs NeuralDepth on every selected stereo device, aligns the depth to the color camera,
turns it into a point cloud in the calibration origin (one host PointCloud node per device; the frame
extrinsics rebased by pipeline.setMultiDeviceCalibration place the points in the common frame) and
sends the concatenated cloud to the DepthAI visualizer. "Panorama" feeds one undistorted camera per
device into dai.beta.node.Stitching composed from the calibration (cylindrical, spherical or
perspective surface) and streams the result to the page as MJPEG. "Detections" runs inference on each
camera and uses ImgDetectionsFilter to remap and merge detections onto that panorama, with an overlay
in the visualizer and live filter controls on the page. "Capture" streams the same cameras as the
calibration, waits for the exposure and the PTP frame lock to settle and saves one synchronized
full-resolution frame of every camera into captures/<timestamp>/ with the timestamps in capture.json.

    /usr/bin/python3 app.py            # then open http://localhost:8093
"""

import argparse
import collections
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import traceback
from datetime import timedelta
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

_PARENTS = Path(__file__).resolve().parents  # scripts/multi_device_apps/<app>/app.py inside the development checkout
DEFAULT_BINDINGS_DIR = _PARENTS[3] / "depthai-core" / "build-python" / "bindings" / "python" if len(_PARENTS) > 3 else Path("/nonexistent")
BINDINGS_DIR = Path(os.environ.get("DEPTHAI_CORE_BINDINGS", DEFAULT_BINDINGS_DIR)).expanduser().resolve()
if BINDINGS_DIR.is_dir():
    sys.path.insert(0, str(BINDINGS_DIR))
import cv2  # noqa: E402
import depthai as dai  # noqa: E402
import numpy as np  # noqa: E402

APP_VERSION = "0.2"
APP_DIR = Path(__file__).resolve().parent
CALIBRATION_DIR = APP_DIR / "calibrations"
CAPTURE_DIR = APP_DIR / "captures"  # one folder per Capture with the PNG frames and capture.json
VISUALIZER_DIR = APP_DIR / "visualizer"  # DepthAI visualizer static files served by this app (if present)
VISUALIZER_MIME = {".html": "text/html", ".css": "text/css", ".js": "application/javascript", ".png": "image/png", ".svg": "image/svg+xml", ".ico": "image/x-icon", ".json": "application/json", ".txt": "text/plain", ".map": "application/json", ".woff": "font/woff", ".woff2": "font/woff2", ".wasm": "application/wasm"}
POINT_CLOUD_TOPIC = "merged_point_cloud"
PROJECTIONS = {"cylindrical": dai.CameraModel.Cylindrical, "spherical": dai.CameraModel.Equirectangular, "perspective": dai.CameraModel.Perspective}
STITCHING_NODE = getattr(dai.node, "Stitching", None) or dai.beta.node.Stitching
DETECTIONS_AVAILABLE = hasattr(dai.node, "ImgDetectionsFilter") and hasattr(dai, "ImgDetectionsFilterConfig") and hasattr(STITCHING_NODE, "outSourceMasks")
DETECTION_DEFAULTS = {"confidence": 0.5, "labels": None, "rejectLabels": None, "minArea": 0.0, "maxDetections": None, "overlap": "nms", "iou": 0.4}
CAMERA_FPS = 10.0  # the PTP frame lock holds within ~0.01 ms at 10 FPS but wanders by 5-20 ms at 5 FPS
PTP_CONF = "/etc/luxonis-ptp.conf"
CAPTURE_TICK_SIZE = (128, 80)  # per-frame "tick" images that carry the timestamps of every camera to the host spread monitor
CAPTURE_PREVIEW_FPS = 2.0  # page previews while capturing: 8 cameras at 640x400 and 2 FPS stay under half of a 100 Mbit link
CAPTURE_RING = 10  # full-resolution frames each camera keeps on the device (1 s at 10 FPS) until the host asks for one
# Runs on every captured camera: keeps the newest CAPTURE_RING full-resolution frames by sequence number and sends the
# requested one to the host (the request is echoed back when that frame is no longer buffered). Streaming full
# resolution continuously is impossible over a 100 Mbit link, so only the chosen frame crosses the network.
CAPTURE_SCRIPT = """
import time
frames = {}
order = []
while True:
    f = node.io['frames'].tryGet()
    if f is not None:
        frames[f.getSequenceNum()] = f
        order.append(f.getSequenceNum())
        while len(order) > RING:
            frames.pop(order.pop(0), None)
    req = node.io['request'].tryGet()
    if req is not None:
        f = frames.get(int(bytes(req.getData()).decode()))
        node.io['out'].send(f if f is not None else req)
    time.sleep(0.005)
""".replace("RING", str(CAPTURE_RING))
# Runs on every device of a point cloud capture: keeps the newest aligned depth and color frames by device timestamp and
# sends the pair closest to the requested timestamp (within half a frame period) to the host, waiting for the depth of
# that slot to arrive (depth lags the tick by the NeuralDepth latency), or echoes the request when the pair never comes
# (depth pauses for a few seconds after start while PTP re-times the stereo sensors)
CAPTURE_CLOUD_SCRIPT = """
import time
depth = {}
color = {}
pending = None  # (request, target timestamp ms, since): depth lags the tick by the NeuralDepth latency, so a request waits for it
def push(store, f):
    store[int(f.getTimestampDevice().total_seconds() * 1000)] = f
    while len(store) > RING:
        del store[min(store)]
def nearest(store, target):
    best = None
    for ts in store:
        if abs(ts - target) <= TOL and (best is None or abs(ts - target) < abs(best - target)):
            best = ts
    return None if best is None else store[best]
while True:
    f = node.io['depth'].tryGet()
    if f is not None:
        push(depth, f)
    f = node.io['color'].tryGet()
    if f is not None:
        push(color, f)
    req = node.io['request'].tryGet()
    if req is not None:
        pending = (req, int(bytes(req.getData()).decode()), time.time())
    if pending is not None:
        req, target, since = pending
        d = nearest(depth, target)
        c = nearest(color, target)
        if d is not None and c is not None:
            node.io['depth_out'].send(d)
            node.io['color_out'].send(c)
            pending = None
        elif (depth and max(depth) > target + TOL) or time.time() - since > 2.0:
            node.error('no depth and color for slot %d: depth %s color %s' % (target, sorted(depth)[-3:], sorted(color)[-3:]))
            node.io['depth_out'].send(req)
            pending = None
    time.sleep(0.005)
""".replace("RING", str(CAPTURE_RING))
PTP_WARNING = "Devices are not PTP-synchronized: the calibration may be less precise."
NO_SCALE_WARNING = "No device provides a stereo pair, so the metric scale must come from a measured distance between two cameras."


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--version", action="version", version=f"multi-device app {APP_VERSION}")
    parser.add_argument("--port", type=int, default=8093, help="Control page HTTP port (default: 8093)")
    parser.add_argument("--http-port", type=int, default=8082, help="DepthAI visualizer HTTP port (default: 8082)")
    parser.add_argument("--websocket-port", type=int, default=8765, help="DepthAI visualizer WebSocket port; the visualizer page always connects to 8765 (default: 8765)")
    parser.add_argument("--pc-fps", type=float, default=10.0, help="Camera FPS for the point cloud (default: 10)")
    parser.add_argument("--model", default="LARGE", choices=["LARGE", "MEDIUM", "SMALL"], help="NeuralDepth model (default: LARGE)")
    parser.add_argument("--min-range", type=float, default=0.3, help="Discard depth closer than this many meters (default: 0.3)")
    parser.add_argument("--max-range", type=float, default=10.0, help="Discard depth farther than this many meters (default: 10)")
    parser.add_argument("--threads", type=int, default=4, help="Host threads per point cloud stream (default: 4)")
    parser.add_argument("--size", type=int, nargs=2, default=None, metavar=("W", "H"), help="Color / aligned depth size per device (default: the NeuralDepth model size; lower it for many devices on one link)")
    parser.add_argument("--pano-fps", type=float, default=10.0, help="Camera FPS for the panorama (default: 10)")
    parser.add_argument("--pano-size", type=int, nargs=2, default=(640, 400), metavar=("W", "H"), help="Camera output size fed to the stitcher (default: 640 400)")
    parser.add_argument("--detections-model", default="luxonis/yolov6-nano:r2-coco-512x288", help="Model Zoo detection model, shared by all cameras (default: luxonis/yolov6-nano:r2-coco-512x288)")
    parser.add_argument("--seam", default="GRAPHCUT_COLOR", choices=["NONE", "VORONOI", "DP_COLOR", "DP_COLOR_GRAD", "GRAPHCUT_COLOR", "GRAPHCUT_COLOR_GRAD"], help="Panorama seam finder; NONE copies the inputs without blending (default: GRAPHCUT_COLOR)")
    parser.add_argument("--sample-count", type=int, default=10, help="Synchronized image groups collected per calibration and saved per capture (default: 10)")
    parser.add_argument("--fps", type=float, default=CAMERA_FPS, help="Camera FPS while calibrating (default: 10)")
    parser.add_argument("--sync-threshold", type=float, default=1.0, help="Sync threshold of the calibration node in seconds; the scene is static, so this only has to catch one frame of every camera (default: 1)")
    parser.add_argument("-c", "--calibration", type=Path, nargs="*", default=[], help="Existing multi-device calibration JSON files to import (copied into calibrations/)")
    parser.add_argument("--no-ptp", action="store_true", help="Never check or configure PTP on the devices (free-running cameras, timestamp sync only)")
    parser.add_argument("--warmup-seconds", type=float, default=4.0, help="Seconds to stream before the node starts collecting, so auto exposure settles (default: 4)")
    parser.add_argument("--ptp-settle-ms", type=float, default=2.0, help="PTP counts as settled once 5 consecutive image groups of all devices lie within this many ms (default: 2)")
    parser.add_argument("--ptp-settle-timeout", type=float, default=20.0, help="Give up waiting for PTP to settle after this many seconds and calibrate anyway; the scene is static, so a few ms of spread do not matter (default: 20)")
    return parser.parse_args()


def detection_config(settings, config=None):
    """Validate browser settings and create a complete config; runtime messages replace every option."""
    if not isinstance(settings, dict):
        raise ValueError("Detection filters must be an object")
    settings = {**DETECTION_DEFAULTS, **settings}
    for key, low, high in (("confidence", 0.1, 0.95), ("iou", 0.0, 1.0), ("minArea", 0.0, float(np.finfo(np.float32).max))):
        value = settings[key]
        if type(value) not in (int, float) or not low <= value <= high:
            raise ValueError(f"{key} must be between {low:g} and {high:g}")
    for key in ("labels", "rejectLabels", "maxDetections"):
        value = settings[key]
        if value is None:
            continue
        values = [value] if key == "maxDetections" else value
        if not isinstance(values, list) or any(type(v) is not int or not 0 <= v <= 0xFFFFFFFF for v in values):
            raise ValueError(f"{key} requires nonnegative integer IDs or counts")
    if settings["overlap"] not in ("off", "nms", "average"):
        raise ValueError("Overlap must be off, nms or average")
    config = config if config is not None else dai.ImgDetectionsFilterConfig()
    config.setConfidenceRange(settings["confidence"])
    config.labelsToKeep = settings["labels"]
    config.labelsToReject = settings["rejectLabels"]
    config.setSizeRange(settings["minArea"])
    config.maxDetections = settings["maxDetections"]
    config.sortByConfidence = True
    config.overlapMode = getattr(dai.ImgDetectionsFilterConfig.OverlapMode, settings["overlap"].upper())
    config.overlapIouThreshold = settings["iou"]
    if not config.validate():
        raise ValueError("Each filter minimum must be below its maximum")
    return settings, config


# ---------------------------------------------------------------- small geometry helpers
def rotation_angle_deg(rotation_matrix):
    r = np.asarray(rotation_matrix, dtype=np.float64)
    return float(np.degrees(np.arccos(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))))


def yaw_pitch_roll_deg(rotation_matrix):
    """Z-Y-X Euler angles (R = Rz(yaw) Ry(pitch) Rx(roll)) in degrees."""
    r = np.asarray(rotation_matrix, dtype=np.float64)
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(math.cos(pitch)) > 1e-6:
        yaw = math.atan2(r[1, 0], r[0, 0])
        roll = math.atan2(r[2, 1], r[2, 2])
    else:
        yaw = math.atan2(-r[0, 1], r[1, 1])
        roll = 0.0
    return [round(math.degrees(a), 2) for a in (yaw, pitch, roll)]


def pose_matrix(rotation, translation):
    m = np.eye(4)
    m[:3, :3] = np.asarray(rotation, dtype=np.float64)
    m[:3, 3] = np.asarray(translation, dtype=np.float64)
    return m


def extrinsics_matrix(ext):
    """4x4 transform of a dai.Extrinsics in meters (X_to = R X_from + t)."""
    t = ext.translation
    scale = {dai.LengthUnit.METER: 1.0, dai.LengthUnit.CENTIMETER: 0.01, dai.LengthUnit.MILLIMETER: 0.001}.get(ext.lengthUnit, 1.0)
    return pose_matrix(ext.rotationMatrix, [t.x * scale, t.y * scale, t.z * scale])


def write_ply(path, points, colors):
    """Binary little-endian PLY with float32 xyz (meters) and uint8 rgb."""
    data = np.empty(len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    data["x"], data["y"], data["z"] = points[:, 0], points[:, 1], points[:, 2]
    data["red"], data["green"], data["blue"] = colors[:, 0], colors[:, 1], colors[:, 2]
    header = (f"ply\nformat binary_little_endian 1.0\ncomment multi-device app point cloud, meters, calibration origin frame\nelement vertex {len(points)}\n"
              "property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(data.tobytes())


def cloud_arrays(cloud):
    """(points (N, 3) float32, colors (N, 3) uint8) of a PointCloudData without the invalid (all-zero) points."""
    if cloud.isColor():
        points, colors = cloud.getPointsRGB()
        colors = np.asarray(colors)[:, :3]
    else:
        points = cloud.getPoints()
        colors = np.full((len(points), 3), 255, dtype=np.uint8)
    points = np.asarray(points, dtype=np.float32)
    valid = np.isfinite(points).all(axis=1) & np.any(points != 0, axis=1)
    return np.ascontiguousarray(points[valid]), np.ascontiguousarray(colors[valid].astype(np.uint8))


def seconds(timestamp):
    """Seconds of a message timestamp: a timedelta (monotonic clocks) or a datetime (the PTP-disciplined system clock)."""
    return timestamp.total_seconds() if hasattr(timestamp, "total_seconds") else timestamp.timestamp()


def socket_name(socket):
    return socket.name if hasattr(socket, "name") else str(socket).split(".")[-1]


def socket_from_name(name):
    return dai.CameraBoardSocket.__members__[name]


def calibration_key(device_ids):
    return "_".join(sorted(device_ids))


def local_origin_socket(calib, socket):
    """Root of a socket's extrinsics chain in the device calibration (the device's local calibration origin)."""
    cameras = calib.getEepromData().cameraData
    seen = set()
    while socket in cameras and socket not in seen:
        seen.add(socket)
        nxt = cameras[socket].extrinsics.toCameraSocket
        if nxt == dai.CameraBoardSocket.AUTO or nxt not in cameras:
            break
        socket = nxt
    return socket


# ---------------------------------------------------------------- ssh / PTP (same approach as the point cloud app)
class DeviceSsh:
    """Runs commands on OAK devices as root over ssh. Uses paramiko when it is installed (works on Windows too);
    otherwise the OpenSSH client, with the password handed over through SSH_ASKPASS. None tries key / agent
    authentication; an empty string explicitly selects passwordless login."""

    def __init__(self):
        try:
            import paramiko  # noqa: F401

            self.paramiko = True
        except ImportError:
            self.paramiko = False
        self.askpass = None
        if not self.paramiko and os.name != "nt":
            self.askpass = Path(tempfile.mkdtemp(prefix="oak-askpass-")) / "askpass.sh"
            self.askpass.write_text('#!/bin/sh\nprintf "%s" "$DEVICE_PASSWORD"\n')
            self.askpass.chmod(0o700)

    def run(self, ip, password, command, timeout=25):
        """Returns (status, output): status is ok | needs_password | unreachable | failed."""
        if self.paramiko:
            return self._run_paramiko(ip, password, command, timeout)
        if self.askpass is None:
            return "failed", "ssh support missing: install the paramiko package (pip install paramiko)"
        env = dict(os.environ, SSH_ASKPASS=str(self.askpass), SSH_ASKPASS_REQUIRE="force", DEVICE_PASSWORD=password or "")
        args = ["ssh", "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null", "-o", "LogLevel=ERROR",
                "-o", "ConnectTimeout=5", "-o", "NumberOfPasswordPrompts=1", "-o", "PreferredAuthentications=publickey,password"]
        if password is None:
            args += ["-o", "BatchMode=yes"]
        args += [f"root@{ip}", command]
        try:
            proc = subprocess.run(args, env=env, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=timeout, start_new_session=True)
        except subprocess.TimeoutExpired:
            return "unreachable", "ssh timed out"
        except FileNotFoundError:
            return "failed", "ssh client not installed"
        if proc.returncode == 255:
            if "Permission denied" in proc.stderr:
                return "needs_password", proc.stderr.strip()
            return "unreachable", proc.stderr.strip() or "ssh connection failed"
        if proc.returncode != 0:
            return "failed", (proc.stderr.strip() or proc.stdout.strip() or f"exit code {proc.returncode}")
        return "ok", proc.stdout

    @staticmethod
    def _run_paramiko(ip, password, command, timeout):
        import paramiko

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            try:
                client.connect(ip, username="root", password=password, timeout=5, banner_timeout=10, auth_timeout=10,
                               allow_agent=password is None, look_for_keys=password is None)
            except paramiko.AuthenticationException:
                if password != "":
                    raise
                client.get_transport().auth_none("root")
        except paramiko.AuthenticationException:
            client.close()
            return "needs_password", "authentication failed"
        except paramiko.SSHException as e:
            client.close()
            if password is None or "auth" in str(e).lower():
                return "needs_password", str(e)
            return "unreachable", str(e) or "ssh connection failed"
        except OSError as e:
            client.close()
            return "unreachable", str(e) or "ssh connection failed"
        try:
            _, stdout, stderr = client.exec_command(command, timeout=timeout)
            out = stdout.read().decode(errors="replace")
            err = stderr.read().decode(errors="replace")
            code = stdout.channel.recv_exit_status()
        except (paramiko.SSHException, OSError) as e:
            return "unreachable", str(e)
        finally:
            client.close()
        if code != 0:
            return "failed", (err.strip() or out.strip() or f"exit code {code}")
        return "ok", out


def parse_ptp_conf(text):
    conf = {}
    for line in text.splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            key, value = line.split("=", 1)
            conf[key.strip()] = value.strip()
    return {
        "enabled": conf.get("PTP_ENABLE", "false").lower() == "true",
        "mode": conf.get("PTP_MODE", "").lower(),
        "syncFrames": conf.get("PTP_SYNC_FRAMES", "false").lower() == "true",
    }


class App:
    """Owns the device list, the inspection results and the one running job (inspect or calibrate)."""

    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.phase = "idle"  # idle | discovering | inspecting | calibrating | distance_input | ptp_password | pointcloud | panorama | detections | error
        self.message = "Select devices."
        self.stats = None  # live numbers of the running mode
        self.live = None  # {"mode": "pointcloud"|"panorama"|"detections", ...} while running
        self.streaming = False  # the viewer has content (clouds are being sent / panoramas arrive)
        self.pano_jpeg = None
        self.pano_seq = 0
        self.pano_cond = threading.Condition()
        self.pending_detection_config = None
        self.detection_topics = []
        self.progress = None
        self.devices = []  # discovered devices (dicts)
        self.device_info = {}  # device name -> inspection dict (cameras, stereo pair, local origin)
        self.calibrations = {}  # path -> calibration info dict (metrics, poses)
        self.latest_calibration = None  # path of the result of the last calibration job
        self.latest_capture = None  # info dict of the last Capture job of this session (files, timestamps, spread)
        self.capture_thumbs = {}  # "<capture dir>/<file>" -> JPEG thumbnail bytes served to the page
        self.log_lines = collections.deque(maxlen=80)
        self.stop_event = threading.Event()
        self.job = None
        self.job_devices = []
        self.last_job = None  # "inspect" | "calibrate"
        self.previews = {}  # stream name -> {"queue": MessageQueue, "jpeg": bytes}; only while calibrating
        self.ssh = DeviceSsh()
        self.passwords = {}
        self.credentials_event = threading.Event()
        self.ptp_skip_requested = False
        self.ptp = {"active": False, "master": None, "statuses": {}, "needPassword": [], "warning": None, "spreadMs": None, "spreadAt": None, "groups": 0, "settled": None}
        self.distance_request = None  # {"pairs": [...], "required": bool} while the page is asked for distances
        self.distance_event = threading.Event()
        self.known_distances = {}  # "name/SOCKET|name/SOCKET" -> cm, remembered for the app lifetime
        self.distance_answer = None
        self.serve_visualizer = (VISUALIZER_DIR / "index.html").is_file()
        self.remote = dai.RemoteConnection(webSocketPort=args.websocket_port, serveFrontend=not self.serve_visualizer, httpPort=args.http_port)
        if self.serve_visualizer:
            visualizer_server = ThreadingHTTPServer(("0.0.0.0", args.http_port), VisualizerHandler)
            visualizer_server.daemon_threads = True
            threading.Thread(target=visualizer_server.serve_forever, daemon=True).start()
        self.cloud_queue = self.remote.addTopic(POINT_CLOUD_TOPIC, "3d", maxSize=2, blocking=False)
        CALIBRATION_DIR.mkdir(exist_ok=True)
        CAPTURE_DIR.mkdir(exist_ok=True)
        self._load_saved_calibrations()
        self.log(f"DepthAI {dai.__version__}")
        self.discover_async()

    # ---------------------------------------------------------------- state helpers
    def log(self, text):
        line = f"{time.strftime('%H:%M:%S')}  {text}"
        print(line, flush=True)
        with self.lock:
            self.log_lines.append(line)

    def set_state(self, phase=None, message=None, progress="keep", stats="keep"):
        with self.lock:
            if phase is not None:
                self.phase = phase
            if message is not None:
                self.message = message
            if progress != "keep":
                self.progress = progress
            if stats != "keep":
                self.stats = stats

    def set_ptp(self, **fields):
        with self.lock:
            self.ptp.update(fields)

    def snapshot(self):
        with self.lock:
            return {
                "phase": self.phase,
                "message": self.message,
                "progress": self.progress,
                "devices": list(self.devices),
                "deviceInfo": dict(self.device_info),
                "calibrations": sorted(self.calibrations.values(), key=lambda c: c["time"], reverse=True),
                "latestCalibration": self.latest_calibration,
                "latestCapture": self.latest_capture,
                "log": list(self.log_lines),
                "version": APP_VERSION,
                "busy": self.job is not None and self.job.is_alive(),
                "jobDevices": list(self.job_devices),
                "lastJob": self.last_job,
                "previews": list(self.previews.keys()),
                "ptp": {**self.ptp, "spreadAge": (round(time.monotonic() - self.ptp["spreadAt"], 1) if self.ptp["spreadAt"] else None)},
                "distanceRequest": self.distance_request,
                "knownDistances": dict(self.known_distances),
                "stats": self.stats,
                "live": self.live,
                "streaming": self.streaming,
                "visualizerPort": self.args.http_port,
                "detectionsAvailable": DETECTIONS_AVAILABLE,
            }

    def preview_jpeg(self, name):
        with self.lock:
            preview = self.previews.get(name)
        if preview is None:
            return None
        frame = preview["queue"].tryGet()
        if frame is not None:
            ok, encoded = cv2.imencode(".jpg", frame.getCvFrame(), [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                preview["jpeg"] = encoded.tobytes()
        return preview["jpeg"]

    # ---------------------------------------------------------------- stored calibrations
    def _load_saved_calibrations(self):
        import shutil

        for source in self.args.calibration:
            handler = dai.beta.MultiDeviceCalibrationHandler(source)
            target = self._calibration_path(self._graph_device_ids(handler), source.stat().st_mtime)
            if not target.exists():
                shutil.copyfile(source, target)
                self.log(f"Imported calibration {source} -> {target.name}")
        for path in sorted(CALIBRATION_DIR.glob("*.json")):
            if path.name.endswith(".meta.json"):
                continue
            try:
                info = self._load_calibration_info(path)
                self.calibrations[str(path)] = info
            except Exception as e:  # noqa: BLE001
                print(f"Ignoring {path}: {e}")

    @staticmethod
    def _meta_path(path):
        return path.with_name(path.stem + ".meta.json")

    def _load_calibration_info(self, path):
        """Calibration info of a stored file: the metrics sidecar when present, else what the graph alone tells."""
        meta = self._meta_path(path)
        if meta.is_file():
            info = json.loads(meta.read_text())
            info["path"] = str(path)
            return info
        handler = dai.beta.MultiDeviceCalibrationHandler(path)
        return self._calibration_info(handler, path, self._calibration_time(path))

    @staticmethod
    def _calibration_time(path):
        stamp = path.stem.rsplit("__", 1)[-1]
        try:
            return time.strftime("%Y-%m-%d %H:%M:%S", time.strptime(stamp, "%Y%m%d-%H%M%S"))
        except ValueError:
            return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(path.stat().st_mtime))

    @staticmethod
    def _calibration_path(device_ids, timestamp):
        return CALIBRATION_DIR / f"{calibration_key(device_ids)}__{time.strftime('%Y%m%d-%H%M%S', time.localtime(timestamp))}.json"

    @staticmethod
    def _graph_device_ids(handler):
        graph = handler.getGraph()
        return sorted({e.fromDeviceId for e in graph} | {e.extrinsics.toDeviceId for e in graph})

    def _calibration_info(self, handler, path, when, result=None, names=None, cameras_used=None, known=None, extra=None):
        """Everything the page shows about a calibration: device poses in the common origin, the cameras that were
        registered (with their poses), pairwise distances, metrics. Poses are 4x4 row-major lists in meters."""
        device_ids = self._graph_device_ids(handler)
        names = names or {}
        cameras_used = cameras_used or {}
        known = known or []
        devices = []
        origin = None
        for device_id in device_ids:
            socket = handler.getDeviceSocket(device_id)
            ext = handler.getExtrinsicsToOrigin(device_id, socket) if socket is not None else None
            if ext is None:
                continue
            pose = extrinsics_matrix(ext)
            origin = f"{ext.toDeviceId}/{socket_name(ext.toCameraSocket)}"
            cameras = []
            for cam in cameras_used.get(device_id, []):
                cam_pose = pose @ np.asarray(cam["poseLocal"], dtype=np.float64)
                cameras.append({"socket": cam["socket"], "pose": cam_pose.round(6).tolist(), "sensor": cam.get("sensor")})
            devices.append({
                "deviceId": device_id,
                "name": names.get(device_id, device_id),
                "originSocket": socket_name(socket),
                "isOrigin": device_id == ext.toDeviceId,
                "pose": pose.round(6).tolist(),
                "translation_m": [round(float(v), 4) for v in pose[:3, 3]],
                "distance_m": round(float(np.linalg.norm(pose[:3, 3])), 4),
                "rotation_deg": round(rotation_angle_deg(pose[:3, :3]), 2),
                "ypr_deg": yaw_pitch_roll_deg(pose[:3, :3]),
                "cameras": cameras,
            })
        edges = [{"from": e.fromDeviceId, "to": e.extrinsics.toDeviceId} for e in handler.getGraph()]
        pairs = self._pair_distances(devices, known)
        info = {
            "key": calibration_key(device_ids), "deviceIds": device_ids, "path": str(path), "time": when, "origin": origin,
            "confidence": round(result.dataConfidence, 4) if result is not None else None,
            "sampsonError": round(result.sampsonError, 6) if result is not None else None,
            "devices": devices, "edges": edges, "pairs": pairs, "knownDistances": known,
        }
        info.update(extra or {})
        return info

    @staticmethod
    def _pair_distances(devices, known):
        """Distance between the registered cameras of every device pair, next to the measured value when one was entered."""
        by_id = {d["deviceId"]: d for d in devices}
        known_by_pair = {}
        for k in known:
            known_by_pair[(k["fromDeviceId"], k["fromSocket"], k["toDeviceId"], k["toSocket"])] = k["cm"]
            known_by_pair[(k["toDeviceId"], k["toSocket"], k["fromDeviceId"], k["fromSocket"])] = k["cm"]
        pairs = []
        ids = [d["deviceId"] for d in devices]
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                da, db = by_id[a], by_id[b]
                origin_distance = float(np.linalg.norm(np.asarray(da["pose"])[:3, 3] - np.asarray(db["pose"])[:3, 3]))
                camera_distances = []
                for ca in da["cameras"]:
                    for cb in db["cameras"]:
                        dist = float(np.linalg.norm(np.asarray(ca["pose"])[:3, 3] - np.asarray(cb["pose"])[:3, 3]))
                        entry = {"a": f"{da['name']}/{ca['socket']}", "b": f"{db['name']}/{cb['socket']}", "distance_cm": round(100 * dist, 2)}
                        measured = known_by_pair.get((a, ca["socket"], b, cb["socket"]))
                        if measured is not None:
                            entry["measured_cm"] = measured
                            entry["residual_cm"] = round(100 * dist - measured, 2)
                        camera_distances.append(entry)
                pairs.append({"a": da["name"], "b": db["name"], "origin_distance_cm": round(100 * origin_distance, 2), "cameras": camera_distances})
        return pairs

    def _compare_with_previous(self, info):
        """Largest pose change of any device against the newest earlier calibration of exactly the same devices."""
        with self.lock:
            previous = [c for c in self.calibrations.values() if c["key"] == info["key"] and c["path"] != info["path"]]
        if not previous:
            return None
        previous.sort(key=lambda c: c["time"], reverse=True)
        prev = previous[0]
        if prev.get("origin") != info.get("origin"):
            return {"path": prev["path"], "time": prev["time"], "note": "different origin device"}
        prev_by_id = {d["deviceId"]: d for d in prev["devices"]}
        worst_t, worst_r, per_device = 0.0, 0.0, []
        for d in info["devices"]:
            p = prev_by_id.get(d["deviceId"])
            if p is None:
                continue
            new, old = np.asarray(d["pose"]), np.asarray(p["pose"])
            dt = float(np.linalg.norm(new[:3, 3] - old[:3, 3]))
            dr = rotation_angle_deg(new[:3, :3] @ old[:3, :3].T)
            per_device.append({"name": d["name"], "translation_cm": round(100 * dt, 2), "rotation_deg": round(dr, 3)})
            worst_t, worst_r = max(worst_t, dt), max(worst_r, dr)
        return {"path": prev["path"], "time": prev["time"], "maxTranslation_cm": round(100 * worst_t, 2), "maxRotation_deg": round(worst_r, 3), "devices": per_device}

    # ---------------------------------------------------------------- jobs
    def _start_job(self, target, *job_args, name=None):
        with self.lock:
            if self.job is not None and self.job.is_alive():
                return False
            self.stop_event.clear()
            self.credentials_event.clear()
            self.distance_event.clear()
            self.ptp_skip_requested = False
            self.distance_request = None
            self.job_devices = list(job_args[0]) if job_args and isinstance(job_args[0], list) else []
            self.last_job = name or {self._inspect_job: "inspect", self._calibrate: "calibrate", self._pointcloud: "pointcloud", self._panorama: "panorama", self._capture: "capture"}.get(target)
            self.streaming = False
            self.live = None
            self.stats = None
            self.pending_detection_config = None
            self.job = threading.Thread(target=self._run_job, args=(target, *job_args), daemon=True)
            self.job.start()
            return True

    def _run_job(self, target, *job_args):
        try:
            target(*job_args)
        except Exception as e:  # noqa: BLE001
            traceback.print_exc()
            self.log(f"ERROR: {e}")
            self.set_state("error", "Calibration failed." if target == self._calibrate else f"Failed: {e}", progress=None, stats=None)
        finally:
            for topic in self.detection_topics:
                self.remote.removeTopic(topic)
            self.detection_topics.clear()
            with self.lock:
                self.passwords.clear()
                self.distance_request = None
                self.streaming = False
                self.live = None
                self.pending_detection_config = None

    def discover_async(self):
        return self._start_job(self._discover)

    def _discover(self):
        self.set_state("discovering", "Looking for devices on the network...")
        found = []
        for info in dai.Device.getAllAvailableDevices():
            platform = str(info.platform).split(".")[-1].replace("X_LINK_", "")
            state = str(info.state).split(".")[-1].replace("X_LINK_", "")
            found.append({
                "name": info.name,
                "deviceId": info.deviceId,
                "platform": platform,
                "state": state,
                "selectable": platform == "RVC4" and state == "GATE",
            })
        found.sort(key=lambda d: (not d["selectable"], d["name"]))
        with self.lock:
            self.devices = found
        self.log(f"Found {len(found)} devices, {sum(d['selectable'] for d in found)} selectable (RVC4, free)")
        self.set_state("idle", "Select at least two devices.")

    def stop(self):
        self.stop_event.set()
        self.credentials_event.set()
        self.distance_event.set()

    def submit_passwords(self, passwords):
        with self.lock:
            self.passwords.update(passwords)
        self.credentials_event.set()

    def skip_ptp(self):
        self.ptp_skip_requested = True
        self.credentials_event.set()

    def submit_distances(self, distances):
        """distances: list of {from, fromSocket, to, toSocket, cm}; an empty list means "none known"."""
        with self.lock:
            self.distance_answer = distances
            for d in distances:
                self.known_distances[self._distance_key(d["from"], d["fromSocket"], d["to"], d["toSocket"])] = float(d["cm"])
        self.distance_event.set()

    @staticmethod
    def _distance_key(a, a_socket, b, b_socket):
        ends = sorted([f"{a}/{a_socket}", f"{b}/{b_socket}"])
        return "|".join(ends)

    # ---------------------------------------------------------------- device inspection
    def inspect_async(self, device_names):
        return self._start_job(self._inspect_job, device_names)

    def _inspect_job(self, device_names):
        self._inspect(device_names)
        n = len(device_names)
        stereo = sum(1 for name in device_names if self.device_info.get(name, {}).get("stereoPair"))
        self.set_state("idle", f"Checked {n} devices, {stereo} with a stereo pair.", progress=None)

    def _inspect(self, device_names, force=False):
        """Connects to every device (one after another) and records its cameras, stereo pair and local origin."""
        self.set_state("inspecting", "Checking what cameras the devices have...", progress=None)
        for index, name in enumerate(device_names):
            if self.stop_event.is_set():
                raise InterruptedError
            if not force and name in self.device_info:
                continue
            self.set_state(message=f"Connecting to {name} ({index + 1}/{len(device_names)}) to read its cameras and calibration...")
            with dai.Device(dai.DeviceInfo(name)) as device:
                info = self._describe_device(device)
            with self.lock:
                self.device_info[name] = info
            pair = info["stereoPair"]
            self.log(f"{name}: {info['productName']} ({info['deviceId']}), cameras {[c['socket'] for c in info['cameras']]}, "
                     + (f"stereo pair {pair[0]} + {pair[1]}" if pair else "no stereo pair") + f", local origin {info['origin']}")

    @staticmethod
    def _describe_device(device):
        calib = device.readCalibration()
        features = device.getConnectedCameraFeatures()
        cameras = []
        sockets = [f.socket for f in features]
        for f in features:
            origin = local_origin_socket(calib, f.socket)
            try:
                pose_local = np.asarray(calib.getCameraExtrinsics(f.socket, origin, False, dai.LengthUnit.METER), dtype=np.float64) if origin != f.socket else np.eye(4)
                calibrated = True
            except Exception:  # noqa: BLE001 - socket without calibration data
                pose_local = np.eye(4)
                calibrated = False
            types = [str(t).split(".")[-1] for t in f.supportedTypes]
            cameras.append({
                "socket": socket_name(f.socket), "sensor": f.sensorName, "width": f.width, "height": f.height,
                "type": types[0] if types else "", "origin": socket_name(origin), "calibrated": calibrated, "poseLocal": pose_local.round(8).tolist(),
            })
        pair = None
        try:
            left, right = calib.getStereoLeftCameraId(), calib.getStereoRightCameraId()
            if left in sockets and right in sockets and left != right and left != dai.CameraBoardSocket.AUTO and right != dai.CameraBoardSocket.AUTO:
                calib.getCameraExtrinsics(left, right, False, dai.LengthUnit.METER)  # raises when the pair is not calibrated
                pair = [socket_name(left), socket_name(right)]
        except Exception as e:  # noqa: BLE001
            print(f"{device.getDeviceId()}: stereo pair unusable: {e}")
        calibrated_cameras = [c for c in cameras if c["calibrated"]] or cameras
        origin = calibrated_cameras[0]["origin"] if calibrated_cameras else None
        if pair:
            default_mode = "stereo"
        else:
            preferred = [c["socket"] for c in calibrated_cameras if c["type"] == "COLOR"] or [c["socket"] for c in calibrated_cameras]
            default_mode = f"single:{preferred[0]}" if preferred else None
        return {
            "deviceId": device.getDeviceId(), "productName": device.getDeviceName(), "cameras": cameras,
            "stereoPair": pair, "origin": origin, "defaultMode": default_mode,
        }

    @staticmethod
    def _cameras_for_mode(info, mode):
        """Sockets to register for a device given the page's choice (stereo | single:<SOCKET>)."""
        mode = mode or info["defaultMode"]
        if mode == "stereo" and info["stereoPair"]:
            return list(info["stereoPair"]), True
        if mode and mode.startswith("single:"):
            socket = mode.split(":", 1)[1]
            if any(c["socket"] == socket for c in info["cameras"]):
                return [socket], False
        if info["defaultMode"] == "stereo":
            return list(info["stereoPair"]), True
        if info["defaultMode"]:
            return [info["defaultMode"].split(":", 1)[1]], False
        raise RuntimeError(f"{info['deviceId']} has no usable camera")

    # ---------------------------------------------------------------- PTP
    def _ptp_status(self, ip):
        status, output = self.ssh.run(ip, self.passwords.get(ip), f"cat {PTP_CONF}")
        info = {"status": status, "detail": "" if status == "ok" else output}
        if status == "ok":
            info.update(parse_ptp_conf(output))
        return info

    def _prepare_ptp(self, device_names):
        """Checks and, when needed, configures PTP (one master, the rest slaves, frame sync on) on every device.
        Asks the page for root passwords when key login does not work. Returns True when all devices are
        PTP-configured, False when the job should run without PTP (a warning is stored in the state)."""
        if self.args.no_ptp:
            self.set_ptp(active=False, warning=None, master=None)
            return False
        self.set_ptp(active=False, warning=None, needPassword=[], spreadMs=None, spreadAt=None, groups=0, settled=None)
        while True:
            self.set_state(message="Checking PTP configuration on the devices (ssh)...")
            statuses = {ip: self._ptp_status(ip) for ip in device_names}
            self.set_ptp(statuses=statuses)
            need = [ip for ip, st in statuses.items() if st["status"] == "needs_password"]
            if not need:
                break
            self.set_state("ptp_password", f"Root password needed for {', '.join(need)} to check / configure PTP.")
            self.set_ptp(needPassword=need)
            self.credentials_event.clear()
            got = self.credentials_event.wait(timeout=900)
            self.set_ptp(needPassword=[])
            self.set_state("calibrating")
            if self.stop_event.is_set():
                return False
            if not got or self.ptp_skip_requested:
                self.set_ptp(warning=PTP_WARNING + " (no password given)")
                self.log("PTP skipped by the user")
                return False
        bad = {ip: st for ip, st in statuses.items() if st["status"] != "ok"}
        if bad:
            detail = "; ".join(f"{ip}: {st['detail'].splitlines()[-1] if st['detail'] else st['status']}" for ip, st in bad.items())
            self.set_ptp(warning=f"{PTP_WARNING} Could not reach over ssh: {detail}")
            self.log(f"PTP check failed: {detail}")
            return False

        masters = [ip for ip in device_names if statuses[ip]["enabled"] and statuses[ip]["mode"] == "master"]
        master = masters[0] if masters else device_names[0]
        changed = []
        for ip in device_names:
            st = statuses[ip]
            mode = "master" if ip == master else "slave"
            if st["enabled"] and st["mode"] == mode and st["syncFrames"]:
                continue
            self.set_state(message=f"Configuring PTP on {ip} as {mode}...")
            status, output = self.ssh.run(ip, self.passwords.get(ip), f"luxonis-ptp-config enable && luxonis-ptp-config mode {mode} && luxonis-ptp-config sync_frames true", timeout=60)
            if status != "ok":
                self.set_ptp(warning=f"{PTP_WARNING} Configuring {ip} failed: {output.splitlines()[-1] if output else status}")
                self.log(f"PTP configuration of {ip} failed: {output}")
                return False
            changed.append(f"{ip}={mode}")
            statuses[ip].update(enabled=True, mode=mode, syncFrames=True)
        self.set_ptp(active=True, master=master, statuses=statuses, warning=None)
        self.log(f"PTP: master {master}, slaves {[ip for ip in device_names if ip != master]}" + (f"; configured {', '.join(changed)}" if changed else " (already configured)"))
        return True

    def _wait_for_lock(self, pipeline, spread_queue, ptp, total, action):
        """Streams until the auto exposure has warmed up (--warmup-seconds) and, with PTP, the frame timestamps of all
        cameras have locked: 5 consecutive groups of the spread monitor within --ptp-settle-ms, or --ptp-settle-timeout
        with a warning. Returns the number of monitor groups seen; raises InterruptedError when Stop was pressed."""
        args = self.args
        started = time.monotonic()
        monitor_groups = 0
        recent = collections.deque(maxlen=5)
        settled = not ptp
        self.set_state(
            message="Warming up: letting the exposure settle" + (f" and waiting for PTP to lock the frames (spread below {args.ptp_settle_ms:g} ms)" if ptp else "") + "...",
            progress={"stage": "warmup", "done": 0, "total": total, "percent": 0},
        )
        while pipeline.isRunning() and not self.stop_event.is_set():
            for group in spread_queue.tryGetAll():
                monitor_groups += 1
                recent.append(group.getIntervalNs() / 1e6)
                self.set_ptp(spreadMs=round(recent[-1], 2), spreadAt=time.monotonic(), groups=monitor_groups)
            elapsed = time.monotonic() - started
            warm = min(elapsed / args.warmup_seconds, 1.0) if args.warmup_seconds > 0 else 1.0
            if warm < 1.0 or settled or not ptp:
                self.set_state(progress={"stage": "warmup", "done": 0, "total": total, "percent": int(100 * warm)})
            else:
                locked = 0
                for spread in reversed(recent):
                    if spread >= args.ptp_settle_ms:
                        break
                    locked += 1
                self.set_state(progress={"stage": "ptplock", "done": 0, "total": total, "percent": int(100 * locked / recent.maxlen) if locked else None,
                                         "detail": f"frame spread {recent[-1]:.1f} ms, needs {recent.maxlen} groups below {args.ptp_settle_ms:g} ms" if recent else "waiting for the first synchronized frames"})
            if ptp and len(recent) == recent.maxlen and max(recent) < args.ptp_settle_ms:
                settled = True
            if elapsed >= args.warmup_seconds and settled:
                break
            if ptp and elapsed >= args.ptp_settle_timeout:
                self.set_ptp(warning=f"PTP did not settle within {args.ptp_settle_timeout:.0f} s (frame spread {max(recent) if recent else float('nan'):.1f} ms, limit {args.ptp_settle_ms:g} ms). {action} anyway; check that the switch passes PTP traffic.")
                self.log(f"PTP did not settle: recent spreads {[round(x, 1) for x in recent]} ms")
                break
            time.sleep(0.05)
        self.set_ptp(settled=settled if ptp else None)
        if ptp and settled:
            self.log(f"PTP settled: frame spread {max(recent):.2f} ms after {time.monotonic() - started:.1f} s")
        if self.stop_event.is_set():
            raise InterruptedError
        return monitor_groups

    # ---------------------------------------------------------------- calibration
    def calibrate_async(self, device_names, modes, distances, ask_distances=False):
        return self._start_job(self._calibrate, device_names, modes or {}, distances or [], ask_distances)

    def _ask_distances(self, device_names, registered, has_scale):
        """Pauses the job until the page sent measured distances (or none). Returns the list of
        {from, fromSocket, to, toSocket, cm} entries (device names, not IDs)."""
        pairs = []
        for i, a in enumerate(device_names):
            for b in device_names[i + 1:]:
                sa, sb = registered[a][0], registered[b][0]
                key = self._distance_key(a, sa, b, sb)
                pairs.append({"from": a, "fromSocket": sa, "to": b, "toSocket": sb, "cm": self.known_distances.get(key)})
        with self.lock:
            self.distance_request = {"pairs": pairs, "required": not has_scale}
            self.distance_answer = None
        self.distance_event.clear()
        self.set_state("distance_input", "Enter the measured distances between the cameras." if not has_scale else "Optionally enter measured distances between the cameras.")
        got = self.distance_event.wait(timeout=1800)
        with self.lock:
            self.distance_request = None
            answer = self.distance_answer
        self.set_state("calibrating")
        if self.stop_event.is_set():
            raise InterruptedError
        if not got or answer is None:
            if not has_scale:
                raise RuntimeError("No measured distance was entered; without a stereo pair the calibration has no metric scale.")
            return []
        return [d for d in answer if d.get("cm") not in (None, "")]

    def _calibrate(self, device_names, modes, distances, ask_distances=False):
        args = self.args
        self.set_state("calibrating", "Checking the devices...", progress={"stage": "inspecting", "done": 0, "total": args.sample_count, "percent": None})
        self.log(f"Calibration of {device_names} started ({args.sample_count} groups)")
        started_job = time.monotonic()
        self._inspect(device_names)
        self.set_state("calibrating")
        registered = {}  # device name -> sockets registered
        stereo_devices = []
        for name in device_names:
            sockets, is_stereo = self._cameras_for_mode(self.device_info[name], modes.get(name))
            registered[name] = sockets
            if is_stereo:
                stereo_devices.append(name)
        has_scale = bool(stereo_devices)
        # Measured distances: required when no stereo pair fixes the scale, otherwise only when the page asked for them
        if ask_distances or not has_scale:
            distances = self._ask_distances(device_names, registered, has_scale)
        distances = [d for d in (distances or []) if d.get("cm") not in (None, "")]
        with self.lock:
            for d in distances:
                self.known_distances[self._distance_key(d["from"], d["fromSocket"], d["to"], d["toSocket"])] = float(d["cm"])
        if not has_scale and not distances:
            raise RuntimeError(NO_SCALE_WARNING)
        for name in device_names:
            self.log(f"{name}: using {' + '.join(registered[name])}" + (" (stereo pair, fixes the scale)" if name in stereo_devices else " (single camera)"))
        for d in distances:
            self.log(f"Known distance {d['from']}/{d['fromSocket']} <-> {d['to']}/{d['toSocket']}: {float(d['cm']):.1f} cm")

        self.set_state(progress={"stage": "ptp", "done": 0, "total": args.sample_count, "percent": None})
        ptp = self._prepare_ptp(device_names)
        if self.stop_event.is_set():
            self.set_state("idle", "Calibration stopped.", progress=None)
            return
        result = None
        device_ids = {}
        groups = 0
        self.set_state("calibrating", "Connecting to devices...", progress={"stage": "connecting", "done": 0, "total": args.sample_count, "percent": None})
        with dai.Pipeline(createImplicitDevice=False) as pipeline:
            pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)
            calibration = pipeline.create(dai.beta.node.MultiDeviceCalibration)
            calibration.setSampleCount(args.sample_count)
            calibration.sync.setSyncThreshold(timedelta(seconds=args.sync_threshold))
            if ptp:
                calibration.sync.setTimestampSource(dai.node.Sync.TimestampSource.SYSTEM)
            # Monitor of the cross-device timestamp spread on the previews; its threshold is just under one frame
            # period so it only groups frames of the same capture slot (a looser one pairs frames a slot apart)
            monitor = pipeline.create(dai.node.Sync)
            monitor.setRunOnHost(True)
            monitor.setSyncThreshold(timedelta(milliseconds=0.9 * 1000.0 / args.fps))
            monitor.setSyncAttempts(-1)
            monitor.setTimestampSource(dai.node.Sync.TimestampSource.SYSTEM if ptp else dai.node.Sync.TimestampSource.HOST)
            for name in device_names:
                device = pipeline.addDevice(dai.DeviceInfo(name))
                device_id = device.getDeviceId()
                device_ids[name] = device_id
                for socket_str in registered[name]:
                    socket = socket_from_name(socket_str)
                    camera = pipeline.create(dai.node.Camera, device).build(socket, sensorFps=args.fps)
                    if ptp:
                        camera.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.TIME_PTP)
                    calibration.addCamera(camera.requestFullResolutionOutput(fps=args.fps))
                    preview = camera.requestOutput((640, 400), fps=args.fps)
                    with self.lock:
                        self.previews[f"{name} {socket_str}"] = {"queue": preview.createOutputQueue(maxSize=1, blocking=False), "jpeg": None}
                    monitor_input = monitor.inputs[f"{name} {socket_str}"]
                    monitor_input.setBlocking(False)
                    monitor_input.setMaxSize(8)
                    preview.link(monitor_input)
                if name in stereo_devices:
                    calibration.setStereoPair(device_id, socket_from_name(registered[name][0]), socket_from_name(registered[name][1]))
                self.log(f"{name}: {device.getDeviceName()} ({device_id})")
            known_entries = []
            for d in distances:
                calibration.setKnownDistance(device_ids[d["from"]], socket_from_name(d["fromSocket"]), device_ids[d["to"]], socket_from_name(d["toSocket"]), float(d["cm"]), dai.LengthUnit.CENTIMETER)
                known_entries.append({"fromDeviceId": device_ids[d["from"]], "fromSocket": d["fromSocket"], "toDeviceId": device_ids[d["to"]], "toSocket": d["toSocket"], "cm": float(d["cm"])})

            control_queue = calibration.inputControl.createInputQueue()
            result_queue = calibration.calibrationOutput.createOutputQueue()
            group_queue = calibration.sync.out.createOutputQueue(maxSize=4, blocking=False)
            spread_queue = monitor.out.createOutputQueue(maxSize=8, blocking=False)
            pipeline.start()
            collect_started = None
            try:
                monitor_groups = self._wait_for_lock(pipeline, spread_queue, ptp, args.sample_count, "Calibrating")
                group_queue.tryGetAll()
                collect_started = time.monotonic()
                control_queue.send(dai.beta.MultiDeviceCalibrationControl.start())
                self.set_state(message="Collecting synchronized image groups. Keep the devices still and looking at the same textured scene.",
                               progress={"stage": "collecting", "done": 0, "total": args.sample_count, "percent": 0})
                while pipeline.isRunning() and not self.stop_event.is_set():
                    for group in spread_queue.tryGetAll():
                        monitor_groups += 1
                        self.set_ptp(spreadMs=round(group.getIntervalNs() / 1e6, 2), spreadAt=time.monotonic(), groups=monitor_groups)
                    for _ in group_queue.tryGetAll():
                        groups = min(groups + 1, args.sample_count)
                        self.set_state(progress={"stage": "collecting", "done": groups, "total": args.sample_count, "percent": int(100 * groups / args.sample_count)})
                    result = result_queue.tryGet()
                    if result is not None:
                        break
                    time.sleep(0.05)
                self.set_state(message="Solving the calibration..." if result is not None else "Stopping...",
                               progress={"stage": "solving" if result is not None else "stopping", "done": groups, "total": args.sample_count, "percent": None})
            except InterruptedError:
                pass
            finally:
                with self.lock:
                    self.previews = {}
                pipeline.stop()

        if result is None:
            self.log("Calibration stopped")
            self.set_state("idle", "Calibration stopped.", progress=None)
            return
        if not result.passed or result.graph is None:
            self.log(f"Calibration failed: {result.info}")
            self.set_state("error", f"Calibration failed: {result.info}", progress=None)
            return

        handler = result.getHandler()
        now = time.time()
        path = self._calibration_path(list(device_ids.values()), now)
        if not handler.toJsonFile(path):
            raise RuntimeError(f"Failed to save calibration to {path}")
        names = {device_id: name for name, device_id in device_ids.items()}
        cameras_used = {}
        for name, device_id in device_ids.items():
            cams = {c["socket"]: c for c in self.device_info[name]["cameras"]}
            cameras_used[device_id] = [cams[s] for s in registered[name]]
        with self.lock:
            ptp_summary = {"active": self.ptp["active"], "settled": self.ptp["settled"], "spreadMs": self.ptp["spreadMs"], "warning": self.ptp["warning"]}
        extra = {
            "samples": args.sample_count, "fps": args.fps,
            "collectSeconds": round(time.monotonic() - collect_started, 1) if collect_started else None,
            "totalSeconds": round(time.monotonic() - started_job, 1),
            "ptp": ptp_summary,
            "products": {device_id: self.device_info[name]["productName"] for name, device_id in device_ids.items()},
        }
        info = self._calibration_info(handler, path, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)), result, names, cameras_used, known_entries, extra)
        info["previous"] = self._compare_with_previous(info)
        self._meta_path(path).write_text(json.dumps(info, indent=1))
        for d in info["devices"]:
            self.log(f"{d['name']}: position {d['translation_m']} m ({d['distance_m']:.3f} m), rotation {d['rotation_deg']} deg to origin {info['origin']}" + ("  [origin]" if d["isOrigin"] else ""))
        for pair in info["pairs"]:
            for c in pair["cameras"]:
                self.log(f"{c['a']} <-> {c['b']}: {c['distance_cm']:.1f} cm" + (f" (measured {c['measured_cm']:.1f}, residual {c['residual_cm']:+.1f} cm)" if "measured_cm" in c else ""))
        if info["previous"] and "maxTranslation_cm" in info["previous"]:
            self.log(f"Change vs calibration of {info['previous']['time']}: max {info['previous']['maxTranslation_cm']} cm / {info['previous']['maxRotation_deg']} deg")
        with self.lock:
            self.calibrations[str(path)] = info
            self.latest_calibration = str(path)
        self.log(f"Calibration saved to {path.name} (confidence {info['confidence']}, Sampson error {info['sampsonError']})")
        self.set_state("idle", f"Calibrated {len(device_ids)} devices.", progress=None)

    # ---------------------------------------------------------------- capture
    def capture_async(self, device_names, modes):
        return self._start_job(self._capture, device_names, modes)

    def _capture(self, device_names, modes):
        """Saves --sample-count synchronized full-resolution image groups of the registered cameras (the stereo pair
        or the chosen single camera of each device, as for the calibration) once the exposure and, with PTP, the frame
        lock have settled: the same material the calibration consumes, so the setup can be calibrated offline. Every
        group goes to captures/<timestamp>/group_NN/ as PNG; capture.json holds the timestamps and spreads and the
        factory calibration of every device is saved next to it."""
        args = self.args
        self.set_state("capturing", "Checking the devices...", progress={"stage": "inspecting", "done": 0, "total": args.sample_count, "percent": None}, stats=None)
        self.log(f"Capture of {device_names} started ({args.sample_count} groups)")
        self._inspect(device_names)
        self.set_state("capturing")
        registered = {name: self._cameras_for_mode(self.device_info[name], modes.get(name))[0] for name in device_names}
        for name in device_names:
            self.log(f"{name}: capturing {' + '.join(registered[name])}")
        self.set_state(progress={"stage": "ptp", "done": 0, "total": args.sample_count, "percent": None})
        if len(device_names) > 1:
            ptp = self._prepare_ptp(device_names)
        else:
            ptp = False  # a single device has nothing to synchronize with
            self.set_ptp(active=False, warning=None, master=None, needPassword=[], spreadMs=None, spreadAt=None, groups=0, settled=None)
        if self.stop_event.is_set():
            self.set_state("idle", "Capture stopped.", progress=None)
            return
        self.set_state("capturing", "Connecting to devices...", progress={"stage": "connecting", "done": 0, "total": args.sample_count, "percent": None})
        captured_groups = []  # [{"spreadMs": float, "frames": [frame dicts]}] in capture order
        device_ids = {}
        calibrations = {}  # device name -> factory CalibrationHandler, saved with the frames
        with dai.Pipeline(createImplicitDevice=False) as pipeline:
            pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)
            # One group = the ticks of every camera from the same capture slot. The threshold is just under a frame
            # period with PTP (frames of neighbouring slots never mix) and a full period for free-running cameras,
            # which can never be closer than that
            sync = pipeline.create(dai.node.Sync)
            sync.setRunOnHost(True)
            sync.setSyncThreshold(timedelta(milliseconds=(900.0 if ptp else 1000.0) / args.fps))
            sync.setSyncAttempts(-1)
            sync.setTimestampSource(dai.node.Sync.TimestampSource.SYSTEM if ptp else dai.node.Sync.TimestampSource.HOST)
            streams = []  # (device name, socket, sync input / preview name, request queue, full-frame queue)
            for name in device_names:
                device = pipeline.addDevice(dai.DeviceInfo(name))
                device_ids[name] = device.getDeviceId()
                calibrations[name] = device.readCalibration()
                for socket_str in registered[name]:
                    camera = pipeline.create(dai.node.Camera, device).build(socket_from_name(socket_str), sensorFps=args.fps)
                    camera.setOutputsNumFramesPool(2 * CAPTURE_RING)  # the ring holds CAPTURE_RING of them; the default pool stalls the camera
                    if ptp:
                        camera.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.TIME_PTP)
                    key = f"{name} {socket_str}"
                    sync_input = sync.inputs[key]
                    sync_input.setBlocking(False)
                    sync_input.setMaxSize(8)
                    camera.requestOutput(CAPTURE_TICK_SIZE, fps=args.fps).link(sync_input)
                    ring = pipeline.create(dai.node.Script, device)
                    ring.setScript(CAPTURE_SCRIPT)
                    ring.inputs["frames"].setBlocking(False)
                    ring.inputs["frames"].setMaxSize(4)
                    camera.requestFullResolutionOutput(fps=args.fps).link(ring.inputs["frames"])
                    preview = camera.requestOutput((640, 400), fps=min(CAPTURE_PREVIEW_FPS, args.fps))
                    with self.lock:
                        self.previews[key] = {"queue": preview.createOutputQueue(maxSize=1, blocking=False), "jpeg": None}
                    streams.append((name, socket_str, key, ring.inputs["request"].createInputQueue(), ring.outputs["out"].createOutputQueue(maxSize=4, blocking=False)))
                self.log(f"{name}: {device.getDeviceName()} ({device_ids[name]})")
            group_queue = sync.out.createOutputQueue(maxSize=8, blocking=False)
            pipeline.start()
            try:
                groups = self._wait_for_lock(pipeline, group_queue, ptp, args.sample_count, "Capturing")
                order = [(name, socket_str) for name, socket_str, *_ in streams]
                while len(captured_groups) < args.sample_count:
                    done = len(captured_groups)
                    self.set_state(message=f"Capturing synchronized image group {done + 1} of {args.sample_count}. Keep the devices still and looking at the same textured scene.",
                                   progress={"stage": "capturing", "done": done, "total": args.sample_count, "percent": int(100 * done / args.sample_count), "detail": f"{done} of {args.sample_count} image groups received"})
                    # A fresh group after the previous transfer, so its frames are still in the rings on the devices
                    group_queue.tryGetAll()
                    group = None
                    deadline = time.monotonic() + max(5.0, 5 * 1000.0 / args.fps)
                    while pipeline.isRunning() and not self.stop_event.is_set() and time.monotonic() < deadline:
                        group = group_queue.tryGet()
                        if group is not None:
                            break
                        time.sleep(0.02)
                    if self.stop_event.is_set():
                        raise InterruptedError
                    if group is None:
                        raise RuntimeError("No synchronized frame group arrived: the cameras are not delivering frames in step.")
                    spread_ms = group.getIntervalNs() / 1e6
                    groups += 1
                    self.set_ptp(spreadMs=round(spread_ms, 2), spreadAt=time.monotonic(), groups=groups)
                    # Ask every camera's ring for the full-resolution frame with the sequence number of its tick in the group
                    wanted = {}
                    for name, socket_str, key, request_queue, _ in streams:
                        wanted[key] = group[key].getSequenceNum()
                        request = dai.Buffer()
                        request.setData(list(str(wanted[key]).encode()))
                        request_queue.send(request)
                    self.log(f"Group {done + 1}: spread {spread_ms:.2f} ms; fetching " + ", ".join(f"{key} #{seq}" for key, seq in wanted.items()))
                    frames = []
                    pending = {key: (name, socket_str, frame_queue) for name, socket_str, key, _, frame_queue in streams}
                    deadline = time.monotonic() + 30.0  # full frames of all cameras over a 100 Mbit link
                    while pending and pipeline.isRunning() and not self.stop_event.is_set() and time.monotonic() < deadline:
                        for key in list(pending):
                            name, socket_str, frame_queue = pending[key]
                            message = frame_queue.tryGet()
                            if message is None:
                                continue
                            if not isinstance(message, dai.ImgFrame):
                                raise RuntimeError(f"{key}: frame #{wanted[key]} was no longer buffered on the device when the request arrived; try again.")
                            if message.getSequenceNum() != wanted[key]:
                                self.log(f"{key}: ignoring frame #{message.getSequenceNum()}, waiting for #{wanted[key]}")
                                continue
                            frames.append({
                                "device": name, "deviceId": device_ids[name], "socket": socket_str, "image": message.getCvFrame(),
                                "width": message.getWidth(), "height": message.getHeight(), "type": str(message.getType()).split(".")[-1], "sequenceNum": message.getSequenceNum(),
                                "timestamp_s": seconds(message.getTimestamp()), "timestampDevice_s": seconds(message.getTimestampDevice()), "timestampSystem_s": seconds(message.getTimestampSystem()),
                            })
                            del pending[key]
                            self.set_state(progress={"stage": "capturing", "done": done, "total": args.sample_count, "percent": int(100 * (done + len(frames) / len(streams)) / args.sample_count),
                                                     "detail": f"group {done + 1}: {len(frames)} of {len(streams)} full-resolution frames received"})
                        time.sleep(0.02)
                    if self.stop_event.is_set():
                        raise InterruptedError
                    if pending:
                        raise RuntimeError(f"Full-resolution frames of {', '.join(pending)} did not arrive within 30 s.")
                    frames.sort(key=lambda c: order.index((c["device"], c["socket"])))  # device order, not arrival order
                    captured_groups.append({"spreadMs": spread_ms, "frames": frames})
            except InterruptedError:
                captured_groups = []
            finally:
                with self.lock:
                    self.previews = {}
                pipeline.stop()
        if not captured_groups:
            self.log("Capture stopped")
            self.set_state("idle", "Capture stopped.", progress=None)
            return

        self.set_state(message="Saving the frames...", progress={"stage": "saving", "done": len(captured_groups), "total": args.sample_count, "percent": None})
        now = time.time()
        folder = CAPTURE_DIR / time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
        folder.mkdir(parents=True, exist_ok=True)
        source = "timestampSystem_s" if ptp else "timestamp_s"  # the clock the Sync grouped by
        thumbs = {}
        saved_groups = []
        for index, group in enumerate(captured_groups, 1):
            group_dir = folder / f"group_{index:02d}"
            group_dir.mkdir(exist_ok=True)
            first = min(c[source] for c in group["frames"])
            entries = []
            for c in group["frames"]:
                file = f"{group_dir.name}/{c['device']}_{c['socket']}.png".replace(" ", "_")
                if not cv2.imwrite(str(folder / file), c["image"]):
                    raise RuntimeError(f"Failed to write {folder / file}")
                thumbs[file] = self._thumbnail(c["image"])
                entries.append({**{k: v for k, v in c.items() if k != "image"}, "file": file, "offsetMs": round((c[source] - first) * 1000.0, 3)})
            saved_groups.append({"index": index, "dir": group_dir.name, "spreadMs": round(group["spreadMs"], 3), "frames": entries})
        calibration_files = {}
        for name, handler in calibrations.items():
            file = f"calibration_{device_ids[name]}.json"
            if not handler.eepromToJsonFile(str(folder / file)):
                self.log(f"WARNING: could not save the factory calibration of {name}")
                continue
            calibration_files[name] = file
        with self.lock:
            ptp_summary = {"active": self.ptp["active"], "settled": self.ptp["settled"], "master": self.ptp["master"], "warning": self.ptp["warning"]}
        spread_max = max(g["spreadMs"] for g in saved_groups)
        info = {
            "kind": "frames", "dir": folder.name, "time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)), "fps": args.fps,
            "sampleCount": args.sample_count, "spreadMaxMs": round(spread_max, 3), "ptp": ptp_summary,
            "devices": [{"name": name, "deviceId": device_ids[name], "product": self.device_info[name]["productName"], "sockets": registered[name],
                         "stereoPair": self.device_info[name]["stereoPair"] if len(registered[name]) == 2 else None, "calibrationFile": calibration_files.get(name)} for name in device_names],
            "groups": saved_groups,
        }
        (folder / "capture.json").write_text(json.dumps(info, indent=1))
        with self.lock:
            self.latest_capture = info
            self.capture_thumbs = {f"{folder.name}/{file}": jpeg for file, jpeg in thumbs.items() if jpeg is not None}
        for g in saved_groups:
            self.log(f"{g['dir']}: spread {g['spreadMs']:.2f} ms, " + ", ".join(f"{f['device']} {f['socket']} #{f['sequenceNum']} {f['offsetMs']:+.2f} ms" for f in g["frames"]))
        frames_total = sum(len(g["frames"]) for g in saved_groups)
        sync_label = "PTP locked" if ptp_summary["settled"] else "PTP not settled" if ptp else "no PTP"
        self.log(f"Captured {len(saved_groups)} groups ({frames_total} frames, spread up to {spread_max:.2f} ms, {sync_label}) and {len(calibration_files)} device calibrations into captures/{folder.name}")
        self.set_state("idle", f"Captured {len(saved_groups)} image groups ({frames_total} frames, spread up to {spread_max:.2f} ms, {sync_label}) into captures/{folder.name}.", progress=None)

    def capture_pointcloud_async(self, device_names):
        return self._start_job(self._capture_pointcloud, device_names, name="capture")

    def _capture_pointcloud(self, device_names):
        """Saves one merged point cloud of the selected stereo devices as PLY (the Point cloud pipeline: NeuralDepth per
        device, every cloud in the calibration origin), plus the cloud of every device, once the frames have locked."""
        args = self.args
        model = getattr(dai.DeviceModelZoo, f"NEURAL_DEPTH_{args.model}")
        size = tuple(args.size) if args.size else tuple(dai.node.NeuralDepth.getInputSize(model))
        self.set_state("capturing", "Checking the devices...", progress={"stage": "inspecting", "done": 0, "total": 1, "percent": None}, stats=None)
        self.log(f"Point cloud capture of {device_names} started (NeuralDepth {args.model}, {args.pc_fps:g} FPS)")
        device_ids, calibration = self._require_calibration(device_names)
        for name in device_names:
            if not self.device_info[name]["stereoPair"]:
                raise RuntimeError(f"{name} has no stereo pair, so it cannot produce depth.")
        self.set_state("capturing", progress={"stage": "ptp", "done": 0, "total": 1, "percent": None})
        ptp = self._prepare_ptp(device_names)
        if self.stop_event.is_set():
            self.set_state("idle", "Capture stopped.", progress=None)
            return
        self.set_state("capturing", "Connecting to devices...", progress={"stage": "connecting", "done": 0, "total": 1, "percent": None})
        handler = dai.beta.MultiDeviceCalibrationHandler(Path(calibration["path"]))
        self.log(f"Using calibration {Path(calibration['path']).name}")
        clouds = None
        spread_ms = None
        tolerance_ms = int(500.0 / args.pc_fps)  # half a frame period: the depth and color frames of one capture slot
        with dai.Pipeline(createImplicitDevice=False) as pipeline:
            pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)
            devices = [pipeline.addDevice(dai.DeviceInfo(name)) for name in device_names]
            for device in devices:
                device.setLogOutputLevel(dai.LogLevel.ERR)  # the on-device stereo sync warns for seconds while PTP locks the sensors
            # The aligned frames carry the extrinsics rebased onto the common origin device, which the host PointCloud nodes apply
            pipeline.setMultiDeviceCalibration(handler.getGraph())
            # Several devices' depth and color at the depth size do not fit through a gigabit link in step, so every device
            # keeps its newest aligned depth and color frames on the device and only tiny tick frames of the color camera
            # stream for the spread monitor; after the lock one slot is pulled and converted into clouds on the host
            monitor = self._monitor_sync(pipeline, args.pc_fps, ptp)
            streams = {}  # device id -> (request queue, depth queue, color queue, cloud depth queue, cloud color queue, cloud queue)
            for name, device, device_id in zip(device_names, devices, device_ids):
                info = self.device_info[name]
                if not device.isNeuralDepthSupported():
                    raise RuntimeError(f"{name} does not support NeuralDepth")
                left_socket, right_socket = (socket_from_name(s) for s in info["stereoPair"])
                left = pipeline.create(dai.node.Camera, device).build(left_socket, sensorFps=args.pc_fps)
                right = pipeline.create(dai.node.Camera, device).build(right_socket, sensorFps=args.pc_fps)
                color_socket = self._color_socket(info)
                color = pipeline.create(dai.node.Camera, device).build(color_socket, sensorFps=args.pc_fps)
                color.setOutputsNumFramesPool(2 * CAPTURE_RING)  # the ring holds CAPTURE_RING color frames
                for camera in (left, right, color):
                    if ptp:
                        camera.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.TIME_PTP)
                neural = pipeline.create(dai.node.NeuralDepth, device).build(left.requestFullResolutionOutput(fps=args.pc_fps), right.requestFullResolutionOutput(fps=args.pc_fps), model)
                neural.initialConfig.postProcessing.thresholdFilter.minRange = int(args.min_range * 1000)
                neural.initialConfig.postProcessing.thresholdFilter.maxRange = int(args.max_range * 1000)
                color_out = color.requestOutput(size, type=dai.ImgFrame.Type.RGB888i, fps=args.pc_fps, resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True)
                align = pipeline.create(dai.node.ImageAlign, device)
                align.setNumFramesPool(2 * CAPTURE_RING)  # the ring holds CAPTURE_RING aligned frames
                neural.depth.link(align.input)
                color_out.link(align.inputAlignTo)
                ring = pipeline.create(dai.node.Script, device)
                ring.setScript(CAPTURE_CLOUD_SCRIPT.replace("TOL", str(tolerance_ms)))
                for input_name, output in (("depth", align.outputAligned), ("color", color_out)):
                    ring.inputs[input_name].setBlocking(False)
                    ring.inputs[input_name].setMaxSize(4)
                    output.link(ring.inputs[input_name])
                monitor_input = monitor.inputs[device_id]
                monitor_input.setBlocking(False)
                monitor_input.setMaxSize(8)
                color.requestOutput(CAPTURE_TICK_SIZE, fps=args.pc_fps).link(monitor_input)
                pc = pipeline.create(dai.node.PointCloud)
                pc.setRunOnHost(True)
                pc.useCPUMT(args.threads)
                pc.initialConfig.setLengthUnit(dai.LengthUnit.METER)
                streams[device_id] = (ring.inputs["request"].createInputQueue(), ring.outputs["depth_out"].createOutputQueue(maxSize=4, blocking=False),
                                      ring.outputs["color_out"].createOutputQueue(maxSize=4, blocking=False), pc.inputDepth.createInputQueue(),
                                      pc.inputColor.createInputQueue(), pc.outputPointCloud.createOutputQueue(maxSize=4, blocking=False))
                self.log(f"{name}: NeuralDepth {args.model} on {info['stereoPair'][0]}+{info['stereoPair'][1]}, aligned to {color_socket.name} {size[0]}x{size[1]}")
            group_queue = monitor.out.createOutputQueue(maxSize=8, blocking=False)
            pipeline.start()
            try:
                self._wait_for_lock(pipeline, group_queue, ptp, 1, "Capturing")
                self.set_state(message="Capturing the point clouds...", progress={"stage": "capturing", "done": 0, "total": 1, "percent": None, "detail": "fetching the depth and color frames of one capture slot"})
                frames = None
                for attempt in range(1, 11):
                    group_queue.tryGetAll()  # a fresh slot whose frames are still in the rings
                    group = None
                    deadline = time.monotonic() + max(5.0, 5 * 1000.0 / args.pc_fps)
                    while pipeline.isRunning() and not self.stop_event.is_set() and time.monotonic() < deadline:
                        group = group_queue.tryGet()
                        if group is not None:
                            break
                        time.sleep(0.02)
                    if self.stop_event.is_set():
                        raise InterruptedError
                    if group is None:
                        raise RuntimeError("No synchronized frame group arrived: the cameras are not delivering frames in step.")
                    spread_ms = group.getIntervalNs() / 1e6
                    self.set_ptp(spreadMs=round(spread_ms, 2), spreadAt=time.monotonic())
                    targets = {device_id: int(seconds(group[device_id].getTimestampDevice()) * 1000) for device_id in device_ids}
                    for device_id, (request_queue, *_) in streams.items():
                        request = dai.Buffer()
                        request.setData(list(str(targets[device_id]).encode()))
                        request_queue.send(request)
                    frames = {}
                    missing = []
                    deadline = time.monotonic() + 30.0
                    pending = set(device_ids)
                    while pending and pipeline.isRunning() and not self.stop_event.is_set() and time.monotonic() < deadline:
                        for device_id in list(pending):
                            _, depth_queue, color_queue, *_ = streams[device_id]
                            message = depth_queue.tryGet()
                            if message is None:
                                continue
                            if not isinstance(message, dai.ImgFrame):
                                missing.append(device_id)
                                pending.discard(device_id)
                                continue
                            color_frame = color_queue.get(timedelta(seconds=5))
                            if color_frame is None:
                                raise RuntimeError(f"{device_id}: the color frame did not follow the depth frame")
                            frames[device_id] = (message, color_frame)
                            pending.discard(device_id)
                        time.sleep(0.02)
                    if self.stop_event.is_set():
                        raise InterruptedError
                    if pending:
                        raise RuntimeError(f"Frames of {', '.join(pending)} did not arrive within 30 s.")
                    if not missing:
                        break
                    names = [name for name, device_id in zip(device_names, device_ids) if device_id in missing]
                    self.log(f"Attempt {attempt}: no depth yet for the slot on {', '.join(names)} (depth pauses while PTP re-times the stereo sensors), trying the next slot")
                    self.set_state(progress={"stage": "capturing", "done": 0, "total": 1, "percent": None, "detail": f"attempt {attempt}: waiting for depth of {', '.join(names)}"})
                    frames = None
                    time.sleep(1.0)
                if frames is None:
                    raise RuntimeError("The devices never had depth and color of the same capture slot: are all devices producing depth?")
                self.set_state(progress={"stage": "capturing", "done": 0, "total": 1, "percent": None, "detail": "converting the frames into point clouds"})
                clouds = []
                for name, device_id in zip(device_names, device_ids):
                    depth_frame, color_frame = frames[device_id]
                    _, _, _, cloud_depth_queue, cloud_color_queue, cloud_queue = streams[device_id]
                    cloud_queue.tryGetAll()
                    cloud_depth_queue.send(depth_frame)
                    cloud_color_queue.send(color_frame)
                    cloud = cloud_queue.get(timedelta(seconds=15))
                    if cloud is None:
                        raise RuntimeError(f"{name}: the point cloud node produced nothing for the captured frames")
                    points, colors = cloud_arrays(cloud)
                    # The pose the point cloud node applied: the depth camera relative to the common origin (rebased calibration)
                    extrinsics = depth_frame.getTransformation().getExtrinsics()
                    pose = {"toDeviceId": extrinsics.toDeviceId, "toCameraSocket": socket_name(extrinsics.toCameraSocket), "unit": str(extrinsics.lengthUnit).split(".")[-1],
                            "matrix": np.asarray(extrinsics.getTransformationMatrix(), dtype=np.float64).round(6).tolist()}
                    clouds.append({"device": name, "deviceId": device_id, "points": points, "colors": colors, "sequenceNum": depth_frame.getSequenceNum(), "pose": pose,
                                   "timestamp_s": seconds(depth_frame.getTimestamp()), "timestampDevice_s": seconds(depth_frame.getTimestampDevice()),
                                   "timestampSystem_s": seconds(depth_frame.getTimestampSystem()), "colorTimestampDevice_s": seconds(color_frame.getTimestampDevice())})
            except InterruptedError:
                clouds = None
            finally:
                pipeline.stop()
        if not clouds:
            self.log("Capture stopped")
            self.set_state("idle", "Capture stopped.", progress=None)
            return

        self.set_state(message="Saving the point clouds...", progress={"stage": "saving", "done": 0, "total": 1, "percent": None})
        now = time.time()
        folder = CAPTURE_DIR / (time.strftime("%Y%m%d-%H%M%S", time.localtime(now)) + "-pointcloud")
        folder.mkdir(parents=True, exist_ok=True)
        entries = []
        source = "timestampSystem_s" if ptp else "timestamp_s"  # the clock the monitor grouped by
        first = min(c[source] for c in clouds)
        for c in clouds:
            file = f"{c['device']}.ply".replace("/", "_").replace(" ", "_")
            write_ply(folder / file, c["points"], c["colors"])
            entries.append({**{k: v for k, v in c.items() if k not in ("points", "colors")}, "file": file, "points": len(c["points"]), "offsetMs": round((c[source] - first) * 1000.0, 3)})
        merged_points = np.concatenate([c["points"] for c in clouds])
        write_ply(folder / "merged.ply", merged_points, np.concatenate([c["colors"] for c in clouds]))
        with self.lock:
            ptp_summary = {"active": self.ptp["active"], "settled": self.ptp["settled"], "master": self.ptp["master"], "warning": self.ptp["warning"]}
        info = {
            "kind": "pointcloud", "dir": folder.name, "time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
            "calibration": calibration["path"], "origin": calibration["origin"], "model": args.model, "fps": args.pc_fps, "size": list(size),
            "minRange_m": args.min_range, "maxRange_m": args.max_range, "spreadMs": round(spread_ms, 3), "ptp": ptp_summary,
            "merged": {"file": "merged.ply", "points": int(len(merged_points))}, "clouds": entries,
        }
        (folder / "pointcloud.json").write_text(json.dumps(info, indent=1))
        with self.lock:
            self.latest_capture = info
        for e in entries:
            self.log(f"{e['device']}: {e['points']} points, depth seq {e['sequenceNum']}, {e['offsetMs']:+.2f} ms -> {e['file']}")
        sync_label = "PTP locked" if ptp_summary["settled"] else "PTP not settled" if ptp else "no PTP"
        self.log(f"Captured a merged point cloud of {len(merged_points)} points ({spread_ms:.2f} ms spread, {sync_label}) into captures/{folder.name}")
        self.set_state("idle", f"Captured a merged point cloud of {len(merged_points):,} points ({spread_ms:.2f} ms spread, {sync_label}) into captures/{folder.name}.", progress=None)

    @staticmethod
    def _thumbnail(image, width=640):
        """JPEG bytes of the image scaled down to at most `width` pixels wide (None when encoding fails)."""
        h, w = image.shape[:2]
        if w > width:
            image = cv2.resize(image, (width, max(1, round(h * width / w))), interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return encoded.tobytes() if ok else None

    def capture_file(self, relative, thumb=False):
        """(bytes, content type) of a file in captures/, or its 640-wide JPEG thumbnail; None when there is no such file."""
        if thumb:
            with self.lock:
                jpeg = self.capture_thumbs.get(relative)
            if jpeg is not None:
                return jpeg, "image/jpeg"
        path = (CAPTURE_DIR / relative).resolve()
        if CAPTURE_DIR.resolve() not in path.parents or not path.is_file():
            return None
        if thumb:
            image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
            jpeg = self._thumbnail(image) if image is not None else None
            if jpeg is None:
                return None
            with self.lock:
                self.capture_thumbs[relative] = jpeg
            return jpeg, "image/jpeg"
        return path.read_bytes(), VISUALIZER_MIME.get(path.suffix, "application/octet-stream")

    # ---------------------------------------------------------------- shared by the run modes
    def _require_calibration(self, device_names):
        """Inspection info and the newest stored calibration covering the devices (raises when there is none)."""
        self._inspect(device_names)
        device_ids = [self.device_info[name]["deviceId"] for name in device_names]
        info = self.find_calibration(device_ids)
        if info is None:
            raise RuntimeError("These devices have not been calibrated together yet. Press Calibrate first.")
        return device_ids, info

    def find_calibration(self, device_ids):
        """Newest calibration that covers every given device (an exact match wins)."""
        wanted = set(device_ids)
        with self.lock:
            candidates = [c for c in self.calibrations.values() if wanted <= set(c["deviceIds"])]
        candidates.sort(key=lambda c: (set(c["deviceIds"]) == wanted, c["time"]), reverse=True)
        return candidates[0] if candidates else None

    @staticmethod
    def _color_socket(info):
        colors = [c["socket"] for c in info["cameras"] if c["type"] == "COLOR" and c["calibrated"]]
        return socket_from_name(colors[0] if colors else "CAM_A")

    def _panorama_socket(self, info, calibration):
        """Camera used for the panorama: the first camera the calibration registered for this device, else the left
        stereo camera, else the device's first calibrated camera."""
        for d in calibration.get("devices", []):
            if d["deviceId"] == info["deviceId"] and d.get("cameras"):
                return socket_from_name(d["cameras"][0]["socket"])
        if info["stereoPair"]:
            return socket_from_name(info["stereoPair"][0])
        calibrated = [c["socket"] for c in info["cameras"] if c["calibrated"]] or [c["socket"] for c in info["cameras"]]
        return socket_from_name(calibrated[0])

    def _monitor_sync(self, pipeline, fps, ptp):
        """Host Sync measuring the cross-device timestamp spread of one capture slot (threshold just under a period)."""
        monitor = pipeline.create(dai.node.Sync)
        monitor.setRunOnHost(True)
        monitor.setSyncThreshold(timedelta(milliseconds=0.9 * 1000.0 / fps))
        monitor.setSyncAttempts(-1)
        monitor.setTimestampSource(dai.node.Sync.TimestampSource.SYSTEM if ptp else dai.node.Sync.TimestampSource.HOST)
        return monitor

    # ---------------------------------------------------------------- point cloud
    def pointcloud_async(self, device_names):
        return self._start_job(self._pointcloud, device_names)

    def _pointcloud(self, device_names):
        args = self.args
        model = getattr(dai.DeviceModelZoo, f"NEURAL_DEPTH_{args.model}")
        size = tuple(args.size) if args.size else tuple(dai.node.NeuralDepth.getInputSize(model))
        self.set_state("pointcloud", "Checking the devices...", progress=None, stats=None)
        self.log(f"Point cloud of {device_names} started (NeuralDepth {args.model}, {args.pc_fps:g} FPS)")
        device_ids, calibration = self._require_calibration(device_names)
        for name in device_names:
            if not self.device_info[name]["stereoPair"]:
                raise RuntimeError(f"{name} has no stereo pair, so it cannot produce depth.")
        with self.lock:
            self.live = {"mode": "pointcloud", "calibration": calibration["path"]}
        ptp = self._prepare_ptp(device_names)
        if self.stop_event.is_set():
            self.set_state("idle", "Point cloud stopped.", stats=None)
            return
        self.set_state("pointcloud", "Connecting to devices...")
        # Just under one frame period with PTP: frames of different capture slots are never merged, but the merge keeps
        # going while the PTP frame sync pulls the sensors onto the shared schedule after start
        sync_threshold_ms = 900.0 / args.pc_fps if ptp else 1000.0 / args.pc_fps
        handler = dai.beta.MultiDeviceCalibrationHandler(Path(calibration["path"]))
        self.log(f"Using calibration {Path(calibration['path']).name}")
        with dai.Pipeline(createImplicitDevice=False) as pipeline:
            pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)
            devices = [pipeline.addDevice(dai.DeviceInfo(name)) for name in device_names]
            for device in devices:
                device.setLogOutputLevel(dai.LogLevel.ERR)  # the on-device stereo sync warns for seconds while PTP locks the sensors
            pipeline.setMultiDeviceCalibration(handler.getGraph())
            merge = pipeline.create(dai.node.Sync)
            merge.setRunOnHost(True)
            merge.setSyncThreshold(timedelta(milliseconds=sync_threshold_ms))
            merge.setSyncAttempts(-1)
            if ptp:
                merge.setTimestampSource(dai.node.Sync.TimestampSource.SYSTEM)
            monitor = self._monitor_sync(pipeline, args.pc_fps, ptp)
            for name, device, device_id in zip(device_names, devices, device_ids):
                info = self.device_info[name]
                if not device.isNeuralDepthSupported():
                    raise RuntimeError(f"{name} does not support NeuralDepth")
                left_socket, right_socket = (socket_from_name(s) for s in info["stereoPair"])
                left = pipeline.create(dai.node.Camera, device).build(left_socket, sensorFps=args.pc_fps)
                right = pipeline.create(dai.node.Camera, device).build(right_socket, sensorFps=args.pc_fps)
                color_socket = self._color_socket(info)
                color = pipeline.create(dai.node.Camera, device).build(color_socket, sensorFps=args.pc_fps)
                for camera in (left, right, color):
                    if ptp:
                        camera.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.TIME_PTP)
                neural = pipeline.create(dai.node.NeuralDepth, device).build(left.requestFullResolutionOutput(fps=args.pc_fps), right.requestFullResolutionOutput(fps=args.pc_fps), model)
                neural.initialConfig.postProcessing.thresholdFilter.minRange = int(args.min_range * 1000)
                neural.initialConfig.postProcessing.thresholdFilter.maxRange = int(args.max_range * 1000)
                color_out = color.requestOutput(size, type=dai.ImgFrame.Type.RGB888i, fps=args.pc_fps, resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True)
                align = pipeline.create(dai.node.ImageAlign, device)
                neural.depth.link(align.input)
                color_out.link(align.inputAlignTo)
                # Host point cloud in the calibration origin: the node applies the aligned frame's extrinsics, which
                # pipeline.setMultiDeviceCalibration rebased onto the common origin device
                pc = pipeline.create(dai.node.PointCloud)
                pc.setRunOnHost(True)
                pc.useCPUMT(args.threads)
                pc.initialConfig.setLengthUnit(dai.LengthUnit.METER)
                align.outputAligned.link(pc.inputDepth)
                color_out.link(pc.inputColor)
                merge_input = merge.inputs[device_id]
                merge_input.setBlocking(False)
                merge_input.setMaxSize(4)
                pc.outputPointCloud.link(merge_input)
                monitor_input = monitor.inputs[device_id]
                monitor_input.setBlocking(False)
                monitor_input.setMaxSize(4)
                align.outputAligned.link(monitor_input)
                self.log(f"{name}: NeuralDepth {args.model} on {info['stereoPair'][0]}+{info['stereoPair'][1]}, aligned to {color_socket.name} {size[0]}x{size[1]}")

            group_queue = merge.out.createOutputQueue(maxSize=4, blocking=False)
            spread_queue = monitor.out.createOutputQueue(maxSize=4, blocking=False)
            pipeline.start()
            start = time.monotonic()
            last_report = start
            count = 0
            window_count = 0
            # With PTP the left and right sensors are re-timed independently after start, the on-device stereo sync
            # rejects them for a few seconds and depth pauses; clouds are forwarded only once they arrive steadily
            steady = not ptp
            arrivals = collections.deque(maxlen=10)
            self.set_state(message="Waiting for the cameras to lock onto the shared clock (around 10 s)..." if ptp else "Starting cameras...")
            try:
                while pipeline.isRunning() and not self.stop_event.is_set():
                    group = group_queue.get(timedelta(milliseconds=200))
                    now = time.monotonic()
                    for spread in spread_queue.tryGetAll():
                        self.set_ptp(spreadMs=round(spread.getIntervalNs() / 1e6, 2), spreadAt=now)
                    if group is None:
                        if count == 0 and now - start > 20 and now - last_report > 5:
                            self.set_state(message="No merged cloud yet. Are all devices producing depth?")
                            last_report = now
                        continue
                    count += 1
                    window_count += 1
                    if not steady:
                        arrivals.append(now)
                        if len(arrivals) == arrivals.maxlen and now - arrivals[0] < 1.5 * arrivals.maxlen / args.pc_fps:
                            steady = True
                            self.log(f"Point cloud stream steady {now - start:.1f} s after start")
                            self.set_state(message="Streaming the merged point cloud.")
                        else:
                            continue
                    merged = self._merge_clouds([group[device_id] for device_id in device_ids])
                    with self.lock:
                        self.streaming = True
                    self.cloud_queue.send(merged)
                    if now - last_report >= 2.0:
                        self.set_state(stats={"rate": round(window_count / (now - last_report), 1), "clouds": count, "points": merged.getWidth() * merged.getHeight(), "origin": calibration["origin"]})
                        last_report = now
                        window_count = 0
            finally:
                pipeline.stop()
        with self.lock:
            self.streaming = False
        self.log(f"Point cloud stopped after {count} merged clouds")
        self.set_state("idle", "Point cloud stopped.", stats=None)

    @staticmethod
    def _merge_clouds(clouds):
        """One PointCloudData with the points of all clouds (they are already in the common origin frame)."""
        points, colors = [], []
        for cloud in clouds:
            if cloud.isColor():
                p, c = cloud.getPointsRGB()
            else:
                p = cloud.getPoints()
                c = np.full((len(p), 4), 255, dtype=np.uint8)
            points.append(p)
            colors.append(c)
        merged = dai.PointCloudData()
        all_points = np.concatenate(points) if points else np.zeros((0, 3), np.float32)
        merged.setPointsRGB(np.ascontiguousarray(all_points, dtype=np.float32), np.ascontiguousarray(np.concatenate(colors) if colors else np.zeros((0, 4), np.uint8)))
        merged.setSize(len(all_points), 1)
        merged.setTimestamp(clouds[0].getTimestamp())
        merged.setSequenceNum(clouds[0].getSequenceNum())
        merged.updateBoundingBox()
        return merged

    # ---------------------------------------------------------------- panorama
    def panorama_async(self, device_names, projection, filters=None):
        if filters is not None:
            if not DETECTIONS_AVAILABLE:
                raise ValueError("Detections requires an updated DepthAI wheel with ImgDetectionsFilter and Stitching source masks. See USAGE.md.")
            filters, _ = detection_config(filters)
        return self._start_job(self._panorama, device_names, projection, filters, name="detections" if filters is not None else "panorama")

    def update_detection_filters(self, filters):
        with self.lock:
            if not self.live or self.live["mode"] != "detections":
                raise ValueError("Start Detections before applying filters")
            _, config = detection_config(filters)
            self.pending_detection_config = config

    def _panorama(self, device_names, projection, filters=None):
        args = self.args
        detections = filters is not None
        mode = "detections" if detections else "panorama"
        title = "Detections" if detections else "Panorama"
        projection = projection if projection in PROJECTIONS else "cylindrical"
        self.set_state(mode, "Checking the devices...", progress=None, stats=None)
        self.log(f"{title} of {device_names} started ({projection}, {args.pano_size[0]}x{args.pano_size[1]} at {args.pano_fps:g} FPS)")
        _, calibration = self._require_calibration(device_names)
        with self.lock:
            self.live = {"mode": mode, "projection": projection, "calibration": calibration["path"]}
            self.pano_jpeg = None
        ptp = self._prepare_ptp(device_names)
        if self.stop_event.is_set():
            self.set_state("idle", f"{title} stopped.", stats=None)
            return
        self.set_state(mode, "Connecting to devices (the first detection run may download the model)..." if detections else "Connecting to devices...")
        handler = dai.beta.MultiDeviceCalibrationHandler(Path(calibration["path"]))
        self.log(f"Using calibration {Path(calibration['path']).name}")
        with dai.Pipeline(createImplicitDevice=False) as pipeline:
            pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)
            devices = [pipeline.addDevice(dai.DeviceInfo(name)) for name in device_names]
            pipeline.setMultiDeviceCalibration(handler.getGraph())
            topics = []
            if detections:
                detection_sync = self._monitor_sync(pipeline, args.pano_fps, ptp)
                demux = pipeline.create(dai.node.MessageDemux)
                demux.setRunOnHost(True)
                detection_sync.out.link(demux.input)
                filtered = pipeline.create(dai.node.ImgDetectionsFilter)
                filtered.setRunOnHost(True)
                detection_config(filters, filtered.initialConfig)
                config_queues = [filtered.inputConfig.createInputQueue(maxSize=1, blocking=False)]
                detection_queue = filtered.out.createOutputQueue(maxSize=2, blocking=False)
                spread_queue = detection_sync.out.createOutputQueue(maxSize=2, blocking=False)
            outputs = []
            for index, (name, device) in enumerate(zip(device_names, devices)):
                info = self.device_info[name]
                socket = self._panorama_socket(info, calibration)
                camera = pipeline.create(dai.node.Camera, device).build(socket, sensorFps=args.pano_fps)
                if ptp:
                    camera.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.TIME_PTP)
                outputs.append(camera.requestOutput(tuple(args.pano_size), dai.ImgFrame.Type.NV12, fps=args.pano_fps, enableUndistortion=True))
                if detections:
                    model = dai.NNModelDescription(args.detections_model, platform=device.getPlatformAsString())
                    network = pipeline.create(dai.node.DetectionNetwork, device).build(camera, model, fps=args.pano_fps, resizeMode=dai.ImgResizeMode.LETTERBOX)
                    network.setConfidenceThreshold(0.1)
                    key = f"cam{index}"
                    network.out.link(detection_sync.inputs[key])
                    # Demux outputs default to Buffer; filters require a concrete ImgDetections type.
                    demux.outputs[key].setPossibleDatatypes([(dai.DatatypeEnum.ImgDetections, False)])
                    demux.outputs[key].link(filtered.inputs[key])
                    source_filter = pipeline.create(dai.node.ImgDetectionsFilter)
                    source_filter.setRunOnHost(True)
                    detection_config(filters, source_filter.initialConfig)
                    network.out.link(source_filter.inputs[key])
                    config_queues.append(source_filter.inputConfig.createInputQueue(maxSize=1, blocking=False))
                    group = f"Camera {index + 1}: {name} / {socket.name}"
                    topics.extend([(f"Source image {index + 1}", network.passthrough, group), (f"Source detections {index + 1}", source_filter.out, group)])
                self.log(f"{name}: {socket.name} undistorted {args.pano_size[0]}x{args.pano_size[1]}")
            stitching = pipeline.create(STITCHING_NODE).build(outputs)
            stitching.setRunOnHost(True)
            stitching.setMode(STITCHING_NODE.Mode.PANORAMA)
            stitching.setCameraModel(PROJECTIONS[projection])
            stitching.setUseInputCalibration(True)
            stitching.setSeamFinder(getattr(STITCHING_NODE.SeamFinder, args.seam))
            stitching.setMaxPanoramaSize(2560 if detections else 8192, 1440 if detections else 4096)
            stitching.setSyncThreshold(timedelta(milliseconds=(900.0 if ptp else 1000.0) / args.pano_fps))
            stitching.sync.setTimestampSource(dai.node.Sync.TimestampSource.SYSTEM if ptp else dai.node.Sync.TimestampSource.HOST)
            stitching.sync.setSyncAttempts(-1)
            if detections:
                stitching.out.link(filtered.inputReference)
                for index in range(len(outputs)):
                    stitching.outSourceMasks[f"input{index}"].link(filtered.inputSourceMasks[f"cam{index}"])
                topics.extend([
                    ("Panorama", stitching.out, "Detections"),
                    ("Filtered detections", filtered.out, "Detections"),
                ])
                for topic, output, group in topics:
                    self.remote.addTopic(topic, output, group)
                    self.detection_topics.append(topic)
                self.log(f"Detection model: {args.detections_model}; source masks enabled, filters: {filters}")
            out_queue = stitching.out.createOutputQueue(maxSize=2, blocking=False)
            pipeline.start()
            start = time.monotonic()
            last_report = start
            count = 0
            window_count = 0
            detection_counts = None
            self.set_state(message="Starting cameras...")
            try:
                while pipeline.isRunning() and not self.stop_event.is_set():
                    if detections:
                        pipeline.processTasks()
                        with self.lock:
                            pending = self.pending_detection_config
                            self.pending_detection_config = None
                        if pending is not None:
                            for queue in config_queues:
                                queue.send(pending)
                    frame = out_queue.get(timedelta(milliseconds=200))
                    now = time.monotonic()
                    if detections:
                        for spread in spread_queue.tryGetAll():
                            self.set_ptp(spreadMs=round(spread.getIntervalNs() / 1e6, 2), spreadAt=now)
                        for result in detection_queue.tryGetAll():
                            detection_counts = {"detections": len(result.detections)}
                        if detection_counts is not None:
                            with self.lock:
                                self.streaming = True
                            self.set_state(message="Streaming the panorama with merged detections.")
                        elif count and now - start > 20:
                            self.set_state(message="Panorama is streaming; waiting for synchronized detections from every camera.")
                    if frame is None:
                        if count == 0 and now - start > 20 and now - last_report > 5:
                            self.set_state(message="Waiting for panorama and detections. Check that every camera streams." if detections else "No panorama yet. Do the cameras stream?")
                            last_report = now
                        continue
                    count += 1
                    window_count += 1
                    if not detections:
                        ok, encoded = cv2.imencode(".jpg", frame.getCvFrame(), [cv2.IMWRITE_JPEG_QUALITY, 85])
                        if not ok:
                            continue
                        with self.pano_cond:
                            self.pano_jpeg = encoded.tobytes()
                            self.pano_seq += 1
                            self.pano_cond.notify_all()
                    if count == 1:
                        t = frame.getTransformation()
                        fx = t.getIntrinsicMatrix()[0][0]
                        span = math.degrees(frame.getWidth() / fx) if fx > 0 and projection != "perspective" else None
                        self.log(f"First panorama after {now - start:.1f} s: {frame.getWidth()}x{frame.getHeight()}" + (f", {span:.0f} deg" if span else ""))
                        if not detections:
                            with self.lock:
                                self.streaming = True
                            self.set_state(message="Streaming the panorama.")
                    if now - last_report >= 2.0:
                        self.set_state(stats={"rate": round(window_count / (now - last_report), 1), "frames": count, "width": frame.getWidth(), "height": frame.getHeight(), "projection": projection, **(detection_counts or {})})
                        last_report = now
                        window_count = 0
            finally:
                pipeline.stop()
        with self.lock:
            self.streaming = False
        self.log(f"{title} stopped after {count} frames")
        self.set_state("idle", f"{title} stopped.", stats=None)

    def wait_panorama(self, seen_seq, timeout=1.0):
        """Blocks until a panorama newer than seen_seq exists; returns (jpeg, seq) or (None, seen_seq)."""
        with self.pano_cond:
            if self.pano_seq <= seen_seq:
                self.pano_cond.wait(timeout)
            if self.pano_seq > seen_seq and self.pano_jpeg is not None:
                return self.pano_jpeg, self.pano_seq
            return None, seen_seq

    # ---------------------------------------------------------------- shutdown
    def close(self):
        self.stop_event.set()
        self.credentials_event.set()
        self.distance_event.set()
        if self.job is not None and self.job.is_alive():
            self.job.join(timeout=20)
        # ~RemoteConnection joins its topic threads with the GIL held; drain the queue first and let an external
        # watchdog kill us if it still hangs
        self.cloud_queue.close()
        time.sleep(0.5)
        self.cloud_queue = None
        self.remote = None


class VisualizerHandler(BaseHTTPRequestHandler):
    """Serves the DepthAI visualizer static files from the app's own `visualizer/` folder."""

    def log_message(self, format, *args):  # noqa: A002 - quiet
        pass

    def do_GET(self):
        relative = self.path.split("?")[0].lstrip("/") or "index.html"
        path = (VISUALIZER_DIR / relative).resolve()
        if VISUALIZER_DIR.resolve() not in path.parents or not path.is_file():
            self.send_response(404)
            self.end_headers()
            return
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", VISUALIZER_MIME.get(path.suffix, "application/octet-stream"))
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class Handler(BaseHTTPRequestHandler):
    app: App = None

    def log_message(self, format, *args):  # noqa: A002 - quiet
        pass

    def _json(self, payload, status=200):
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            index_html = (APP_DIR / "index.html").read_bytes()  # read per request so page edits need no restart
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(index_html)))
            self.end_headers()
            self.wfile.write(index_html)
        elif self.path == "/api/state":
            self._json(self.app.snapshot())
        elif self.path.startswith("/api/panorama.mjpeg"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = 0
            try:
                while True:
                    jpeg, seq = self.app.wait_panorama(seq, timeout=1.0)
                    if jpeg is None:
                        if self.app.live is None:
                            break
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
        elif self.path.startswith("/api/panorama.jpg"):
            jpeg = self.app.pano_jpeg
            if jpeg is None:
                return self._json({"error": "no frame"}, 404)
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
        elif self.path.startswith("/api/capture/"):
            from urllib.parse import unquote

            relative, _, query = self.path[len("/api/capture/"):].partition("?")
            found = self.app.capture_file(unquote(relative), thumb="thumb" in query)
            if found is None:
                return self._json({"error": "not found"}, 404)
            data, mime = found
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Cache-Control", "max-age=3600")  # captures never change once written
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif self.path.startswith("/api/preview/"):
            from urllib.parse import unquote

            jpeg = self.app.preview_jpeg(unquote(self.path[len("/api/preview/"):].split("?")[0]))
            if jpeg is None:
                return self._json({"error": "no frame"}, 404)
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length") or 0)
        body = json.loads(self.rfile.read(length) or b"{}")
        devices = body.get("devices", [])
        if self.path == "/api/refresh":
            ok = self.app.discover_async()
        elif self.path == "/api/inspect":
            if not devices:
                return self._json({"error": "Select at least one device"}, 400)
            ok = self.app.inspect_async(devices)
        elif self.path == "/api/calibrate":
            if len(devices) < 2:
                return self._json({"error": "Select at least two devices"}, 400)
            ok = self.app.calibrate_async(devices, body.get("modes") or {}, body.get("distances"), bool(body.get("askDistances")))
        elif self.path == "/api/capture":
            if not devices:
                return self._json({"error": "Select at least one device"}, 400)
            ok = self.app.capture_async(devices, body.get("modes") or {})
        elif self.path == "/api/capture/pointcloud":
            if len(devices) < 2:
                return self._json({"error": "Select at least two devices"}, 400)
            ok = self.app.capture_pointcloud_async(devices)
        elif self.path == "/api/pointcloud":
            if len(devices) < 2:
                return self._json({"error": "Select at least two devices"}, 400)
            ok = self.app.pointcloud_async(devices)
        elif self.path in ("/api/panorama", "/api/detections"):
            if len(devices) < 2:
                return self._json({"error": "Select at least two devices"}, 400)
            try:
                filters = body.get("filters", {}) if self.path == "/api/detections" else None
                if self.path == "/api/detections" and not isinstance(filters, dict):
                    raise ValueError("Detection filters must be an object")
                if len(set(devices)) != len(devices):
                    raise ValueError("Select distinct devices")
                ok = self.app.panorama_async(devices, body.get("projection") or "cylindrical", filters)
            except ValueError as e:
                return self._json({"error": str(e)}, 400)
        elif self.path == "/api/detections/filters":
            try:
                self.app.update_detection_filters(body.get("filters", {}))
            except ValueError as e:
                return self._json({"error": str(e)}, 400)
            ok = True
        elif self.path == "/api/distances":
            self.app.submit_distances(body.get("distances") or [])
            ok = True
        elif self.path == "/api/stop":
            self.app.stop()
            ok = True
        elif self.path == "/api/ptp/passwords":
            self.app.submit_passwords(body.get("passwords", {}))
            ok = True
        elif self.path == "/api/ptp/skip":
            self.app.skip_ptp()
            ok = True
        else:
            return self._json({"error": "not found"}, 404)
        if not ok:
            return self._json({"error": "Another job is still running, press Stop first"}, 409)
        self._json({"ok": True})


def start_shutdown_watchdog(timeout_s):
    """External process that kills this one if the shutdown hangs (C++ destructors can block with the GIL held)."""
    pid = os.getpid()
    if os.name == "nt":
        script = f"timeout /t {int(timeout_s)} /nobreak >nul & taskkill /F /PID {pid} >nul 2>&1"
        return subprocess.Popen(["cmd", "/c", script], stdin=subprocess.DEVNULL, creationflags=subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.DETACHED_PROCESS)
    script = f"sleep {timeout_s}; kill -0 {pid} 2>/dev/null || exit 0; echo 'shutdown hung for {timeout_s:.0f} s, killing pid {pid}' >&2; kill -KILL {pid}"
    return subprocess.Popen(["/bin/sh", "-c", script], stdin=subprocess.DEVNULL, start_new_session=True)


def main():
    args = parse_args()
    app = App(args)
    Handler.app = app
    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    print(f"\nMulti-device app {APP_VERSION}: http://localhost:{args.port}   (visualizer: http://localhost:{args.http_port}, {'bundled' if app.serve_visualizer else 'embedded in depthai'}; ssh via {'paramiko' if app.ssh.paramiko else 'OpenSSH client'})\n", flush=True)
    while not stop.is_set():
        stop.wait(0.2)
    print("Shutting down", flush=True)
    watchdog = start_shutdown_watchdog(25)
    server.shutdown()
    app.close()
    watchdog.kill()


if __name__ == "__main__":
    main()
