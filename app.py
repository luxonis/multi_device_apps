#!/usr/bin/env python3
"""Multi-device app: pick OAK 4 devices on the network, calibrate them against each other
(dai.beta.node.MultiDeviceCalibration), then run a merged point cloud or a calibrated panorama on them.

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
perspective surface) and streams the result to the page as MJPEG.

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

APP_VERSION = "0.1"
APP_DIR = Path(__file__).resolve().parent
CALIBRATION_DIR = APP_DIR / "calibrations"
VISUALIZER_DIR = APP_DIR / "visualizer"  # DepthAI visualizer static files served by this app (if present)
VISUALIZER_MIME = {".html": "text/html", ".css": "text/css", ".js": "application/javascript", ".png": "image/png", ".svg": "image/svg+xml", ".ico": "image/x-icon", ".json": "application/json", ".txt": "text/plain", ".map": "application/json", ".woff": "font/woff", ".woff2": "font/woff2", ".wasm": "application/wasm"}
POINT_CLOUD_TOPIC = "merged_point_cloud"
PROJECTIONS = {"cylindrical": dai.CameraModel.Cylindrical, "spherical": dai.CameraModel.Equirectangular, "perspective": dai.CameraModel.Perspective}
CAMERA_FPS = 10.0  # the PTP frame lock holds within ~0.01 ms at 10 FPS but wanders by 5-20 ms at 5 FPS
PTP_CONF = "/etc/luxonis-ptp.conf"
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
    parser.add_argument("--seam", default="GRAPHCUT_COLOR", choices=["NONE", "VORONOI", "DP_COLOR", "DP_COLOR_GRAD", "GRAPHCUT_COLOR", "GRAPHCUT_COLOR_GRAD"], help="Panorama seam finder; NONE copies the inputs without blending (default: GRAPHCUT_COLOR)")
    parser.add_argument("--sample-count", type=int, default=10, help="Synchronized image groups collected per calibration (default: 10)")
    parser.add_argument("--fps", type=float, default=CAMERA_FPS, help="Camera FPS while calibrating (default: 10)")
    parser.add_argument("--sync-threshold", type=float, default=1.0, help="Sync threshold of the calibration node in seconds; the scene is static, so this only has to catch one frame of every camera (default: 1)")
    parser.add_argument("-c", "--calibration", type=Path, nargs="*", default=[], help="Existing multi-device calibration JSON files to import (copied into calibrations/)")
    parser.add_argument("--no-ptp", action="store_true", help="Never check or configure PTP on the devices (free-running cameras, timestamp sync only)")
    parser.add_argument("--warmup-seconds", type=float, default=4.0, help="Seconds to stream before the node starts collecting, so auto exposure settles (default: 4)")
    parser.add_argument("--ptp-settle-ms", type=float, default=2.0, help="PTP counts as settled once 5 consecutive image groups of all devices lie within this many ms (default: 2)")
    parser.add_argument("--ptp-settle-timeout", type=float, default=20.0, help="Give up waiting for PTP to settle after this many seconds and calibrate anyway; the scene is static, so a few ms of spread do not matter (default: 20)")
    return parser.parse_args()


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
    otherwise the OpenSSH client, with the password handed over through SSH_ASKPASS. Without a password only
    key / agent authentication is tried."""

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
            client.connect(ip, username="root", password=password, timeout=5, banner_timeout=10, auth_timeout=10,
                           allow_agent=password is None, look_for_keys=password is None)
        except paramiko.AuthenticationException:
            return "needs_password", "authentication failed"
        except paramiko.SSHException as e:
            if password is None or "auth" in str(e).lower():
                return "needs_password", str(e)
            return "unreachable", str(e) or "ssh connection failed"
        except OSError as e:
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
        self.phase = "idle"  # idle | discovering | inspecting | calibrating | distance_input | ptp_password | pointcloud | panorama | error
        self.message = "Select devices."
        self.stats = None  # live numbers of the running point cloud / panorama
        self.live = None  # {"mode": "pointcloud"|"panorama", ...} while one of them runs
        self.streaming = False  # the viewer has content (clouds are being sent / panoramas arrive)
        self.pano_jpeg = None
        self.pano_seq = 0
        self.pano_cond = threading.Condition()
        self.progress = None
        self.devices = []  # discovered devices (dicts)
        self.device_info = {}  # device name -> inspection dict (cameras, stereo pair, local origin)
        self.calibrations = {}  # path -> calibration info dict (metrics, poses)
        self.latest_calibration = None  # path of the result of the last calibration job
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
    def _start_job(self, target, *job_args):
        with self.lock:
            if self.job is not None and self.job.is_alive():
                return False
            self.stop_event.clear()
            self.credentials_event.clear()
            self.distance_event.clear()
            self.ptp_skip_requested = False
            self.distance_request = None
            self.job_devices = list(job_args[0]) if job_args and isinstance(job_args[0], list) else []
            self.last_job = {self._inspect_job: "inspect", self._calibrate: "calibrate", self._pointcloud: "pointcloud", self._panorama: "panorama"}.get(target)
            self.streaming = False
            self.live = None
            self.stats = None
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
            with self.lock:
                self.passwords.clear()
                self.distance_request = None
                self.streaming = False
                self.live = None

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
            self.passwords.update({ip: pw for ip, pw in passwords.items() if pw})
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
                started = time.monotonic()
                monitor_groups = 0
                recent = collections.deque(maxlen=5)
                settled = not ptp
                self.set_state(
                    message="Warming up: letting the exposure settle" + (f" and waiting for PTP to lock the frames (spread below {args.ptp_settle_ms:g} ms)" if ptp else "") + "...",
                    progress={"stage": "warmup", "done": 0, "total": args.sample_count, "percent": 0},
                )
                while pipeline.isRunning() and not self.stop_event.is_set():
                    for group in spread_queue.tryGetAll():
                        monitor_groups += 1
                        recent.append(group.getIntervalNs() / 1e6)
                        self.set_ptp(spreadMs=round(recent[-1], 2), spreadAt=time.monotonic(), groups=monitor_groups)
                    elapsed = time.monotonic() - started
                    warm = min(elapsed / args.warmup_seconds, 1.0) if args.warmup_seconds > 0 else 1.0
                    if warm < 1.0 or settled or not ptp:
                        self.set_state(progress={"stage": "warmup", "done": 0, "total": args.sample_count, "percent": int(100 * warm)})
                    else:
                        locked = 0
                        for spread in reversed(recent):
                            if spread >= args.ptp_settle_ms:
                                break
                            locked += 1
                        self.set_state(progress={"stage": "ptplock", "done": 0, "total": args.sample_count, "percent": int(100 * locked / recent.maxlen) if locked else None,
                                                 "detail": f"frame spread {recent[-1]:.1f} ms, needs {recent.maxlen} groups below {args.ptp_settle_ms:g} ms" if recent else "waiting for the first synchronized frames"})
                    if ptp and len(recent) == recent.maxlen and max(recent) < args.ptp_settle_ms:
                        settled = True
                    if elapsed >= args.warmup_seconds and settled:
                        break
                    if ptp and elapsed >= args.ptp_settle_timeout:
                        self.set_ptp(warning=f"PTP did not settle within {args.ptp_settle_timeout:.0f} s (frame spread {max(recent) if recent else float('nan'):.1f} ms, limit {args.ptp_settle_ms:g} ms). Calibrating anyway; check that the switch passes PTP traffic.")
                        self.log(f"PTP did not settle: recent spreads {[round(x, 1) for x in recent]} ms")
                        break
                    time.sleep(0.05)
                self.set_ptp(settled=settled if ptp else None)
                if ptp and settled:
                    self.log(f"PTP settled: frame spread {max(recent):.2f} ms after {time.monotonic() - started:.1f} s")
                if self.stop_event.is_set():
                    raise InterruptedError
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
    def panorama_async(self, device_names, projection):
        return self._start_job(self._panorama, device_names, projection)

    def _panorama(self, device_names, projection):
        args = self.args
        projection = projection if projection in PROJECTIONS else "cylindrical"
        self.set_state("panorama", "Checking the devices...", progress=None, stats=None)
        self.log(f"Panorama of {device_names} started ({projection}, {args.pano_size[0]}x{args.pano_size[1]} at {args.pano_fps:g} FPS)")
        device_ids, calibration = self._require_calibration(device_names)
        with self.lock:
            self.live = {"mode": "panorama", "projection": projection, "calibration": calibration["path"]}
            self.pano_jpeg = None
        ptp = self._prepare_ptp(device_names)
        if self.stop_event.is_set():
            self.set_state("idle", "Panorama stopped.", stats=None)
            return
        self.set_state("panorama", "Connecting to devices...")
        handler = dai.beta.MultiDeviceCalibrationHandler(Path(calibration["path"]))
        self.log(f"Using calibration {Path(calibration['path']).name}")
        with dai.Pipeline(createImplicitDevice=False) as pipeline:
            pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)
            devices = [pipeline.addDevice(dai.DeviceInfo(name)) for name in device_names]
            pipeline.setMultiDeviceCalibration(handler.getGraph())
            outputs = []
            for name, device in zip(device_names, devices):
                info = self.device_info[name]
                socket = self._panorama_socket(info, calibration)
                camera = pipeline.create(dai.node.Camera, device).build(socket, sensorFps=args.pano_fps)
                if ptp:
                    camera.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.TIME_PTP)
                outputs.append(camera.requestOutput(tuple(args.pano_size), dai.ImgFrame.Type.NV12, fps=args.pano_fps, enableUndistortion=True))
                self.log(f"{name}: {socket.name} undistorted {args.pano_size[0]}x{args.pano_size[1]}")
            stitching = pipeline.create(dai.beta.node.Stitching).build(outputs)
            stitching.setMode(dai.beta.node.Stitching.Mode.PANORAMA)
            stitching.setCameraModel(PROJECTIONS[projection])
            stitching.setUseInputCalibration(True)
            stitching.setSeamFinder(getattr(dai.beta.node.Stitching.SeamFinder, args.seam))
            stitching.setMaxPanoramaSize(8192, 4096)
            stitching.setSyncThreshold(timedelta(milliseconds=(900.0 if ptp else 1000.0) / args.pano_fps))
            stitching.sync.setTimestampSource(dai.node.Sync.TimestampSource.SYSTEM if ptp else dai.node.Sync.TimestampSource.HOST)
            stitching.sync.setSyncAttempts(-1)
            out_queue = stitching.out.createOutputQueue(maxSize=2, blocking=False)
            pipeline.start()
            start = time.monotonic()
            last_report = start
            count = 0
            window_count = 0
            self.set_state(message="Starting cameras...")
            try:
                while pipeline.isRunning() and not self.stop_event.is_set():
                    frame = out_queue.get(timedelta(milliseconds=200))
                    now = time.monotonic()
                    if frame is None:
                        if count == 0 and now - start > 20 and now - last_report > 5:
                            self.set_state(message="No panorama yet. Do the cameras stream?")
                            last_report = now
                        continue
                    count += 1
                    window_count += 1
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
                        with self.lock:
                            self.streaming = True
                        self.set_state(message="Streaming the panorama.")
                    if now - last_report >= 2.0:
                        self.set_state(stats={"rate": round(window_count / (now - last_report), 1), "frames": count, "width": frame.getWidth(), "height": frame.getHeight(), "projection": projection})
                        last_report = now
                        window_count = 0
            finally:
                pipeline.stop()
        with self.lock:
            self.streaming = False
        self.log(f"Panorama stopped after {count} frames")
        self.set_state("idle", "Panorama stopped.", stats=None)

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
        elif self.path == "/api/pointcloud":
            if len(devices) < 2:
                return self._json({"error": "Select at least two devices"}, 400)
            ok = self.app.pointcloud_async(devices)
        elif self.path == "/api/panorama":
            if len(devices) < 2:
                return self._json({"error": "Select at least two devices"}, 400)
            ok = self.app.panorama_async(devices, body.get("projection") or "cylindrical")
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
