"""Browser control tests without DepthAI binaries or connected devices."""

import importlib.util
import io
import json
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, call, patch

import numpy as np


class FilterConfig:
    OverlapMode = SimpleNamespace(OFF="off", NMS="nms", AVERAGE="average")

    def setConfidenceRange(self, low=0, high=1):
        self.minConfidence, self.maxConfidence = np.float32(low), np.float32(high)

    def setSizeRange(self, low=0, high=np.finfo(np.float32).max):
        self.minArea, self.maxArea = np.float32(low), np.float32(high)

    def validate(self):
        return self.minConfidence < self.maxConfidence and self.minArea < self.maxArea


sdk = SimpleNamespace(
    CameraModel=SimpleNamespace(Cylindrical=1, Equirectangular=2, Perspective=3),
    node=SimpleNamespace(Stitching=SimpleNamespace(outSourceMasks=None), ImgDetectionsFilter=object()),
    ImgDetectionsFilterConfig=FilterConfig,
)
spec = importlib.util.spec_from_file_location("multi_device_app_test", Path(__file__).resolve().parents[1] / "app.py")
app = importlib.util.module_from_spec(spec)
with patch.dict("sys.modules", depthai=sdk):
    spec.loader.exec_module(app)


class DetectionControlsTest(unittest.TestCase):
    def setUp(self):
        self.app = object.__new__(app.App)
        self.app.lock = threading.Lock()
        self.app.live = None
        self.app.pending_detection_config = None
        self.app._start_job = Mock(return_value=True)

    def post(self, path, body):
        handler = object.__new__(app.Handler)
        handler.app = self.app
        handler.path = path
        payload = json.dumps(body).encode()
        handler.headers = {"Content-Length": str(len(payload))}
        handler.rfile = io.BytesIO(payload)
        handler._json = Mock()
        handler.do_POST()
        return handler._json.call_args.args

    def test_complete_runtime_config_preserves_controls_and_zero_limit(self):
        filters = {"labels": [0, 2], "rejectLabels": [2], "confidence": 0.7,
                   "minArea": 500, "maxDetections": 0, "overlap": "average", "iou": 0.3}
        self.app.live = {"mode": "detections"}
        self.assertEqual(self.post("/api/detections/filters", {"filters": filters}), ({"ok": True},))
        config = self.app.pending_detection_config
        self.assertEqual(config.labelsToKeep, [0, 2])
        self.assertEqual(config.labelsToReject, [2])
        self.assertAlmostEqual(config.minConfidence, 0.7)
        self.assertEqual(config.minArea, 500)
        self.assertEqual(config.maxDetections, 0)
        self.assertEqual(config.overlapMode, "average")
        self.assertEqual(config.overlapIouThreshold, 0.3)
        self.assertTrue(config.sortByConfidence)
        self.app.update_detection_filters({**filters, "overlap": "off"})
        self.assertEqual(self.app.pending_detection_config.labelsToKeep, [0, 2])
        self.assertEqual(self.app.pending_detection_config.overlapMode, "off")

    def test_invalid_filters_never_start_a_job(self):
        invalid = [None, [], {"confidence": 1}, {"confidence": 0.05}, {"confidence": True},
                   {"confidence": "0.5"}, {"iou": -0.1}, {"iou": float("nan")},
                   {"minArea": -1}, {"minArea": float("inf")}, {"minArea": 10**400},
                   {"minArea": float(np.finfo(np.float32).max)}, {"labels": "0"},
                   {"labels": [-1]}, {"labels": [True]}, {"rejectLabels": [2**32]},
                   {"maxDetections": -1}, {"maxDetections": 1.5}, {"overlap": "bad"}]
        for filters in invalid:
            with self.subTest(filters=filters):
                response = self.post("/api/detections", {"devices": ["a", "b"], "filters": filters})
                self.assertEqual(response[1], 400)
        self.app._start_job.assert_not_called()

    def test_start_uses_defaults_and_detection_job_name(self):
        self.assertEqual(self.post("/api/detections", {"devices": ["a", "b"]}), ({"ok": True},))
        self.app._start_job.assert_called_once_with(
            self.app._panorama, ["a", "b"], "cylindrical", app.DETECTION_DEFAULTS, name="detections")

    def test_unsupported_wheel_keeps_plain_panorama_available(self):
        with patch.object(app, "DETECTIONS_AVAILABLE", False):
            response = self.post("/api/detections", {"devices": ["a", "b"]})
            self.assertEqual(response[1], 400)
            self.assertIn("updated DepthAI wheel", response[0]["error"])
            self.app._start_job.assert_not_called()
            self.assertEqual(self.post("/api/panorama", {"devices": ["a", "b"]}), ({"ok": True},))
        self.app._start_job.assert_called_once_with(
            self.app._panorama, ["a", "b"], "cylindrical", None, name="panorama")

    def test_device_selection_and_busy_job_errors(self):
        for devices in ([], ["a"], ["a", "a"]):
            with self.subTest(devices=devices):
                self.assertEqual(self.post("/api/detections", {"devices": devices})[1], 400)
        self.app._start_job.assert_not_called()
        self.app._start_job.return_value = False
        self.assertEqual(self.post("/api/detections", {"devices": ["a", "b"]})[1], 409)

    def test_updates_require_detection_mode_and_leave_pending_config_on_error(self):
        for live in (None, {"mode": "pointcloud"}, {"mode": "panorama"}):
            self.app.live = live
            self.assertEqual(self.post("/api/detections/filters", {"filters": {}})[1], 400)
        self.app.live = {"mode": "detections"}
        self.app.update_detection_filters({})
        pending = self.app.pending_detection_config
        self.assertEqual(self.post("/api/detections/filters", {"filters": {"iou": 2}})[1], 400)
        self.assertIs(self.app.pending_detection_config, pending)

    def test_source_overlays_are_filtered_and_all_views_receive_updates(self):
        sdk = MagicMock(ImgDetectionsFilterConfig=FilterConfig)
        pipeline = sdk.Pipeline.return_value.__enter__.return_value
        pipeline.isRunning.side_effect = [True, False]
        frame = MagicMock()
        frame.getWidth.return_value, frame.getHeight.return_value = 1280, 400
        frame.getTransformation.return_value.getIntrinsicMatrix.return_value = [[400]]
        filters = []

        def create(kind, *args):
            node = MagicMock()
            node.build.return_value = node
            if kind is sdk.node.ImgDetectionsFilter:
                node.initialConfig = FilterConfig()
                node.out.createOutputQueue.return_value.tryGetAll.return_value = [SimpleNamespace(detections=[])]
                filters.append(node)
            elif kind is sdk.node.Stitching:
                node.out.createOutputQueue.return_value.get.return_value = frame
            return node

        pipeline.create.side_effect = create
        vars(self.app).update(
            args=SimpleNamespace(pano_size=(640, 400), pano_fps=10, seam="GRAPHCUT_COLOR", detections_model="model"),
            stop_event=threading.Event(), device_info={"a": {}, "b": {}},
            remote=Mock(), detection_topics=[], log=Mock(), set_state=Mock(), set_ptp=Mock(),
            _require_calibration=Mock(return_value=(["a", "b"], {"path": "/tmp/calibration.json"})),
            _prepare_ptp=Mock(return_value=False), _panorama_socket=Mock(return_value=SimpleNamespace(name="CAM_A")),
        )
        initial = {"labels": [0], "confidence": 0.6, "minArea": 100, "maxDetections": 5}
        update = {"labels": [2], "rejectLabels": [3], "confidence": 0.8, "minArea": 400, "maxDetections": 0}
        _, pending = app.detection_config(update)
        self.app.pending_detection_config = pending
        with patch.object(app, "dai", sdk), patch.object(app, "STITCHING_NODE", sdk.node.Stitching):
            self.app._panorama(["a", "b"], "cylindrical", initial)

        self.assertEqual(len(filters), 3)  # panorama plus both source views
        source_outputs = [c.args[1] for c in self.app.remote.addTopic.call_args_list if c.args[0].startswith("Source detections")]
        self.assertEqual(source_outputs, [node.out for node in filters[1:]])
        for node in filters:
            self.assertEqual(node.initialConfig.labelsToKeep, [0])
            self.assertAlmostEqual(node.initialConfig.minConfidence, 0.6)
            self.assertEqual(node.initialConfig.minArea, 100)
            self.assertEqual(node.initialConfig.maxDetections, 5)
            queue = node.inputConfig.createInputQueue.return_value
            queue.send.assert_called_once_with(pending)
            config = queue.send.call_args.args[0]
            self.assertEqual(config.labelsToKeep, [2])
            self.assertEqual(config.labelsToReject, [3])
            self.assertAlmostEqual(config.minConfidence, 0.8)
            self.assertEqual(config.minArea, 400)
            self.assertEqual(config.maxDetections, 0)
        self.assertIsNone(self.app.pending_detection_config)
        pipeline.stop.assert_called_once()

    def test_topic_cleanup_after_startup_failure_and_restart(self):
        self.app.remote = Mock()
        self.app.detection_topics = ["Panorama", "Filtered detections"]
        self.app.passwords = {"a": "secret"}
        self.app.log = Mock()
        self.app.set_state = Mock()
        self.app.live = {"mode": "detections"}
        self.app.pending_detection_config = FilterConfig()
        with patch.object(app.traceback, "print_exc"):
            self.app._run_job(Mock(side_effect=RuntimeError("Model unavailable")))
        self.app.remote.removeTopic.assert_has_calls([call("Panorama"), call("Filtered detections")])
        self.assertEqual(self.app.detection_topics, [])
        self.assertEqual(self.app.passwords, {})
        self.assertIsNone(self.app.live)
        self.assertIsNone(self.app.pending_detection_config)
        self.assertFalse(self.app.streaming)
        self.app.remote.removeTopic.reset_mock()
        self.app._run_job(Mock())
        self.app.remote.removeTopic.assert_not_called()

    def test_ptp_password_submission_preserves_empty_password(self):
        self.app.passwords = {"a": "old password"}
        self.app.credentials_event = threading.Event()
        self.assertEqual(self.post("/api/ptp/passwords", {"passwords": {"a": "", "b": "secret"}}), ({"ok": True},))
        self.assertEqual(self.app.passwords, {"a": "", "b": "secret"})
        self.assertTrue(self.app.credentials_event.is_set())


class SshPasswordsTest(unittest.TestCase):
    def setUp(self):
        self.client = Mock()
        stdout, stderr = Mock(), Mock()
        stdout.read.return_value = b"PTP_ENABLE=true\n"
        stdout.channel.recv_exit_status.return_value = 0
        stderr.read.return_value = b""
        self.client.exec_command.return_value = (None, stdout, stderr)
        self.ssh_client = patch("paramiko.SSHClient", return_value=self.client)
        self.ssh_client.start()
        self.addCleanup(self.ssh_client.stop)

    def test_empty_password_and_key_login_remain_distinct(self):
        for password in (None, "", "secret"):
            with self.subTest(password=password):
                self.assertEqual(app.DeviceSsh._run_paramiko("device", password, "check", 25), ("ok", "PTP_ENABLE=true\n"))
                args = self.client.connect.call_args.kwargs
                self.assertEqual(args["password"], password)
                self.assertEqual(args["allow_agent"], password is None)
                self.assertEqual(args["look_for_keys"], password is None)

    def test_passwordless_server_accepting_none_authentication(self):
        import paramiko

        self.client.connect.side_effect = paramiko.BadAuthenticationType("No password authentication", ["none"])
        self.assertEqual(app.DeviceSsh._run_paramiko("device", "", "check", 25), ("ok", "PTP_ENABLE=true\n"))
        self.client.get_transport().auth_none.assert_called_once_with("root")
        self.client.close.assert_called_once()

    def test_rejected_empty_password_closes_connection(self):
        import paramiko

        self.client.connect.side_effect = paramiko.AuthenticationException("Wrong password")
        self.client.get_transport().auth_none.side_effect = paramiko.AuthenticationException("Password required")
        self.assertEqual(app.DeviceSsh._run_paramiko("device", "", "check", 25)[0], "needs_password")
        self.client.exec_command.assert_not_called()
        self.client.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
