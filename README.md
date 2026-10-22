# Multi-device app

Calibrate several OAK 4 devices against each other, then show a merged point cloud, a calibrated
panorama, or merged detections across their views. Browser UI, one Python process.

## Run

Ubuntu 22.04 or 24.04, the devices on the same network:

```bash
./install.sh     # once: creates .venv and installs depthai, numpy, opencv, paramiko
./run.sh         # then open http://localhost:8093
```

Detections requires the updated `ImgDetectionsFilter` and Stitching source masks. Installation uses
the snapshot pinned in [requirements-depthai.txt](requirements-depthai.txt), matching the oak-examples
image-filter-detections tutorial. To use your own updated build: `./install.sh /path/to/depthai.whl`.

1. Tick two or more devices.
2. Point them at the same static, textured scene and press **Calibrate**.
3. Press **Point cloud** (devices with a stereo pair), **Panorama**, or **Detections** (stereo or single-camera devices).
4. **Extras → Capture frames** saves the calibration material to `captures/<timestamp>/`: the same number of
   synchronized full-resolution image groups the calibration collects, the timestamps in `capture.json` and the factory
   calibration of every device, so the setup can be calibrated offline. With PTP it first waits for the frame lock.
4. In **Detections**, view the panorama with merged detections in the embedded visualizer.
   Expand **Detection filters** to change confidence, labels, box area, detection count, or duplicate
   handling (**Off**, **NMS**, **Average**), then press **Apply filters** while running. Filters also
   apply to the source camera overlays; duplicate merging happens in the panorama.

Before each step the app checks PTP time sync on the devices over ssh and may ask for their root password
(OAK 4 default `oelinux123`; leave blank for passwordless devices). Details and options:
[USAGE.md](USAGE.md), `./run.sh --help`.

## Hand it to someone

`./make_package.sh` builds `multi_device_app-<version>-ubuntu.zip` one folder up, with the DepthAI wheel
included so the recipient only needs Python 3 and `./install.sh`.
For a custom build, put only compatible updated DepthAI wheels in `wheels/` before packaging; otherwise
the pinned snapshot is downloaded for Python 3.10 and 3.12. The first detection run also needs internet
to download the model.
