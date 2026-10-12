# Multi-device app

Calibrate several OAK 4 devices against each other, then show a merged point cloud or a calibrated
panorama of all of them. Browser UI, one Python process, DepthAI 3.11.0.

## Run

Ubuntu 22.04 or 24.04, the devices on the same network:

```bash
./install.sh     # once: creates .venv and installs depthai, numpy, opencv, paramiko
./run.sh         # then open http://localhost:8093
```

1. Tick two or more devices.
2. Point them at the same static, textured scene and press **Calibrate**.
3. Press **Point cloud** (devices with a stereo pair) or **Panorama** (projection selectable next to it).

Before each step the app checks PTP time sync on the devices over ssh and may ask for their root password
(OAK 4 default `oelinux123`). Details and options: [USAGE.md](USAGE.md), `./run.sh --help`.

## Hand it to someone

`./make_package.sh` builds `multi_device_app-<version>-ubuntu.zip` one folder up, with the DepthAI wheel
included so the recipient only needs Python 3 and `./install.sh`.
