# Multi-device app (version 0.1)

Pick two or more OAK 4 devices on your network, calibrate them against each other, then see one merged
point cloud of all of them or a calibrated panorama. Everything is in this folder: the app, the DepthAI
visualizer and (in the zip) the official DepthAI 3.11.0 wheel. No other DepthAI installation is needed or
touched.

## Install (once)

Ubuntu 22.04 (Python 3.10) or 24.04 (Python 3.12), x86_64, from the zip or the git checkout:

```bash
./install.sh
```

The installer creates a virtual environment in `.venv` and installs DepthAI 3.11.0 (the bundled wheel, or
the same version from PyPI) plus numpy, OpenCV and paramiko from PyPI (internet needed). If `python3-venv` is
missing it installs it with sudo.

## Run

```bash
./run.sh
```

Open **http://localhost:8093** in a browser.

1. **Devices**: every reachable device is listed; free OAK 4 devices can be ticked. Refresh rescans.
2. **Check devices** (optional) reads the cameras of the ticked devices. Under each device choose what the
   calibration uses: its stereo pair (default, fixes the metric scale) or one camera.
3. **Calibrate**: point all cameras at the same static, well textured scene with plenty of overlap and keep
   the devices still. If no device uses a stereo pair you are asked for the tape-measured distance between
   two camera lenses (tick *Enter measured distances* to add such measurements in any case). The live
   streams are shown while it runs. The result appears as a 3D view of the device coordinate systems with
   the metrics (confidence 0 … 1, Sampson error, poses, camera distances, change against the previous
   calibration). Calibrations are kept in `calibrations/`; click one in the list to show it.
4. **Point cloud** (devices with stereo pairs): NeuralDepth on every device, merged into one cloud in the
   calibration origin and shown in the DepthAI visualizer.
5. **Panorama**: one camera per device stitched from the calibration onto a cylinder (default), a sphere or a
   plane, chosen next to the button.

**Stop** ends the running mode. Recalibrate whenever a device is moved.

### PTP time sync

Before each job the app checks that PTP is enabled on the selected devices (one master, the others slaves,
frame sync on) using `luxonis-ptp-config` over ssh as root. If a device does not accept your ssh key, a dialog
asks for its root password (OAK 4 default: `oelinux123`; passwords stay in memory). The readout at the bottom
left shows the live timestamp spread between the devices. **Continue without PTP** skips it; `./run.sh --no-ptp`
disables the check. PTP needs a switch that passes PTP traffic.

## Notes

- Ports 8093 (app), 8082 (visualizer page) and 8765 (visualizer WebSocket) must be free.
- Many devices on one network link: `./run.sh --pc-fps 5` or `./run.sh --size 640 400`. All options:
  `./run.sh --help`.
- If the calibration fails, check that every camera sees the same part of a textured, lit scene; the Log
  section and the terminal output show the details.
- Stop the app with Ctrl+C in the terminal.
