# Multi-device app (version 0.2)

Pick two or more OAK 4 devices on your network, calibrate them against each other, then see one merged
point cloud of all of them, a calibrated panorama, or merged image detections, or capture synchronized frames. Everything is in this folder:
the app, the DepthAI visualizer and (in the zip) the updated DepthAI wheel. No other DepthAI installation is needed or
touched.

## Install (once)

Ubuntu 22.04 (Python 3.10) or 24.04 (Python 3.12), x86_64, from the zip or the git checkout:

```bash
./install.sh
```

The installer creates a virtual environment in `.venv` and installs the bundled DepthAI wheel, or the
development snapshot in `requirements-depthai.txt`, plus numpy, OpenCV and paramiko from PyPI (internet needed). If `python3-venv` is
missing it installs it with sudo.

The snapshot matches the oak-examples `image-filter-detections` tutorial and provides
`dai.node.ImgDetectionsFilter` and `dai.node.Stitching.outSourceMasks`. The original released 3.11.0 wheel
does not provide this updated workflow. The installer checks these APIs. To install your own updated wheel:

```bash
./install.sh /absolute/path/to/depthai.whl
```

For development, `DEPTHAI_CORE_BINDINGS=/path/to/built/bindings/python ./run.sh` can select existing local
bindings instead. The app logs the imported DepthAI version at startup. With the original 3.11.0 wheel,
the existing modes remain available and the Detections button is disabled.

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
6. **Detections**: runs the same detector on one calibrated camera per device, then maps their boxes into
   the panorama with the native host `ImgDetectionsFilter`. Works with stereo and single-camera devices;
   no depth is needed. The embedded visualizer shows source camera groups plus a **Detections** group.
   Select its **Panorama** and **Filtered detections** topics to see the merged overlay. The live
   statistics show the number of filtered detections.

### Detection filters

Expand **Detection filters** before starting or press **Apply filters** during a detection run:
Label, confidence, area, and count filters apply to the panorama and every source camera overlay.

- **Overlap**: **Off** preserves duplicates, **NMS** keeps one overlapping box per class, and **Average**
  merges overlapping boxes of the same class across cameras. **Overlap IoU** controls the matching threshold.
- **Confidence**: 0.1 to 0.95. Each detector has a confidence floor of 0.1.
- **Keep / Reject label IDs**: comma-separated class IDs; blank means all / none. For the default COCO model,
  person is `0`. Rejection takes precedence over keeping.
- **Minimum area**: box area in each view's pixels squared. Source images and the panorama can have
  different resolutions and projections, so the same box can pass in one view and fail in another.
- **Maximum detections per view**: blank means unlimited; `0` keeps none. Results are sorted by confidence.
  The limit applies independently to each source camera and to the merged panorama.

The panorama uses Stitching source visibility masks, so boxes fully hidden by a seam are removed
even with overlap **Off**. Runtime changes send a complete configuration, preserving the other controls.
One **Apply filters** updates every view. Cross-camera overlap merging applies to the panorama; source
overlays retain their own camera coordinates and show the filtered detections visible to that camera.
All cameras use `--detections-model` (default `luxonis/yolov6-nano:r2-coco-512x288`), resolved for each
device platform. The first run needs internet for the model download. Detection mode uses `--pano-fps`,
`--pano-size`, `--seam`, and the projection selector; its panorama is capped at 2560×1440.

Remapping uses camera rotation and intrinsics without depth or camera translation. Nearby objects can
retain parallax errors; evaluate merging with more distant objects and overlapping views. Boxes crossing
a full 360-degree panorama seam are unsupported. This demonstrates image detection merging, without
cross-camera tracking IDs or depth-aware spatial fusion.

### Capture

**Capture** (one or more devices, no calibration needed) saves one synchronized full-resolution frame of every selected
camera: the stereo pair or the single camera chosen under each device, as for the calibration. It streams the cameras,
lets the exposure settle and, with PTP, waits for the frame lock exactly like the calibration (`--warmup-seconds`,
`--ptp-settle-ms`, `--ptp-settle-timeout`), then takes the next synchronized group. The frames are saved as PNG in
`captures/<date>-<time>/` together with `capture.json` (device, socket, size, sequence number, the three timestamps of
every frame, its offset to the earliest frame of the group in ms, the group spread and the PTP state). The page shows the
result with the offsets; click a frame for the full-resolution PNG.

Full resolution cannot stream continuously from several cameras over a 100 Mbit link, so every camera keeps its newest
frames on the device and only the chosen frame crosses the network. Only small per-frame timestamp messages and 2 FPS
previews stream while waiting. With a single device no PTP is involved.

**Stop** ends the running mode. Recalibrate whenever a device is moved.

### PTP time sync

Before each job the app checks that PTP is enabled on the selected devices (one master, the others slaves,
frame sync on) using `luxonis-ptp-config` over ssh as root. If a device does not accept your ssh key, a dialog
asks for its root password (OAK 4 default: `oelinux123`; passwords stay in memory). Leave the field blank
and press **Set up PTP** for a device without a password. The readout at the bottom
left shows the live timestamp spread between the devices. **Continue without PTP** skips it; `./run.sh --no-ptp`
disables the check. PTP needs a switch that passes PTP traffic.

## Notes

- Ports 8093 (app), 8082 (visualizer page) and 8765 (visualizer WebSocket) must be free.
- Many devices on one network link: `./run.sh --pc-fps 5` or `./run.sh --size 640 400`. All options:
  `./run.sh --help`.
- If the calibration fails, check that every camera sees the same part of a textured, lit scene; the Log
  section and the terminal output show the details.
- Stop the app with Ctrl+C in the terminal.
