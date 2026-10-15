#!/usr/bin/env bash
# Installs the multi-device app into a local virtual environment (.venv next to this script).
# Ubuntu 22.04 (Python 3.10) and 24.04 (Python 3.12), x86_64. Needs internet for the PyPI packages.
set -euo pipefail
cd "$(dirname "$0")"

PY=${PYTHON:-python3}
if ! command -v "$PY" >/dev/null; then echo "python3 not found. Install it with: sudo apt install python3 python3-venv"; exit 1; fi
# Supply a custom wheel as the first argument, or use the wheel bundled in wheels/.
# Otherwise install the snapshot pinned by the ImgDetectionsFilter example.
WHEEL=${1:-}

if ! "$PY" -c 'import venv, ensurepip' 2>/dev/null; then
  echo "The python3-venv package is missing; installing it (sudo)."
  sudo apt-get update && sudo apt-get install -y python3-venv
fi
[ -d .venv ] || "$PY" -m venv .venv
.venv/bin/python -m pip install --upgrade pip >/dev/null
if [ -n "$WHEEL" ]; then
  echo "Installing $WHEEL"
  .venv/bin/python -m pip install --force-reinstall --no-deps "$WHEEL"
elif compgen -G 'wheels/depthai-*manylinux*.whl' >/dev/null; then
  echo "Installing the compatible DepthAI wheel from wheels/"
  .venv/bin/python -m pip install --force-reinstall --no-deps --no-index --find-links wheels depthai
else
  echo "Installing the DepthAI snapshot from requirements-depthai.txt"
  .venv/bin/python -m pip install --no-deps -r requirements-depthai.txt
fi
.venv/bin/python -m pip install numpy opencv-python-headless paramiko

# USB devices need udev rules; network (PoE) devices do not.
if [ ! -f /etc/udev/rules.d/80-movidius.rules ] && [ -t 0 ]; then
  echo
  echo "Optional: for USB-connected OAK devices run"
  echo '  echo '"'"'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"'"'"' | sudo tee /etc/udev/rules.d/80-movidius.rules && sudo udevadm control --reload-rules && sudo udevadm trigger'
fi

.venv/bin/python - <<'PY'
import depthai as dai, cv2, numpy, paramiko
print("depthai", dai.__version__, "| opencv", cv2.__version__, "| numpy", numpy.__version__, "| paramiko", paramiko.__version__)
if not (hasattr(dai.node, "ImgDetectionsFilter") and hasattr(dai, "ImgDetectionsFilterConfig")
        and hasattr(getattr(dai.node, "Stitching", None), "outSourceMasks")):
    raise SystemExit("This wheel lacks ImgDetectionsFilter or Stitching source masks. Run ./install.sh /path/to/updated-depthai.whl")
PY
echo
echo "Installed. Start the app with:  ./run.sh   and open http://localhost:8093"
