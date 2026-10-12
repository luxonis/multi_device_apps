#!/usr/bin/env bash
# Installs the multi-device app into a local virtual environment (.venv next to this script).
# Ubuntu 22.04 (Python 3.10) and 24.04 (Python 3.12), x86_64. Needs internet for the PyPI packages.
set -euo pipefail
cd "$(dirname "$0")"

PY=${PYTHON:-python3}
if ! command -v "$PY" >/dev/null; then echo "python3 not found. Install it with: sudo apt install python3 python3-venv"; exit 1; fi
# The zip ships the official depthai 3.11.0 wheel in wheels/ (one file for Python 3.9 to 3.14); from a git
# checkout the folder is empty and the same version is installed from PyPI.
WHEEL=$(ls wheels/depthai-*manylinux*.whl 2>/dev/null | head -1 || true)

if ! "$PY" -c 'import venv, ensurepip' 2>/dev/null; then
  echo "The python3-venv package is missing; installing it (sudo)."
  sudo apt-get update && sudo apt-get install -y python3-venv
fi
[ -d .venv ] || "$PY" -m venv .venv
.venv/bin/python -m pip install --upgrade pip >/dev/null
if [ -n "$WHEEL" ]; then
  echo "Installing $WHEEL"
  .venv/bin/python -m pip install --force-reinstall --no-deps "$WHEEL"
else
  echo "Installing depthai==3.11.0 from PyPI"
  .venv/bin/python -m pip install --no-deps "depthai==3.11.0"
fi
.venv/bin/python -m pip install numpy opencv-python-headless paramiko

# USB devices need udev rules; network (PoE) devices do not.
if [ ! -f /etc/udev/rules.d/80-movidius.rules ] && [ -t 0 ]; then
  echo
  echo "Optional: for USB-connected OAK devices run"
  echo '  echo '"'"'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"'"'"' | sudo tee /etc/udev/rules.d/80-movidius.rules && sudo udevadm control --reload-rules && sudo udevadm trigger'
fi

.venv/bin/python -c 'import depthai as dai, cv2, numpy, paramiko; print("depthai", dai.__version__, "| opencv", cv2.__version__, "| numpy", numpy.__version__, "| paramiko", paramiko.__version__)'
echo
echo "Installed. Start the app with:  ./run.sh   and open http://localhost:8093"
