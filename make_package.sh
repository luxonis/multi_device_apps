#!/usr/bin/env bash
# Assembles the distributable zip: app files, visualizer bundle and the DepthAI wheels from wheels/.
#   ./make_package.sh [output directory]
# Produces multi_device_app-<version>-ubuntu.zip with the manylinux wheels for Python 3.10 (Ubuntu 22.04)
# and 3.12 (Ubuntu 24.04); install.sh picks the one matching the system Python.
set -euo pipefail
cd "$(dirname "$0")"
OUTDIR=$(realpath -m "${1:-..}")
VERSION=$(sed -n 's/^APP_VERSION = "\(.*\)"/\1/p' app.py)
[ -n "$VERSION" ] || { echo "APP_VERSION not found in app.py"; exit 1; }
NAME="multi_device_app-$VERSION-ubuntu"
OUT="$OUTDIR/$NAME.zip"
STAGE=$(mktemp -d)
mkdir -p "$STAGE/$NAME/calibrations" "$STAGE/$NAME/captures" "$STAGE/$NAME/wheels"
cp app.py index.html install.sh run.sh README.md USAGE.md requirements-depthai.txt "$STAGE/$NAME/"
cp -r visualizer "$STAGE/$NAME/"
count=0
for whl in wheels/depthai-*manylinux*.whl; do [ -e "$whl" ] && { cp "$whl" "$STAGE/$NAME/wheels/"; count=$((count + 1)); }; done
if [ "$count" -eq 0 ]; then
  echo "Downloading the DepthAI snapshot into wheels/"
  for pyver in 3.10 3.12; do
    python3 -m pip download -r requirements-depthai.txt --no-deps --only-binary=:all: --python-version "$pyver" --implementation cp --abi "cp${pyver/./}" --abi abi3 --platform manylinux_2_28_x86_64 --platform manylinux2014_x86_64 -d wheels -q
  done
  cp wheels/depthai-*manylinux*.whl "$STAGE/$NAME/wheels/"
fi
rm -f "$OUT"
(cd "$STAGE" && zip -qr "$OUT" "$NAME")
rm -rf "$STAGE"
ls -la "$OUT"
