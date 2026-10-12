#!/usr/bin/env bash
# Starts the multi-device app (install.sh first). Extra arguments go to app.py, e.g. ./run.sh --pc-fps 5
cd "$(dirname "$0")"
[ -x .venv/bin/python ] || { echo "Run ./install.sh first"; exit 1; }
exec .venv/bin/python app.py "$@"
