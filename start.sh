#!/usr/bin/env bash
# BeamNG ↔ openpilot Bridge — Control Panel launcher.
# Run from the HOST (Bazzite), not inside the distrobox.
#
# Usage:
#   ./start.sh              # single camera
#   ./start.sh --dual-camera
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/tools/bridge_gui.py" "$@"
