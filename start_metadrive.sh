#!/usr/bin/env bash
# MetaDrive ↔ openpilot Bridge — Control Panel launcher.
# Run from the HOST (Bazzite), not inside the distrobox.
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/tools/bridge_gui.py" --metadrive "$@"
