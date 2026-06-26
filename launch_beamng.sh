#!/usr/bin/env bash
# Launches BeamNG directly with the tech server flags.
# -nosteam bypasses Steam auth so no Steam session is needed.
# -tcom -tport 64256 start the tech server that beamngpy connects to.

BEAMNG_BIN="/home/alex/.local/share/Steam/steamapps/common/BeamNG.drive/BinLinux/BeamNG.drive.x64"

echo "[BeamNG] Launching: $BEAMNG_BIN -nosteam -tcom -tport 64256"
exec "$BEAMNG_BIN" -nosteam -tcom -tport 64256
