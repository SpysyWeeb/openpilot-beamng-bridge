#!/usr/bin/env bash
# Launches BeamNG directly with the tech server flags.
# -nosteam bypasses Steam auth so no Steam session is needed.
# -tcom -tport 64256 start the tech server that beamngpy connects to.
#
# nice +10: modeld runs a CPU-compiled model and must hold 20 Hz; BeamNG
# saturating all cores starved it to ~15 Hz (dips to 6), flagging
# modelV2/cameraOdometry invalid and cascading into commIssue soft-disables.
# Deprioritizing the game lets modeld win the contested cycles.

BEAMNG_BIN="/home/alex/.local/share/Steam/steamapps/common/BeamNG.drive/BinLinux/BeamNG.drive.x64"

echo "[BeamNG] Launching (nice +10): $BEAMNG_BIN -nosteam -tcom -tport 64256"
exec nice -n 10 "$BEAMNG_BIN" -nosteam -tcom -tport 64256
