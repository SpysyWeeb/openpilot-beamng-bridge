"""
Runtime shims applied to stock openpilot sim helpers.

Per project policy the openpilot checkout is never modified — anything the
stock sim gets wrong for our purposes is corrected here at import time.

Current shims
-------------
patch_camerad_timestamps()
    Stock tools/sim/lib/camerad.py stamps vipc frames with a synthetic clock
    (frame_id * 50 ms, i.e. seconds since the first frame), while the sim's
    IMU messages carry real CLOCK_MONOTONIC time.  locationd's Kalman filter
    follows the IMU clock and computes the vision-odometry timestamp as
    cameraOdometry.timestampEof - CAM_ODO_POSE_DELAY, so every observation
    lands ~hours in the past and is dropped with "Observation cameraOdometry
    ignored due to failed timing check".  livePose then runs IMU-only.
    Stamping frames with real monotonic time puts both sensors in the same
    clock domain.  (Upstream PR candidate — the stock MetaDrive bridge has
    the same defect.)
"""
import time

import openpilot.cereal.messaging as messaging
from openpilot.tools.sim.lib import camerad as _camerad


def patch_camerad_timestamps() -> None:
    # Mirrors Camerad._send_yuv() with eof = frame_id * 0.05 * 1e9 replaced
    # by the real clock.  Method-level patch so it applies regardless of how
    # callers imported the Camerad class.
    def _send_yuv(self, yuv, frame_id, pub_type, yuv_type):
        eof = time.monotonic_ns()
        self.vipc_server.send(yuv_type, yuv, frame_id, eof, eof)

        dat = messaging.new_message(pub_type, valid=True)
        msg = {
            "frameId": frame_id,
            "transform": [1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0]
        }
        setattr(dat, pub_type, msg)
        self.pm.send(pub_type, dat)

    _camerad.Camerad._send_yuv = _send_yuv


def apply() -> None:
    """Apply all shims. Call once, before SimulatedSensors is constructed."""
    patch_camerad_timestamps()
    print('[op_shims] camerad timestamps patched to real monotonic time', flush=True)
