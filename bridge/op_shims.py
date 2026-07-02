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


def patch_camerad_colors() -> None:
    """Stock tools/sim rgb_to_nv12 halves chroma amplitude: luma uses
    half-scale coefficients with >>7, but chroma uses half-scale coefficients
    with >>8 — 50% desaturation. The model is trained on full-color road
    footage and the UI feed looks visibly gray. Replace with full-range
    BT.601 (matches real-device ISP output). Upstream PR candidate."""
    import numpy as np

    def rgb_to_nv12_fullrange(rgb):
        h, w = rgb.shape[:2]
        r = rgb[:, :, 0].astype(np.int32)
        g = rgb[:, :, 1].astype(np.int32)
        b = rgb[:, :, 2].astype(np.int32)

        y = (77 * r + 150 * g + 29 * b + 128) >> 8          # 0.299/0.587/0.114
        y = np.clip(y, 0, 255).astype(np.uint8)

        r_s = (r[0::2, 0::2] + r[0::2, 1::2] + r[1::2, 0::2] + r[1::2, 1::2] + 2) >> 2
        g_s = (g[0::2, 0::2] + g[0::2, 1::2] + g[1::2, 0::2] + g[1::2, 1::2] + 2) >> 2
        b_s = (b[0::2, 0::2] + b[0::2, 1::2] + b[1::2, 0::2] + b[1::2, 1::2] + 2) >> 2
        y_s = (77 * r_s + 150 * g_s + 29 * b_s + 128) >> 8

        u = np.clip(((b_s - y_s) * 144 >> 8) + 128, 0, 255).astype(np.uint8)  # 0.564
        v = np.clip(((r_s - y_s) * 183 >> 8) + 128, 0, 255).astype(np.uint8)  # 0.713

        uv = np.empty((h // 2, w), dtype=np.uint8)
        uv[:, 0::2] = u
        uv[:, 1::2] = v
        return np.concatenate([y.ravel(), uv.ravel()]).tobytes()

    _camerad.rgb_to_nv12 = rgb_to_nv12_fullrange


def apply() -> None:
    """Apply all shims. Call once, before SimulatedSensors is constructed."""
    patch_camerad_timestamps()
    patch_camerad_colors()
    print('[op_shims] camerad patched: real monotonic timestamps + full-range color', flush=True)
