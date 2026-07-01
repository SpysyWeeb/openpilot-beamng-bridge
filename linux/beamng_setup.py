#!/usr/bin/env python3
"""
BeamNG scenario launcher — Linux side.

Mirrors windows/beamng_setup.py exactly, except connection uses launch=False
because BeamNG must run on the Bazzite host (distrobox lacks libnspr4.so).
start.sh launches BeamNG with -nosteam -tcom -tport 64256 before this runs.

Public API
----------
setup_beamng() -> (bng, vehicle, imu, camera, camera_wide)
"""

import logging
import os
import sys
import time

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import AdvancedIMU, Camera, Electrics

logging.basicConfig(level=logging.WARNING)
for _name in list(logging.Logger.manager.loggerDict.keys()):
    if _name.startswith("beamngpy"):
        _lg = logging.getLogger(_name)
        _lg.setLevel(logging.WARNING)
        _lg.handlers.clear()

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

BEAMNG_HOME = os.environ.get(
    "BEAMNG_HOME",
    "/home/alex/.local/share/Steam/steamapps/common/BeamNG.drive",
)
BEAMNG_USER = os.environ.get(
    "BEAMNG_USER",
    "/home/alex/.local/share/BeamNG/BeamNG.tech/current",
)
BEAMNG_HOST = os.environ.get("BEAMNG_HOST", "localhost")
BEAMNG_PORT = int(os.environ.get("BEAMNG_PORT", "64256"))

VEHICLE_MODEL = os.environ.get("BEAMNG_MODEL", "bastion")
SCENARIO_MAP  = os.environ.get("BEAMNG_MAP",   "west_coast_usa")
DUAL_CAMERA   = os.environ.get("DUAL_CAMERA",  "0") == "1"

SPAWN_POS      = (-829.5, -499.0, 106.8)
SPAWN_ROT_QUAT = (0.0, 0.0, -0.9272, 0.3746)

CAM_POS = (0.0, -0.4, 1.22)
CAM_DIR = (0.0, -1.0, 0.0)
CAM_UP  = (0.0,  0.0, 1.0)
# Vertical FOVs derived from openpilot's pinhole intrinsics for the sim device
# (DEVICE_CAMERAS[("pc","unknown")] = _ar_ox_config): FOV_y = 2*atan(h / (2*f)).
CAM_FOV      = 25.70   # road cam:  f=2648 px @ 1928x1208 → 2*atan(604/2648)
CAM_WIDE_FOV = 93.62   # wide cam:  f=567 px  @ 1928x1208 → 2*atan(604/567)
# NOTE: the real comma wide cam is a fisheye; 567 px is openpilot's own pinhole
# approximation (upstream comments call it inconsistent across the frame).
# BeamNG renders pinhole only, so matching the pinhole equivalent is optimal.

W, H = 1928, 1208
CAM_RENDER_W = W
CAM_RENDER_H = H


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def setup_beamng(port=BEAMNG_PORT, home=BEAMNG_HOME, user=BEAMNG_USER,
                 dual_camera=DUAL_CAMERA, timeout=120.0):
    """
    Connect to BeamNG (already launched by start.sh) and load the scenario.
    Returns (bng, vehicle, imu, camera, camera_wide).
    Prints BEAMNG_READY when the scenario is live.
    """
    print(f"[Setup] Connecting to BeamNG on {BEAMNG_HOST}:{port}...", flush=True)
    bng = _connect_with_retry(port, home, user, timeout)

    print("[Setup] Connected. Loading scenario...", flush=True)
    vehicle, imu, camera, camera_wide = _setup_scenario(bng, dual_camera)

    # Re-suppress logs — AdvancedIMU creates its logger lazily during setup.
    for _name in list(logging.Logger.manager.loggerDict.keys()):
        _lg = logging.getLogger(_name)
        _lg.setLevel(logging.WARNING)
        _lg.handlers.clear()
        _lg.propagate = False

    print("BEAMNG_READY", flush=True)
    return bng, vehicle, imu, camera, camera_wide


# ---------------------------------------------------------------------------
# Internals
# ---------------------------------------------------------------------------

def _connect_with_retry(port, home, user, timeout):
    deadline = time.monotonic() + timeout
    attempt = 0
    while True:
        attempt += 1
        try:
            bng = BeamNGpy(BEAMNG_HOST, port, home=home, user=user)
            bng.open(launch=False)
            print(f"[Setup] Connected (attempt {attempt})", flush=True)
            return bng
        except Exception as exc:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                print(f"[Setup] ERROR: Could not connect after {timeout:.0f} s: {exc}", flush=True)
                sys.exit(1)
            print(f"[Setup] Waiting for BeamNG... ({remaining:.0f} s left, attempt {attempt})", flush=True)
            time.sleep(3.0)


def _setup_scenario(bng, dual_camera):
    scenario = Scenario(SCENARIO_MAP, "openpilot_bridge",
                        description="openpilot BeamNG bridge")
    vehicle = Vehicle("ego", model=VEHICLE_MODEL, license="OPENPILOT")

    # Electrics is a vehicle-side sensor — attach before scenario loads.
    vehicle.sensors.attach("electrics", Electrics())

    scenario.add_vehicle(vehicle, pos=SPAWN_POS, rot_quat=SPAWN_ROT_QUAT)
    scenario.make(bng)
    bng.load_scenario(scenario)
    bng.start_scenario()

    vehicle.ai.set_mode("disabled")
    vehicle.control(throttle=0.0, brake=0.0, steering=0.0)
    vehicle.set_shift_mode("realistic_automatic")

    imu = AdvancedIMU(
        "imu", bng, vehicle,
        pos=(0.0, 0.0, 1.0),
        dir=CAM_DIR,
        up=CAM_UP,
        is_using_gravity=True,
        is_send_immediately=True,   # VE path — avoids GE-socket race with camera.poll()
        is_visualised=False,
    )
    camera = Camera(
        "road_cam", bng, vehicle,
        pos=CAM_POS,
        dir=CAM_DIR,
        up=CAM_UP,
        field_of_view_y=CAM_FOV,
        resolution=(CAM_RENDER_W, CAM_RENDER_H),
        near_far_planes=(0.1, 1500.0),
        is_render_annotations=False,
        is_render_depth=False,
        is_visualised=False,
        is_streaming=True,
        is_using_shared_memory=True,
        requested_update_time=0.05,   # 20 Hz — matches modeld's consumption; 0.01 made
                                      # BeamNG render both 1928×1208 sensor cams ~100/s,
                                      # starving modeld's CPU inference below 20 Hz
    )
    camera_wide = None
    if dual_camera:
        camera_wide = Camera(
            "wide_cam", bng, vehicle,
            pos=CAM_POS,
            dir=CAM_DIR,
            up=CAM_UP,
            field_of_view_y=CAM_WIDE_FOV,
            resolution=(CAM_RENDER_W, CAM_RENDER_H),
            near_far_planes=(0.1, 1500.0),
            is_render_annotations=False,
            is_render_depth=False,
            is_visualised=False,
            is_streaming=True,
            is_using_shared_memory=True,
            requested_update_time=0.05,   # 20 Hz — see road cam note above
        )

    dual_str = "dual" if dual_camera else "single"
    print(f"[Setup] '{VEHICLE_MODEL}' spawned on '{SCENARIO_MAP}' ({dual_str} camera).", flush=True)
    return vehicle, imu, camera, camera_wide
