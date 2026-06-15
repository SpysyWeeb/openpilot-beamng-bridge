#!/usr/bin/env python3
"""
BeamNG scenario launcher — Windows side.

Responsible for one thing: launching BeamNG.Drive and loading a scenario.
All runtime sensor polling, camera streaming, and vehicle control live in
beamng_runtime.py so this file stays easy to read and modify.

Public API
----------
setup_beamng() -> (bng, vehicle, imu, camera)
    Launch BeamNG, load the configured scenario, and return the raw beamngpy
    objects.  Caller is responsible for closing bng when done.
"""

import logging
import os
import sys

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import AdvancedIMU, Camera, Electrics

# Suppress beamngpy's verbose DEBUG output at import time.
# beamngpy attaches StreamHandlers to its own loggers, so basicConfig alone
# (which only touches the root handler) is not enough.
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
    r"C:\Program Files (x86)\Steam\steamapps\common\BeamNG.drive",
)
BEAMNG_HOST = os.environ.get("BEAMNG_HOST", "localhost")
BEAMNG_PORT = int(os.environ.get("BEAMNG_PORT", "64256"))

VEHICLE_MODEL  = os.environ.get("BEAMNG_MODEL", "bastion")
SCENARIO_MAP   = os.environ.get("BEAMNG_MAP",   "west_coast_usa")

DUAL_CAMERA    = os.environ.get("DUAL_CAMERA", "0") == "1"

SPAWN_POS      = (-829.5, -499.0, 106.8)
SPAWN_ROT_QUAT = (0.0, 0.0, -0.9272, 0.3746)

CAM_POS = (0.0, -0.3, 1.22)
CAM_DIR = (0.0, -1.0, 0.0)
CAM_UP  = (0.0,  0.0, 1.0)
CAM_FOV      = 60.0
CAM_WIDE_FOV = 94.0   # 2 * degrees(arctan(604 / 567)) — matches Comma 3 wide cam focal length of 567 px

W, H = 1928, 1208
CAM_RENDER_W = W // 2   # render at half-res, upscale in WSL bridge
CAM_RENDER_H = H // 2

MAX_STEER_DEG = 495.0         # Corolla TSS2 max wheel travel (±495°); normalises OP's angle commands to BeamNG's ±1
MAX_STEER_RATE_DEG_S = 2000.0  # max steering change rate (°/s) to simulate EPAS response speed


# ---------------------------------------------------------------------------
# Scenario setup
# ---------------------------------------------------------------------------

def setup_beamng():
    """
    Launch BeamNG, load the scenario, and return (bng, vehicle, imu, camera, camera_wide).
    camera_wide is None when DUAL_CAMERA env var is not "1".
    Prints BEAMNG_READY to stdout when the scenario is live.
    """
    print(f"[Setup] Launching BeamNG from: {BEAMNG_HOME}", flush=True)
    bng = BeamNGpy(BEAMNG_HOST, BEAMNG_PORT, home=BEAMNG_HOME)
    try:
        bng.open(launch=True)
    except Exception as exc:
        print(f"[Setup] ERROR: Could not launch BeamNG: {exc}", flush=True)
        sys.exit(1)

    print("[Setup] BeamNG running. Loading scenario...", flush=True)
    vehicle, imu, camera, camera_wide = _setup_scenario(bng)

    # Re-apply log suppression — AdvancedIMU creates its own logger lazily during
    # _setup_scenario(), after our initial filter ran above.
    for _name in list(logging.Logger.manager.loggerDict.keys()):
        _lg = logging.getLogger(_name)
        _lg.setLevel(logging.WARNING)
        _lg.handlers.clear()
        _lg.propagate = False

    print("BEAMNG_READY", flush=True)
    return bng, vehicle, imu, camera, camera_wide


def _setup_scenario(bng: BeamNGpy) -> tuple:
    """Load west_coast_usa, spawn the vehicle, attach sensors."""
    scenario = Scenario(SCENARIO_MAP, "openpilot_bridge",
                        description="openpilot BeamNG bridge")
    vehicle = Vehicle("ego", model=VEHICLE_MODEL, license="OPENPILOT")

    # Phase 1 — lightweight sensor before the scenario loads.
    vehicle.sensors.attach("electrics", Electrics())

    scenario.add_vehicle(vehicle, pos=SPAWN_POS, rot_quat=SPAWN_ROT_QUAT)
    scenario.make(bng)
    bng.load_scenario(scenario)
    bng.start_scenario()

    vehicle.ai.set_mode("disabled")
    vehicle.control(throttle=0.0, brake=0.0, steering=0.0)
    vehicle.set_shift_mode("realistic_automatic")

    # Phase 2 — CommBase sensors need an active simulation.
    imu = AdvancedIMU(
        "imu", bng, vehicle,
        pos=(0.0, 0.0, 1.0),
        dir=CAM_DIR,
        up=CAM_UP,
        is_using_gravity=True,
        is_send_immediately=True,
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
    )
    camera_wide = None
    if DUAL_CAMERA:
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
        )

    dual_str = "dual" if DUAL_CAMERA else "single"
    print(f"[Setup] '{VEHICLE_MODEL}' spawned on '{SCENARIO_MAP}' ({dual_str} camera).", flush=True)
    return vehicle, imu, camera, camera_wide
