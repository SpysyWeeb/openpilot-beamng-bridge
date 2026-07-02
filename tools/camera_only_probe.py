#!/usr/bin/env python3
"""
Camera-only isolation probe — replicates the comma-device bench test on PC.

Feeds ONLY BeamNG camera frames + a fake ignition signal into openpilot.
No SimulatedCar, no CAN, no controls, no IMU/GPS. Pair it with openpilot
launched as:

    BLOCK=card bash ~/openpilot/openpilot/tools/sim/launch_openpilot.sh

and a hand-written MOCK CarParams (card is blocked, so nothing else writes
one). This is the exact configuration the real comma 3X ran on the bench
2026-07-02 (mock car, passive, camera-only), where the model produced a
full healthy path on BeamNG footage.

Interpretation:
  - Path still collapses to a stub here  -> defect is in the frame path
    (rgb_to_nv12, intrinsics, pacing) or the PC model runner itself.
  - Path looks like the device's        -> defect is in the sim runner's
    car/CAN/sensor layer (SimulatedCar, IMU, locationd inputs).

Usage (inside the openpilot-beamng-bridge distrobox, BeamNG already up):
    source ~/openpilot/.venv/bin/activate
    python3 tools/camera_only_probe.py [--no-dual-camera]
"""
import argparse
import os
import sys
import time

OPENPILOT_DIR = os.path.expanduser(os.environ.get('OPENPILOT_DIR', '~/openpilot'))
if OPENPILOT_DIR not in sys.path:
    sys.path.insert(0, OPENPILOT_DIR)
BRIDGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BRIDGE_DIR not in sys.path:
    sys.path.insert(0, BRIDGE_DIR)

# Layout compat: on legacy root-layout trees (e.g. release branches) cereal
# lives at the repo root and openpilot/ is a partial shim without it — alias
# it so op_shims' nested-layout imports resolve on both.
try:
    import openpilot.cereal.messaging  # noqa: F401  (nested layout, master)
except ModuleNotFoundError:
    import cereal as _cereal
    import openpilot as _openpilot
    sys.modules['openpilot.cereal'] = _cereal
    _openpilot.cereal = _cereal

from bridge import op_shims
op_shims.apply()  # real monotonic vipc timestamps + full-range color

import openpilot.cereal.messaging as messaging
from openpilot.tools.sim.lib.camerad import Camerad

from linux.beamng_setup import setup_beamng
from bridge.beamng_world import BeamNGWorld


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dual-camera', action=argparse.BooleanOptionalAction, default=True)
    args = ap.parse_args()

    bng, vehicle, imu, camera, camera_wide = setup_beamng(dual_camera=args.dual_camera)
    electrics = vehicle.sensors['electrics']

    world = BeamNGWorld(bng=bng, vehicle=vehicle, camera=camera, imu=imu,
                        electrics=electrics, dual_camera=args.dual_camera,
                        camera_wide=camera_wide)

    camerad = Camerad(dual_camera=args.dual_camera)
    pm = messaging.PubMaster(['pandaStates'])

    print('[probe] PROBE_READY — feeding cameras + ignition only. Ctrl-C to stop.', flush=True)
    frame = 0
    try:
        while True:
            # BeamNGWorld._camera_loop releases image_lock once per fresh frame (20 Hz cap)
            world.image_lock.acquire()
            camerad.cam_send_yuv_road(camerad.rgb_to_yuv(world.road_image))
            if args.dual_camera:
                camerad.cam_send_yuv_wide_road(camerad.rgb_to_yuv(world.wide_road_image))

            if frame % 10 == 0:  # 2 Hz, same cadence as SimulatedCar
                dat = messaging.new_message('pandaStates', 1)
                dat.valid = True
                dat.pandaStates[0] = {
                    'ignitionLine': True,
                    'pandaType': 'blackPanda',
                    'controlsAllowed': False,
                    'safetyModel': 'noOutput',
                }
                pm.send('pandaStates', dat)

            if frame % 200 == 0:
                print(f'[probe] {frame} frames fed', flush=True)
            frame += 1
    finally:
        world.close('probe exit')


if __name__ == '__main__':
    main()
