"""
Linux bridge entry point.

Usage (from inside the openpilot-beamng-bridge distrobox):
    python3 ~/Documents/BeamNG-Openpilot-Bridge/linux/bridge_runner.py [--dual-camera]

Prerequisites:
    BeamNG is already running on the host with -nosteam -tcom -tport 64256
    (the control panel's BeamNG component / launch_beamng.sh does this).
"""
import argparse
import os
import sys
import threading

OPENPILOT_DIR = os.path.expanduser(os.environ.get('OPENPILOT_DIR', '~/openpilot'))
if OPENPILOT_DIR not in sys.path:
    sys.path.insert(0, OPENPILOT_DIR)

BRIDGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BRIDGE_DIR not in sys.path:
    sys.path.insert(0, BRIDGE_DIR)

from multiprocessing import Queue

from linux.beamng_setup import setup_beamng, BEAMNG_PORT
from bridge.beamng_bridge import BeamNGBridge
from bridge.beamng_world import BeamNGWorld
from bridge import op_shims

# Correct stock-sim defects (openpilot checkout itself stays pristine).
op_shims.apply()

# Written once sensors are live; start.sh polls for this before launching openpilot.
READY_FILE   = '/tmp/openpilot_beamng_bridge_ready'
# Named pipe — GUI writes cruise commands ('cruise_down', 'cruise_cancel', …).
CMD_FIFO     = '/tmp/beamng_bridge_cmd'
# Written each frame by the bridge — '1' if engaged, '0' if not.
ENGAGED_FILE = '/tmp/beamng_bridge_engaged'


def _start_cmd_fifo(q):
    """Background thread: read cruise commands from a FIFO and post to bridge queue."""
    from openpilot.tools.sim.bridge.common import QueueMessage, QueueMessageType
    try:
        os.mkfifo(CMD_FIFO, 0o600)
    except FileExistsError:
        pass

    def _loop():
        while True:
            try:
                with open(CMD_FIFO, 'r') as f:
                    for line in f:
                        cmd = line.strip()
                        if cmd:
                            q.put(QueueMessage(QueueMessageType.CONTROL_COMMAND, cmd))
            except Exception:
                pass

    threading.Thread(target=_loop, daemon=True, name='cmd_fifo').start()


def main():
    parser = argparse.ArgumentParser(description='BeamNG ↔ openpilot bridge (Linux)')
    parser.add_argument('--dual-camera', action=argparse.BooleanOptionalAction, default=True,
                        help='stream the wide road camera too (--no-dual-camera to disable)')
    parser.add_argument('--port', type=int, default=BEAMNG_PORT)
    parser.add_argument('--model', default=None, help='Override BEAMNG_MODEL env var')
    parser.add_argument('--map',   default=None, help='Override BEAMNG_MAP env var')
    args = parser.parse_args()

    if args.model:
        os.environ['BEAMNG_MODEL'] = args.model
    if args.map:
        os.environ['BEAMNG_MAP'] = args.map

    print('=== BeamNG ↔ openpilot bridge (Linux) ===')

    bng, vehicle, imu, camera, camera_wide = setup_beamng(
        port=args.port,
        dual_camera=args.dual_camera,
    )

    # Electrics is now on vehicle.sensors["electrics"] after vehicle.sensors.attach()
    electrics = vehicle.sensors["electrics"]

    world = BeamNGWorld(
        bng=bng,
        vehicle=vehicle,
        camera=camera,
        imu=imu,
        electrics=electrics,
        dual_camera=args.dual_camera,
        camera_wide=camera_wide,
    )

    # BeamNGWorld.__init__ blocks until _state_valid — sensors are flowing now.
    # Signal start.sh so it can launch openpilot with live sensor data from tick 0.
    with open(READY_FILE, 'w') as f:
        f.write('ready\n')
    print('BRIDGE_READY', flush=True)

    bridge = BeamNGBridge(dual_camera=args.dual_camera, high_quality=False)
    bridge.world = world

    q = Queue()
    _start_cmd_fifo(q)
    try:
        bridge._run_with_world(q)
    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        print('[bridge_runner] FATAL exception in bridge loop:', flush=True)
        traceback.print_exc()
    finally:
        world.close('exiting')
        for f in (READY_FILE, ENGAGED_FILE, CMD_FIFO):
            try:
                os.unlink(f)
            except FileNotFoundError:
                pass


if __name__ == '__main__':
    main()
