"""
Linux-side BeamNG scenario setup.

Connects to an already-running BeamNG.tech instance (started via launch_beamng.sh),
loads the west_coast_usa map, spawns a Bastion, and attaches all sensors needed by
the bridge. Intended to be called once from linux/bridge_runner.py before the bridge
loop starts.
"""
import time

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera, AdvancedIMU, Electrics

BEAMNG_HOST = 'localhost'
BEAMNG_PORT = 64256
BEAMNG_HOME = '/home/alex/.local/share/Steam/steamapps/common/BeamNG.drive'
BEAMNG_USER = '/home/alex/.local/share/BeamNG/BeamNG.tech/current'

VEHICLE_MODEL = 'bastion'
MAP_NAME      = 'west_coast_usa'

# Road camera — matches the Comma 3 road camera specs used in the Windows bridge
CAM_RENDER_W  = 964
CAM_RENDER_H  = 604
CAM_FOV       = 60.0   # degrees, matches narrow road cam
CAM_WIDE_FOV  = 94.0   # degrees, matches Comma 3 wide cam
CAM_POS       = (0.0, -0.5, 1.22)  # ~dash height, slightly forward of vehicle center
CAM_DIR       = (0, -1, 0)         # facing forward (BeamNG Y-forward convention)
CAM_UP        = (0, 0, 1)


def connect(port: int = BEAMNG_PORT,
            home: str = BEAMNG_HOME,
            user: str = BEAMNG_USER,
            timeout: float = 120.0) -> BeamNGpy:
    """Connect to already-running BeamNG (launched by start.sh with -nosteam -tcom -tport)."""
    deadline = time.monotonic() + timeout
    attempt = 0
    while True:
        attempt += 1
        try:
            bng = BeamNGpy(BEAMNG_HOST, port, home=home, user=user)
            bng.open(launch=False)
            print(f'[setup] Connected to BeamNG at {BEAMNG_HOST}:{port} (attempt {attempt})')
            return bng
        except Exception as exc:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(
                    f'Could not connect to BeamNG on port {port} after {timeout:.0f} s: {exc}'
                )
            print(f'[setup] Waiting for BeamNG... ({remaining:.0f} s left, attempt {attempt})')
            time.sleep(3.0)


def setup_scenario(bng: BeamNGpy,
                   dual_camera: bool = False,
                   model: str = VEHICLE_MODEL,
                   map_name: str = MAP_NAME):
    """
    Load scenario, spawn ego vehicle, attach sensors.

    Returns:
        (vehicle, camera, camera_wide, imu, electrics)
        camera_wide is None when dual_camera=False.
    """
    print(f'[setup] Building scenario on {map_name}...')
    scenario = Scenario(map_name, 'openpilot_bridge',
                        description='openpilot BeamNG bridge')

    vehicle = Vehicle('ego', model=model, license='OPENPILOT')
    scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot_quat=(0, 0, 0, 1))
    scenario.make(bng)

    bng.scenario.load(scenario)
    bng.scenario.start()
    print('[setup] Scenario running.')

    # Electrics: vehicle-side sensor — attach before connecting
    electrics = Electrics()
    electrics.attach(vehicle, 'electrics')
    electrics.connect(bng, vehicle)

    # Road camera
    camera = Camera(
        'road_cam', bng, vehicle,
        pos=CAM_POS, dir=CAM_DIR, up=CAM_UP,
        resolution=(CAM_RENDER_W, CAM_RENDER_H),
        field_of_view_y=CAM_FOV,
        requested_update_time=0.016,
        is_render_colours=True,
        is_render_annotations=False,
        is_render_depth=False,
    )

    camera_wide = None
    if dual_camera:
        camera_wide = Camera(
            'wide_cam', bng, vehicle,
            pos=CAM_POS, dir=CAM_DIR, up=CAM_UP,
            resolution=(CAM_RENDER_W, CAM_RENDER_H),
            field_of_view_y=CAM_WIDE_FOV,
            requested_update_time=0.016,
            is_render_colours=True,
            is_render_annotations=False,
            is_render_depth=False,
        )

    # AdvancedIMU: provides calibrated accel + gyro in vehicle frame
    imu = AdvancedIMU(
        'imu', bng, vehicle,
        pos=(0, 0, 0), dir=(0, -1, 0), up=(0, 0, 1),
        physics_update_time=0.01,
        is_using_gravity=True,
    )

    print('[setup] Sensors attached. Waiting for first camera frame...')
    _wait_for_camera(camera)
    print('[setup] Ready.')

    return vehicle, camera, camera_wide, imu, electrics


def _wait_for_camera(camera: Camera, timeout: float = 30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            data = camera.poll()
            if data and 'colour' in data and data['colour'] is not None:
                return
        except Exception:
            pass
        time.sleep(0.1)
    raise RuntimeError('BeamNG camera not ready after 30 seconds')
