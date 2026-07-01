"""
BeamNG World implementation for the Linux all-native setup.

Talks to BeamNG directly via beamngpy — no intermediate DataServer TCP hop.
A background sensor thread polls IMU + Electrics + vehicle state at ~60 Hz.
A background camera thread polls the road camera as fast as possible and
releases image_lock whenever a new frame arrives, following the same
semaphore pattern as the MetaDrive bridge.
"""
import math
import threading
import time

import numpy as np

from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import Camera, AdvancedIMU, Electrics

from openpilot.common.realtime import Ratekeeper
from openpilot.tools.sim.lib.camerad import H, W
from openpilot.tools.sim.lib.common import SimulatorState, World, vec3

CAM_RENDER_W = W  # 1928
CAM_RENDER_H = H  # 1208

MAX_STEER_DEG      = 495.0   # Bastion full-lock steering wheel travel
MAX_STEER_RATE_DEG_S = 60.0  # max steering rate to prevent EPAS faults


class BeamNGWorld(World):
    def __init__(self,
                 bng:          BeamNGpy,
                 vehicle:      Vehicle,
                 camera:       Camera,
                 imu:          AdvancedIMU,
                 electrics:    Electrics,
                 dual_camera:  bool  = False,
                 camera_wide:  Camera | None = None):
        super().__init__(dual_camera)

        self.bng          = bng
        self.vehicle      = vehicle
        self.camera       = camera
        self.camera_wide  = camera_wide
        self.imu          = imu
        self.electrics    = electrics

        self._lock         = threading.Lock()
        self._state_valid  = False
        self._velocity     = vec3(0.0, 0.0, 0.0)
        self._steering_deg = 0.0
        self._pos          = (0.0, 0.0, 0.0)
        self._imu_accel    = vec3(0.0, 0.0, -9.81)
        self._imu_gyro     = vec3(0.0, 0.0,  0.0)
        self._bearing      = 0.0

        # Rate-limited steering command (degrees)
        self._steer_cmd    = 0.0
        self._last_ctrl_t  = time.monotonic()

        self._brake_input   = 0.0   # last electrics brake value (player input when driver mode active)

        self._vehicle_reset = threading.Event()
        self._exit          = threading.Event()

        # Current FOV values — updated by set_camera_fov(); used by _rebuild_cameras().
        # Initialised from beamng_setup constants on first use.
        self._road_fov = None
        self._wide_fov = None
        # Protects camera object replacement against concurrent _camera_loop reads.
        self._cam_lock = threading.Lock()

        # Serialises ALL BeamNG VE-socket calls (vehicle.sensors.poll and
        # vehicle.control both use the same per-vehicle TCP connection; concurrent
        # access from multiple threads corrupts the protocol and causes BeamNG to
        # close the socket).
        self._ve_lock = threading.Lock()

        # Latest-wins control slot — apply_controls() writes here; the control
        # thread reads and calls vehicle.control().  This prevents a hung
        # vehicle.control() TCP call from blocking the main bridge loop and
        # stopping IMU publishing (which causes sensorDataInvalid after 10 s).
        self._ctrl_lock = threading.Lock()
        self._ctrl_cmd  = {'steering': 0.0, 'throttle': 0.0, 'brake': 0.0}
        self._ctrl_event = threading.Event()

        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name='beamng_ctrl'
        )
        self._control_thread.start()

        self._sensor_thread = threading.Thread(
            target=self._sensor_loop, daemon=True, name='beamng_sensors'
        )
        self._sensor_thread.start()

        self._camera_thread = threading.Thread(
            target=self._camera_loop, daemon=True, name='beamng_camera'
        )
        self._camera_thread.start()

        # Block until first sensor frame so the bridge sees valid state on start
        print('[BeamNGWorld] Waiting for first sensor frame (up to 60 s)...', flush=True)
        deadline = time.monotonic() + 60.0
        while not self._state_valid and not self._exit.is_set():
            if time.monotonic() > deadline:
                raise RuntimeError('No sensor data from BeamNG within 60 s')
            time.sleep(0.05)
        print('[BeamNGWorld] Ready — sensors live.', flush=True)

    # ------------------------------------------------------------------
    # Background threads
    # ------------------------------------------------------------------

    def _sensor_loop(self):
        """Poll vehicle state + IMU + Electrics at ~60 Hz."""
        rk = Ratekeeper(60, None)
        prev_pos  = None
        # yaw-mapping diagnostic state: compare mapped gyro yaw against the
        # yaw rate derived from the vehicle quaternion (ground truth).
        _diag_next    = time.monotonic() + 5.0
        _diag_bearing = None
        _diag_t       = None

        while not self._exit.is_set():
            try:
                # Poll Electrics (VE socket) + IMU (VE path, is_send_immediately=True)
                # together under _ve_lock so they don't race with vehicle.control().
                with self._ve_lock:
                    self.vehicle.sensors.poll()
                    imu_readings = self.imu.poll()

                state = self.vehicle.state  # populated by poll()

                # Electrics (steering angle, wheel speed, brake pedal)
                vehicle_electrics = self.electrics
                # beamngpy 1.35: electrics['steering'] is already in degrees.
                steering_deg = float(vehicle_electrics.get('steering', 0.0))
                wheelspeed   = float(vehicle_electrics.get('wheelspeed', 0.0))
                brake_input  = float(vehicle_electrics.get('brake', 0.0))
                imu_data = {}
                if isinstance(imu_readings, list) and imu_readings:
                    imu_data = imu_readings[-1]
                elif isinstance(imu_readings, dict):
                    imu_data = imu_readings
                # Only update if we actually got data; keep last valid reading otherwise
                # so a crash/tumble that returns zeros doesn't poison the IMU stream.
                if 'accSmooth' in imu_data:
                    acc = imu_data['accSmooth']
                    gyr = imu_data.get('angVelSmooth', [0.0, 0.0, 0.0])
                    # ── BeamNG → openpilot IMU frame mapping ─────────────────
                    # openpilot expects sensor messages in a [up, left, backward]
                    # frame: locationd converts them to device [fwd, right, down]
                    # as [-v2, -v1, -v0], and the accelerometer must read
                    # [+9.81, 0, 0] at rest.  BeamNG's AdvancedIMU (with our
                    # dir=forward, up=+Z config) reports in [forward, up, right]
                    # with accSmooth sign-flipped specific force — at rest it
                    # reads -9.81 on index 1 (verified from monitor_imu logs).
                    # Feeding the raw vector made locationd's gyro/vision yaw
                    # cross-check reject every gyroscope observation.
                    # Horizontal/yaw signs are checked live by the [IMU] yaw
                    # diagnostic below — flip here if it reports a mismatch.
                    accel = vec3(-float(acc[1]),  float(acc[2]),  float(acc[0]))
                    gyro  = vec3( float(gyr[1]), -float(gyr[2]), -float(gyr[0]))
                else:
                    # No new IMU data — keep whatever is in _imu_accel/_imu_gyro
                    with self._lock:
                        accel = self._imu_accel
                        gyro  = self._imu_gyro

                # Vehicle position and velocity from state
                pos = state.get('pos', (0.0, 0.0, 0.0))
                vel = state.get('vel', (0.0, 0.0, 0.0))
                rot = state.get('rotation', (0.0, 0.0, 0.0, 1.0))  # xyzw quaternion

                # Bearing from quaternion yaw
                x, y, z, w = float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])
                yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
                bearing = math.degrees(yaw) % 360.0

                # Detect vehicle recovery teleport (position jump > 5 m)
                curr_pos = (float(pos[0]), float(pos[1]), float(pos[2]))
                if prev_pos is not None:
                    dx = curr_pos[0] - prev_pos[0]
                    dy = curr_pos[1] - prev_pos[1]
                    if math.hypot(dx, dy) > 5.0:
                        self._vehicle_reset.set()
                prev_pos = curr_pos

                with self._lock:
                    self._velocity     = vec3(float(vel[0]), float(vel[1]), float(vel[2]))
                    self._steering_deg = steering_deg
                    self._brake_input  = brake_input
                    self._pos          = curr_pos
                    self._imu_accel    = accel
                    self._imu_gyro     = gyro
                    self._bearing      = bearing
                    self._state_valid  = True

                # 5 s diagnostic: mapped gyro yaw (gyro.x = up axis, CCW+) must
                # match the bearing-derived yaw rate in sign and magnitude.
                _dnow = time.monotonic()
                if _dnow >= _diag_next:
                    if _diag_bearing is not None:
                        db = (bearing - _diag_bearing + 180.0) % 360.0 - 180.0
                        brate = math.radians(db) / (_dnow - _diag_t)
                        print(f'[IMU] yaw_gyro={gyro.x:+.3f} rad/s  yaw_from_bearing={brate:+.3f} rad/s  '
                              f'acc=[up {accel.x:+5.2f}, left {accel.y:+5.2f}, back {accel.z:+5.2f}]',
                              flush=True)
                    _diag_bearing, _diag_t = bearing, _dnow
                    _diag_next = _dnow + 5.0

            except BaseException as exc:
                if isinstance(exc, (SystemExit, KeyboardInterrupt)):
                    raise
                if not self._exit.is_set():
                    print(f'[BeamNGWorld] Sensor error ({type(exc).__name__}): {exc}', flush=True)

            rk.keep_time()

    @staticmethod
    def _to_rgb(colour) -> np.ndarray:
        """Return a uint8 (H, W, 3) RGB array from a beamngpy colour frame."""
        from PIL import Image as _PILImage
        if isinstance(colour, _PILImage.Image):
            return np.array(colour.convert('RGB'), dtype=np.uint8)

        arr = np.asarray(colour)
        if arr.ndim == 3 and arr.shape[2] == 4:
            return arr[:, :, :3].copy()
        return np.array(arr, dtype=np.uint8)

    def _control_loop(self):
        """Send queued control commands to BeamNG in a dedicated thread.

        Decoupled from the main bridge loop so a hung vehicle.control() TCP call
        (which can happen during crash/recovery) does not stall IMU publishing.

        Uses _ve_lock to serialise VE-socket access with _sensor_loop — both
        vehicle.sensors.poll() and vehicle.control() share the same TCP connection.
        """
        while not self._exit.is_set():
            triggered = self._ctrl_event.wait(timeout=0.1)
            self._ctrl_event.clear()
            if self._exit.is_set():
                break
            if not triggered:
                continue  # timeout with no new command — don't spam vehicle.control()
            with self._ctrl_lock:
                cmd = dict(self._ctrl_cmd)
            try:
                with self._ve_lock:
                    self.vehicle.control(
                        steering=cmd['steering'],
                        throttle=cmd['throttle'],
                        brake=cmd['brake'],
                    )
            except Exception as exc:
                print(f'[BeamNGWorld] Control error: {exc}', flush=True)

    def _camera_loop(self):
        """Stream road camera via shared memory; release image_lock on each new frame.

        Uses camera.stream() instead of camera.poll() so BeamNG writes frames
        directly into a shared memory buffer — no TCP round-trip per frame.

        Publish rate is capped to stay at or below modeld's actual evaluation
        rate.  Publishing faster causes vipc frame-ID gaps that make modeld
        mark its output invalid ("skipping model eval. Dropped N frames"),
        preventing engagement.
        """
        _first_frame = True
        _frame_interval = 1.0 / 20.0  # 20 Hz publish cap (= modeld's frame rate)
        _next_t = time.monotonic()

        while not self._exit.is_set():
            # Pace before sampling so we always grab the freshest BeamNG frame
            # (shared memory is updated continuously by BeamNG between sleeps).
            now = time.monotonic()
            wait = _next_t - now
            if wait > 0:
                time.sleep(wait)
            _next_t = max(_next_t + _frame_interval, time.monotonic())

            try:
                with self._cam_lock:
                    data = self.camera.stream()
                if data and 'colour' in data and data['colour'] is not None:
                    if _first_frame:
                        c = data['colour']
                        print(f'[Camera] first frame type={type(c).__name__} '
                              f'mode={getattr(c, "mode", "n/a")} '
                              f'size={getattr(c, "size", "n/a")}', flush=True)
                        _first_frame = False
                    rgb = self._to_rgb(data['colour'])
                    self.road_image[...] = rgb

                    if self.dual_camera and self.camera_wide is not None:
                        with self._cam_lock:
                            wide = self.camera_wide.stream()
                        if wide and 'colour' in wide and wide['colour'] is not None:
                            w_rgb = self._to_rgb(wide['colour'])
                            self.wide_road_image[...] = w_rgb

            except Exception as exc:
                if not self._exit.is_set():
                    print(f'[BeamNGWorld] Camera error ({type(exc).__name__}): {exc}')

            # Always release image_lock regardless of whether we got a new frame.
            # send_camera_images() does a blocking acquire() with NO timeout — if we
            # ever skip the release (bad frame, exception, None data) the bridge's
            # camera thread hangs forever and openpilot goes blank.
            while self.image_lock.acquire(block=False):
                pass
            self.image_lock.release()

    # ------------------------------------------------------------------
    # World interface
    # ------------------------------------------------------------------

    def apply_controls(self, steer_angle_deg: float, throttle: float,
                       brake: float, engaged: bool = False):
        now = time.monotonic()
        dt  = min(now - self._last_ctrl_t, 0.1)  # cap first-frame dt so rate limiter can't jump
        self._last_ctrl_t = now

        # Rate-limit steering to avoid EPAS fault
        max_delta = MAX_STEER_RATE_DEG_S * dt
        self._steer_cmd = float(np.clip(
            steer_angle_deg,
            self._steer_cmd - max_delta,
            self._steer_cmd + max_delta,
        ))

        # BeamNG expects steering normalised to [-1, 1]; negate for sign convention
        steer_norm = float(np.clip(-self._steer_cmd / MAX_STEER_DEG, -1.0, 1.0))

        # Post to the dedicated control thread — never block the main loop here.
        with self._ctrl_lock:
            self._ctrl_cmd = {
                'steering': steer_norm,
                'throttle': float(np.clip(throttle, 0.0, 1.0)),
                'brake':    float(np.clip(brake,    0.0, 1.0)),
            }
        self._ctrl_event.set()

    def read_sensors(self, state: SimulatorState):
        with self._lock:
            if not self._state_valid:
                return
            state.velocity          = self._velocity
            state.steering_angle    = self._steering_deg
            state.imu.accelerometer = self._imu_accel
            state.imu.gyroscope     = self._imu_gyro
            state.imu.bearing       = self._bearing
            state.bearing           = self._bearing
            state.gps.from_xy((self._pos[0], self._pos[1]))
            state.valid             = True

    def read_state(self):
        if not self._sensor_thread.is_alive():
            print('[BeamNGWorld] Sensor thread died — restarting', flush=True)
            self._sensor_thread = threading.Thread(
                target=self._sensor_loop, daemon=True, name='beamng_sensors'
            )
            self._sensor_thread.start()

    @property
    def player_brake(self) -> float:
        """Electrics brake value — reflects actual player input when driver mode is active
        (i.e. when apply_controls() is not being called and BeamNG has control)."""
        with self._lock:
            return self._brake_input

    def consume_vehicle_reset(self) -> bool:
        if self._vehicle_reset.is_set():
            self._vehicle_reset.clear()
            return True
        return False

    def read_cameras(self):
        pass  # camera frames arrive asynchronously via _camera_loop

    def tick(self):
        pass  # BeamNG advances its own physics

    def reset(self):
        # Run recover() in a background thread so the main bridge loop (and IMU
        # publishing) keeps running during the BeamNG teleport, which can take
        # several seconds and would otherwise trip the 10-s sensorDataInvalid watchdog.
        def _do_recover():
            try:
                self.vehicle.recover()
            except Exception as exc:
                print(f'[BeamNGWorld] recover error: {exc}')
        threading.Thread(target=_do_recover, daemon=True, name='beamng_recover').start()

    def set_camera_fov(self, road_fov: float | None = None,
                       wide_fov: float | None = None) -> None:
        """Update road/wide camera vertical FOV on the fly.

        Rebuilds only the changed camera(s) in a background thread so the
        main bridge loop is never blocked.  Changes smaller than 0.5° are
        ignored to avoid thrashing while a GUI slider is being dragged.
        """
        # Lazy-init from beamng_setup defaults on first call.
        if self._road_fov is None:
            from linux.beamng_setup import CAM_FOV, CAM_WIDE_FOV
            self._road_fov = CAM_FOV
            self._wide_fov = CAM_WIDE_FOV

        changed = False
        if road_fov is not None and abs(road_fov - self._road_fov) >= 0.5:
            self._road_fov = road_fov
            changed = True
        if wide_fov is not None and self.camera_wide is not None \
                and abs(wide_fov - (self._wide_fov or 0)) >= 0.5:
            self._wide_fov = wide_fov
            changed = True

        if changed:
            threading.Thread(target=self._rebuild_cameras, daemon=True,
                             name='cam_fov_rebuild').start()

    def _rebuild_cameras(self) -> None:
        """Remove and re-create camera sensor(s) with updated FOV values."""
        from beamngpy.sensors import Camera as _Camera
        from linux.beamng_setup import CAM_POS, CAM_DIR, CAM_UP
        _cam_kwargs = dict(
            pos=CAM_POS, dir=CAM_DIR, up=CAM_UP,
            resolution=(CAM_RENDER_W, CAM_RENDER_H),
            near_far_planes=(0.1, 1500.0),
            is_render_annotations=False, is_render_depth=False,
            is_visualised=False, is_streaming=True,
            is_using_shared_memory=True, requested_update_time=0.05,
        )
        with self._cam_lock:
            try:
                self.camera.remove()
            except Exception:
                pass
            self.camera = _Camera('road_cam', self.bng, self.vehicle,
                                  field_of_view_y=self._road_fov, **_cam_kwargs)
            print(f'[BeamNGWorld] Road cam FOV → {self._road_fov:.1f}°', flush=True)

            if self.camera_wide is not None and self._wide_fov is not None:
                try:
                    self.camera_wide.remove()
                except Exception:
                    pass
                self.camera_wide = _Camera('wide_cam', self.bng, self.vehicle,
                                           field_of_view_y=self._wide_fov, **_cam_kwargs)
                print(f'[BeamNGWorld] Wide cam FOV → {self._wide_fov:.1f}°', flush=True)

    def close(self, reason: str):
        print(f'[BeamNGWorld] Closing: {reason}')
        self._exit.set()
        try:
            self.bng.close()
        except Exception:
            pass
