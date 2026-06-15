import json
import math
import os
import socket
import struct
import subprocess
import threading
import time

import numpy as np

from openpilot.tools.sim.lib.camerad import H, W
from openpilot.tools.sim.lib.common import SimulatorState, World, vec3

DATA_PORT = int(os.environ.get("DATA_PORT", "12321"))

# Wire protocol — must match beamng_setup.py
MSG_SENSOR  = 0x01
MSG_CAMERA  = 0x02
MSG_CONTROL     = 0x03
MSG_CAMERA_WIDE = 0x04
_HDR = struct.Struct("<BI")   # 1-byte type + 4-byte uint32 length

# beamng_setup.py renders at half resolution to reduce BeamNG render latency.
# We upscale here with a pixel-double (numpy repeat) before writing road_image.
CAM_RENDER_W = W // 2   # 964
CAM_RENDER_H = H // 2   # 604
CAMERA_FRAME_BYTES = CAM_RENDER_W * CAM_RENDER_H * 3


def _candidate_hosts() -> list[str]:
    """
    Return a prioritised list of IPs to try when connecting to the Windows data server.

    WSL2 networking has two modes:
      - Mirrored (Windows 11 22H2+, opt-in): 'localhost' reaches Windows directly.
      - NAT (default / older): 'localhost' loops to WSL itself; the Windows host is
        the default-route gateway (commonly 172.x.0.1).

    We try both so the bridge works regardless of which mode is active.
    If DATA_HOST is set explicitly in the environment, use that only.
    """
    explicit = os.environ.get("DATA_HOST", "")
    if explicit:
        return [explicit]

    hosts: list[str] = ["localhost"]
    try:
        r = subprocess.run(
            ["bash", "-c",
             "ip route show default | awk '/default/{print $3; exit}'"],
            capture_output=True, text=True, timeout=3,
        )
        gw = r.stdout.strip()
        if gw and gw != "localhost" and gw != "127.0.0.1":
            hosts.append(gw)
    except Exception:
        pass
    return hosts


class BeamNGWorld(World):
    """
    World implementation that receives sensor data from beamng_setup.py (Windows)
    via a TCP socket. Does NOT connect to BeamNG directly — that's handled entirely
    by beamng_setup.py on the Windows side.
    """

    def __init__(self, q, dual_camera: bool = False):
        super().__init__(dual_camera)
        self.q = q

        self._lock = threading.Lock()
        self._velocity        = vec3(0.0, 0.0, 0.0)
        self._steering_angle  = 0.0
        self._pos             = (0.0, 0.0, 0.0)
        self._imu_accel       = vec3(0.0, 0.0, -9.81)
        self._imu_gyro        = vec3(0.0, 0.0, 0.0)
        self._bearing         = 0.0
        self._state_valid     = False
        self._wide_frame_buf  = None
        self._wide_lock       = threading.Lock()

        self._io_exit = threading.Event()
        self._vehicle_reset = threading.Event()
        hosts = _candidate_hosts()
        print(f"[BeamNGWorld] Connecting to data server on port {DATA_PORT} "
              f"(candidates: {hosts})")
        connected = False
        for attempt in range(1, 21):
            for host in hosts:
                try:
                    self._sock = socket.create_connection((host, DATA_PORT), timeout=5.0)
                    self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self._sock.settimeout(None)
                    print(f"[BeamNGWorld] Connected to {host}:{DATA_PORT} "
                          f"on attempt {attempt}.")
                    connected = True
                    break
                except Exception as exc:
                    print(f"[BeamNGWorld] {host}:{DATA_PORT} attempt {attempt} "
                          f"failed: {exc}")
            if connected:
                break
            if attempt < 20:
                time.sleep(3.0)
        if not connected:
            raise RuntimeError(
                f"Could not connect to data server on port {DATA_PORT} "
                f"after 20 attempts (tried: {hosts})"
            )

        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name="beamng_rx"
        )
        self._rx_thread.start()

        # Wait for first sensor frame before handing control to the bridge
        deadline = time.monotonic() + 30.0
        while not self._state_valid and not self._io_exit.is_set():
            if time.monotonic() > deadline:
                raise RuntimeError("No sensor data from beamng_setup.py within 30 s")
            time.sleep(0.05)

        print("[BeamNGWorld] Ready.")

    # -------------------------------------------------------------------------
    # Receive loop
    # -------------------------------------------------------------------------

    def _rx_loop(self):
        buf = b""
        hdr = _HDR.size

        while not self._io_exit.is_set():
            try:
                chunk = self._sock.recv(65536)
                if not chunk:
                    print("[BeamNGWorld] Data server closed connection.")
                    self._io_exit.set()
                    self.exit_event.set()
                    break
                buf += chunk

                while len(buf) >= hdr:
                    msg_type, length = _HDR.unpack(buf[:hdr])
                    if len(buf) < hdr + length:
                        break
                    payload = buf[hdr:hdr + length]
                    buf = buf[hdr + length:]

                    if msg_type == MSG_SENSOR:
                        self._handle_sensor(json.loads(payload.decode()))
                    elif msg_type == MSG_CAMERA:
                        self._handle_camera(payload)
                    elif msg_type == MSG_CAMERA_WIDE:
                        self._handle_camera_wide(payload)

            except Exception as exc:
                if not self._io_exit.is_set():
                    print(f"[BeamNGWorld] RX error: {exc}")
                    self._io_exit.set()
                    self.exit_event.set()
                break

    def consume_vehicle_reset(self) -> bool:
        """Return True (once) if beamng_setup detected a vehicle recovery teleport."""
        if self._vehicle_reset.is_set():
            self._vehicle_reset.clear()
            return True
        return False

    def _handle_sensor(self, data: dict):
        if data.get("vehicle_reset", False):
            self._vehicle_reset.set()
        vel  = data.get("vel",  [0.0, 0.0, 0.0])
        pos  = data.get("pos",  [0.0, 0.0, 0.0])
        rot  = data.get("rot",  [0.0, 0.0, 0.0, 1.0])
        a    = data.get("imu_accel", [0.0, 0.0, -9.81])
        g    = data.get("imu_gyro",  [0.0, 0.0,  0.0])

        x, y, z, w = float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        bearing = math.degrees(yaw) % 360.0

        with self._lock:
            self._velocity       = vec3(float(vel[0]), float(vel[1]), float(vel[2]))
            self._steering_angle = float(data.get("steering", 0.0))
            self._pos            = (float(pos[0]), float(pos[1]), float(pos[2]))
            self._imu_accel      = vec3(float(a[0]), float(a[1]), float(a[2]))
            self._imu_gyro       = vec3(float(g[0]), float(g[1]), float(g[2]))
            self._bearing        = bearing
            self._state_valid    = True

    def _handle_camera_wide(self, payload: bytes):
        if len(payload) != CAMERA_FRAME_BYTES:
            return
        arr = np.frombuffer(payload, dtype=np.uint8).reshape((CAM_RENDER_H, CAM_RENDER_W, 3))
        with self._wide_lock:
            self._wide_frame_buf = arr.repeat(2, axis=0).repeat(2, axis=1)

    def _handle_camera(self, payload: bytes):
        if len(payload) != CAMERA_FRAME_BYTES:
            return
        arr = np.frombuffer(payload, dtype=np.uint8).reshape((CAM_RENDER_H, CAM_RENDER_W, 3))
        # Pixel-double to full openpilot resolution — fast nearest-neighbour via repeat
        self.road_image[...] = arr.repeat(2, axis=0).repeat(2, axis=1)
        if self.dual_camera:
            with self._wide_lock:
                if self._wide_frame_buf is not None:
                    self.wide_road_image[...] = self._wide_frame_buf
        # Drain any pending signals before releasing so the semaphore never
        # accumulates above 1.  Without this, if frames arrive faster than the
        # consumer (simulated_camera_thread), the semaphore count builds up and
        # the thread later processes a burst of frames in rapid succession,
        # flooding vipc and then going silent — causing modeld to time out.
        # NOTE: image_lock is multiprocessing.Semaphore — its acquire() uses
        # block=False (not blocking=False like threading.Semaphore).
        while self.image_lock.acquire(block=False):
            pass
        self.image_lock.release()

    # -------------------------------------------------------------------------
    # Control output
    # -------------------------------------------------------------------------

    def _send_control(self, msg: bytes):
        try:
            self._sock.sendall(msg)
        except Exception:
            pass

    # -------------------------------------------------------------------------
    # World interface
    # -------------------------------------------------------------------------

    def apply_controls(self, steer_angle_deg: float, throttle: float, brake: float,
                       engaged: bool = False):
        payload = json.dumps({
            "steering": float(steer_angle_deg),
            "throttle": float(max(0.0, min(1.0, throttle))),
            "brake":    float(max(0.0, min(1.0, brake))),
            "engaged":  bool(engaged),
        }).encode()
        msg = _HDR.pack(MSG_CONTROL, len(payload)) + payload
        self._send_control(msg)

    def read_sensors(self, state: SimulatorState):
        with self._lock:
            if not self._state_valid:
                return
            state.velocity          = self._velocity
            state.steering_angle    = self._steering_angle
            state.imu.accelerometer = self._imu_accel
            state.imu.gyroscope     = self._imu_gyro
            state.imu.bearing       = self._bearing
            state.bearing           = self._bearing
            state.gps.from_xy((self._pos[0], self._pos[1]))
            state.valid             = True

    def read_state(self):
        if not self._rx_thread.is_alive():
            self.exit_event.set()

    def read_cameras(self):
        pass  # camera frames arrive via _rx_loop / _handle_camera

    def tick(self):
        pass

    def reset(self):
        pass  # vehicle recovery would need a MSG_RESET command to beamng_setup.py

    def close(self, reason: str):
        print(f"[BeamNGWorld] Closing: {reason}")
        self._io_exit.set()
        try:
            self._sock.close()
        except Exception:
            pass
