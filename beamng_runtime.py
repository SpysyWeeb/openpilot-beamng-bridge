#!/usr/bin/env python3
"""
BeamNG runtime — sensor loop, data server, and vehicle control relay.

Calls beamng_setup.setup_beamng() to launch BeamNG and load the scenario,
then runs a 60 Hz loop that:
  - Polls vehicle sensors (Electrics, State, AdvancedIMU)
  - Streams camera frames (background thread)
  - Sends all sensor data to the WSL bridge over TCP
  - Receives translated control commands (steering ±270 °, throttle/brake 0-1)
    from the WSL bridge and relays them to BeamNG via vehicle.control()

The control pump loop calls vehicle.control() repeatedly for the remainder
of each 16.7 ms frame window to prevent BeamNG's physics engine from
resetting ephemeral steering input to zero between frames.
"""

import json
import os
import queue
import socket
import struct
import sys
import threading
import time
import traceback

import numpy as np
from PIL import Image

from beamng_setup import (
    BEAMNG_HOME, CAM_RENDER_W, CAM_RENDER_H, MAX_STEER_DEG, MAX_STEER_RATE_DEG_S,
    setup_beamng,
)

# ---------------------------------------------------------------------------
# Wire protocol  (must match bridge/beamng_world.py)
# ---------------------------------------------------------------------------

DATA_HOST = "0.0.0.0"
DATA_PORT = int(os.environ.get("DATA_PORT", "12321"))

IO_HZ = 60

MSG_SENSOR       = 0x01
MSG_CAMERA       = 0x02
MSG_CONTROL      = 0x03
MSG_CAMERA_WIDE  = 0x04

_HDR = struct.Struct("<BI")   # 1-byte type + 4-byte uint32 length


def _pack(msg_type: int, payload: bytes) -> bytes:
    return _HDR.pack(msg_type, len(payload)) + payload


# ---------------------------------------------------------------------------
# IMU helper
# ---------------------------------------------------------------------------

def _quat_rotate(v: list, q: list) -> list:
    """Rotate 3-vector v by unit quaternion q = [x, y, z, w]."""
    x, y, z, w = q
    t = [
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    ]
    return [
        v[0] + w * t[0] + y * t[2] - z * t[1],
        v[1] + w * t[1] + z * t[0] - x * t[2],
        v[2] + w * t[2] + x * t[1] - y * t[0],
    ]


# ---------------------------------------------------------------------------
# Camera poller
# ---------------------------------------------------------------------------

class CameraPoller:
    """
    Polls BeamNG's camera sensor in a daemon thread so it never stalls the
    60 Hz main loop.  The camera uses the GE socket (independent of the VE
    socket used by Electrics/State/IMU) so concurrent access is safe.
    """

    def __init__(self, camera, name: str = "cam_poll"):
        self._camera = camera
        self._frame: bytes | None = None
        self._lock   = threading.Lock()
        self._errors = 0
        t = threading.Thread(target=self._loop, daemon=True, name=name)
        t.start()

    def _loop(self):
        while True:
            try:
                data = self._camera.poll()
                if data and data.get("colour") is not None:
                    img = data["colour"]
                    if isinstance(img, Image.Image):
                        if img.mode != "RGB":
                            img = img.convert("RGB")
                        raw = img.tobytes()
                    else:
                        raw = np.asarray(img, dtype=np.uint8).tobytes()
                    if len(raw) == CAM_RENDER_W * CAM_RENDER_H * 3:
                        with self._lock:
                            self._frame = raw
                        self._errors = 0
            except Exception as exc:
                self._errors += 1
                if self._errors <= 3 or self._errors % 50 == 0:
                    print(f"[Runtime] Camera error #{self._errors}: {exc}", flush=True)
                time.sleep(0.05)

    def get_frame(self) -> bytes | None:
        with self._lock:
            frame, self._frame = self._frame, None
            return frame


# ---------------------------------------------------------------------------
# Data server
# ---------------------------------------------------------------------------

class DataServer:
    """
    Single-client TCP server.
    Outbound: sensor JSON + raw camera RGB.
    Inbound:  control JSON {"steering": ±270, "throttle": 0-1, "brake": 0-1}.

    Camera frames use a latest-wins slot rather than a queue — if a new frame
    arrives before the previous one was sent, the old one is silently replaced.
    This eliminates multi-second camera lag from frame backlog.
    Sensor messages still go through a small FIFO so they are never silently dropped.
    """

    def __init__(self, host: str, port: int):
        self._host = host
        self._port = port
        self._conn: socket.socket | None = None
        self._conn_lock  = threading.Lock()
        self._tx_queue: queue.Queue = queue.Queue(maxsize=8)  # sensors only
        self._cam_lock   = threading.Lock()
        self._cam_road:  bytes | None = None  # latest road frame (replace-on-arrival)
        self._cam_wide:  bytes | None = None  # latest wide frame (replace-on-arrival)
        self._cam_event  = threading.Event()
        self._control    = {"steering": 0.0, "throttle": 0.0, "brake": 0.0}
        self._ctrl_lock  = threading.Lock()

    def start(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self._host, self._port))
        srv.listen(1)
        print(f"[DataServer] Listening on {self._host}:{self._port}", flush=True)
        threading.Thread(target=self._accept_loop, args=(srv,),
                         daemon=True, name="ds_accept").start()

    def _accept_loop(self, srv: socket.socket):
        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._conn_lock:
                if self._conn is not None:
                    try:
                        conn.close()
                    except Exception:
                        pass
                    print(f"[DataServer] Rejected probe from {addr}", flush=True)
                    continue
                self._conn = conn
            print(f"[DataServer] Bridge connected from {addr}", flush=True)
            threading.Thread(target=self._tx_loop, args=(conn,),
                             daemon=True, name="ds_tx").start()
            threading.Thread(target=self._rx_loop, args=(conn,),
                             daemon=True, name="ds_rx").start()

    def _tx_loop(self, conn: socket.socket):
        try:
            while True:
                # Drain any pending sensor messages first (small, high-priority).
                try:
                    while True:
                        conn.sendall(self._tx_queue.get_nowait())
                except queue.Empty:
                    pass

                # Send latest camera frames if available (latest-wins, no backlog).
                with self._cam_lock:
                    road = self._cam_road
                    wide = self._cam_wide
                    self._cam_road = None
                    self._cam_wide = None

                if road is not None:
                    conn.sendall(road)
                if wide is not None:
                    conn.sendall(wide)

                # Block until a sensor message or camera frame arrives.
                self._cam_event.wait(timeout=0.005)
                self._cam_event.clear()
        except Exception:
            pass
        finally:
            with self._conn_lock:
                if self._conn is conn:
                    self._conn = None

    def _rx_loop(self, conn: socket.socket):
        buf = b""
        hdr = _HDR.size
        try:
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= hdr:
                    msg_type, length = _HDR.unpack(buf[:hdr])
                    if len(buf) < hdr + length:
                        break
                    payload  = buf[hdr:hdr + length]
                    buf      = buf[hdr + length:]
                    if msg_type == MSG_CONTROL:
                        with self._ctrl_lock:
                            self._control = json.loads(payload.decode())
        except Exception:
            pass

    def send_sensors(self, data: dict):
        try:
            self._tx_queue.put_nowait(_pack(MSG_SENSOR, json.dumps(data).encode()))
        except queue.Full:
            pass
        self._cam_event.set()

    def send_camera(self, raw_rgb: bytes):
        with self._cam_lock:
            self._cam_road = _pack(MSG_CAMERA, raw_rgb)
        self._cam_event.set()

    def send_camera_wide(self, raw_rgb: bytes):
        with self._cam_lock:
            self._cam_wide = _pack(MSG_CAMERA_WIDE, raw_rgb)
        self._cam_event.set()

    def get_control(self) -> dict:
        with self._ctrl_lock:
            return dict(self._control)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    bng, vehicle, imu, camera, camera_wide = setup_beamng()

    cam_poller      = CameraPoller(camera, "cam_road")
    cam_wide_poller = CameraPoller(camera_wide, "cam_wide") if camera_wide is not None else None

    srv = DataServer(DATA_HOST, DATA_PORT)
    srv.start()

    print("READY", flush=True)

    # Dedicated steering debug log — one line per frame showing openpilot's
    # request vs BeamNG's measured output side by side.
    _steer_log_path = os.path.join(os.path.dirname(__file__), "steer_debug.log")
    _steer_log = open(_steer_log_path, "w", buffering=1, encoding="utf-8")
    _steer_log.write("time_s | op_cmd_deg | beamng_norm | beamng_elec | echo_sent\n")
    _steer_log.write("-" * 63 + "\n")
    _t_start = time.monotonic()

    dt         = 1.0 / IO_HZ
    loop_errors = 0
    frame_cnt  = 0
    _echo_steer_deg = 0.0
    _prev_steer_cmd = 0.0
    _last_pos       = None
    steer_norm = throttle = brake = 0.0

    try:
        while True:
            t0 = time.monotonic()
            frame_cnt += 1

            try:
                vehicle.sensors.poll()
                elec = vehicle.sensors["electrics"]
                st   = vehicle.sensors["state"]

                vel = st.get("vel",      [0.0, 0.0, 0.0])
                pos = st.get("pos",      [0.0, 0.0, 0.0])
                rot = st.get("rotation", [0.0, 0.0, 0.0, 1.0])

                inv_rot   = [-rot[0], -rot[1], -rot[2], rot[3]]
                imu_accel = _quat_rotate([0.0, 0.0, -9.81], inv_rot)
                imu_gyro  = [0.0, 0.0, 0.0]
                try:
                    r = imu.poll()
                    if isinstance(r, dict) and r:
                        imu_accel = list(r.get("accSmooth", imu_accel))
                        imu_gyro  = list(r.get("angVel",    imu_gyro))
                except Exception:
                    pass

                # Detect vehicle recovery teleport (position jump > 5 m in one frame).
                speed_ms = (float(vel[0])**2 + float(vel[1])**2 + float(vel[2])**2) ** 0.5

                vehicle_reset = False
                if _last_pos is not None:
                    dx = float(pos[0]) - _last_pos[0]
                    dy = float(pos[1]) - _last_pos[1]
                    dz = float(pos[2]) - _last_pos[2]
                    if dx*dx + dy*dy + dz*dz > 25.0:
                        vehicle_reset = True
                        _echo_steer_deg = 0.0
                        _prev_steer_cmd = 0.0
                        print("[Runtime] Vehicle reset detected (position jump).", flush=True)
                _last_pos = (float(pos[0]), float(pos[1]), float(pos[2]))

                srv.send_sensors({
                    "steering":      _echo_steer_deg,
                    "vel":           [float(v) for v in vel],
                    "pos":           [float(v) for v in pos],
                    "rot":           [float(v) for v in rot],
                    "imu_accel":     imu_accel,
                    "imu_gyro":      imu_gyro,
                    "vehicle_reset": vehicle_reset,
                })

                frame = cam_poller.get_frame()
                if frame is not None:
                    srv.send_camera(frame)

                if cam_wide_poller is not None:
                    wide_frame = cam_wide_poller.get_frame()
                    if wide_frame is not None:
                        srv.send_camera_wide(wide_frame)

                # Receive translated control from the WSL bridge.
                # steering is in openpilot degrees (±270); normalise to BeamNG's ±1.
                ctrl       = srv.get_control()
                throttle   = max(0.0, min(1.0,  ctrl["throttle"]))
                brake      = max(0.0, min(1.0,  ctrl["brake"]))
                engaged    = bool(ctrl.get("engaged", False))
                _max_delta = MAX_STEER_RATE_DEG_S / IO_HZ
                _raw_cmd = ctrl["steering"]
                _limited_cmd = max(_prev_steer_cmd - _max_delta,
                                   min(_prev_steer_cmd + _max_delta, _raw_cmd))
                _prev_steer_cmd = _limited_cmd
                steer_norm = max(-1.0, min(1.0, -_limited_cmd / MAX_STEER_DEG))
                _echo_steer_deg = float(elec.get("steering", 0.0)) * MAX_STEER_DEG / 495.0

                # One line per frame: OP request vs BeamNG output
                _steer_log.write(
                    f"{time.monotonic() - _t_start:7.2f} | "
                    f"{ctrl['steering']:+8.2f}   | "
                    f"{steer_norm:+7.3f}     | "
                    f"{float(elec.get('steering', 0)):+7.3f}     | "
                    f"{_echo_steer_deg:+8.2f}\n"
                )

                if frame_cnt % IO_HZ == 1:
                    print(
                        f"[Runtime #{frame_cnt // IO_HZ}] "
                        f"cmd_steer_norm={steer_norm:.3f} cmd_deg={ctrl['steering']:.1f} | "
                        f"echo_deg={_echo_steer_deg:.1f} | "
                        f"thr={throttle:.3f} brk={brake:.3f} "
                        f"speed={speed_ms * 3.6:.1f} kph",
                        flush=True,
                    )

            except Exception as exc:
                loop_errors += 1
                if loop_errors <= 5 or loop_errors % 100 == 0:
                    print(f"[Runtime] Loop error #{loop_errors}: {type(exc).__name__}: {exc}",
                          flush=True)
                    traceback.print_exc(file=sys.stdout)
                    sys.stdout.flush()

            # When engaged: pump vehicle.control() for the rest of the frame window.
            # BeamNG resets ephemeral inputs every physics step (~15 ms); a single call
            # per sensor frame leaves gaps where the wheel snaps to center.
            # When not engaged: do nothing — let BeamNG receive native input (keyboard /
            # controller) without interference. Calling vehicle.control() even once would
            # override native input for that physics tick.
            frame_end = t0 + dt
            _pump_calls = 0
            if engaged:
                while True:
                    try:
                        vehicle.control(
                            steering=steer_norm,
                            throttle=throttle,
                            brake=brake,
                            parkingbrake=0,
                        )
                        _pump_calls += 1
                    except Exception:
                        pass
                    if time.monotonic() >= frame_end:
                        break
            else:
                remaining = frame_end - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
            if frame_cnt % IO_HZ == 1:
                print(f"[Runtime]   pump_calls_last_frame={_pump_calls}", flush=True)

    except BaseException as fatal:
        print(f"\n[Runtime] FATAL {type(fatal).__name__}: {fatal}", flush=True)
        traceback.print_exc(file=sys.stdout)
        sys.stdout.flush()
        raise
    finally:
        try:
            _steer_log.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
