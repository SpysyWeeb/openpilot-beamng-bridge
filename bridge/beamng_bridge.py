import functools
import os
import time
import numpy as np

ENGAGED_FILE = '/tmp/beamng_bridge_engaged'

from multiprocessing import Queue

from opendbc.car.honda.values import CruiseButtons
from openpilot.common.params import Params
from openpilot.tools.sim.bridge.common import (
    SimulatorBridge, QueueMessageType, rk_loop,
)
from openpilot.tools.sim.lib.common import World
from openpilot.tools.sim.lib.simulated_car import SimulatedCar
from openpilot.tools.sim.lib.simulated_sensors import SimulatedSensors

from bridge.beamng_world import MAX_STEER_DEG


class BeamNGBridge(SimulatorBridge):
    """
    SimulatorBridge subclass for BeamNG.drive.

    Overrides _run() to handle BeamNG-specific control translation:
      - Honda Civic 2022 fingerprint → torque-based lateral control; actuator
        torque (±1) is scaled to a ±MAX_STEER_DEG wheel angle for the world.
      - cruise_speed_<mph> FIFO commands ramp the set speed via one RES/SET
        button press per frame.
      - Detects vehicle recovery teleports, clears the excessive-actuation
        fault, and cancels cruise so the driver can re-engage.

    openpilot's common.py is left completely stock.
    """

    def spawn_world(self, q: Queue) -> World:
        raise NotImplementedError(
            'BeamNGWorld needs live beamngpy handles and must be built '
            'in-process — see linux/bridge_runner.py')

    def _run_with_world(self, q: Queue):
        """Like _run() but uses self.world as already set (no spawn_world call).
        Use this on Linux where beamngpy sockets can't survive a subprocess fork."""
        self._run(q)

    def _run(self, q: Queue):
        if self.world is None:
            self.world = self.spawn_world(q)

        self.simulated_car = SimulatedCar()
        self.simulated_sensors = SimulatedSensors(self.dual_camera)

        import threading
        self._exit_event = threading.Event()

        self.simulated_car_thread = threading.Thread(
            target=rk_loop,
            args=(functools.partial(self.simulated_car.update, self.simulator_state),
                  100, self._exit_event),
        )
        self.simulated_car_thread.start()

        def _camera_loop():
            # Process each frame immediately as it arrives rather than on a fixed
            # 20 Hz tick.  send_camera_images() blocks on image_lock.acquire() until
            # _handle_camera() signals a new frame, so this loop runs exactly once
            # per received frame with no artificial inter-frame sleep.
            while not self._exit_event.is_set():
                try:
                    self.simulated_sensors.send_camera_images(self.world)
                except Exception:
                    time.sleep(0.01)

        self.simulated_camera_thread = threading.Thread(
            target=_camera_loop, daemon=True, name="cam_send",
        )
        self.simulated_camera_thread.start()

        for _ in range(20):
            self.world.tick()

        # Pre-populate simulator_state with real sensor data so the first
        # send_imu_message() doesn't emit zeros (which triggers sensorDataInvalid).
        self.world.read_sensors(self.simulator_state)

        # Local state — kept here rather than on self so common.py stays unmodified.
        pending_reengage   = False
        _controls_active   = False   # whether we drove BeamNG last frame
        _last_watchdog     = time.monotonic()
        _driver_mode       = False   # Option B: stop sending controls, let player drive in BeamNG
        _pending_spd_presses = 0     # remaining cruise button presses for cruise_speed_X
        _pending_spd_btn   = None    # CruiseButtons value for speed adjustment

        while self._keep_alive:
            throttle_out = steer_out = brake_out = 0.0
            throttle_op = steer_op = brake_op = 0.0

            self.simulator_state.cruise_button = 0
            self.simulator_state.left_blinker = False
            self.simulator_state.right_blinker = False

            throttle_manual = steer_manual = brake_manual = 0.

            if not q.empty():
                message = q.get()
                if message.type == QueueMessageType.CONTROL_COMMAND:
                    m = message.info.split('_')
                    if m[0] == "steer":
                        steer_manual = float(m[1])
                    elif m[0] == "throttle":
                        throttle_manual = float(m[1])
                    elif m[0] == "brake":
                        brake_manual = float(m[1])
                    elif m[0] == "cruise":
                        if m[1] == "down":
                            self.simulator_state.cruise_button = CruiseButtons.DECEL_SET
                        elif m[1] == "up":
                            self.simulator_state.cruise_button = CruiseButtons.RES_ACCEL
                        elif m[1] == "cancel":
                            self.simulator_state.cruise_button = CruiseButtons.CANCEL
                        elif m[1] == "main":
                            self.simulator_state.cruise_button = CruiseButtons.MAIN
                        elif m[1] == "speed" and len(m) >= 3:
                            # cruise_speed_45 → ramp cruise to 45 mph via delta button presses
                            try:
                                target_mph = float(m[2])
                                v_cruise_kph = self.simulated_car.sm['controlsState'].vCruise
                                v_cruise_mph = v_cruise_kph / 1.609
                                delta = int(round(target_mph - v_cruise_mph))
                                if delta != 0:
                                    _pending_spd_presses = abs(delta)
                                    _pending_spd_btn = (CruiseButtons.RES_ACCEL if delta > 0
                                                        else CruiseButtons.DECEL_SET)
                            except (ValueError, IndexError):
                                pass
                    elif m[0] == "driver":
                        if len(m) >= 3 and m[1] == "mode":
                            _driver_mode = (m[2] == "on")
                            print(f'[BRG] driver_mode={_driver_mode}', flush=True)
                    elif m[0] == "blinker":
                        if m[1] == "left":
                            self.simulator_state.left_blinker = True
                        elif m[1] == "right":
                            self.simulator_state.right_blinker = True
                    elif m[0] == "ignition":
                        self.simulator_state.ignition = not self.simulator_state.ignition
                    elif m[0] == "fov" and len(m) >= 3:
                        try:
                            new_fov = float(m[2])
                            if m[1] == "road":
                                self.world.set_camera_fov(road_fov=new_fov)
                            elif m[1] == "wide":
                                self.world.set_camera_fov(wide_fov=new_fov)
                        except (ValueError, AttributeError):
                            pass
                    elif m[0] == "reset":
                        self.world.reset()
                    elif m[0] == "quit":
                        break

            self.simulator_state.user_brake = brake_manual
            self.simulator_state.user_gas = throttle_manual
            self.simulator_state.user_torque = steer_manual * -10000

            steer_manual = steer_manual * -40

            self.simulated_sensors.update(self.simulator_state, self.world)

            self.simulated_car.sm.update(0)
            self.simulator_state.is_engaged = self.simulated_car.sm['selfdriveState'].active

            if self.simulator_state.is_engaged:
                act = self.simulated_car.sm['carControl'].actuators
                throttle_op = np.clip(act.accel / 1.6, 0.0, 1.0)
                brake_op    = np.clip(-act.accel / 4.0, 0.0, 1.0)

                # Honda Civic 2022 is torque-controlled. Scale ±1 torque to a wheel
                # angle; beamng_world.apply_controls() divides by MAX_STEER_DEG,
                # cancelling it back to BeamNG's ±1 steering input.
                steer_op = act.torque * MAX_STEER_DEG

            # ── Cruise speed ramp (one press per frame when pending) ───────────
            # Only fire if no other button event claimed this frame.
            if _pending_spd_presses > 0 and self.simulator_state.cruise_button == 0:
                self.simulator_state.cruise_button = _pending_spd_btn
                _pending_spd_presses -= 1

            # ── Control priority ──────────────────────────────────────────────
            # Manual FIFO inputs (Option A) are primary over openpilot when engaged.
            # Brake overrides openpilot and cancels cruise (longitudinal) while MADS
            # lateral stays active in sunnypilot.
            if self.simulator_state.is_engaged:
                steer_out    = steer_manual    if steer_manual    != 0 else steer_op
                throttle_out = throttle_manual if throttle_manual != 0 else throttle_op
                if brake_manual > 0:
                    brake_out = brake_manual
                    if self.simulator_state.cruise_button == 0:
                        self.simulator_state.cruise_button = CruiseButtons.CANCEL
                else:
                    brake_out = brake_op
            else:
                steer_out    = steer_manual
                throttle_out = throttle_manual
                brake_out    = brake_manual

            if self.rk.frame % 100 == 0:
                act = self.simulated_car.sm['carControl'].actuators
                dm = ' [DRIVER]' if _driver_mode else ''
                print(
                    f"[BRG]{dm} eng={self.simulator_state.is_engaged} "
                    f"steer_out={steer_out:.2f} beamng_steer_deg={self.simulator_state.steering_angle:.1f} "
                    f"(angDeg={act.steeringAngleDeg:.2f} torque={act.torque:.3f} curv={act.curvature:.4f}) "
                    f"thr={throttle_out:.3f} brk={brake_out:.3f}",
                    flush=True,
                )

            # ── Option B: driver mode ─────────────────────────────────────────
            # When active, stop sending controls so BeamNG returns to player input.
            # Watch electrics.brake to detect player braking and cancel cruise.
            if _driver_mode:
                player_brake = getattr(self.world, 'player_brake', 0.0)
                if player_brake > 0.05 and self.simulator_state.is_engaged:
                    if self.simulator_state.cruise_button == 0:
                        self.simulator_state.cruise_button = CruiseButtons.CANCEL
            else:
                # Only drive BeamNG while openpilot is engaged or a manual FIFO
                # input is active.  Sending zeros every frame overrides the
                # player's native BeamNG inputs, which made hand-driving feel
                # like controls were being dropped.
                _want_control = (self.simulator_state.is_engaged
                                 or steer_manual != 0 or throttle_manual != 0
                                 or brake_manual != 0)
                if _want_control:
                    self.world.apply_controls(steer_out, throttle_out, brake_out,
                                              engaged=self.simulator_state.is_engaged)
                    _controls_active = True
                elif _controls_active:
                    # One neutral command on release so throttle/steer don't stick.
                    self.world.apply_controls(0.0, 0.0, 0.0, engaged=False)
                    _controls_active = False

            self.world.read_state()
            self.world.read_sensors(self.simulator_state)

            _now = time.monotonic()
            if _now - _last_watchdog >= 5.0:
                print(f'[BRG] heartbeat frame={self.rk.frame} eng={self.simulator_state.is_engaged} '
                      f'driver_mode={_driver_mode} sensor_thread_alive={self.world._sensor_thread.is_alive()}',
                      flush=True)
                _last_watchdog = _now

            # Vehicle recovery teleport — clear excessive-actuation flag.
            if hasattr(self.world, 'consume_vehicle_reset') and self.world.consume_vehicle_reset():
                print('[BRG] Vehicle reset — sending CANCEL; re-engage manually via Controls panel.',
                      flush=True)
                Params().remove('Offroad_ExcessiveActuation')
                self.simulator_state.cruise_button = CruiseButtons.CANCEL
                pending_reengage = True

            if pending_reengage and not self.simulator_state.is_engaged:
                pending_reengage = False
                print('[BRG] Disengaged after reset — use Controls panel to re-engage.', flush=True)

            if self.rk.frame % 10 == 0:
                try:
                    with open(ENGAGED_FILE, 'w') as _ef:
                        _ef.write('1' if self.simulator_state.is_engaged else '0')
                except Exception:
                    pass

            if self.world.exit_event.is_set():
                self.shutdown()

            if self.rk.frame % self.TICKS_PER_FRAME == 0:
                self.world.tick()
                self.world.read_cameras()

            if not self.test_run and self.rk.frame % 25 == 0:
                self.print_status()

            self.started.value = True

            self.rk.keep_time()
