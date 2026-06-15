import functools
import time
import numpy as np

from multiprocessing import Queue

from opendbc.car.honda.values import CruiseButtons
from openpilot.common.params import Params
from openpilot.tools.sim.bridge.common import (
    SimulatorBridge, QueueMessageType, rk_loop,
)
from openpilot.tools.sim.lib.common import World
from openpilot.tools.sim.lib.simulated_car import SimulatedCar
from openpilot.tools.sim.lib.simulated_sensors import SimulatedSensors

from bridge.beamng_world import BeamNGWorld



class BeamNGBridge(SimulatorBridge):
    """
    SimulatorBridge subclass for BeamNG.Drive.

    Overrides _run() to handle BeamNG-specific control translation:
      - Toyota Corolla TSS2 uses angle-based lateral control (actuators.steeringAngleDeg).
      - Pulses RES_ACCEL after auto-engage to ramp cruise speed from 0.
      - Detects vehicle recovery teleports and re-arms auto-engage.

    openpilot's common.py is left completely stock.
    """

    def spawn_world(self, q: Queue) -> World:
        return BeamNGWorld(q, dual_camera=self.dual_camera)

    def _run(self, q: Queue):
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

        # Local state — kept here rather than on self so common.py stays unmodified.
        post_engage_cnt = 0
        pending_reengage = False

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
                    elif m[0] == "blinker":
                        if m[1] == "left":
                            self.simulator_state.left_blinker = True
                        elif m[1] == "right":
                            self.simulator_state.right_blinker = True
                    elif m[0] == "ignition":
                        self.simulator_state.ignition = not self.simulator_state.ignition
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

                # Honda Civic 2022 is torque-controlled. Scale ±1 torque to ±495 so the
                # runtime's existing /MAX_STEER_DEG normalization cancels it back to ±1.
                steer_op = act.torque * 495.0

                self.past_startup_engaged = True

                # Pulse RES_ACCEL to ramp cruise speed up from 0 kph after first engage.
                if post_engage_cnt < 120:
                    post_engage_cnt += 1
                    if post_engage_cnt % 12 == 1:
                        self.simulator_state.cruise_button = CruiseButtons.RES_ACCEL

            elif not self.past_startup_engaged and self.simulated_car.sm['selfdriveState'].engageable:
                self.simulator_state.cruise_button = (
                    CruiseButtons.DECEL_SET if self.startup_button_prev else CruiseButtons.MAIN
                )
                self.startup_button_prev = not self.startup_button_prev

            throttle_out = throttle_op if self.simulator_state.is_engaged else throttle_manual
            brake_out    = brake_op    if self.simulator_state.is_engaged else brake_manual
            steer_out    = steer_op    if self.simulator_state.is_engaged else steer_manual

            if self.rk.frame % 100 == 0:
                act = self.simulated_car.sm['carControl'].actuators
                print(
                    f"[BRG] eng={self.simulator_state.is_engaged} "
                    f"steer_out={steer_out:.2f} "
                    f"(angDeg={act.steeringAngleDeg:.2f} torque={act.torque:.3f} curv={act.curvature:.4f}) "
                    f"thr={throttle_out:.3f} brk={brake_out:.3f}",
                    flush=True,
                )

            self.world.apply_controls(steer_out, throttle_out, brake_out,
                                       engaged=self.simulator_state.is_engaged)
            self.world.read_state()
            self.world.read_sensors(self.simulator_state)

            # Vehicle recovery teleport — clear excessive-actuation flag and re-arm engage.
            if hasattr(self.world, 'consume_vehicle_reset') and self.world.consume_vehicle_reset():
                print('[BRG] Vehicle reset — sending CANCEL, re-arming engage once disengaged', flush=True)
                Params().remove('Offroad_ExcessiveActuation')
                self.simulator_state.cruise_button = CruiseButtons.CANCEL
                pending_reengage = True

            # Wait until is_engaged actually drops before clearing past_startup_engaged,
            # otherwise the 1-2 frame lag before CANCEL is processed re-sets the flag.
            if pending_reengage and not self.simulator_state.is_engaged:
                self.past_startup_engaged = False
                self.startup_button_prev = True
                post_engage_cnt = 0
                pending_reengage = False
                print('[BRG] Auto-engage re-armed after vehicle reset', flush=True)

            if self.world.exit_event.is_set():
                self.shutdown()

            if self.rk.frame % self.TICKS_PER_FRAME == 0:
                self.world.tick()
                self.world.read_cameras()

            if not self.test_run and self.rk.frame % 25 == 0:
                self.print_status()

            self.started.value = True

            self.rk.keep_time()
