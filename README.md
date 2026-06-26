# openpilot-beamng-bridge

A bridge that connects [BeamNG.drive](https://www.beamng.com/) to [openpilot](https://github.com/commaai/openpilot) or even forks like [sunnypilot](https://github.com/sunnyhaibin/sunnypilot), allowing BeamNG to serve as a simulation environment for testing openpilot's driving model, lateral control, and longitudinal tuning.

---

## What We're Trying to Accomplish

openpilot has an existing MetaDrive-based simulator, but MetaDrive's road geometry and visual appearance differ significantly from real-world dashcam footage — the very data the openpilot driving model was trained on. BeamNG.drive offers photorealistic road rendering, physically accurate vehicle dynamics, and a rich sensor API, making it a much more representative test environment.

The goal is to:

1. **Feed BeamNG camera frames into openpilot** so the driving model sees something resembling real road footage
2. **Forward IMU, speed, and steering data** from BeamNG to openpilot so the lateral and longitudinal controllers have accurate state feedback
3. **Apply openpilot's control outputs back to BeamNG** so the simulated vehicle steers and accelerates under openpilot's direction
4. **Enable real-world-style tuning validation** — for openpilot and any fork — without requiring a physical test vehicle for every iteration

The longer-term goal is to close the loop completely: BeamNG feeds all sensor data openpilot expects, and openpilot drives the BeamNG vehicle as it would a real car.

---

## Architecture

BeamNG cannot run inside a Linux container, so the setup is split across two environments:

```
Bazzite host (Linux)
└── BeamNG.drive (Steam, via Proton or native)
    └── Exposes TCP control socket on port 64256 (-tcom flag)

openpilot-beamng-bridge distrobox (Ubuntu)
├── openpilot (or a fork) — all openpilot processes
├── bridge/beamng_bridge.py — main control loop
├── bridge/beamng_world.py — BeamNG sensor/control abstraction
├── linux/beamng_setup.py — scenario setup and sensor attachment
└── tools/bridge_gui.py — GTK4 control panel GUI
```

The bridge connects to BeamNG over the local network using [beamngpy](https://github.com/BeamNG/BeamNGpy), polls camera frames and IMU/electrics data, and feeds them into openpilot's cereal messaging bus. openpilot's `carControl` outputs are translated back to BeamNG vehicle controls each frame.

---

## Requirements

### BeamNG.tech License

This bridge **requires a BeamNG.tech license**. This is separate from the consumer BeamNG.drive game available on Steam.

**Why:** BeamNG.tech is the research/developer edition of BeamNG.drive. It enables the external TCP control interface (`-tcom`) that beamngpy uses to:
- Attach virtual sensors (cameras, IMU, electrics) to a vehicle programmatically
- Poll sensor data in real time from an external process
- Send vehicle control commands (steer, throttle, brake) from outside the game

Without a tech license, launching BeamNG with `-tcom` either fails or exposes a restricted API that does not allow vehicle control or sensor attachment. The consumer Steam version alone is not sufficient.

A BeamNG.tech license can be obtained from [BeamNG's website](https://www.beamng.com/beamng-tech/). The underlying game files are the same — the license unlocks the research API.

### Other Requirements

- **openpilot** (or a compatible fork) checked out and built inside the distrobox
- **distrobox** (Ubuntu container on the Bazzite host)
- **beamngpy** installed in the distrobox environment (`pip install beamngpy`)
- **GTK4** available in the distrobox for the control panel GUI

---

## What's Been Built

### Linux Bridge (`linux/` + `bridge/`)
- Connects to BeamNG running on the host from inside a distrobox over localhost TCP
- Streams road camera (and optionally wide camera) frames as NV12 to openpilot's camerad
- Polls AdvancedIMU for accelerometer and gyroscope data
- Reads vehicle electrics for steering angle, wheel speed, and brake state
- Sends vehicle control commands each frame via beamngpy

### GTK4 Control Panel (`tools/bridge_gui.py`)
- Per-component start/stop for openpilot and the BeamNG bridge
- **Controls popover** with:
  - Cruise control buttons (MAIN, CANCEL, SET▼, RES▲) for engaging openpilot
  - Set Speed input — ramps openpilot's cruise speed to a target mph via button presses
  - Driver Override toggle — stops sending openpilot commands to BeamNG so the player can drive; brake auto-cancels cruise
  - Road and wide camera FOV sliders for live tuning (hot-swaps BeamNG cameras without restart)
- MetaDrive mode (`--metadrive` flag / `start_metadrive.sh`) for running the stock openpilot MetaDrive bridge as a comparison baseline

### Manual Input Priority (Option A)
When openpilot is engaged, physical controller inputs (steer/throttle/brake via FIFO) take priority over openpilot's commands. Braking cancels longitudinal cruise while lateral control remains active, mirroring real-world driver override behavior.

### Driver Mode (Option B)
A full driver takeover mode that stops forwarding any openpilot commands to BeamNG, returning full control to the player. Physical braking while in this mode sends a cruise CANCEL to openpilot.

---

## Roadblocks and Known Issues

### 1. Vehicle Dynamics Mismatch
openpilot's sim infrastructure fingerprints the car as a **Honda Civic 2022**. The Honda Civic uses a torque-based lateral controller (`latcontrol_torque`) whose gains (`liveTorqueParameters`) and steer ratio (`liveParameters`) are learned from real Honda Civic driving data. BeamNG's simulated vehicle has completely different steering dynamics, so the controller operates with wrong feedback gains. The car can steer, but it doesn't track lanes accurately until `liveTorqueParameters` and `liveParameters` converge — which requires sustained driving and may never fully match BeamNG's dynamics.

### 2. MetaDrive Also Not Lane-Keeping
The same dynamics mismatch affects the stock openpilot MetaDrive bridge. Additionally, `latcontrol_torque` returns `steeringAngleDeg = 0.0` (hardcoded), so MetaDrive's bridge — which reads `actuators.steeringAngleDeg` — was receiving zero steer signal. Fixed here by converting `actuators.curvature` to a steering angle via the Ackermann formula using the car's actual wheelbase and steer ratio from `carParams`.

### 3. Camera Calibration
`liveCalibration` is pre-seeded with `rpyCalib = [0, 0, 0]` (camera perfectly level, no pitch/yaw offset). This is a reasonable starting point but may not match the actual camera mounting angle in the BeamNG scene. `calibrationd` re-learns this from visual odometry over time.

### 4. STEER_MOTOR_TORQUE Not Provided
The Honda Civic carState parser reads `STEER_MOTOR_TORQUE` from CAN to determine actuator torque feedback. The sim sends this as an empty CAN message, meaning `actuatorsOutput.torque` is always zero. This affects the torque controller's saturation detection and the live torque learning loop.

### 5. Wide Camera FOV
The openpilot wide camera model expects a ~120° horizontal FOV equivalent to an undistorted view from a fisheye lens. BeamNG uses a pinhole camera model with no distortion. The current setup approximates the correct FOV, but the absence of fisheye distortion means the wide camera output may not fully match the model's training distribution.

### 6. BeamNG on Linux via Proton
BeamNG.drive runs on Linux via Steam/Proton. The `-nosteam -tcom -tport 64256` launch flags need to be passed through the Proton launch configuration. See `launch_beamng.sh` and `start.sh` for the working launch setup.

---

## Branches

| Branch | Description |
|--------|-------------|
| `linux` | **Main branch.** Full Linux setup — distrobox bridge, GTK4 GUI, all current features. |
| `windows` | Original Windows prototype — runs entirely on a single Windows machine, earlier architecture. |
