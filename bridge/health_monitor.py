"""
In-process estimator health monitor.

Runs as a daemon thread inside the bridge (started by linux/bridge_runner.py)
and writes one line per second to logs/health_current.log — no manual step
needed during test drives.

Records the things selfdrived's commIssue events cannot show:
  - livePose health fields (inputsOK / sensorsOK / posenetOK) that torqued,
    lagd and paramsd gate their own validity on
  - measured publish rates of the estimator chain, because each daemon's
    SubMaster also fails freq_ok checks when an upstream rate dips (modeld's
    CPU-compiled model competes with BeamNG for the same cores)
  - set speed / accel target / lead so longitudinal decisions are explainable
"""
import os
import threading
import time
from datetime import datetime

SERVICES = ['livePose', 'liveCalibration', 'liveDelay', 'liveParameters',
            'liveTorqueParameters', 'selfdriveState', 'carState',
            'longitudinalPlan', 'radarState', 'modelV2', 'cameraOdometry']

# nominal Hz, for the rate report
RATED = ['carState', 'modelV2', 'cameraOdometry', 'livePose',
         'liveCalibration', 'liveDelay', 'liveParameters', 'liveTorqueParameters']


def run(log_path: str, echo: bool = False) -> None:
    """Blocking monitor loop. Set echo=True to also print lines to stdout."""
    try:
        import openpilot.cereal.messaging as messaging   # nested layout (new openpilot master)
    except ImportError:
        import cereal.messaging as messaging             # legacy layout (older trees / forks)

    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    log_file = open(log_path, 'w', buffering=1)

    def emit(msg: str):
        line = f'[{datetime.now().strftime("%H:%M:%S")}] {msg}'
        log_file.write(line + '\n')
        if echo:
            print(line, flush=True)

    emit('health monitor started')
    sm = messaging.SubMaster(SERVICES)
    counts = dict.fromkeys(SERVICES, 0)
    window_start = time.monotonic()

    while True:
        sm.update(100)
        for s in SERVICES:
            if sm.updated[s]:
                counts[s] += 1

        now = time.monotonic()
        dt = now - window_start
        if dt < 1.0:
            continue

        flags = []
        for s in SERVICES:
            if not sm.alive[s]:
                flags.append(f'{s}=A')
            elif not sm.valid[s]:
                flags.append(f'{s}=V')
        bad = ' '.join(flags) if flags else 'all-ok'

        lp = sm['livePose']
        cal = sm['liveCalibration']
        sd = sm['selfdriveState']
        car = sm['carState']
        plan = sm['longitudinalPlan']
        lead = sm['radarState'].leadOne
        rates = ' '.join(f'{s}={counts[s] / dt:.1f}' for s in RATED)

        emit(
            f'{bad:<52s} | pose: in={int(lp.inputsOK)} sen={int(lp.sensorsOK)} '
            f'net={int(lp.posenetOK)} av={int(lp.angularVelocityDevice.valid)} '
            f'| cal {cal.calPerc}% | eng={int(sd.enabled)}/{int(sd.active)} '
            f'vEgo={car.vEgo:.1f} vCruise={car.vCruise:.0f} aT={plan.aTarget:+.2f} '
            f'lead={int(bool(lead.status))}@{lead.dRel:.0f}m '
            f'| Hz: {rates}'
        )
        counts = dict.fromkeys(SERVICES, 0)
        window_start = now


def start(log_path: str) -> threading.Thread:
    """Start the monitor as a daemon thread (used by the bridge)."""
    t = threading.Thread(target=run, args=(log_path,), daemon=True,
                         name='health_monitor')
    t.start()
    return t
