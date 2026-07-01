#!/usr/bin/env python3
"""
Estimator-health probe — pinpoints what is tripping selfdrived's commIssue.

Logs, once per second, the valid/alive flags of the estimator chain plus the
livePose health fields (inputsOK / sensorsOK / posenetOK) that torqued, lagd
and paramsd gate their own validity on.  Text logs only show *that* services
went invalid; this shows *why*.

Run inside the distrobox while the stack is up:
    python3 tools/health_probe.py [--log-dir /path/to/logs]
"""
import argparse
import os
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.expanduser(os.environ.get('OPENPILOT_DIR', '~/openpilot')))
try:
    import openpilot.cereal.messaging as messaging   # nested layout (new openpilot master)
except ImportError:
    import cereal.messaging as messaging             # legacy layout (older trees / forks)

SERVICES = ['livePose', 'liveCalibration', 'liveDelay', 'liveParameters',
            'liveTorqueParameters', 'selfdriveState', 'carState']

parser = argparse.ArgumentParser()
parser.add_argument('--log-dir', default=os.path.join(os.path.dirname(__file__), '..', 'logs'))
args = parser.parse_args()

log_dir = os.path.realpath(args.log_dir)
os.makedirs(log_dir, exist_ok=True)
log_path = os.path.join(log_dir, 'health_current.log')
log_file = open(log_path, 'w', buffering=1)  # line-buffered so tail -f works


def emit(msg: str):
    line = f'[{datetime.now().strftime("%H:%M:%S")}] {msg}'
    print(line, flush=True)
    log_file.write(line + '\n')


emit(f'health probe started (log → {log_path})')
sm = messaging.SubMaster(SERVICES)
last_print = 0.0

while True:
    sm.update(100)
    now = time.monotonic()
    if now - last_print < 1.0:
        continue
    last_print = now

    # per-service alive/valid: '.' = ok, 'A' = not alive, 'V' = not valid
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
    emit(
        f'{bad:<60s} | livePose: inputsOK={bool(lp.inputsOK)} sensorsOK={bool(lp.sensorsOK)} '
        f'posenetOK={bool(lp.posenetOK)} angVelValid={bool(lp.angularVelocityDevice.valid)} '
        f'| cal: status={cal.calStatus} perc={cal.calPerc}% '
        f'| enabled={bool(sd.enabled)} active={bool(sd.active)} vEgo={sm["carState"].vEgo:.1f}'
    )
