#!/usr/bin/env python3
"""
IMU message-rate monitor. Started automatically by start.sh.

Waits for the bridge to create the accelerometer/gyroscope shared-memory
publishers (which only exist after BeamNG has loaded and the bridge is
running), then monitors message rate and logs everything to a timestamped
file in logs/.

Usage (manual):
  python3 tools/monitor_imu.py [--log-dir /path/to/logs]
"""
import argparse
import os
import sys
import time
from datetime import datetime

sys.path.insert(0, '/home/alex/sunnypilot')
import cereal.messaging as messaging

parser = argparse.ArgumentParser()
parser.add_argument('--log-dir', default=os.path.join(os.path.dirname(__file__), '..', 'logs'))
args = parser.parse_args()

log_dir = os.path.realpath(args.log_dir)
os.makedirs(log_dir, exist_ok=True)
log_path = os.path.join(log_dir, f'monitor_imu_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
log_file = open(log_path, 'w', buffering=1)  # line-buffered so tail -f works

def emit(msg: str):
    line = f'[{datetime.now().strftime("%H:%M:%S")}] {msg}'
    print(line, flush=True)
    log_file.write(line + '\n')

emit(f'IMU monitor started  (log → {log_path})')
emit('Waiting for bridge to come up...')

# ── Wait for publishers ────────────────────────────────────────────────────
# The bridge only creates accelerometer/gyroscope shared-memory regions
# after BeamNG has loaded (can take 60–120 s). Retry until it works.
sm = None
while sm is None:
    try:
        sm = messaging.SubMaster(['accelerometer', 'gyroscope'])
        sm.update(100)   # one poll to confirm sockets are live
        emit('Publishers found — monitoring started.')
    except Exception as exc:
        sm = None
        emit(f'Not ready yet ({type(exc).__name__}: {exc}) — retrying in 3 s...')
        time.sleep(3)

# ── Main monitoring loop ───────────────────────────────────────────────────
accel_n = gyro_n = 0
last_accel = last_gyro = time.monotonic()
last_print  = time.monotonic()
warned      = False

try:
    while True:
        try:
            sm.update(50)
        except Exception as exc:
            emit(f'SubMaster error ({type(exc).__name__}: {exc}) — reconnecting...')
            time.sleep(1)
            try:
                sm = messaging.SubMaster(['accelerometer', 'gyroscope'])
            except Exception:
                pass
            continue

        now = time.monotonic()

        if sm.updated['accelerometer']:
            accel_n   += 1
            last_accel = now
            warned     = False
        if sm.updated['gyroscope']:
            gyro_n   += 1
            last_gyro = now

        accel_gap = now - last_accel
        gyro_gap  = now - last_gyro

        if (accel_gap > 0.5 or gyro_gap > 0.5) and not warned:
            emit(f'!! SENSOR GAP !!  accel silent {accel_gap:.2f}s  gyro silent {gyro_gap:.2f}s')
            warned = True

        if now - last_print >= 1.0:
            v = sm['accelerometer'].acceleration.v if accel_n else [0, 0, 0]
            emit(
                f'accel {accel_n:4d}/s  gyro {gyro_n:4d}/s  '
                f'last_accel=[{v[0]:6.2f},{v[1]:6.2f},{v[2]:6.2f}]  '
                f'gap accel={accel_gap:.2f}s gyro={gyro_gap:.2f}s'
            )
            accel_n = gyro_n = 0
            last_print = now

finally:
    emit('Monitor exiting.')
    log_file.close()
