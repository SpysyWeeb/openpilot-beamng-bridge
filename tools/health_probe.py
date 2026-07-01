#!/usr/bin/env python3
"""
Standalone CLI for the estimator health monitor.

The bridge starts the same monitor automatically (bridge/health_monitor.py →
logs/health_current.log); this wrapper exists for running it by hand against
an openpilot stack when the bridge isn't up.

    python3 tools/health_probe.py [--log-dir /path/to/logs]
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.expanduser(os.environ.get('OPENPILOT_DIR', '~/openpilot')))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bridge.health_monitor import run

parser = argparse.ArgumentParser()
parser.add_argument('--log-dir', default=os.path.join(os.path.dirname(__file__), '..', 'logs'))
args = parser.parse_args()

log_path = os.path.join(os.path.realpath(args.log_dir), 'health_current.log')
print(f'health probe → {log_path}')
run(log_path, echo=True)
