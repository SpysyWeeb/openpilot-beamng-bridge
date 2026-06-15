#!/usr/bin/env python3
"""
WSL worker — started by main.py via 'wsl python bridge_runner.py'.
Runs the BeamNG bridge using the openpilot venv.
"""
import argparse
from multiprocessing import Queue
from bridge.beamng_bridge import BeamNGBridge


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dual-camera", action="store_true")
    args = parser.parse_args()

    bridge = BeamNGBridge(dual_camera=args.dual_camera, high_quality=False)
    q = Queue()
    proc = bridge.run(q)
    proc.join()


if __name__ == "__main__":
    main()
