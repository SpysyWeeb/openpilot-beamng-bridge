#!/usr/bin/env python3
"""Quick connectivity test — verifies beamngpy can reach an already-running BeamNG.

Run from inside the distrobox with BeamNG already launched on the host
(launch_beamng.sh / the control panel's BeamNG component).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from linux.beamng_setup import BEAMNG_HOME, BEAMNG_HOST, BEAMNG_PORT, BEAMNG_USER
from beamngpy import BeamNGpy


def main():
    print(f"Connecting to BeamNG at {BEAMNG_HOST}:{BEAMNG_PORT}")
    print(f"  home: {BEAMNG_HOME}")
    print(f"  user: {BEAMNG_USER}")

    bng = BeamNGpy(BEAMNG_HOST, BEAMNG_PORT, home=BEAMNG_HOME, user=BEAMNG_USER)

    try:
        print("Connecting to already-running BeamNG (launch=False)...")
        bng.open(launch=False)
        print("\n✓ Connected to BeamNG successfully!")

        info = bng.get_system_info() if hasattr(bng, 'get_system_info') else {}
        if info:
            print(f"  BeamNG version: {info.get('version', 'unknown')}")

        bng.close()
        print("✓ Connection closed cleanly.")
    except Exception as e:
        print(f"\n✗ Connection failed: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
