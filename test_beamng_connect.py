#!/usr/bin/env python3
"""Quick connectivity test — launches BeamNG and verifies beamngpy can connect."""
import sys
from beamngpy import BeamNGpy

BEAMNG_HOME = "/home/alex/.local/share/Steam/steamapps/common/BeamNG.drive"
BEAMNG_USER = "/home/alex/.local/share/BeamNG/BeamNG.drive/current"
BEAMNG_PORT = 64256

def main():
    print(f"Connecting to BeamNG at localhost:{BEAMNG_PORT}")
    print(f"  home: {BEAMNG_HOME}")
    print(f"  user: {BEAMNG_USER}")

    bng = BeamNGpy('localhost', BEAMNG_PORT, home=BEAMNG_HOME, user=BEAMNG_USER)

    print(f"\nLaunch command would be:\n  {bng.get_launch_arguments()}\n")

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
