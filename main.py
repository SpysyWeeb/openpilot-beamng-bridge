#!/usr/bin/env python3
"""
openpilot <-> BeamNG.Drive Bridge — Master Launcher

Run this from PowerShell in the project directory:
    python main.py

Starts in order:
  1. beamng_setup.py (Windows Python) — launches BeamNG via beamngpy, loads the
     scenario, spawns the vehicle, polls sensors, and runs a TCP data server on
     port 12321 so the WSL bridge can receive sensor frames and send controls.
  2. openpilot in sim mode (WSL subprocess)
  3. Bridge (WSL) — connects to beamng_setup.py's data server (not to BeamNG
     directly), feeds sensor data into openpilot, and relays control commands.

Then shows a live status display until Ctrl+C.

Flags:
    --skip-beamng      BeamNG + scenario already running — skip step 1
    --skip-openpilot   openpilot already running — skip step 2
    --dual-camera      Enable wide road camera in the bridge

Requires (Windows Python):
    pip install beamngpy pillow numpy
"""

import argparse
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque

# Enable ANSI escape codes in Windows console (virtualised terminal processing).
# Without this, colour/dim codes show as literal characters like ←[2m in PowerShell.
if sys.platform == "win32":
    try:
        import ctypes
        kernel32 = ctypes.windll.kernel32
        # GetStdHandle(-11) = stdout; OR in ENABLE_VIRTUAL_TERMINAL_PROCESSING (0x0004)
        handle = kernel32.GetStdHandle(-11)
        mode = ctypes.c_ulong()
        kernel32.GetConsoleMode(handle, ctypes.byref(mode))
        kernel32.SetConsoleMode(handle, mode.value | 0x0004)
    except Exception:
        pass

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# BeamNG tech server (used only by beamng_setup.py; bridge does not connect here)
BEAMNG_HOST = os.environ.get("BEAMNG_HOST", "localhost")
BEAMNG_PORT = int(os.environ.get("BEAMNG_PORT", "64256"))

# Data server (beamng_setup.py → bridge); WSL2 localhost reaches Windows host
DATA_PORT = int(os.environ.get("DATA_PORT", "12321"))

# WSL-side paths (these are Linux paths, used inside wsl commands)
WSL_OPENPILOT_DIR  = "~/sunnypilot"
WSL_VENV_PYTHON    = "~/sunnypilot/.venv/bin/python"
WSL_LAUNCH_OP      = "~/sunnypilot/tools/sim/launch_openpilot.sh"
# Bridge project root (Windows path — converted to WSL in win_to_wsl())
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))

# beamng_setup.py calls bng.open(launch=True) which internally waits for BeamNG to boot.
# Give it generous time since BeamNG can take 60–90 s on first load.
SETUP_READY_TIMEOUT  = 180   # seconds waiting for beamng_setup.py READY signal
OP_STARTUP_WAIT      = 8     # seconds for openpilot to initialise

# ANSI colours (work in Windows Terminal / PowerShell 7)
_GRN = "\033[92m"
_RED = "\033[91m"
_YLW = "\033[93m"
_DIM = "\033[2m"
_RST = "\033[0m"
_CLR = "\033[2J\033[H"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _dot(ok) -> str:
    if ok is None:
        return f"{_YLW}●{_RST}"
    return f"{_GRN}●{_RST}" if ok else f"{_RED}●{_RST}"


def tcp_reachable(host: str, port: int, timeout: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def win_to_wsl(win_path: str) -> str:
    """Convert a Windows absolute path to its /mnt/... WSL equivalent."""
    drive, rest = os.path.splitdrive(win_path)
    letter = drive.rstrip(":").lower()
    posix = rest.replace("\\", "/")
    return f"/mnt/{letter}{posix}"


# ---------------------------------------------------------------------------
# Process launchers
# ---------------------------------------------------------------------------

def launch_beamng_setup(dual_camera: bool) -> tuple:
    """
    Run beamng_runtime.py (Windows Python) which launches BeamNG, loads the
    scenario, then starts the 60 Hz sensor/control loop and TCP data server.
    Blocks until the script prints 'READY', then returns (proc, log_capture).
    """
    script = os.path.join(_THIS_DIR, "beamng_runtime.py")
    if not os.path.exists(script):
        print(f"{_RED}[Setup]{_RST} beamng_runtime.py not found at {script}")
        return None, None

    env = os.environ.copy()
    env["DUAL_CAMERA"] = "1" if dual_camera else "0"

    try:
        proc = subprocess.Popen(
            [sys.executable, script],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )
    except Exception as exc:
        print(f"{_RED}[Setup]{_RST} Could not start beamng_runtime.py: {exc}")
        return None, None

    print(f"[Setup] beamng_runtime.py started (PID {proc.pid}), waiting for scenario…")
    deadline = time.monotonic() + SETUP_READY_TIMEOUT

    for line in proc.stdout:
        line = line.rstrip()
        print(f"  {_DIM}{line}{_RST}")
        if line == "READY":
            print(f"{_GRN}[Setup]{_RST} Scenario ready.")
            # Hand off stdout drain to a background thread so the pipe never fills.
            log = LogCapture("setup", proc, maxlines=1000)
            return proc, log
        if proc.poll() is not None:
            print(f"{_RED}[Setup]{_RST} beamng_runtime.py exited (code {proc.returncode}) before 'READY'.")
            return proc, None
        if time.monotonic() > deadline:
            proc.terminate()
            print(f"{_RED}[Setup]{_RST} Timed out waiting for scenario after {SETUP_READY_TIMEOUT}s.")
            return proc, None

    print(f"{_RED}[Setup]{_RST} beamng_runtime.py stdout closed before 'READY'.")
    return proc, None


def clear_openpilot_params():
    """Clear any offroad alerts that would block engagement on the next launch."""
    cmd = (
        f"cd {WSL_OPENPILOT_DIR} && "
        f".venv/bin/python -c \""
        f"from openpilot.common.params import Params; "
        f"p = Params(); "
        f"p.remove('Offroad_ExcessiveActuation'); "
        f"print('[params] Offroad_ExcessiveActuation cleared')"
        f"\""
    )
    subprocess.run(["wsl", "-e", "bash", "-c", cmd], timeout=10)


def launch_openpilot():
    """Launch openpilot sim-mode manager inside WSL."""
    clear_openpilot_params()
    cmd = (
        f"cd {WSL_OPENPILOT_DIR} && "
        f"PATH={WSL_OPENPILOT_DIR}/.venv/bin:$PATH "
        f"BLOCK=soundd "
        f"bash {WSL_LAUNCH_OP}"
    )
    try:
        proc = subprocess.Popen(
            ["wsl", "-e", "bash", "-c", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        print(f"{_GRN}[openpilot]{_RST} Manager launched in WSL (PID {proc.pid})")
        return proc
    except Exception as exc:
        print(f"{_RED}[openpilot]{_RST} Launch failed: {exc}")
        return None


def launch_bridge(dual_camera: bool):
    """Launch bridge_runner.py inside WSL using the openpilot venv."""
    dc_flag = "--dual-camera" if dual_camera else ""
    wsl_dir = win_to_wsl(_THIS_DIR)
    cmd = (
        f"cd '{wsl_dir}' && "
        f"DATA_PORT={DATA_PORT} "
        f"PYTHONPATH={WSL_OPENPILOT_DIR}:$PYTHONPATH "
        f"{WSL_VENV_PYTHON} bridge_runner.py {dc_flag}"
    )
    try:
        proc = subprocess.Popen(
            ["wsl", "-e", "bash", "-c", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        print(f"{_GRN}[Bridge]{_RST} Bridge launched in WSL (PID {proc.pid})")
        return proc
    except Exception as exc:
        print(f"{_RED}[Bridge]{_RST} Launch failed: {exc}")
        return None


# ---------------------------------------------------------------------------
# Log capture (ring buffer)
# ---------------------------------------------------------------------------

class LogCapture:
    def __init__(self, label: str, proc, maxlines: int = 200):
        self.label = label
        self.lines: deque = deque(maxlen=maxlines)
        if proc is not None and proc.stdout is not None:
            self._t = threading.Thread(target=self._drain, args=(proc,), daemon=True)
            self._t.start()

    def _drain(self, proc):
        try:
            for line in proc.stdout:
                self.lines.append(line.rstrip())
        except Exception:
            pass

    def dump(self, path: str):
        """Write full captured log to a file."""
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write("\n".join(self.lines))
        except Exception as exc:
            print(f"[LogCapture] Could not write log to {path}: {exc}")


# ---------------------------------------------------------------------------
# Status display
# ---------------------------------------------------------------------------

def _proc_alive(proc):
    if proc is None:
        return None
    return proc.poll() is None


def render_status(
    setup_proc, op_proc, bridge_proc,
    beamng_skipped, op_skipped, bridge_started,
    op_log, bridge_log, setup_log, start_time, tcp_ok,
):
    elapsed = int(time.monotonic() - start_time)
    h, m, s = elapsed // 3600, (elapsed % 3600) // 60, elapsed % 60

    setup_alive  = _proc_alive(setup_proc)
    op_alive     = _proc_alive(op_proc)
    bridge_alive = _proc_alive(bridge_proc)

    lines = []
    lines.append(f"{'─'*58}")
    lines.append(f"  openpilot <-> BeamNG.Drive    uptime {h:02d}:{m:02d}:{s:02d}")
    lines.append(f"{'─'*58}")

    if beamng_skipped:
        lines.append(f"  {_dot(None)}  BeamNG + scenario  skipped (assumed running)")
    else:
        s2 = "running" if setup_alive else ("stopped" if setup_alive is False else "—")
        lines.append(f"  {_dot(setup_alive)}  BeamNG + scenario  {s2}")

    if op_skipped:
        lines.append(f"  {_dot(None)}  openpilot       skipped (assumed running)")
    else:
        s2 = "running" if op_alive else ("stopped" if op_alive is False else "—")
        lines.append(f"  {_dot(op_alive)}  openpilot (WSL) {s2}")

    if not bridge_started:
        lines.append(f"  {_dot(None)}  Bridge (WSL)    starting…")
    else:
        s2 = "running" if bridge_alive else ("stopped" if bridge_alive is False else "—")
        lines.append(f"  {_dot(bridge_alive)}  Bridge (WSL)    {s2}")

    lines.append(f"{'─'*58}")
    lines.append(f"  {_dot(tcp_ok)}  BeamNG setup   {'running' if tcp_ok else 'not running'}")
    lines.append(f"{'─'*58}")

    if bridge_log and bridge_log.lines:
        lines.append(f"  {_DIM}bridge log:{_RST}")
        for l in list(bridge_log.lines)[-5:]:
            lines.append(f"  {_DIM}{l[:74]}{_RST}")

    if op_log and op_log.lines:
        lines.append(f"  {_DIM}openpilot log:{_RST}")
        for l in list(op_log.lines)[-5:]:
            lines.append(f"  {_DIM}{l[:74]}{_RST}")

    if setup_log and setup_log.lines:
        lines.append(f"  {_DIM}scenario log:{_RST}")
        for l in list(setup_log.lines)[-3:]:
            lines.append(f"  {_DIM}{l[:74]}{_RST}")

    lines.append(f"{'─'*58}")
    lines.append(f"  {_DIM}Ctrl+C to shut down{_RST}")
    print(_CLR + "\n".join(lines), flush=True)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="openpilot <-> BeamNG bridge launcher")
    parser.add_argument("--skip-beamng",    action="store_true",
                        help="BeamNG already running — skip launching it (bridge will still load the scenario)")
    parser.add_argument("--skip-openpilot", action="store_true",
                        help="openpilot already running — skip launch")
    parser.add_argument("--dual-camera",    action="store_true",
                        help="Enable wide road camera")
    args = parser.parse_args()

    setup_proc    = None
    setup_log     = None
    op_proc       = None
    bridge_proc   = None
    op_log        = None
    bridge_log    = None
    bridge_started = False
    start_time    = time.monotonic()

    def shutdown(sig=None, frame=None):
        print(f"\n{_YLW}[Launcher] Shutting down…{_RST}")
        for p in [bridge_proc, op_proc, setup_proc]:
            if p is not None and p.poll() is None:
                p.terminate()
        # BeamNG.Drive is left running so the user can inspect the scene.
        print("[Launcher] Done. BeamNG left running — close it manually if needed.")
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # ── 1. Launch BeamNG (beamng_setup.py owns the entire BeamNG connection) ───
    if not args.skip_beamng:
        # beamng_setup.py uses beamngpy (launch=True) to start BeamNG, load the
        # scenario, spawn the vehicle, and run the data server. The bridge (WSL)
        # never connects to BeamNG directly — it only talks to the data server.
        setup_proc, setup_log = launch_beamng_setup(args.dual_camera)
        if setup_proc is None or setup_log is None:
            print(f"{_RED}[Launcher] BeamNG launch/setup failed — cannot continue.{_RST}")
            print("           Check beamng_setup.py output above for details.")
            print("           Make sure 'pip install beamngpy pillow numpy' has been run.")
            shutdown()
    else:
        print(f"{_YLW}[BeamNG]{_RST} Skipping launch — assuming beamng_setup.py is "
              f"already running with data server on port {DATA_PORT}")

    # ── 2. openpilot ─────────────────────────────────────────────────────────
    if not args.skip_openpilot:
        op_proc = launch_openpilot()
        if op_proc is not None:
            op_log = LogCapture("openpilot", op_proc, maxlines=500)
            print(f"[Launcher] Giving openpilot {OP_STARTUP_WAIT}s to initialise…")
            time.sleep(OP_STARTUP_WAIT)
        else:
            print(f"{_YLW}[openpilot]{_RST} Launch failed — continuing without it.")
    else:
        print(f"{_YLW}[openpilot]{_RST} Skipping launch — assuming already running in WSL")

    # ── 3. Bridge (WSL — connects to data server, feeds openpilot) ──────────
    # Ensure Windows Firewall allows inbound TCP on DATA_PORT so WSL can reach
    # the data server. This is a best-effort add — silently ignored if already
    # present or if the process lacks privileges.
    try:
        subprocess.run(
            ["netsh", "advfirewall", "firewall", "add", "rule",
             f"name=openpilot-beamng-data-{DATA_PORT}",
             "dir=in", "action=allow", "protocol=TCP",
             f"localport={DATA_PORT}"],
            capture_output=True, timeout=5,
        )
    except Exception:
        pass

    # Confirm beamng_setup.py is still alive before launching the bridge.
    # NOTE: do NOT use tcp_reachable() here — connecting to the DataServer
    # occupies its single client slot and blocks the real bridge from connecting.
    if setup_proc is not None and setup_proc.poll() is None:
        print(f"{_GRN}[Bridge]{_RST} BeamNG/data server ready (setup_proc alive)")
    elif not args.skip_beamng:
        print(f"{_RED}[Bridge]{_RST} WARNING: beamng_setup.py is not running — bridge will fail to connect")

    print("[Bridge] Launching bridge in WSL…")
    bridge_proc = launch_bridge(args.dual_camera)
    if bridge_proc is not None:
        bridge_log = LogCapture("bridge", bridge_proc)
    bridge_started = True

    # Health monitor — tracks whether beamng_setup.py is alive.
    # IMPORTANT: do NOT use tcp_reachable() here. It establishes a real TCP
    # connection to the DataServer which evicts the bridge as the active client.
    # For --skip-beamng mode we don't track setup_proc, so default to True.
    tcp_status = [args.skip_beamng]
    def _tcp_watcher():
        while True:
            if bridge_proc is None or bridge_proc.poll() is not None:
                break
            if not args.skip_beamng:
                tcp_status[0] = (setup_proc is not None and setup_proc.poll() is None)
            time.sleep(2.0)
    threading.Thread(target=_tcp_watcher, daemon=True).start()

    # ── 4. Status loop ───────────────────────────────────────────────────────
    print(f"{_GRN}[Launcher] All systems started.{_RST}\n")
    time.sleep(1.5)
    try:
        while bridge_proc is None or bridge_proc.poll() is None:
            # Also exit if beamng_setup.py died — bridge will disconnect soon
            if setup_proc is not None and setup_proc.poll() is not None:
                print(f"\n{_RED}[Launcher] beamng_setup.py exited unexpectedly "
                      f"(code {setup_proc.returncode}) — stopping.{_RST}")
                time.sleep(1.0)  # let bridge log the RST before we dump logs
                break
            render_status(
                setup_proc, op_proc, bridge_proc,
                args.skip_beamng, args.skip_openpilot, bridge_started,
                op_log, bridge_log, setup_log, start_time, tcp_status[0],
            )
            time.sleep(2.0)
    except (KeyboardInterrupt, SystemExit):
        # SystemExit is raised when shutdown() calls sys.exit(0) via signal handler.
        # Catching it here lets the dump code below run before we exit.
        time.sleep(1.5)  # give terminated processes time to flush final log output

    print("\n[Launcher] Bridge process ended.")

    # Dump full logs so errors are visible even though the status display
    # clears the screen on each refresh.
    if setup_log and setup_log.lines:
        log_path = os.path.join(_THIS_DIR, "setup_last_run.log")
        setup_log.dump(log_path)
        print(f"\n{'─'*58}")
        print(f"  SCENARIO LOG (last {len(setup_log.lines)} lines):")
        print(f"{'─'*58}")
        for l in setup_log.lines:
            print(f"  {l}")
        print(f"{'─'*58}")
        print(f"  Full log written to: {log_path}")
        print(f"{'─'*58}\n")

    if bridge_log and bridge_log.lines:
        log_path = os.path.join(_THIS_DIR, "bridge_last_run.log")
        bridge_log.dump(log_path)
        print(f"\n{'─'*58}")
        print(f"  BRIDGE LOG (last {len(bridge_log.lines)} lines):")
        print(f"{'─'*58}")
        for l in bridge_log.lines:
            print(f"  {l}")
        print(f"{'─'*58}")
        print(f"  Full log written to: {log_path}")
        print(f"{'─'*58}\n")

    if op_log and op_log.lines:
        log_path = os.path.join(_THIS_DIR, "op_last_run.log")
        op_log.dump(log_path)
        print(f"\n{'─'*58}")
        print(f"  OPENPILOT LOG (last {len(op_log.lines)} lines):")
        print(f"{'─'*58}")
        for l in op_log.lines:
            print(f"  {l}")
        print(f"{'─'*58}")
        print(f"  Full log written to: {log_path}")
        print(f"{'─'*58}\n")

    shutdown()


if __name__ == "__main__":
    main()
