#!/usr/bin/env python3
"""
Status monitor for the openpilot ↔ BeamNG bridge.

Shows a live curses TUI with green/red health indicators and log tails
for every component.  Launched automatically by start.sh, or run manually:

    python3 tools/status_monitor.py

Keys:
    q / Q / Esc  → quit
"""
import curses
import glob
import os
import subprocess
import time
from datetime import datetime

# ── Configuration ─────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR    = os.path.join(SCRIPT_DIR, 'logs')

BEAMNG_GAME_LOG = os.path.expanduser(
    '~/.local/share/BeamNG/BeamNG.drive/current/beamng.log'
)

REFRESH_S = 0.5   # seconds between screen refreshes

COMPONENTS = [
    {
        'name'  : 'BeamNG',
        'log'   : BEAMNG_GAME_LOG,
        'pgrep' : 'BeamNG.drive.x64',
    },
    {
        'name'  : 'Bridge',
        'log'   : os.path.join(LOG_DIR, 'bridge_current.log'),
        'pgrep' : 'bridge_runner.py',
    },
    {
        'name'  : 'openpilot',
        'log'   : os.path.join(LOG_DIR, 'openpilot_current.log'),
        'pgrep' : 'manager.py',
    },
    {
        'name'  : 'IMU Monitor',
        'log'   : os.path.join(LOG_DIR, 'monitor_current.log'),
        'pgrep' : 'monitor_imu.py',
    },
]

# Keywords that highlight a log line in yellow
_WARN_KW = (
    'error', 'fatal', 'crash', 'exception', 'traceback', 'failed',
    'sensor gap', '!! sensor', 'warning', 'errordevice', 'killed',
    'address already in use', 'connectionreset', 'broken pipe',
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def is_running(pattern: str) -> bool:
    try:
        r = subprocess.run(['pgrep', '-f', pattern],
                           capture_output=True, timeout=1)
        return r.returncode == 0
    except Exception:
        return False


def resolve_log(comp: dict) -> str | None:
    """Return the best available log path for a component."""
    path = comp['log']
    if path and os.path.exists(path) and os.path.getsize(path) > 0:
        return path
    # Fallback: latest timestamped IMU log
    if comp['name'] == 'IMU Monitor':
        files = glob.glob(os.path.join(LOG_DIR, 'monitor_imu_*.log'))
        if files:
            return max(files, key=os.path.getmtime)
    return path   # may not exist yet — caller handles that


def tail_file(path: str | None, n: int) -> list[str]:
    """Return last n lines of path. Returns [] if missing or unreadable."""
    if not path or not os.path.exists(path):
        return []
    try:
        with open(path, 'r', errors='replace') as fh:
            fh.seek(0, 2)
            size = fh.tell()
            fh.seek(max(0, size - 32768))   # read tail ~32 KB
            return [l.rstrip('\n') for l in fh.readlines()[-n:]]
    except Exception:
        return []


def safe_add(win, row: int, col: int, text: str, attr: int = 0) -> None:
    h, w = win.getmaxyx()
    if row < 0 or row >= h or col < 0 or col >= w:
        return
    try:
        win.addstr(row, col, text[: w - col], attr)
    except curses.error:
        pass


# ── Main TUI ──────────────────────────────────────────────────────────────────

def draw(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.use_default_colors()

    # Colour pairs (number, fg, bg)
    C_TITLE  = 1;  curses.init_pair(C_TITLE,  curses.COLOR_BLACK,  curses.COLOR_CYAN)
    C_GREEN  = 2;  curses.init_pair(C_GREEN,  curses.COLOR_GREEN,  -1)
    C_RED    = 3;  curses.init_pair(C_RED,    curses.COLOR_RED,    -1)
    C_HDR    = 4;  curses.init_pair(C_HDR,    curses.COLOR_CYAN,   -1)
    C_WARN   = 5;  curses.init_pair(C_WARN,   curses.COLOR_YELLOW, -1)
    C_FOOT   = 6;  curses.init_pair(C_FOOT,   curses.COLOR_BLACK,  curses.COLOR_WHITE)
    C_DIM    = 7;  curses.init_pair(C_DIM,    curses.COLOR_WHITE,  -1)

    stopped_at: dict[str, float] = {}

    while True:
        key = stdscr.getch()
        if key in (ord('q'), ord('Q'), 27):
            break

        h, w = stdscr.getmaxyx()
        stdscr.erase()

        # ── Title bar ─────────────────────────────────────────────────────
        ts    = datetime.now().strftime('%H:%M:%S')
        title = '  openpilot ↔ BeamNG Bridge Monitor  '
        right = f'  {ts}  '
        pad   = max(0, w - len(title) - len(right))
        safe_add(stdscr, 0, 0,
                 (title + ' ' * pad + right)[:w],
                 curses.color_pair(C_TITLE) | curses.A_BOLD)

        # ── Status row ────────────────────────────────────────────────────
        row, col = 2, 2
        for comp in COMPONENTS:
            name    = comp['name']
            running = is_running(comp['pgrep'])

            if running:
                stopped_at.pop(name, None)
                dot, cp, note = '●', C_GREEN, 'running'
            else:
                if name not in stopped_at:
                    stopped_at[name] = time.monotonic()
                secs = int(time.monotonic() - stopped_at[name])
                dot, cp = '○', C_RED
                note = (f'stopped {secs}s ago' if secs < 60
                        else f'stopped {secs // 60}m ago')

            label = f' {name} ({note})   '
            if col + len(dot) + len(label) + 2 > w - 2:
                row += 1
                col  = 2

            safe_add(stdscr, row, col, dot, curses.color_pair(cp) | curses.A_BOLD)
            safe_add(stdscr, row, col + 2, label, curses.color_pair(C_DIM))
            col += 2 + len(label)

        row += 2

        # ── Log panels (one per component, equal height) ──────────────────
        available = h - row - 1    # leave one line for footer
        panel_h   = max(3, available // len(COMPONENTS))

        for comp in COMPONENTS:
            if row >= h - 1:
                break

            name     = comp['name']
            log_path = resolve_log(comp)
            running  = is_running(comp['pgrep'])
            dot_cp   = C_GREEN if running else C_RED
            dot      = '●' if running else '○'

            # Section header
            filler = '─' * max(0, w - len(name) - 6)
            hdr    = f'── {dot} {name} {filler}'
            safe_add(stdscr, row, 0, hdr[:w], curses.color_pair(C_HDR))
            # Redraw dot with its own colour on top of the header colour
            safe_add(stdscr, row, 3, dot, curses.color_pair(dot_cp) | curses.A_BOLD)
            row += 1

            # Log lines
            n_lines = panel_h - 1
            lines   = tail_file(log_path, n_lines)

            if not lines:
                msg = ('  [waiting for output…]' if running
                       else '  [not started — no log yet]')
                safe_add(stdscr, row, 0, msg,
                         curses.color_pair(C_DIM) | curses.A_DIM)
                row += 1
            else:
                for line in lines[-n_lines:]:
                    if row >= h - 1:
                        break
                    low  = line.lower()
                    attr = (curses.color_pair(C_WARN) | curses.A_BOLD
                            if any(k in low for k in _WARN_KW) else 0)
                    safe_add(stdscr, row, 0, '  ' + line, attr)
                    row += 1

        # ── Footer ────────────────────────────────────────────────────────
        foot = f'  Logs → {LOG_DIR}   |   q: quit   |   refreshes {int(1/REFRESH_S)}×/s  '
        safe_add(stdscr, h - 1, 0, foot.ljust(w)[:w], curses.color_pair(C_FOOT))

        stdscr.refresh()
        time.sleep(REFRESH_S)


def main():
    try:
        curses.wrapper(draw)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'[status_monitor] Error: {exc}')
        raise


if __name__ == '__main__':
    main()
