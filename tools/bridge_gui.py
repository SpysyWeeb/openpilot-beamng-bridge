#!/usr/bin/env python3
"""
BeamNG ↔ openpilot Bridge Control Panel  (GTK4 / PyGObject)

Run on the HOST (not in distrobox):
    python3 tools/bridge_gui.py [--dual-camera]

Left panel  — status dots + per-component Launch / Stop / View Log buttons,
              plus a "Start All" button that launches in order, waiting for
              each stage to signal readiness before moving on.
Right panel — live log viewer; tabs switch between component logs.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import signal as _signal
import subprocess
import threading
import time
from datetime import datetime

import gi
gi.require_version('Gtk', '4.0')
from gi.repository import Gtk, GLib, Gdk, Pango   # noqa: E402

# ── Paths ──────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OPENPILOT  = os.path.expanduser('~/sunnypilot')
DISTROBOX  = 'openpilot-beamng-bridge'
LOG_DIR    = os.path.join(SCRIPT_DIR, 'logs')
READY_FILE   = '/tmp/openpilot_beamng_bridge_ready'
CMD_FIFO     = '/tmp/beamng_bridge_cmd'
ENGAGED_FILE = '/tmp/beamng_bridge_engaged'
BEAMNG_LOG = os.path.expanduser(
    '~/.local/share/BeamNG/BeamNG.drive/current/beamng.log')

os.makedirs(LOG_DIR, exist_ok=True)

CONFIG_FILE = os.path.expanduser('~/.config/beamng_bridge_gui.json')

POLL_MS     = 1000   # status-light refresh interval
LOG_POLL_MS = 500    # log-append interval
MAX_LINES   = 3000   # lines kept in the text buffer before trimming

MODE_BEAMNG     = 'beamng'
MODE_METADRIVE  = 'metadrive'

# ── GTK4 CSS (Tokyo Night palette) ────────────────────────────────────────────
_CSS = b"""
* { font-family: monospace; }

window  { background-color: #1a1b26; color: #c0caf5; }

.sidebar {
    background-color: #1f2335;
    margin:  8px 0px 8px 8px;
    padding: 0px;
}
.log-panel { background-color: #1a1b26; margin: 8px; }

/* log text area */
textview          { background-color: #0d0e17; }
textview text     { background-color: #0d0e17; color: #c0caf5; font-size: 9pt; }

/* tab buttons */
button.tab        { background-color: #292e42; color: #c0caf5;
                    border: none; border-radius: 4px;
                    padding: 4px 12px; margin-right: 4px; }
button.tab:hover  { background-color: #3d4166; }
button.tab.active { background-color: #7aa2f7; color: white; }

/* action buttons per component */
button.act        { background-color: #292e42; color: #c0caf5;
                    border: none; border-radius: 4px; padding: 4px 10px; }
button.act:hover  { background-color: #3d4166; }
button.act:disabled { opacity: 0.35; }
button.stop-btn   { background-color: #4a2020; color: #ff9898; }
button.log-btn    { color: #7dcfff; }

/* top-level action buttons */
button.start-all  { background-color: #1a5c38; color: white;
                    font-size: 10pt; font-weight: bold;
                    border: none; border-radius: 4px; padding: 7px 16px; }
button.stop-all   { background-color: #5c1a1a; color: white;
                    font-size: 10pt; font-weight: bold;
                    border: none; border-radius: 4px; padding: 7px 16px; }
button.cancel-all { background-color: #7c4010; color: white;
                    font-size: 10pt; font-weight: bold;
                    border: none; border-radius: 4px; padding: 7px 16px; }
button.clear-btn  { background-color: #292e42; color: #e0af68;
                    border: none; border-radius: 4px; padding: 4px 10px; }

button.engage-btn { font-size: 10pt; font-weight: bold;
                    border: none; border-radius: 4px; padding: 7px 16px; }
button.engage-off { background-color: #292e42; color: #565f89; }
button.engage-on  { background-color: #1a5c38; color: #9ece6a; }
button.engage-btn:disabled { opacity: 0.35; }

/* labels */
.title   { font-size: 13pt; font-weight: bold; color: #7dcfff; }
.sub     { font-size: 8pt;  color: #565f89; }
.cname   { font-size: 10pt; font-weight: bold; }
.cdesc   { font-size: 8pt;  color: #565f89; font-style: italic; }
.running { font-size: 8pt;  color: #9ece6a; }
.stopped { font-size: 8pt;  color: #565f89; }
.dot-on  { font-size: 18pt; color: #9ece6a; }
.dot-off { font-size: 18pt; color: #f7768e; }
.statusbar { font-size: 8pt; color: #e0af68; padding: 6px 10px; }
.logpath   { font-size: 8pt; color: #565f89; padding: 2px 0; }
"""

# ── Window geometry persistence ───────────────────────────────────────────────

def _load_gui_config() -> dict:
    try:
        with open(CONFIG_FILE) as f:
            return json.load(f)
    except Exception:
        return {}


def _save_gui_config(cfg: dict) -> None:
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        with open(CONFIG_FILE, 'w') as f:
            json.dump(cfg, f)
    except Exception:
        pass


# ── Process / file helpers ─────────────────────────────────────────────────────

def _pgrep(pattern: str) -> bool:
    try:
        r = subprocess.run(['pgrep', '-f', pattern],
                           capture_output=True, timeout=2)
        return r.returncode == 0
    except Exception:
        return False


def _kill(pattern: str) -> None:
    """Kill all processes matching `pattern`.  Works on processes started
    outside the GUI.  Tries SIGTERM first, then SIGKILL for survivors."""
    try:
        r = subprocess.run(['pgrep', '-f', pattern],
                           capture_output=True, timeout=2, text=True)
        pids = [int(p) for p in r.stdout.split() if p.strip().isdigit()]
    except Exception:
        pids = []

    for pid in pids:
        try:
            os.kill(pid, _signal.SIGTERM)
        except ProcessLookupError:
            pass
        except Exception:
            pass

    if not pids:
        # No pids from pgrep (maybe pattern matches nothing) — fall back to pkill
        try:
            subprocess.run(['pkill', '-f', pattern], capture_output=True, timeout=3)
        except Exception:
            pass
        return

    time.sleep(0.6)

    # SIGKILL any survivors
    for pid in pids:
        try:
            os.kill(pid, _signal.SIGKILL)
        except ProcessLookupError:
            pass   # already gone — good
        except Exception:
            pass


def _read_tail(path: str, limit: int = 131072) -> tuple[str, int]:
    """Return (tail-text, file-size-after-read)."""
    if not path or not os.path.exists(path):
        return '', 0
    try:
        with open(path, 'r', errors='replace') as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - limit))
            return f.read(), f.tell()
    except Exception:
        return '', 0


def _read_from(path: str, offset: int) -> tuple[str, int]:
    """Return (new-text-since-offset, new-offset)."""
    if not path or not os.path.exists(path):
        return '', offset
    try:
        with open(path, 'r', errors='replace') as f:
            f.seek(offset)
            text = f.read()
            return text, f.tell()
    except Exception:
        return '', offset


# ── Component definitions ──────────────────────────────────────────────────────

def _db(inner: str) -> list[str]:
    """Wrap a shell snippet in a distrobox-enter command.

    NOTE: do NOT add 'exec' here — inner commands that want process-replacement
    supply their own exec (e.g. 'exec python3 ...').  Putting exec in the
    wrapper breaks multi-part inners like 'cd foo && exec bash ...'.
    """
    return [
        'distrobox', 'enter', DISTROBOX, '--',
        'bash', '-c',
        (f"source '{OPENPILOT}/.venv/bin/activate' && "
         f"export PYTHONPATH='{OPENPILOT}:{SCRIPT_DIR}' && "
         f"{inner}"),
    ]


def make_metadrive_components(dual_camera: bool = False) -> list[dict]:
    dual = '--dual_camera' if dual_camera else ''
    return [
        {
            'id'            : 'openpilot',
            'name'          : 'openpilot',
            'desc'          : 'ADAS stack (manager + modeld + …)',
            'pgrep'         : r'system/manager/manager\.py',
            'stop_pat'      : r'system/manager/manager\.py',
            'log_path'      : os.path.join(LOG_DIR, 'openpilot_current.log'),
            'log_clear'     : True,
            'launch_cmd'    : _db(
                f"cd '{OPENPILOT}' && exec bash tools/sim/launch_openpilot.sh"),
            'ready'         : 'process',
            'ready_timeout' : 60,
            'ready_delay'   : 5.0,
        },
        {
            'id'            : 'bridge',
            'name'          : 'MetaDrive Bridge',
            'desc'          : 'MetaDrive simulator + sensor bridge',
            'pgrep'         : 'run_bridge.py',
            'stop_pat'      : 'run_bridge.py',
            'log_path'      : os.path.join(LOG_DIR, 'bridge_current.log'),
            'log_clear'     : True,
            'launch_cmd'    : _db(
                f"cd '{OPENPILOT}' && exec python3 tools/sim/run_bridge.py"
                + (f" {dual}" if dual else "")),
            'ready'         : 'process',
            'ready_timeout' : 60,
            'ready_delay'   : 2.0,
        },
    ]


def make_components(dual_camera: bool = False) -> list[dict]:
    dual = '--dual-camera' if dual_camera else ''
    return [
        {
            'id'            : 'beamng',
            'name'          : 'BeamNG',
            # BeamNG alone opens the game with the tech-server port.
            # The SCENARIO is loaded by the Bridge step below.
            'desc'          : 'game engine + tech server (no scenario yet)',
            'pgrep'         : 'BeamNG.drive.x64',
            'stop_pat'      : 'BeamNG.drive.x64',
            'log_path'      : BEAMNG_LOG,
            'log_clear'     : False,
            'launch_cmd'    : ['bash', os.path.join(SCRIPT_DIR, 'launch_beamng.sh')],
            'ready'         : 'process',
            'ready_timeout' : 120,
            'ready_delay'   : 4.0,
        },
        {
            'id'            : 'bridge',
            'name'          : 'Bridge',
            'desc'          : 'connects to BeamNG, loads scenario, polls sensors',
            'pgrep'         : 'bridge_runner.py',
            'stop_pat'      : 'bridge_runner.py',
            'log_path'      : os.path.join(LOG_DIR, 'bridge_current.log'),
            'log_clear'     : True,
            # exec python3 replaces the bash wrapper so pgrep sees bridge_runner.py
            'launch_cmd'    : _db(
                f"exec python3 '{SCRIPT_DIR}/linux/bridge_runner.py'"
                + (f" {dual}" if dual else "")),
            'ready'         : 'sentinel',
            'ready_timeout' : 180,
            'ready_delay'   : 0.0,
        },
        {
            'id'            : 'openpilot',
            'name'          : 'openpilot',
            'desc'          : 'ADAS stack (manager + modeld + …)',
            'pgrep'         : r'system/manager/manager\.py',
            'stop_pat'      : r'system/manager/manager\.py',
            'log_path'      : os.path.join(LOG_DIR, 'openpilot_current.log'),
            'log_clear'     : True,
            # cd is a shell builtin — can't exec it; exec bash replaces the wrapper
            'launch_cmd'    : _db(
                f"cd '{OPENPILOT}' && exec bash tools/sim/launch_openpilot.sh"),
            'ready'         : 'process',
            'ready_timeout' : 60,
            'ready_delay'   : 5.0,
        },
        {
            'id'            : 'imu_monitor',
            'name'          : 'IMU Monitor',
            'desc'          : 'sensor diagnostics',
            'pgrep'         : 'monitor_imu.py',
            'stop_pat'      : 'monitor_imu.py',
            'log_path'      : os.path.join(LOG_DIR, 'monitor_current.log'),
            'log_clear'     : True,
            'launch_cmd'    : _db(
                f"exec python3 '{SCRIPT_DIR}/tools/monitor_imu.py'"
                f" --log-dir '{LOG_DIR}'"),
            'ready'         : 'process',
            'ready_timeout' : 30,
            'ready_delay'   : 2.0,
        },
    ]


# ── Line-colouring regexes ─────────────────────────────────────────────────────
_ANSI_SPLIT_RE = re.compile(r'(\x1b\[[0-9;]*m)')

# ANSI SGR colour codes → Tokyo Night foreground colours
_ANSI_FG: dict[str, str] = {
    '30': '#565f89',  # black   → comment
    '31': '#f7768e',  # red
    '32': '#9ece6a',  # green
    '33': '#e0af68',  # yellow
    '34': '#7aa2f7',  # blue
    '35': '#bb9af7',  # magenta
    '36': '#7dcfff',  # cyan
    '37': '#c0caf5',  # white
    '90': '#444b6a',  # bright black
    '91': '#ff7a93',  # bright red
    '92': '#b9f27c',  # bright green
    '93': '#ff9e64',  # bright yellow
    '94': '#7da6ff',  # bright blue
    '95': '#c0caf5',  # bright magenta
    '96': '#89ddff',  # bright cyan
    '97': '#ffffff',  # bright white
}

_WARN_RE = re.compile(
    r'error|fatal|crash|exception|traceback|warning|killed|failed|invalid',
    re.IGNORECASE)
_OK_RE = re.compile(
    r'\bready\b|\bconnected\b|\bsuccess\b|\bloaded\b',
    re.IGNORECASE)


# ── GTK4 helpers ───────────────────────────────────────────────────────────────

def _label(text: str, css: str = '', markup: bool = False) -> Gtk.Label:
    lbl = Gtk.Label()
    if markup:
        lbl.set_markup(text)
    else:
        lbl.set_text(text)
    lbl.set_xalign(0.0)
    if css:
        lbl.get_style_context().add_class(css)
    return lbl


def _button(label: str, *css_classes: str, handler=None) -> Gtk.Button:
    btn = Gtk.Button(label=label)
    sc  = btn.get_style_context()
    for cls in css_classes:
        sc.add_class(cls)
    if handler:
        btn.connect('clicked', handler)
    return btn


def _sep() -> Gtk.Separator:
    return Gtk.Separator(orientation=Gtk.Orientation.HORIZONTAL)


def _vbox(spacing: int = 0) -> Gtk.Box:
    return Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=spacing)


def _hbox(spacing: int = 0) -> Gtk.Box:
    return Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=spacing)


def _css_add(widget: Gtk.Widget, *classes: str) -> None:
    sc = widget.get_style_context()
    for c in classes:
        sc.add_class(c)


def _css_remove(widget: Gtk.Widget, *classes: str) -> None:
    sc = widget.get_style_context()
    for c in classes:
        sc.remove_class(c)


# ── Main window ────────────────────────────────────────────────────────────────

class BridgeWindow(Gtk.ApplicationWindow):

    def __init__(self, app: Gtk.Application, dual_camera: bool = False,
                 mode: str = MODE_BEAMNG) -> None:
        super().__init__(application=app)
        self._mode = mode
        self.components = (make_metadrive_components(dual_camera)
                           if mode == MODE_METADRIVE
                           else make_components(dual_camera))

        # ── state ──────────────────────────────────────────────────────────────
        self._procs:      dict[str, subprocess.Popen | None] = {
            c['id']: None for c in self.components}
        self._running:    dict[str, bool]        = {
            c['id']: False for c in self.components}
        self._stopped_at: dict[str, float | None] = {
            c['id']: None for c in self.components}

        self._sa_thread: threading.Thread | None = None
        self._sa_cancel = threading.Event()

        self._active_log = self.components[0]['id']
        self._log_offset: dict[str, int] = {c['id']: 0 for c in self.components}
        self._autoscroll = True

        # ── window setup ───────────────────────────────────────────────────────
        _title = ('MetaDrive ↔ openpilot  Bridge Control'
                  if mode == MODE_METADRIVE
                  else 'BeamNG ↔ openpilot  Bridge Control')
        self.set_title(_title)
        self._gui_cfg = _load_gui_config()
        self.set_default_size(
            self._gui_cfg.get('width',  1180),
            self._gui_cfg.get('height', 720),
        )
        self._apply_css()
        self._build()
        self.connect('close-request', self._on_close_request)

        # ── clean slate on startup ─────────────────────────────────────────────
        # Kill any leftover processes from a previous session so the panel
        # always starts with everything stopped.
        threading.Thread(target=self._kill_all_on_start,
                         daemon=True, name='startup-cleanup').start()

        # ── timers ─────────────────────────────────────────────────────────────
        GLib.timeout_add(POLL_MS,     self._poll_status)
        GLib.timeout_add(LOG_POLL_MS, self._poll_log)

    # ── CSS ────────────────────────────────────────────────────────────────────

    def _apply_css(self) -> None:
        prov = Gtk.CssProvider()
        prov.load_from_data(_CSS)
        Gtk.StyleContext.add_provider_for_display(
            Gdk.Display.get_default(), prov,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

    # ── Layout ─────────────────────────────────────────────────────────────────

    def _build(self) -> None:
        self._paned = Gtk.Paned(orientation=Gtk.Orientation.HORIZONTAL)
        self.set_child(self._paned)

        left  = self._build_left()
        right = self._build_right()

        self._paned.set_start_child(left)
        self._paned.set_end_child(right)
        self._paned.set_position(self._gui_cfg.get('paned_pos', 320))
        self._paned.set_shrink_start_child(False)
        self._paned.set_shrink_end_child(False)

    # ── Left panel ─────────────────────────────────────────────────────────────

    def _build_left(self) -> Gtk.Widget:
        root = _vbox()
        _css_add(root, 'sidebar')
        root.set_size_request(300, -1)

        # Header
        hdr = _vbox(4)
        hdr.set_margin_top(14)
        hdr.set_margin_bottom(6)
        hdr.set_margin_start(14)
        hdr.set_margin_end(14)
        _hdr_title = ('MetaDrive ↔ openpilot'
                      if self._mode == MODE_METADRIVE
                      else 'BeamNG ↔ openpilot')
        hdr.append(_label(_hdr_title, 'title'))
        hdr.append(_label('BRIDGE CONTROL PANEL', 'sub'))
        root.append(hdr)

        root.append(_sep())

        # Start All / Stop All
        top_btns = _hbox(8)
        top_btns.set_margin_top(10)
        top_btns.set_margin_bottom(10)
        top_btns.set_margin_start(12)
        top_btns.set_margin_end(12)

        self._sa_btn = _button('▶  Start All', 'start-all',
                               handler=lambda _: self._on_start_all())
        self._sa_btn.set_hexpand(True)
        top_btns.append(self._sa_btn)

        stop_all_btn = _button('■  Stop All', 'stop-all',
                               handler=lambda _: self._on_stop_all())
        stop_all_btn.set_hexpand(True)
        top_btns.append(stop_all_btn)

        root.append(top_btns)
        root.append(_sep())

        # Engage / Disengage — BeamNG mode only (MetaDrive uses keyboard controls)
        self._engage_btn = None
        if self._mode == MODE_BEAMNG:
            engage_row = _hbox(8)
            engage_row.set_margin_top(8)
            engage_row.set_margin_bottom(8)
            engage_row.set_margin_start(12)
            engage_row.set_margin_end(12)

            self._engage_btn = _button('⚡  Engage', 'engage-btn', 'engage-off',
                                       handler=lambda _: self._on_engage_toggle())
            self._engage_btn.set_hexpand(True)
            self._engage_btn.set_sensitive(False)
            engage_row.append(self._engage_btn)
            root.append(engage_row)
            root.append(_sep())

        # Component rows
        self._cw: dict[str, dict] = {}
        for i, comp in enumerate(self.components):
            self._cw[comp['id']] = self._build_comp_row(root, comp)
            if i < len(self.components) - 1:
                root.append(_sep())

        # Controls popover — attached to bridge component's button row (BeamNG mode only)
        self._controls_popover = None
        self._driver_mode_on   = False
        self._speed_spin       = None
        self._road_fov_scale   = None
        self._wide_fov_scale   = None
        if self._mode == MODE_BEAMNG and 'bridge' in self._cw:
            self._controls_popover = self._build_controls_popover()
            menu_btn = Gtk.MenuButton()
            menu_btn.set_label('⚙  Controls')
            sc = menu_btn.get_style_context()
            sc.add_class('act')
            sc.add_class('log-btn')
            menu_btn.set_popover(self._controls_popover)
            self._cw['bridge']['btns'].append(menu_btn)

        # Spacer
        spacer = Gtk.Box()
        spacer.set_vexpand(True)
        root.append(spacer)

        root.append(_sep())

        # Status bar
        self._status_lbl = _label('Idle — click Launch or Start All.', 'statusbar')
        self._status_lbl.set_wrap(True)
        self._status_lbl.set_max_width_chars(38)
        self._status_lbl.set_margin_start(10)
        self._status_lbl.set_margin_end(10)
        self._status_lbl.set_margin_top(4)
        self._status_lbl.set_margin_bottom(8)
        root.append(self._status_lbl)

        return root

    def _build_comp_row(self, parent: Gtk.Box, comp: dict) -> dict:
        row = _vbox(4)
        row.set_margin_top(10)
        row.set_margin_bottom(10)
        row.set_margin_start(12)
        row.set_margin_end(12)

        # Top: dot + name/status/desc
        top = _hbox(10)
        top.set_valign(Gtk.Align.START)

        dot_lbl = _label('●', 'dot-off')
        dot_lbl.set_valign(Gtk.Align.CENTER)
        top.append(dot_lbl)

        info = _vbox(2)
        name_lbl   = _label(comp['name'],  'cname')
        status_lbl = _label('stopped',     'stopped')
        desc_lbl   = _label(comp['desc'],  'cdesc')
        info.append(name_lbl)
        info.append(status_lbl)
        info.append(desc_lbl)
        info.set_hexpand(True)
        top.append(info)

        row.append(top)

        # Bottom: action buttons
        btns = _hbox(6)
        btns.set_margin_top(6)

        launch_btn = _button('Launch', 'act',
                             handler=lambda _, c=comp: self._on_launch(c))
        stop_btn   = _button('Stop',   'act', 'stop-btn',
                             handler=lambda _, c=comp: self._on_stop(c))
        log_btn    = _button('View Log', 'act', 'log-btn',
                             handler=lambda _, c=comp: self._switch_log(c['id']))

        # Initial state: we don't know yet — first poll will correct within 1 s.
        # Disable Stop by default; it will be enabled if the process is found running.
        stop_btn.set_sensitive(False)

        btns.append(launch_btn)
        btns.append(stop_btn)
        btns.append(log_btn)
        row.append(btns)

        parent.append(row)

        return {
            'dot': dot_lbl,
            'status': status_lbl,
            'launch_btn': launch_btn,
            'stop_btn': stop_btn,
            'btns': btns,
        }

    # ── Right panel ────────────────────────────────────────────────────────────

    def _build_right(self) -> Gtk.Widget:
        root = _vbox(0)
        _css_add(root, 'log-panel')

        # Tab bar
        tab_bar = _hbox(0)
        tab_bar.set_margin_bottom(6)

        self._tab_btns: dict[str, Gtk.Button] = {}
        for comp in self.components:
            b = _button(comp['name'], 'tab',
                        handler=lambda _, cid=comp['id']: self._switch_log(cid))
            self._tab_btns[comp['id']] = b
            tab_bar.append(b)

        # Right side of tab bar
        spacer = Gtk.Box()
        spacer.set_hexpand(True)
        tab_bar.append(spacer)

        self._autoscroll_chk = Gtk.CheckButton(label='Auto-scroll')
        self._autoscroll_chk.set_active(True)
        self._autoscroll_chk.connect('toggled',
            lambda c: setattr(self, '_autoscroll', c.get_active()))
        tab_bar.append(self._autoscroll_chk)

        clear_btn = _button('Clear', 'act', 'clear-btn',
                            handler=lambda _: self._clear_log())
        clear_btn.set_margin_start(6)
        tab_bar.append(clear_btn)

        root.append(tab_bar)

        # Text view
        self._buf = Gtk.TextBuffer()
        self._buf.create_tag('err',  foreground='#f7768e',
                             weight=Pango.Weight.BOLD)
        self._buf.create_tag('warn', foreground='#e0af68')
        self._buf.create_tag('ok',   foreground='#9ece6a')
        self._buf.create_tag('ansi_bold', weight=Pango.Weight.BOLD)
        for _code, _color in _ANSI_FG.items():
            self._buf.create_tag(f'ansi_{_code}', foreground=_color)

        self._txt = Gtk.TextView(buffer=self._buf)
        self._txt.set_editable(False)
        self._txt.set_cursor_visible(False)
        self._txt.set_wrap_mode(Gtk.WrapMode.NONE)
        self._txt.set_monospace(True)
        self._txt.set_left_margin(6)
        self._txt.set_top_margin(4)
        self._txt.set_bottom_margin(4)

        self._scrolled = Gtk.ScrolledWindow()
        self._scrolled.set_child(self._txt)
        self._scrolled.set_vexpand(True)
        self._scrolled.set_hexpand(True)
        self._scrolled.set_policy(Gtk.PolicyType.AUTOMATIC,
                                  Gtk.PolicyType.AUTOMATIC)
        root.append(self._scrolled)

        # Log-path footer
        self._logpath_lbl = _label('', 'logpath')
        self._logpath_lbl.set_margin_top(3)
        root.append(self._logpath_lbl)

        self._switch_log(self.components[0]['id'])
        return root

    # ── Actions ────────────────────────────────────────────────────────────────

    def _on_launch(self, comp: dict) -> None:
        cid = comp['id']
        if _pgrep(comp['pgrep']):
            self._status(f"{comp['name']} is already running.")
            return

        lp = comp.get('log_path')
        if comp.get('log_clear') and lp:
            try:
                open(lp, 'w').close()
            except Exception:
                pass
            self._log_offset[cid] = 0

        try:
            fh   = open(lp, 'a') if lp else subprocess.DEVNULL
            proc = subprocess.Popen(comp['launch_cmd'], stdout=fh, stderr=fh,
                                    start_new_session=True)
            if fh is not subprocess.DEVNULL:
                fh.close()
        except Exception as exc:
            self._status(f"ERROR launching {comp['name']}: {exc}")
            return

        self._procs[cid] = proc
        self._status(f"Launched {comp['name']}  (pid {proc.pid})")
        self._switch_log(cid)

    def _on_stop(self, comp: dict) -> None:
        cid = comp['id']
        # Kill by pattern — works regardless of whether the GUI launched the process
        _kill(comp['stop_pat'])
        # Also terminate any Popen handle we own (belt-and-suspenders)
        proc = self._procs.get(cid)
        if proc:
            try:
                proc.terminate()
            except Exception:
                pass
            self._procs[cid] = None
        self._status(f"Stopped {comp['name']}.")

    # ── FIFO helper ────────────────────────────────────────────────────────────

    def _send_cmd(self, cmd: str) -> None:
        try:
            fd = os.open(CMD_FIFO, os.O_WRONLY | os.O_NONBLOCK)
            os.write(fd, (cmd + '\n').encode())
            os.close(fd)
        except Exception:
            pass

    # ── Controls popover ───────────────────────────────────────────────────────

    def _build_controls_popover(self) -> Gtk.Popover:
        pop = Gtk.Popover()

        content = _vbox(6)
        content.set_margin_top(12)
        content.set_margin_bottom(12)
        content.set_margin_start(14)
        content.set_margin_end(14)
        content.set_size_request(210, -1)

        # ── Cruise ────────────────────────────────────────────────────────────
        content.append(_label('Cruise Control', 'cname'))

        row1 = _hbox(6)
        row1.set_margin_top(6)
        main_btn   = _button('MAIN',   'act', handler=lambda _: self._send_cmd('cruise_main'))
        cancel_btn = _button('CANCEL', 'act', 'stop-btn',
                             handler=lambda _: self._send_cmd('cruise_cancel'))
        main_btn.set_hexpand(True)
        cancel_btn.set_hexpand(True)
        row1.append(main_btn)
        row1.append(cancel_btn)
        content.append(row1)

        row2 = _hbox(6)
        row2.set_margin_top(4)
        set_btn = _button('▼  SET', 'act', handler=lambda _: self._send_cmd('cruise_down'))
        res_btn = _button('▲  RES', 'act', handler=lambda _: self._send_cmd('cruise_up'))
        set_btn.set_hexpand(True)
        res_btn.set_hexpand(True)
        row2.append(set_btn)
        row2.append(res_btn)
        content.append(row2)

        # Speed target
        content.append(_sep())
        content.append(_label('Set Speed (mph)', 'cdesc'))

        speed_row = _hbox(6)
        speed_row.set_margin_top(4)
        self._speed_spin = Gtk.SpinButton()
        adj = Gtk.Adjustment(value=30.0, lower=5.0, upper=130.0,
                             step_increment=1.0, page_increment=5.0, page_size=0.0)
        self._speed_spin.set_adjustment(adj)
        self._speed_spin.set_numeric(True)
        self._speed_spin.set_digits(0)
        self._speed_spin.set_hexpand(True)
        go_btn = _button('Go', 'act', handler=lambda _: self._on_set_speed())
        speed_row.append(self._speed_spin)
        speed_row.append(go_btn)
        content.append(speed_row)

        # ── Camera FOV ────────────────────────────────────────────────────────
        content.append(_sep())
        content.append(_label('Camera FOV', 'cname'))

        road_lbl = Gtk.Label(label=f'Road: {25.7:.1f}°')
        road_lbl.set_halign(Gtk.Align.START)
        road_lbl.set_margin_top(4)
        content.append(road_lbl)

        self._road_fov_scale = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 5.0, 60.0, 0.5)
        self._road_fov_scale.set_value(25.69)
        self._road_fov_scale.set_hexpand(True)
        self._road_fov_scale.set_draw_value(False)
        self._road_fov_scale.connect('value-changed',
                                     lambda s: self._on_fov_change('road', s, road_lbl))
        content.append(self._road_fov_scale)

        wide_lbl = Gtk.Label(label=f'Wide: {94.7:.1f}°')
        wide_lbl.set_halign(Gtk.Align.START)
        wide_lbl.set_margin_top(4)
        content.append(wide_lbl)

        self._wide_fov_scale = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 40.0, 120.0, 0.5)
        self._wide_fov_scale.set_value(94.68)
        self._wide_fov_scale.set_hexpand(True)
        self._wide_fov_scale.set_draw_value(False)
        self._wide_fov_scale.connect('value-changed',
                                     lambda s: self._on_fov_change('wide', s, wide_lbl))
        content.append(self._wide_fov_scale)

        # ── Driver Override ───────────────────────────────────────────────────
        content.append(_sep())
        content.append(_label('Driver Override', 'cname'))

        self._driver_mode_btn = _button('⬜  Driver Mode: OFF', 'act',
                                        handler=lambda _: self._on_driver_mode_toggle())
        self._driver_mode_btn.set_margin_top(4)
        content.append(self._driver_mode_btn)

        pop.set_child(content)
        return pop

    def _on_set_speed(self) -> None:
        if self._speed_spin is None:
            return
        mph = int(self._speed_spin.get_value())
        self._send_cmd(f'cruise_speed_{mph}')

    def _on_fov_change(self, cam: str, scale: Gtk.Scale, lbl: Gtk.Label) -> None:
        val = scale.get_value()
        lbl.set_label(f'{"Road" if cam == "road" else "Wide"}: {val:.1f}°')
        self._send_cmd(f'fov_{cam}_{val:.1f}')

    def _on_driver_mode_toggle(self) -> None:
        self._driver_mode_on = not self._driver_mode_on
        self._send_cmd('driver_mode_on' if self._driver_mode_on else 'driver_mode_off')
        lbl = '■  Driver Mode: ON' if self._driver_mode_on else '⬜  Driver Mode: OFF'
        if self._driver_mode_btn is not None:
            self._driver_mode_btn.set_label(lbl)

    def _on_engage_toggle(self) -> None:
        try:
            engaged = os.path.exists(ENGAGED_FILE) and open(ENGAGED_FILE).read().strip() == '1'
            cmd = 'cruise_cancel' if engaged else 'cruise_down'
            self._send_cmd(cmd)
        except Exception:
            pass

    def _on_stop_all(self) -> None:
        self._sa_cancel.set()
        for comp in self.components:
            self._on_stop(comp)
        self._status('All components stopped.')
        self._sa_btn_reset()

    def _on_start_all(self) -> None:
        if self._sa_thread and self._sa_thread.is_alive():
            self._sa_cancel.set()
            self._sa_btn_reset()
            self._status('Start All cancelled.')
            return
        self._sa_cancel.clear()
        _css_remove(self._sa_btn, 'start-all')
        _css_add(self._sa_btn,    'cancel-all')
        self._sa_btn.set_label('◼  Cancel')
        t = threading.Thread(target=self._sa_sequence, daemon=True,
                             name='start-all')
        self._sa_thread = t
        t.start()

    def _sa_btn_reset(self) -> None:
        _css_remove(self._sa_btn, 'cancel-all')
        _css_add(self._sa_btn,    'start-all')
        self._sa_btn.set_label('▶  Start All')

    # ── Start-All sequence (background thread) ─────────────────────────────────

    def _sa_sequence(self) -> None:
        cancel = self._sa_cancel

        def st(msg: str) -> None:
            GLib.idle_add(self._status, msg)

        def launch(comp: dict) -> None:
            GLib.idle_add(self._on_launch, comp)

        def wait_proc(pat: str, timeout: int, delay: float) -> bool:
            dl = time.monotonic() + timeout
            while time.monotonic() < dl:
                if cancel.is_set():
                    return False
                if _pgrep(pat):
                    for _ in range(int(delay * 4)):
                        if cancel.is_set():
                            return False
                        time.sleep(0.25)
                    return True
                time.sleep(1.0)
            return False

        def wait_sentinel(path: str, timeout: int) -> bool:
            dl = time.monotonic() + timeout
            while time.monotonic() < dl:
                if cancel.is_set():
                    return False
                if os.path.exists(path):
                    return True
                time.sleep(1.0)
            return False

        def done(msg: str = '') -> None:
            if msg:
                st(msg)
            GLib.idle_add(self._sa_btn_reset)

        # Clear logs
        for comp in self.components:
            lp = comp.get('log_path')
            if comp.get('log_clear') and lp:
                try:
                    open(lp, 'w').close()
                except Exception:
                    pass
                self._log_offset[comp['id']] = 0
        try:
            os.unlink(READY_FILE)
        except FileNotFoundError:
            pass

        # ── MetaDrive sequence ──────────────────────────────────────────────────
        if self._mode == MODE_METADRIVE:
            # 1 / 2 — openpilot
            c = self._comp('openpilot')
            if not _pgrep(c['pgrep']):
                st('1/2 — Launching openpilot...')
                launch(c)
                st(f"1/2 — Waiting for manager  (up to {c['ready_timeout']} s)...")
                if not wait_proc(c['pgrep'], c['ready_timeout'], c['ready_delay']):
                    done('ERROR: openpilot did not start — check logs.'); return
            else:
                st('1/2 — openpilot already running, skipping.')
            if cancel.is_set():
                done(); return

            # 2 / 2 — MetaDrive bridge
            c = self._comp('bridge')
            if not _pgrep(c['pgrep']):
                st('2/2 — Launching MetaDrive bridge...')
                launch(c)
                st(f"2/2 — Waiting for bridge  (up to {c['ready_timeout']} s)...")
                if not wait_proc(c['pgrep'], c['ready_timeout'], c['ready_delay']):
                    done('ERROR: MetaDrive bridge did not start — check logs.'); return
            else:
                st('2/2 — MetaDrive bridge already running, skipping.')

            done('All components launched!')
            return

        # ── BeamNG sequence ─────────────────────────────────────────────────────
        # 1 / 4 — BeamNG
        c = self._comp('beamng')
        if not _pgrep(c['pgrep']):
            st('1/4 — Launching BeamNG...')
            launch(c)
            st(f"1/4 — Waiting for BeamNG  (up to {c['ready_timeout']} s)...")
            if not wait_proc(c['pgrep'], c['ready_timeout'], c['ready_delay']):
                done('ERROR: BeamNG did not start — check logs.'); return
        else:
            st('1/4 — BeamNG already running, skipping.')
        if cancel.is_set():
            done(); return

        # 2 / 4 — Bridge
        c = self._comp('bridge')
        if not _pgrep(c['pgrep']):
            st('2/4 — Launching Bridge  (scenario load: 60–120 s)...')
            launch(c)
            st(f"2/4 — Waiting for sensors  (up to {c['ready_timeout']} s)...")
            if not wait_sentinel(READY_FILE, c['ready_timeout']):
                done('ERROR: Bridge sensors never went live — check logs.'); return
            try:
                os.unlink(READY_FILE)
            except FileNotFoundError:
                pass
        else:
            st('2/4 — Bridge already running, skipping.')
        if cancel.is_set():
            done(); return

        # 3 / 4 — openpilot
        c = self._comp('openpilot')
        if not _pgrep(c['pgrep']):
            st('3/4 — Launching openpilot...')
            launch(c)
            st(f"3/4 — Waiting for manager  (up to {c['ready_timeout']} s)...")
            if not wait_proc(c['pgrep'], c['ready_timeout'], c['ready_delay']):
                done('ERROR: openpilot did not start — check logs.'); return
        else:
            st('3/4 — openpilot already running, skipping.')
        if cancel.is_set():
            done(); return

        # 4 / 4 — IMU Monitor
        c = self._comp('imu_monitor')
        if not _pgrep(c['pgrep']):
            st('4/4 — Launching IMU Monitor...')
            launch(c)
        else:
            st('4/4 — IMU Monitor already running, skipping.')

        done('All components launched!')

    def _comp(self, cid: str) -> dict:
        return next(c for c in self.components if c['id'] == cid)

    # ── Log viewer ─────────────────────────────────────────────────────────────

    def _switch_log(self, cid: str) -> None:
        self._active_log = cid
        comp = self._comp(cid)
        lp   = comp.get('log_path') or ''
        self._logpath_lbl.set_text(f'  {lp}')

        # Highlight active tab
        for tid, b in self._tab_btns.items():
            sc = b.get_style_context()
            if tid == cid:
                sc.add_class('active')
            else:
                sc.remove_class('active')

        # Full re-read of tail
        self._buf.set_text('')
        content, size = _read_tail(lp)
        if content:
            self._log_offset[cid] = size
            self._insert(content)
        else:
            self._log_offset[cid] = 0

        if self._autoscroll:
            self._scroll_to_end()

    def _clear_log(self) -> None:
        self._buf.set_text('')

    def _insert(self, text: str) -> None:
        """Append colour-tagged lines to the buffer."""
        for line in text.splitlines(keepends=True):
            if '\x1b[' in line:
                self._insert_ansi(line)
            else:
                ll  = line.lower()
                it  = self._buf.get_end_iter()
                tag = None
                if any(kw in ll for kw in ('traceback', 'fatal', 'crash')):
                    tag = 'err'
                elif 'error' in ll or 'exception' in ll:
                    tag = 'err'
                elif _WARN_RE.search(ll):
                    tag = 'warn'
                elif _OK_RE.search(ll):
                    tag = 'ok'
                if tag:
                    self._buf.insert_with_tags_by_name(it, line, tag)
                else:
                    self._buf.insert(it, line)

        # Trim excess lines
        line_count = self._buf.get_line_count()
        if line_count > MAX_LINES:
            start = self._buf.get_start_iter()
            trim  = self._buf.get_iter_at_line(line_count - MAX_LINES)
            self._buf.delete(start, trim)

    def _insert_ansi(self, line: str) -> None:
        """Insert one line that contains ANSI SGR escape codes."""
        parts    = _ANSI_SPLIT_RE.split(line)
        fg_tag   = None   # current foreground tag name
        bold_tag = None   # 'ansi_bold' when bold is active

        for part in parts:
            if not part:
                continue
            if _ANSI_SPLIT_RE.match(part):
                # Parse SGR codes (e.g. \x1b[1;32m → ['1','32'])
                inner  = part[2:-1]   # strip \x1b[ and m
                codes  = inner.split(';') if inner else ['0']
                for code in codes:
                    if code in ('0', ''):
                        fg_tag   = None
                        bold_tag = None
                    elif code == '1':
                        bold_tag = 'ansi_bold'
                    elif code in _ANSI_FG:
                        fg_tag = f'ansi_{code}'
            else:
                it   = self._buf.get_end_iter()
                tags = [t for t in (fg_tag, bold_tag) if t]
                if tags:
                    self._buf.insert_with_tags_by_name(it, part, *tags)
                else:
                    self._buf.insert(it, part)

    def _append_new_log(self) -> None:
        cid = self._active_log
        comp = self._comp(cid)
        lp   = comp.get('log_path')
        if not lp or not os.path.exists(lp):
            return

        try:
            fsize = os.path.getsize(lp)
        except Exception:
            return

        # File was truncated — reset
        if fsize < self._log_offset[cid]:
            self._log_offset[cid] = 0
            self._buf.set_text('')

        new_text, new_off = _read_from(lp, self._log_offset[cid])
        if not new_text:
            return

        self._log_offset[cid] = new_off
        adj    = self._scrolled.get_vadjustment()
        at_end = adj.get_value() >= adj.get_upper() - adj.get_page_size() - 10

        self._insert(new_text)

        if self._autoscroll and at_end:
            self._scroll_to_end()

    def _scroll_to_end(self) -> None:
        end = self._buf.get_end_iter()
        self._txt.scroll_to_iter(end, 0.0, False, 0.0, 1.0)

    # ── Poll timers ────────────────────────────────────────────────────────────

    def _poll_status(self) -> bool:
        for comp in self.components:
            cid   = comp['id']
            was   = self._running[cid]
            now   = _pgrep(comp['pgrep'])
            self._running[cid] = now
            w     = self._cw[cid]

            if now and not was:
                self._stopped_at[cid] = None
            elif not now and was:
                self._stopped_at[cid] = time.monotonic()

            dot = w['dot']
            sc  = dot.get_style_context()
            if now:
                sc.remove_class('dot-off'); sc.add_class('dot-on')
                w['status'].set_text('running')
                w['status'].get_style_context().remove_class('stopped')
                w['status'].get_style_context().add_class('running')
                w['launch_btn'].set_sensitive(False)
                w['stop_btn'].set_sensitive(True)
            else:
                sc.remove_class('dot-on'); sc.add_class('dot-off')
                st = self._stopped_at[cid]
                if st is None:
                    w['status'].set_text('stopped')
                else:
                    secs = int(time.monotonic() - st)
                    txt  = (f'stopped  {secs}s ago' if secs < 60
                            else f'stopped  {secs // 60}m {secs % 60}s ago')
                    w['status'].set_text(txt)
                w['status'].get_style_context().remove_class('running')
                w['status'].get_style_context().add_class('stopped')
                w['launch_btn'].set_sensitive(True)
                w['stop_btn'].set_sensitive(False)

        # Engage button — BeamNG mode only
        if self._engage_btn is not None:
            bridge_up = self._running.get('bridge', False)
            self._engage_btn.set_sensitive(bridge_up)
            try:
                engaged = bridge_up and os.path.exists(ENGAGED_FILE) and \
                          open(ENGAGED_FILE).read().strip() == '1'
            except Exception:
                engaged = False
            sc = self._engage_btn.get_style_context()
            if engaged:
                sc.remove_class('engage-off'); sc.add_class('engage-on')
                self._engage_btn.set_label('⚡  Disengage')
            else:
                sc.remove_class('engage-on'); sc.add_class('engage-off')
                self._engage_btn.set_label('⚡  Engage')

        return GLib.SOURCE_CONTINUE

    def _poll_log(self) -> bool:
        self._append_new_log()
        return GLib.SOURCE_CONTINUE

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _status(self, msg: str) -> bool:
        ts = datetime.now().strftime('%H:%M:%S')
        self._status_lbl.set_text(f'[{ts}]  {msg}')
        return GLib.SOURCE_REMOVE   # safe to use as idle callback

    def _on_close_request(self, *_) -> bool:
        cfg = {
            'width':     self.get_width(),
            'height':    self.get_height(),
            'paned_pos': self._paned.get_position(),
        }
        _save_gui_config(cfg)
        return False   # False = allow the window to close

    def _kill_all_on_start(self) -> None:
        """Kill every known process at startup so the panel always opens clean."""
        GLib.idle_add(self._status,
                      'Startup cleanup — stopping any running processes...')
        for comp in self.components:
            _kill(comp['stop_pat'])
        _ready_msg = ('Ready — all clear. Click Start All to launch MetaDrive.'
                      if self._mode == MODE_METADRIVE
                      else 'Ready — all clear. Load BeamNG, then click Launch or Start All.')
        GLib.idle_add(self._status, _ready_msg)


# ── Application ────────────────────────────────────────────────────────────────

class BridgeApp(Gtk.Application):
    def __init__(self, dual_camera: bool = False, mode: str = MODE_BEAMNG) -> None:
        super().__init__(application_id='org.beamng.bridge.control')
        self._dual = dual_camera
        self._mode = mode
        self.connect('activate', self._activate)

    def _activate(self, app: Gtk.Application) -> None:
        BridgeWindow(app, dual_camera=self._dual, mode=self._mode).present()


def main() -> None:
    ap = argparse.ArgumentParser(
        description='BeamNG / MetaDrive ↔ openpilot Bridge Control Panel')
    ap.add_argument('--dual-camera', action='store_true', default=True,
                    help='Pass --dual-camera to bridge_runner.py / run_bridge.py')
    ap.add_argument('--metadrive', action='store_true', default=False,
                    help='Use MetaDrive instead of BeamNG')
    args = ap.parse_args()

    mode = MODE_METADRIVE if args.metadrive else MODE_BEAMNG
    BridgeApp(dual_camera=args.dual_camera, mode=mode).run([])


if __name__ == '__main__':
    main()
