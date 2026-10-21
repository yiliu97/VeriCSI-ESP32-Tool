"""
CSI live capture + visualization tool.

Single process, no shell pipe in the data path. csi_live.py spawns
`idf.py monitor` as a subprocess (with PYTHONIOENCODING=utf-8 and
PYTHONUNBUFFERED=1 in its env), reads its stdout directly via
subprocess.PIPE in binary mode, and either renders a live plot in the
SAME process or runs headless. This solves three independent Windows
issues at once:
  1. GBK-locale UnicodeEncodeError when boot ROM bytes pass through
     the Python text wrapper of a downstream pipe (utf-8 in env);
  2. 8 KB block-buffer delay before any data appears (PYTHONUNBUFFERED);
  3. Window stays in background or seems to "freeze" because the
     PowerShell pipe + ANSI conversion layers buffer the data flow
     until idf_monitor exits.

Three independent functions, mix and match:
  * **Live plot** - default, requires no flag. Disable with `--no-plot`.
  * **Record CSV** - `-o FILE` to specify a name, or bare `-o` (no
    filename) to auto-name as `YYYYMMDDHHMM_<CHIP>_<ROLE>.csv` from
    the first CSI frame's chip_id + role fields. All bare-filename
    `-o` outputs land in the project's `code/data/` directory
    (resolved relative to this script — no hard-coded paths). Pass a
    path with separators (`-o subdir/foo.csv` or `-o C:\\foo.csv`)
    to override the destination.
  * **Capture source** - `-p PORT` spawns idf.py monitor (recommended);
    `-i / --stdin` reads from stdin (legacy `idf.py monitor | csi_live.py`
    pipe pattern; needs PYTHONIOENCODING=utf-8 + PYTHONUNBUFFERED=1 set
    in the shell first).

At least one of {live plot, -o FILE} must be active; `--no-plot` without
`-o` is rejected (the script would have nothing to do).

Three clean exit paths share the same cleanup: `-t SECONDS` auto-exit,
window close (plot mode), and Ctrl+C - all flush the CSV, kill the
spawned idf.py subprocess tree, and exit.

Plot modes: line (<=16 subcarriers) / heatmap (>=32; required for
HE-LTF 242 / 484).

Usage (all `-o`-driven CSV files land in `code/data/` unless the path has
separators):
    cd H:\\esp32_csi_project\\code\\active_ap

    # Live plot + record. `-o plot.csv` -> code/data/plot.csv
    python ..\\python_utils\\csi_live.py -p COM3 \\
        --subcarriers 0,8,16,24,32,40,48,56 -t 30 -o plot.csv

    # Live plot + auto-named file. Bare `-o` writes to a
    # code/data/csi_live_pending_<ts>.csv temp file first, then renames
    # to e.g. code/data/202605090150_ESP32C5_AP.csv on the first CSI frame.
    python ..\\python_utils\\csi_live.py -p COM3 \\
        --subcarriers 0,8,16,24,32,40,48,56 -t 30 -o

    # Headless record only (long capture, no GUI). Same data/ destination.
    python ..\\python_utils\\csi_live.py -p COM3 -o capture.csv -t 300 --no-plot

    # Override the destination explicitly with a path
    python ..\\python_utils\\csi_live.py -p COM3 -o ../some/elsewhere.csv -t 30

    # Live plot only (no file written)
    python ..\\python_utils\\csi_live.py -p COM3 -t 30
"""

from __future__ import annotations

import argparse
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from typing import List, Optional, Tuple, IO

# Wall-clock the import phase so we can later report whether slow startup
# was matplotlib import vs figure creation vs IDF data delay.
_t_import_start = time.monotonic()

import numpy as np

_t_import_done = time.monotonic()

# matplotlib import is deferred to main() so that --no-plot mode can run
# in environments without a GUI backend (CI, headless servers, SSH).

from parse_csi import parse_line


# Sentinel for `-o` with no filename argument — argparse stores this as the
# value when the user writes bare `-o` (and nothing or another flag follows).
# Picked to be unambiguous with any plausible filename.
AUTO_OUTPUT_SENTINEL = "<AUTO>"


def _resolve_data_dir() -> str:
    """Return the absolute path of the project data directory (`code/data/`),
    computed relative to this script's location at `code/python_utils/csi_live.py`.
    Creates the directory if missing. No hard-coded absolute path is used."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.normpath(os.path.join(script_dir, "..", "data"))
    os.makedirs(data_dir, exist_ok=True)
    return data_dir


def _resolve_output_path(arg: str) -> str:
    """Map a `-o` argument to the actual file path. Bare filenames land in
    `code/data/`; any argument containing a path separator or being absolute
    is honoured as-given (escape hatch for users who want a custom location)."""
    has_sep = (os.sep in arg) or (os.altsep is not None and os.altsep in arg)
    if has_sep or os.path.isabs(arg):
        return arg
    return os.path.join(_resolve_data_dir(), arg)


def _sanitize_for_filename(s: str, fallback: str = "UNKNOWN") -> str:
    """Replace any character that's not alnum with '_', uppercase the result.
    Returns `fallback` if the cleaned string is empty or was '?'."""
    if not s or s == "?":
        return fallback
    cleaned = "".join(c if c.isalnum() else "_" for c in s).upper()
    cleaned = cleaned.strip("_")
    return cleaned or fallback


def _safe_stderr(text: str) -> None:
    """Write a line to stderr via the binary buffer, bypassing Python's
    text-mode encoder. Required on Chinese Windows where sys.stderr's
    TextIOWrapper is GBK-encoded and either crashes (UnicodeEncodeError
    on U+FFFD) or is block-buffered (so prints stay invisible until
    program exit). Always flushes immediately."""
    try:
        sys.stderr.buffer.write(text.encode("utf-8", errors="replace"))
        if not text.endswith("\n"):
            sys.stderr.buffer.write(b"\n")
        sys.stderr.buffer.flush()
    except Exception:
        pass


def parse_subcarrier_arg(s: str) -> List[int]:
    """Comma list ('11,22,33,44,55') or slice spec ('start:stop[:step]')."""
    s = s.strip()
    if ":" in s:
        spec = s.split(":")
        if len(spec) == 2:
            start, stop = int(spec[0]), int(spec[1])
            step = 1
        elif len(spec) == 3:
            start, stop, step = int(spec[0]), int(spec[1]), int(spec[2])
        else:
            raise argparse.ArgumentTypeError(
                f"slice spec must be 'start:stop' or 'start:stop:step', got '{s}'")
        return sorted(set(range(start, stop, step)))
    return sorted(set(int(x) for x in s.split(",") if x.strip()))


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--subcarriers", default="11,22,33,44,55",
                   help="Subcarriers to plot. Comma list (e.g. '11,22,33,44,55') "
                        "or slice 'start:stop:step' (e.g. '0:64:8'). Default 5 spread.")
    p.add_argument("--mode", choices=["auto", "line", "heatmap"], default="auto",
                   help="auto: line if <=16 subcarriers else heatmap. Default auto.")
    p.add_argument("--window", type=int, default=200,
                   help="Recent samples shown on the x-axis (default 200).")
    p.add_argument("--ymin", type=float, default=0.0,
                   help="Initial y-axis / colorbar lower bound. Default 0.")
    p.add_argument("--ymax", type=float, default=64.0,
                   help="Initial y-axis / colorbar upper bound. Default 64.")
    p.add_argument("--no-autoscale", action="store_true",
                   help="Disable adaptive y-axis growth.")
    p.add_argument("--max-fps", type=float, default=30.0,
                   help="Render frame-rate cap (default 30). 0 = render every sample.")
    p.add_argument("--no-plot", action="store_true",
                   help="Headless mode: no matplotlib window, just stream-and-record. "
                        "Requires -o FILE (otherwise the script would have nothing to do). "
                        "Use this for long unattended captures or on machines without a GUI.")
    p.add_argument("-p", "--port", default=None,
                   help="Serial port (e.g. COM3, /dev/ttyUSB0). When set, csi_live.py "
                        "spawns `idf.py monitor` itself with PYTHONIOENCODING=utf-8 "
                        "+ PYTHONUNBUFFERED=1 in the subprocess env. Recommended.")
    p.add_argument("-i", "--stdin", action="store_true",
                   help="Read from stdin instead of spawning idf.py. Use with "
                        "`idf.py -p COMx monitor | python csi_live.py --stdin ...`. "
                        "Mutually exclusive with -p.")
    p.add_argument("--raw", action="store_true",
                   help="With -p: open the serial port directly via pyserial "
                        "(115200, DTR/RTS deasserted) instead of spawning "
                        "idf.py monitor. REQUIRED on ESP32-C5: the monitor's "
                        "DTR/RTS handling resets or holds the C5 via the "
                        "USB-Serial-JTAG EN/GPIO0 mapping and the capture "
                        "reads 0 samples, while a plain open with both lines "
                        "deasserted streams normally (measured 18.5 fps on "
                        "the same board seconds apart). No panic decoding / "
                        "monitor niceties in this mode - raw line stream only.")
    p.add_argument("-o", "--output-csv", nargs="?", const=AUTO_OUTPUT_SENTINEL,
                   default=None,
                   help="Save every line verbatim (raw bytes, boot log + CSI) "
                        "to a file as it streams. Bare '-o' (no filename) "
                        "auto-generates 'YYYYMMDDHHMM_<CHIP>_<ROLE>.csv' from "
                        "the first CSI line (e.g. 202605090150_ESP32C5_AP.csv); "
                        "until that frame arrives, the script writes to a "
                        "temporary 'csi_live_pending_<ts>.csv' file. All "
                        "bare-filename outputs land in 'code/data/' "
                        "(resolved relative to this script). Pass a path with "
                        "separators ('subdir/x.csv', 'C:\\\\x.csv') to override. "
                        "Default: no save.")
    p.add_argument("-t", "--duration", type=float, default=None,
                   help="Auto-exit after N seconds. Also kills the spawned "
                        "idf.py subprocess if one was started via -p.")
    p.add_argument("--stats-period-s", type=float, default=1.0,
                   help="Stats line cadence to stderr (default 1 s).")
    p.add_argument("--baud", type=int, default=115200,
                   help="Baud rate for --raw mode (default 115200). Irrelevant "
                        "on USB-Serial-JTAG chips (USB CDC ignores baud) but "
                        "REQUIRED on real UART bridges: ESP32-classic/S2 "
                        "consoles run at 921600 (sdkconfig defaults), so pass "
                        "--baud 921600 there or the stream is mojibake.")
    return p


def _spawn_idf_monitor(port: str) -> "subprocess.Popen":
    """Spawn `idf.py -p PORT monitor` with the env vars that prevent the
    GBK-encode crash and the 8 KB-buffer delay. Returns the Popen handle;
    caller reads from .stdout (binary, line-streaming) and is responsible
    for terminating it (e.g. via taskkill /F /T /PID on Windows).

    Resolves idf.py via os.environ['IDF_PATH'] (set by IDF Terminal) and
    invokes it via sys.executable so the IDF venv's Python loads idf_monitor."""
    idf_path = os.environ.get("IDF_PATH")
    if not idf_path:
        raise RuntimeError(
            "IDF_PATH not set; run this script from an IDF Terminal "
            "(start menu -> ESP-IDF X.X PowerShell) or set IDF_PATH manually.")
    idf_py = os.path.join(idf_path, "tools", "idf.py")
    if not os.path.isfile(idf_py):
        raise RuntimeError(f"idf.py not found at {idf_py}")

    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUNBUFFERED"] = "1"

    cmd = [sys.executable, idf_py, "-p", port, "monitor"]
    creationflags = 0
    if sys.platform.startswith("win"):
        creationflags = subprocess.CREATE_NEW_PROCESS_GROUP

    return subprocess.Popen(
        cmd,
        shell=False,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=0,
        creationflags=creationflags,
    )


def _kill_proc_tree(proc: "subprocess.Popen") -> None:
    """Kill subprocess and all descendants. Required on Windows because
    idf.py spawns idf_monitor.py as a grandchild; proc.terminate() alone
    leaves the grandchild running and reading the serial port."""
    if proc.poll() is not None:
        return
    if sys.platform.startswith("win"):
        try:
            subprocess.run(
                ["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5,
            )
            return
        except Exception:
            pass
    try:
        proc.terminate()
    except Exception:
        pass


def amplitude_for_subcarriers(csi_raw: List[int],
                              subcarriers: np.ndarray) -> np.ndarray:
    """Hernandez 2020 convention: csi_raw[2i] = imag, csi_raw[2i+1] = real."""
    arr = np.asarray(csi_raw, dtype=np.int32)
    n_complex = arr.size // 2
    if subcarriers.max() >= n_complex:
        subcarriers = subcarriers[subcarriers < n_complex]
        if subcarriers.size == 0:
            return np.empty(0, dtype=np.float32)
    imag = arr[0::2][subcarriers].astype(np.float32)
    real = arr[1::2][subcarriers].astype(np.float32)
    return np.sqrt(imag * imag + real * real)


def main() -> int:
    args = build_argparser().parse_args()
    if args.port and args.stdin:
        _safe_stderr("error: -p/--port and -i/--stdin are mutually exclusive")
        return 2
    if not args.port and not args.stdin:
        _safe_stderr("error: must specify -p PORT (recommended) or -i/--stdin "
                     "(legacy pipe mode)")
        return 2
    if args.no_plot and not args.output_csv:
        _safe_stderr("error: --no-plot requires -o FILE (script would have "
                     "nothing to do otherwise)")
        return 2
    plot_enabled = not args.no_plot

    # Defer matplotlib import to here so headless mode (e.g. CI / SSH)
    # never touches Tk and the import cost only hits plot-mode runs.
    matplotlib = None
    plt = None
    if plot_enabled:
        import matplotlib as _mpl
        try:
            _mpl.use("TkAgg", force=True)
        except Exception:
            pass
        import matplotlib.pyplot as _plt
        matplotlib, plt = _mpl, _plt

    subcarriers = parse_subcarrier_arg(args.subcarriers)
    if not subcarriers:
        _safe_stderr("error: at least one subcarrier required")
        return 2
    sc_array = np.asarray(subcarriers, dtype=np.int64)
    n_sc = len(subcarriers)
    window = args.window

    mode = args.mode
    if mode == "auto":
        mode = "line" if n_sc <= 16 else "heatmap"
    if plot_enabled:
        _safe_stderr(f"[csi_live] subcarriers={n_sc} mode={mode} window={window} "
                     f"max_fps={args.max_fps}")
        _safe_stderr(f"[csi_live] matplotlib backend: {matplotlib.get_backend()}")
    else:
        _safe_stderr(f"[csi_live] HEADLESS RECORD mode (no matplotlib window)")
    _safe_stderr(f"[csi_live] startup: imports took "
                 f"{_t_import_done - _t_import_start:.2f}s")
    # Note: the `tee -> ...` line is now printed by the file-open block below
    # (which knows whether we're using auto-name temp file or explicit name).
    if args.duration is not None:
        _safe_stderr(f"[csi_live] duration: {args.duration:.0f}s (auto-exit)")
    _t_main_start = time.monotonic()

    # 2D numpy ring buffer (n_sc, window). data[:, filled-1] = newest sample
    # while filling; once filled == window, shift-left + append at last col.
    # Allocated even in headless mode for code symmetry — it's cheap and the
    # autoscale path can stay shared.
    data = np.zeros((n_sc, window), dtype=np.float32)
    filled = 0
    xs_full = np.arange(window, dtype=np.float32)

    fig = None
    ax = None
    lines = []
    im = None
    y_min = float(args.ymin)
    y_max = float(args.ymax)
    window_closed = {"value": False}

    if plot_enabled:
        # No foreground / topmost hacks — they didn't help anyway.
        plt.ion()
        fig, ax = plt.subplots(figsize=(11, 6))
        ax.set_xlim(0, window)
        ax.set_xlabel("Sample index (oldest -> newest, rolling)")

        if mode == "line":
            ax.set_ylim(y_min, y_max)
            ax.set_ylabel("Amplitude")
            ax.set_title(f"CSI amplitude across {n_sc} subcarriers (window={window})")
            cmap = plt.get_cmap("tab10" if n_sc <= 10 else "tab20")
            for i, sc in enumerate(subcarriers):
                line, = ax.plot(xs_full, data[i], color=cmap(i % cmap.N),
                                linewidth=1.0, label=f"sc#{sc}")
                lines.append(line)
            ax.legend(loc="upper right", fontsize=8, ncol=max(1, n_sc // 8))
            ax.grid(True, alpha=0.3)
        else:  # heatmap
            ax.set_ylim(0, n_sc)
            ax.set_ylabel("Subcarrier (selected)")
            ax.set_title(f"CSI amplitude heatmap, {n_sc} subcarriers x {window} samples")
            im = ax.imshow(data, aspect="auto", origin="lower",
                           extent=(0, window, 0, n_sc),
                           interpolation="nearest",
                           vmin=y_min, vmax=y_max,
                           cmap="viridis")
            cbar = fig.colorbar(im, ax=ax, fraction=0.04, pad=0.02)
            cbar.set_label("Amplitude")

        plt.show(block=False)
        fig.canvas.draw_idle()
        plt.pause(0.05)
        _safe_stderr(f"[csi_live] startup: figure ready "
                     f"({time.monotonic() - _t_main_start:.2f}s)")

        # Detect window close so the user can stop the script by closing the
        # plot window (more intuitive than Ctrl+C). Without this, the main
        # loop keeps spinning + idf.py keeps running + serial port stays open
        # after the window is gone.
        def _on_close(event):
            window_closed["value"] = True
        fig.canvas.mpl_connect("close_event", _on_close)

    # Reader thread: blocks on the input stream (either spawned idf.py
    # subprocess stdout or our own stdin), parses CSI lines, queues amps.
    # Reads raw bytes so EOF (b'') is distinguishable from empty line ('\n').
    SAMPLE_QUEUE_MAX = 4096
    sample_q: "queue.Queue[Optional[Tuple[np.ndarray, str, str, int]]]" = \
        queue.Queue(maxsize=SAMPLE_QUEUE_MAX)
    drop_count = {"value": 0}
    # Parsed CSI lines, counted BEFORE the plot-subcarrier filter. The
    # `amps.size != n_sc` check below silently discards every sample whose
    # layout does not match the plot's subcarrier selection (e.g. C5's
    # 53-subcarrier frames vs defaults tuned for ESP32-classic lengths), so
    # `samples=` alone read 0 during a perfectly healthy C5 capture and sent
    # the 2026-08-01 bring-up on a phantom serial-stack chase. The file tee
    # happens before parsing and is unaffected; this counter makes the stats
    # line tell that truth.
    parsed_count = {"value": 0}
    # Samples parsed but NOT queued because their subcarrier layout does not
    # match the plot's subcarrier selection. Non-zero here with plotted=0 is
    # the normal signature of capturing a chip whose layout the plot defaults
    # don't cover (e.g. C5's 53 subcarriers) - the FILE gets everything.
    plot_skip = {"value": 0}
    # Identity of the last PARSED line, for the stats display. Previously the
    # stats showed chip/layout/len only after a sample passed the plot filter,
    # so a healthy C5 capture displayed `chip=? layout=- len=0` forever.
    last_info = {"chip": "?", "layout": "-", "len": 0}
    # output_state["fp"] is the active write target. When bare `-o` is used,
    # we write to a temporary file first and rename it in the reader thread
    # after the first CSI line arrives (which is when chip_id and role become
    # known). The dict-of-fp pattern keeps the reference live across the
    # close+rename+reopen sequence so cleanup_and_exit doesn't get a stale fp.
    # data_dir is the canonical landing zone for `-o` outputs. We resolve it
    # once here so the reader thread can reuse it for the auto-rename target
    # without recomputing.
    data_dir = _resolve_data_dir() if (args.output_csv == AUTO_OUTPUT_SENTINEL
                                       or args.output_csv) else None
    output_state = {"fp": None, "name": None,
                    "auto_rename_pending": False, "data_dir": data_dir}
    if args.output_csv == AUTO_OUTPUT_SENTINEL:
        ts = time.strftime("%Y%m%d%H%M")
        # The pending name MUST be unique per process. It used to be
        # timestamped to the minute only, so two concurrent captures started
        # within the same minute - exactly what a two-window AP + RX capture
        # does - opened the SAME path and silently clobbered each other,
        # leaving one truncated file instead of two. The PID makes collision
        # impossible regardless of start time.
        temp_name = os.path.join(
            data_dir, f"csi_live_pending_{ts}_pid{os.getpid()}.csv")
        try:
            output_state["fp"] = open(temp_name, "wb")
            output_state["name"] = temp_name
            output_state["auto_rename_pending"] = True
            _safe_stderr(f"[csi_live] tee -> {temp_name} "
                         f"(will auto-rename to YYYYMMDDHHMM_<CHIP>_<ROLE>.csv "
                         f"after first CSI line)")
        except Exception as e:
            _safe_stderr(f"[csi_live] could not open temp output {temp_name}: {e}")
    elif args.output_csv:
        resolved = _resolve_output_path(args.output_csv)
        try:
            output_state["fp"] = open(resolved, "wb")
            output_state["name"] = resolved
            if resolved != args.output_csv:
                _safe_stderr(f"[csi_live] tee -> {resolved} "
                             f"(routed bare filename to project data/)")
            else:
                _safe_stderr(f"[csi_live] tee -> {resolved}")
        except Exception as e:
            _safe_stderr(f"[csi_live] could not open --output-csv {resolved}: {e}")

    # Resolve the input stream: raw pyserial vs spawned subprocess vs stdin.
    idf_proc: "Optional[subprocess.Popen]" = None
    input_stream: "IO[bytes]"
    if args.port and args.raw:
        try:
            import serial  # noqa: F401  pyserial ships with the IDF venv

            class _RawSerialStream:
                """readline()-compatible raw port reader with the
                rolling-fresh-handle discipline. A Windows serial handle
                opened across a device reset (USB re-enumeration) can go
                stale WITHOUT raising - it just reads nothing forever. So:
                open with DTR/RTS deasserted (they map to GPIO0/EN on
                USB-Serial-JTAG chips and would reset/hold the target),
                and whenever the port stays silent for ~4 s or throws,
                drop the handle and take a fresh one. Never returns EOF;
                the -t duration timer is the session terminator."""

                def __init__(self, port: str, baud: int = 115200) -> None:
                    self._port = port
                    self._baud = baud
                    self._ser = None

                def _open(self):
                    import serial as _serial
                    s = _serial.Serial()
                    s.port = self._port
                    s.baudrate = self._baud
                    s.dtr = False
                    s.rts = False
                    s.timeout = 2.0
                    s.open()
                    self._ser = s

                def readline(self) -> bytes:
                    silent_reads = 0
                    while True:
                        if self._ser is None:
                            try:
                                self._open()
                            except Exception:
                                time.sleep(0.3)
                                continue
                        try:
                            data = self._ser.readline()
                        except Exception:
                            try:
                                self._ser.close()
                            except Exception:
                                pass
                            self._ser = None
                            continue
                        if data:
                            return data
                        silent_reads += 1
                        if silent_reads >= 2:      # ~4 s of silence
                            silent_reads = 0
                            try:
                                self._ser.close()
                            except Exception:
                                pass
                            self._ser = None       # stale-handle guard

            input_stream = _RawSerialStream(args.port, args.baud)
            _safe_stderr(f"[csi_live] raw serial mode on {args.port} "
                         f"({args.baud}, DTR/RTS deasserted, rolling reopen, "
                         f"no idf.py monitor)")
        except Exception as e:
            _safe_stderr(f"[csi_live] failed to open {args.port} raw: {e}")
            return 3
    elif args.port:
        try:
            idf_proc = _spawn_idf_monitor(args.port)
            _safe_stderr(f"[csi_live] spawned idf.py monitor on {args.port} "
                         f"(PID={idf_proc.pid}, env: PYTHONIOENCODING=utf-8 "
                         f"PYTHONUNBUFFERED=1)")
            input_stream = idf_proc.stdout    # binary pipe, bufsize=0
        except Exception as e:
            _safe_stderr(f"[csi_live] failed to spawn idf.py: {e}")
            return 3
    else:
        input_stream = sys.stdin.buffer
        _safe_stderr(f"[csi_live] reading from stdin (legacy pipe mode)")

    def _reader_loop() -> None:
        """Reads the input stream, tees raw bytes to --output-csv (if any),
        echoes pre-CSI boot log to stderr (so the user can see idf.py's
        startup progress during the ~5-20 s before the first CSI frame),
        parses CSI lines, queues amps tuples to the main thread.

        On the first CSI line, if `-o` was given without a filename, the
        accumulated temp file is closed, renamed to
        'YYYYMMDDHHMM_<CHIP>_<ROLE>.csv' (chip_id + role read from the
        first parsed record), and reopened in append mode so the boot log
        already written stays in the file."""
        seen_first_csi = False
        try:
            while True:
                raw = input_stream.readline()
                if not raw:
                    sample_q.put(None)
                    return
                # Tee raw bytes to file before any decoding.
                fp = output_state["fp"]
                if fp is not None:
                    try:
                        fp.write(raw)
                        fp.flush()
                    except Exception:
                        pass
                try:
                    line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                except Exception:
                    continue
                if "CSI_DATA" not in line:
                    # Echo pre-CSI lines (idf.py monitor banner + ESP32 boot
                    # log) to stderr so the user has visibility into what
                    # idf.py is doing during its 5-20 s startup. After the
                    # first CSI line we suppress to avoid stderr clutter
                    # (CSV header, # DIAG lines etc. will still pass).
                    if not seen_first_csi and line:
                        try:
                            sys.stderr.buffer.write(raw)
                            sys.stderr.buffer.flush()
                        except Exception:
                            pass
                    continue
                seen_first_csi = True
                rec = parse_line(line)
                if rec is None:
                    continue
                parsed_count["value"] += 1
                last_info["chip"] = rec.get("chip_id", "?")
                last_info["layout"] = rec.get("subcarrier_layout_id", "-")
                last_info["len"] = rec.get("len", 0)

                # First CSI line + bare `-o` flag: rename temp file to
                # the canonical 'YYYYMMDDHHMM_<CHIP>_<ROLE>.csv' name in
                # the project data directory.
                if output_state["auto_rename_pending"]:
                    try:
                        ts = time.strftime("%Y%m%d%H%M")
                        chip_part = _sanitize_for_filename(rec.get("chip_id", ""))
                        role_part = _sanitize_for_filename(rec.get("role", ""))
                        base = os.path.join(
                            output_state["data_dir"],
                            f"{ts}_{chip_part}_{role_part}")
                        # Never clobber an existing capture. The timestamp is
                        # minute-resolution, so two captures of the same role
                        # started in the same minute would otherwise have the
                        # second os.replace() silently destroy the first.
                        new_name = f"{base}.csv"
                        dup = 2
                        while os.path.exists(new_name):
                            new_name = f"{base}_{dup}.csv"
                            dup += 1
                        # Cleanup may have run concurrently (auto-exit timer,
                        # Ctrl+C); it clears fp and the pending flag first, so
                        # bail out rather than touching a closed handle.
                        if cleanup_done["value"]:
                            break
                        old_fp = output_state["fp"]
                        old_name = output_state["name"]
                        if old_fp is not None and not old_fp.closed:
                            old_fp.flush()
                            old_fp.close()
                        if old_name and old_name != new_name:
                            try:
                                os.replace(old_name, new_name)
                            except OSError as e:
                                _safe_stderr(f"[csi_live] auto-rename "
                                             f"{old_name} -> {new_name} failed "
                                             f"({e}); keeping {old_name}")
                                new_name = old_name
                        # Reopen in append-binary mode so the boot log already
                        # written stays in the file and subsequent CSI lines
                        # append after it.
                        output_state["fp"] = open(new_name, "ab")
                        output_state["name"] = new_name
                        output_state["auto_rename_pending"] = False
                        _safe_stderr(f"[csi_live] auto-named output: {new_name}")
                    except Exception as e:
                        _safe_stderr(f"[csi_live] auto-rename error: "
                                     f"{type(e).__name__}: {e}")
                        output_state["auto_rename_pending"] = False

                amps = amplitude_for_subcarriers(rec["csi_raw"], sc_array)
                if amps.size != n_sc:
                    plot_skip["value"] += 1
                    continue
                item = (amps,
                        rec.get("chip_id", "?"),
                        rec.get("subcarrier_layout_id", "-"),
                        rec["len"])
                try:
                    sample_q.put(item, block=False)
                except queue.Full:
                    drop_count["value"] += 1
        except Exception as e:
            _safe_stderr(f"[csi_live reader] FATAL: {type(e).__name__}: {e}")
            sample_q.put(None)
            return

    reader = threading.Thread(target=_reader_loop, daemon=True, name="csi_reader")
    reader.start()

    # Centralised cleanup so duration timer / window close / KeyboardInterrupt
    # / signal handler all use the same shutdown path. Idempotent.
    cleanup_done = {"value": False}
    def _cleanup_and_exit(reason: str, exit_code: int = 0):
        if cleanup_done["value"]:
            return
        cleanup_done["value"] = True
        _safe_stderr(f"[csi_live] cleanup: {reason}")
        # Clear the shared state BEFORE closing. The reader thread may be
        # inside the auto-rename block, which flushes output_state["fp"]; if
        # cleanup closes the handle while leaving it in place, that flush
        # raises "ValueError: flush of closed file". Observed 2026-07-30 on a
        # passive capture that produced no CSI at all.
        was_pending = bool(output_state.get("auto_rename_pending"))
        fp = output_state["fp"]
        output_state["fp"] = None
        output_state["auto_rename_pending"] = False
        try:
            if fp is not None:
                fp.flush()
                fp.close()
        except Exception:
            pass
        if was_pending:
            _safe_stderr(f"[csi_live] note: no CSI line arrived, output kept "
                         f"as {output_state['name']} (rename it manually if "
                         f"you want the canonical YYYYMMDDHHMM_<CHIP>_<ROLE> "
                         f"name)")
        if idf_proc is not None:
            _kill_proc_tree(idf_proc)
        os._exit(exit_code)

    # Signal handler for Ctrl+C. Without this, Ctrl+C only delivers
    # KeyboardInterrupt to the main thread but leaves the spawned idf.py
    # subprocess running (holding the COM port). The handler runs in any
    # thread context so it kills the subprocess synchronously before exit.
    def _sigint_handler(signum, frame):
        _cleanup_and_exit("SIGINT (Ctrl+C)", exit_code=130)
    signal.signal(signal.SIGINT, _sigint_handler)

    # Optional auto-exit timer.
    if args.duration is not None:
        threading.Timer(args.duration,
                        lambda: _cleanup_and_exit(
                            f"duration {args.duration:.0f}s elapsed")).start()

    # Main loop: drain queue, render at --max-fps, pump GUI events always.
    target_period = 1.0 / args.max_fps if args.max_fps > 0 else 0.0
    last_render_t = 0.0
    last_stats_t = time.monotonic()
    sample_count_this_period = 0
    render_count_this_period = 0
    total_samples = 0
    parsed_prev = 0
    first_render_done = False
    last_chip = "?"
    last_layout = "-"
    last_len = 0
    QUEUE_GET_TIMEOUT = 0.010

    try:
      while True:
        if plot_enabled and window_closed["value"]:
            _cleanup_and_exit("window closed by user")
        try:
            item = sample_q.get(timeout=QUEUE_GET_TIMEOUT)
        except queue.Empty:
            item = "no-data"

        eof = False
        if item is None:
            eof = True
        elif item != "no-data":
            amps, chip, layout, length = item
            if filled < window:
                data[:, filled] = amps
                filled += 1
            else:
                data[:, :-1] = data[:, 1:]
                data[:, -1] = amps
            total_samples += 1
            sample_count_this_period += 1
            last_chip = chip
            last_layout = layout
            last_len = length

            if plot_enabled and not args.no_autoscale:
                this_max = float(amps.max())
                this_min = float(amps.min())
                rescaled = False
                if this_max > y_max:
                    y_max = max(this_max * 1.1, y_max * 1.1)
                    rescaled = True
                if this_min < y_min:
                    if this_min > 0:
                        y_min = this_min * 0.9
                    else:
                        y_min = this_min - abs(this_min) * 0.1
                    rescaled = True
                if rescaled:
                    if mode == "line":
                        ax.set_ylim(y_min, y_max)
                    else:
                        im.set_clim(y_min, y_max)

            # Drain any additional queued items (without rendering between).
            for _ in range(64):
                try:
                    nxt = sample_q.get_nowait()
                except queue.Empty:
                    break
                if nxt is None:
                    eof = True
                    break
                amps, chip, layout, length = nxt
                if filled < window:
                    data[:, filled] = amps
                    filled += 1
                else:
                    data[:, :-1] = data[:, 1:]
                    data[:, -1] = amps
                total_samples += 1
                sample_count_this_period += 1
                last_chip = chip
                last_layout = layout
                last_len = length

        now = time.monotonic()

        if plot_enabled:
            do_render = (
                (args.max_fps > 0 and now - last_render_t >= target_period)
                or (args.max_fps == 0 and total_samples > 0)
            )

            if do_render and filled > 0:
                last_render_t = now
                if mode == "line":
                    if filled == window:
                        for i, line in enumerate(lines):
                            line.set_data(xs_full, data[i])
                    else:
                        xs_partial = xs_full[:filled]
                        for i, line in enumerate(lines):
                            line.set_data(xs_partial, data[i, :filled])
                else:
                    im.set_data(data)
                fig.canvas.draw()
                render_count_this_period += 1
                if not first_render_done and total_samples > 0:
                    first_render_done = True
                    _safe_stderr(f"[csi_live] first render at total_samples={total_samples} "
                                 f"filled={filled} (T+"
                                 f"{now - _t_main_start:.1f}s)")

            # Pump GUI event loop only in plot mode.
            try:
                fig.canvas.start_event_loop(0.001)
            except Exception:
                fig.canvas.flush_events()
        else:
            # Headless mode: yield CPU briefly so the reader thread can fill
            # the queue and stats lines stay readable.
            time.sleep(0.005)

        elapsed = now - last_stats_t
        if elapsed >= args.stats_period_s:
            parsed_now = parsed_count["value"]
            parse_fps = (parsed_now - parsed_prev) / elapsed
            parsed_prev = parsed_now
            ident = (f"chip={last_info['chip']} layout={last_info['layout']} "
                     f"len={last_info['len']}")
            if plot_enabled:
                sample_fps = sample_count_this_period / elapsed
                render_fps = render_count_this_period / elapsed
                _safe_stderr(f"[csi_live] parsed={parsed_now} "
                             f"parse_fps={parse_fps:.0f} "
                             f"plotted={total_samples} "
                             f"sample_fps={sample_fps:.0f} render_fps={render_fps:.0f} "
                             f"plot_skip={plot_skip['value']} "
                             f"qlen={sample_q.qsize()} drops={drop_count['value']} "
                             f"{ident}")
            else:
                # Headless capture: the plot-side counters (plotted / qlen /
                # drops) are noise here and used to bury the one number that
                # matters; show parse throughput + stream identity only.
                _safe_stderr(f"[csi_live] parsed={parsed_now} "
                             f"parse_fps={parse_fps:.0f} {ident}")
            sample_count_this_period = 0
            render_count_this_period = 0
            last_stats_t = now

        if eof:
            _safe_stderr(f"[csi_live] EOF, total samples={total_samples}")
            _cleanup_and_exit("input EOF")

    except KeyboardInterrupt:
        _cleanup_and_exit("KeyboardInterrupt", exit_code=130)
    except Exception as e:
        _safe_stderr(f"[csi_live] FATAL in main loop: {type(e).__name__}: {e}")
        _cleanup_and_exit(f"exception: {e}", exit_code=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
