"""
GRBL 1.1 serial driver.

Runs the serial port on two background threads:
  * writer  -- pulls G-code off a queue and streams it using GRBL's
               character-counting protocol (keeps the 128-byte RX buffer full
               so motion never stutters between lines)
  * reader  -- consumes 'ok' / 'error:' / '<status>' / '[msg]' lines

Realtime bytes (?, !, ~, 0x18, 0x85) bypass the queue entirely and are written
straight to the port, which is what makes the E-STOP and feed-hold instant.

A SIM mode is included so the UI can be developed and demoed with no hardware
attached: pass port="SIM".
"""

from __future__ import annotations

import re
import threading
import time
from collections import deque
from typing import Callable, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None

RX_BUFFER_SIZE = 127          # GRBL 1.1 serial RX buffer
STATUS_INTERVAL = 0.2         # seconds between '?' polls

# Realtime command bytes
CMD_STATUS = b"?"
CMD_FEED_HOLD = b"!"
CMD_CYCLE_START = b"~"
CMD_SOFT_RESET = b"\x18"
CMD_JOG_CANCEL = b"\x85"

STATUS_RE = re.compile(r"<([^>]*)>")

GRBL_ERRORS = {
    "1": "G-code letter with no number",
    "2": "Bad number format in G-code",
    "3": "Unsupported '$' system command",
    "8": "'$' command needs Idle state",
    "9": "G-code locked out during alarm or jog",
    "10": "Soft limits need homing enabled ($22)",
    "15": "Jog target exceeds machine travel",
    "16": "Jog command missing '=' or has prohibited G-code",
    "20": "Unsupported or invalid G-code command",
    "22": "Feed rate has not been set",
    "24": "Two G-code commands need the same axis words",
    "25": "Repeated G-code word",
    "33": "Invalid target for the motion command",
}

GRBL_ALARMS = {
    "1": "Hard limit triggered. Position is lost -- re-home or re-zero.",
    "2": "Soft limit: the move would exceed machine travel.",
    "3": "Reset while in motion. Position is lost.",
    "8": "Homing failed: could not find the limit switch.",
    "9": "Homing failed: switch not cleared during pull-off.",
}


class GrblController:
    def __init__(self, on_log: Optional[Callable[[str, str], None]] = None):
        self._ser = None
        self._sim = False
        self._lock = threading.RLock()
        self._queue: deque = deque()
        self._sent_lengths: deque = deque()
        self._stop = threading.Event()
        self._threads: list = []
        self._on_log = on_log or (lambda level, msg: None)

        self.connected = False
        self.port_name = ""
        self.version = ""
        self.state = "Disconnected"
        self.mpos = [0.0, 0.0, 0.0]     # machine position, DEGREES
        self.wpos = [0.0, 0.0, 0.0]     # work position, DEGREES
        self.wco = [0.0, 0.0, 0.0]
        self.feed = 0.0
        self.spindle = 0.0
        self.last_error = ""
        self.alarm = ""
        self.settings: dict = {}
        self.last_status_time = 0.0

    # ------------------------------------------------------------------
    # Port discovery / connection
    # ------------------------------------------------------------------
    @staticmethod
    def list_ports() -> list:
        out = [{"device": "SIM", "description": "Simulator (no hardware)"}]
        if list_ports is None:
            return out
        for p in list_ports.comports():
            out.append(
                {
                    "device": p.device,
                    "description": f"{p.description}",
                    "hwid": p.hwid,
                }
            )
        return out

    def connect(self, port: str, baud: int = 115200) -> tuple[bool, str]:
        if self.connected:
            self.disconnect()

        self._stop.clear()
        self._queue.clear()
        self._sent_lengths.clear()
        self.alarm = ""
        self.last_error = ""

        if port.upper() == "SIM":
            self._sim = True
            self._ser = None
            self.connected = True
            self.port_name = "SIM"
            self.version = "Grbl 1.1f (simulated)"
            self.state = "Idle"
            self._log("info", "Connected to simulator -- no motors will move.")
            self._start_threads()
            return True, "Simulator connected"

        if serial is None:
            return False, "pyserial is not installed (pip install pyserial)"

        try:
            self._ser = serial.Serial(port, baud, timeout=0.1, write_timeout=2)
        except Exception as exc:  # noqa: BLE001
            return False, f"Could not open {port}: {exc}"

        self._sim = False
        self.connected = True
        self.port_name = port
        self.state = "Connecting"
        # The UNO reboots when the port opens; GRBL takes ~2 s to print its
        # banner. Swallow the boot noise before we start streaming.
        time.sleep(2.0)
        try:
            self._ser.reset_input_buffer()
        except Exception:  # noqa: BLE001
            pass

        self._start_threads()
        self._log("info", f"Opened {port} at {baud} baud")
        self.write_realtime(CMD_STATUS)
        self.send("$I")     # build info -> version string
        self.send("$$")     # dump settings
        self.send("$G")     # parser state
        return True, f"Connected to {port}"

    def disconnect(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.5)
        self._threads = []
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
        self._ser = None
        self.connected = False
        self._sim = False
        self.state = "Disconnected"
        self._log("info", "Disconnected")

    def _start_threads(self) -> None:
        self._threads = [
            threading.Thread(target=self._writer_loop, daemon=True),
            threading.Thread(target=self._reader_loop, daemon=True),
            threading.Thread(target=self._status_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()

    # ------------------------------------------------------------------
    # Sending
    # ------------------------------------------------------------------
    def send(self, line: str) -> None:
        """Queue one G-code / $ command line."""
        line = line.strip()
        if not line:
            return
        with self._lock:
            self._queue.append(line)

    def send_many(self, lines) -> None:
        with self._lock:
            for line in lines:
                line = line.strip()
                if line:
                    self._queue.append(line)

    def write_realtime(self, byte: bytes) -> None:
        if not self.connected:
            return
        if self._sim:
            self._handle_sim_realtime(byte)
            return
        try:
            self._ser.write(byte)
            self._ser.flush()
        except Exception as exc:  # noqa: BLE001
            self._log("error", f"Serial write failed: {exc}")

    def queue_depth(self) -> int:
        with self._lock:
            return len(self._queue)

    def clear_queue(self) -> None:
        with self._lock:
            self._queue.clear()

    # ---- convenience wrappers ----------------------------------------
    def feed_hold(self) -> None:
        self.write_realtime(CMD_FEED_HOLD)

    def resume(self) -> None:
        self.write_realtime(CMD_CYCLE_START)

    def jog_cancel(self) -> None:
        self.write_realtime(CMD_JOG_CANCEL)

    def soft_reset(self) -> None:
        """E-STOP. Kills queued motion and resets GRBL immediately."""
        self.clear_queue()
        with self._lock:
            self._sent_lengths.clear()
        self.write_realtime(CMD_JOG_CANCEL)
        self.write_realtime(CMD_FEED_HOLD)
        time.sleep(0.05)
        self.write_realtime(CMD_SOFT_RESET)
        self._log("warn", "SOFT RESET sent -- machine position may be lost.")

    def unlock(self) -> None:
        self.alarm = ""
        self.send("$X")

    def home(self) -> None:
        self.send("$H")

    # ------------------------------------------------------------------
    # Threads
    # ------------------------------------------------------------------
    def _writer_loop(self) -> None:
        while not self._stop.is_set():
            line = None
            with self._lock:
                if self._queue:
                    candidate = self._queue[0]
                    pending = sum(self._sent_lengths)
                    if pending + len(candidate) + 1 < RX_BUFFER_SIZE:
                        line = self._queue.popleft()
                        self._sent_lengths.append(len(line) + 1)
            if line is None:
                time.sleep(0.005)
                continue

            self._log("tx", line)
            if self._sim:
                self._handle_sim_line(line)
                continue
            try:
                self._ser.write((line + "\n").encode("ascii", "ignore"))
            except Exception as exc:  # noqa: BLE001
                self._log("error", f"Serial write failed: {exc}")
                with self._lock:
                    if self._sent_lengths:
                        self._sent_lengths.pop()
                time.sleep(0.2)

    def _reader_loop(self) -> None:
        buf = b""
        while not self._stop.is_set():
            if self._sim:
                time.sleep(0.05)
                continue
            try:
                chunk = self._ser.read(256)
            except Exception as exc:  # noqa: BLE001
                self._log("error", f"Serial read failed: {exc}")
                time.sleep(0.3)
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._handle_line(raw.decode("ascii", "ignore").strip())

    def _status_loop(self) -> None:
        while not self._stop.is_set():
            self.write_realtime(CMD_STATUS)
            time.sleep(STATUS_INTERVAL)

    # ------------------------------------------------------------------
    # Parsing
    # ------------------------------------------------------------------
    def _handle_line(self, line: str) -> None:
        if not line:
            return

        if line.startswith("<"):
            self._parse_status(line)
            return

        if line == "ok":
            with self._lock:
                if self._sent_lengths:
                    self._sent_lengths.popleft()
            return

        if line.startswith("error:"):
            code = line.split(":", 1)[1].strip()
            self.last_error = f"error:{code} -- {GRBL_ERRORS.get(code, 'see GRBL docs')}"
            self._log("error", self.last_error)
            with self._lock:
                if self._sent_lengths:
                    self._sent_lengths.popleft()
            return

        if line.startswith("ALARM:"):
            code = line.split(":", 1)[1].strip()
            self.alarm = f"ALARM:{code} -- {GRBL_ALARMS.get(code, 'see GRBL docs')}"
            self.state = "Alarm"
            self.clear_queue()
            with self._lock:
                self._sent_lengths.clear()
            self._log("error", self.alarm)
            return

        if line.startswith("$") and "=" in line:
            key, _, val = line.partition("=")
            self.settings[key.strip()] = val.strip()
            self._log("rx", line)
            return

        if line.startswith("Grbl"):
            self.version = line
            self.state = "Idle"
            with self._lock:
                self._sent_lengths.clear()
            self._log("info", line)
            return

        if line.startswith("[VER:") or line.startswith("[OPT:"):
            self.version = self.version or line
        self._log("rx", line)

    def _parse_status(self, line: str) -> None:
        m = STATUS_RE.search(line)
        if not m:
            return
        fields = m.group(1).split("|")
        self.state = fields[0].split(":")[0]
        if self.state != "Alarm":
            self.alarm = ""
        for f in fields[1:]:
            key, _, val = f.partition(":")
            parts = val.split(",")
            try:
                if key == "MPos":
                    self.mpos = [float(v) for v in parts[:3]]
                    self.wpos = [m_ - w for m_, w in zip(self.mpos, self.wco)]
                elif key == "WPos":
                    self.wpos = [float(v) for v in parts[:3]]
                    self.mpos = [w + o for w, o in zip(self.wpos, self.wco)]
                elif key == "WCO":
                    self.wco = [float(v) for v in parts[:3]]
                elif key == "FS":
                    self.feed = float(parts[0])
                    self.spindle = float(parts[1]) if len(parts) > 1 else 0.0
                elif key == "F":
                    self.feed = float(parts[0])
            except ValueError:
                pass
        self.last_status_time = time.time()

    # ------------------------------------------------------------------
    # Simulator
    # ------------------------------------------------------------------
    def _handle_sim_line(self, line: str) -> None:
        with self._lock:
            if self._sent_lengths:
                self._sent_lengths.popleft()

        up = line.upper()
        if up.startswith("$J=") or up.startswith("G0") or up.startswith("G1") \
                or up.startswith("G90") or up.startswith("G91"):
            self._sim_move(up)
        elif up == "$H":
            self.wpos = [0.0, 90.0, 0.0]
            self.mpos = list(self.wpos)
            self._log("info", "[SIM] homed")
        elif up.startswith("G10") or up.startswith("G92"):
            self._log("info", f"[SIM] zero set: {line}")
        elif up == "$$":
            self.settings.setdefault("$100", "88.889")

    _AXIS_RE = re.compile(r"([XYZ])(-?\d+(?:\.\d+)?)")
    _sim_absolute = True

    def _sim_move(self, up: str) -> None:
        relative = "G91" in up or (up.startswith("$J=") and "G91" in up)
        if "G90" in up:
            self._sim_absolute = True
        if up.strip() in ("G90", "G91"):
            self._sim_absolute = up.strip() == "G90"
            return
        target = list(self.wpos)
        for axis, val in self._AXIS_RE.findall(up):
            i = "XYZ".index(axis)
            v = float(val)
            target[i] = target[i] + v if (relative or not self._sim_absolute) else v
        # Pretend the move takes time so the UI shows a Run state.
        dist = max(abs(a - b) for a, b in zip(target, self.wpos)) if target else 0
        self.state = "Jog" if up.startswith("$J=") else "Run"
        steps = max(1, int(dist / 2))
        for i in range(1, steps + 1):
            if self._stop.is_set():
                return
            self.wpos = [
                s + (t - s) * i / steps for s, t in zip(self.wpos, target)
            ]
            self.mpos = [w + o for w, o in zip(self.wpos, self.wco)]
            time.sleep(0.012)
        self.wpos = target
        self.mpos = [w + o for w, o in zip(self.wpos, self.wco)]
        self.state = "Idle"

    def _handle_sim_realtime(self, byte: bytes) -> None:
        if byte == CMD_SOFT_RESET:
            self.state = "Idle"
            self._log("warn", "[SIM] soft reset")
        elif byte == CMD_FEED_HOLD:
            self.state = "Hold"
        elif byte == CMD_CYCLE_START:
            self.state = "Idle"
        elif byte == CMD_JOG_CANCEL:
            self.clear_queue()

    # ------------------------------------------------------------------
    def _log(self, level: str, msg: str) -> None:
        try:
            self._on_log(level, msg)
        except Exception:  # noqa: BLE001
            pass

    def snapshot(self) -> dict:
        return {
            "connected": self.connected,
            "port": self.port_name,
            "sim": self._sim,
            "version": self.version,
            "state": self.state,
            "alarm": self.alarm,
            "last_error": self.last_error,
            "mpos": self.mpos,
            "wpos": self.wpos,
            "wco": self.wco,
            "feed": self.feed,
            "queued": self.queue_depth(),
            "settings": dict(self.settings),
        }
