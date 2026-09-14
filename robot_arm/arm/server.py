"""
Flask backend for the robotic arm control panel.

Everything the browser does goes through these endpoints; the browser never
touches the serial port directly. Start it with `python run.py`.
"""

from __future__ import annotations

import threading
import time
from collections import deque

from flask import Flask, jsonify, render_template, request

from .config import ArmConfig
from .grbl import GrblController
from . import kinematics as kin

JOINTS = ("j1", "j2", "j3")
AXES = ("X", "Y", "Z")           # GRBL axis letters for j1, j2, j3


class ArmService:
    """Glue between the config, the kinematics and the GRBL link."""

    def __init__(self):
        self.cfg = ArmConfig.load()
        self.log = deque(maxlen=400)
        self._log_seq = 0
        self.grbl = GrblController(on_log=self._append_log)
        self.sequence_thread: threading.Thread | None = None
        self.sequence_stop = threading.Event()
        self.sequence_status = {"running": False, "step": 0, "total": 0, "name": ""}

    # -- logging --------------------------------------------------------
    def _append_log(self, level: str, msg: str) -> None:
        self._log_seq += 1
        self.log.append(
            {"id": self._log_seq, "t": time.strftime("%H:%M:%S"),
             "level": level, "msg": msg}
        )

    def logs_since(self, since: int) -> list:
        return [e for e in self.log if e["id"] > since]

    # -- angle helpers --------------------------------------------------
    def current_angles(self) -> dict:
        w = self.grbl.wpos
        return {"j1": w[0], "j2": w[1], "j3": w[2]}

    def current_xyz(self) -> dict:
        return kin.forward(**self.current_angles(), geometry=self.cfg.geometry)

    def check_limits(self, angles: dict) -> tuple[bool, str]:
        return self.cfg.in_limits(angles)

    # -- motion ---------------------------------------------------------
    def goto_angles(self, angles: dict, feed: float | None = None,
                    rapid: bool = False) -> tuple[bool, str]:
        ok, why = self.check_limits(angles)
        if not ok:
            return False, why
        f = feed or self.cfg["motion"]["feed_deg_per_min"]
        words = " ".join(
            f"{ax}{angles[j]:.3f}" for ax, j in zip(AXES, JOINTS) if j in angles
        )
        cmd = f"G90 G0 {words}" if rapid else f"G90 G1 {words} F{f:.1f}"
        self.grbl.send(cmd)
        return True, cmd

    def jog(self, joint: str, delta: float, feed: float | None = None) -> tuple[bool, str]:
        target = dict(self.current_angles())
        target[joint] = target[joint] + delta
        ok, why = self.check_limits(target)
        if not ok:
            return False, why
        ax = AXES[JOINTS.index(joint)]
        f = feed or self.cfg["motion"]["jog_feed_deg_per_min"]
        cmd = f"$J=G91{ax}{delta:.3f}F{f:.1f}"
        self.grbl.send(cmd)
        return True, cmd

    def move_cartesian(self, x: float, y: float, z: float, linear: bool,
                       feed: float | None = None) -> tuple[bool, str]:
        elbow_up = bool(self.cfg["motion"]["elbow_up"])
        f = feed or self.cfg["motion"]["feed_deg_per_min"]
        try:
            if linear:
                start = self.current_xyz()
                pts = kin.line_waypoints(
                    (start["x"], start["y"], start["z"]),
                    (x, y, z),
                    self.cfg.geometry,
                    self.cfg.limits,
                    elbow_up=elbow_up,
                    segment_mm=self.cfg["motion"]["linear_segment_mm"],
                )
                lines = ["G90"] + [
                    "G1 " + " ".join(f"{ax}{p[j]:.3f}" for ax, j in zip(AXES, JOINTS))
                    + f" F{f:.1f}"
                    for p in pts
                ]
                self.grbl.send_many(lines)
                return True, f"streaming {len(pts)} segments"
            sol = kin.inverse(x, y, z, self.cfg.geometry,
                              elbow_up=elbow_up, limits=self.cfg.limits)
            return self.goto_angles(
                {j: sol[j] for j in JOINTS}, feed=f
            )
        except kin.UnreachableError as exc:
            return False, str(exc)

    # -- sequences ------------------------------------------------------
    def run_sequence(self, name: str, steps: list, loop: bool = False) -> None:
        self.stop_sequence()
        self.sequence_stop.clear()

        def worker():
            self.sequence_status.update(
                {"running": True, "total": len(steps), "step": 0, "name": name}
            )
            self._append_log("info", f"Sequence '{name}' started ({len(steps)} steps)")
            try:
                while not self.sequence_stop.is_set():
                    for i, step in enumerate(steps):
                        if self.sequence_stop.is_set():
                            break
                        self.sequence_status["step"] = i + 1
                        angles = {j: float(step[j]) for j in JOINTS}
                        ok, msg = self.goto_angles(
                            angles, feed=step.get("feed")
                        )
                        if not ok:
                            self._append_log("error", f"Sequence stopped: {msg}")
                            return
                        self._wait_for_idle()
                        dwell = float(step.get("dwell", 0) or 0)
                        end = time.time() + dwell
                        while time.time() < end and not self.sequence_stop.is_set():
                            time.sleep(0.05)
                    if not loop:
                        break
            finally:
                self.sequence_status.update({"running": False, "step": 0})
                self._append_log("info", f"Sequence '{name}' finished")

        self.sequence_thread = threading.Thread(target=worker, daemon=True)
        self.sequence_thread.start()

    def _wait_for_idle(self, timeout: float = 120.0) -> None:
        time.sleep(0.35)   # let GRBL leave Idle before we start watching
        deadline = time.time() + timeout
        while time.time() < deadline and not self.sequence_stop.is_set():
            if self.grbl.queue_depth() == 0 and self.grbl.state in ("Idle", "Check"):
                return
            if self.grbl.state == "Alarm":
                return
            time.sleep(0.05)

    def stop_sequence(self) -> None:
        self.sequence_stop.set()
        if self.sequence_thread and self.sequence_thread.is_alive():
            self.sequence_thread.join(timeout=2.0)
        self.sequence_status.update({"running": False, "step": 0})

    # -- GRBL settings push ---------------------------------------------
    def grbl_setting_lines(self) -> list:
        c = self.cfg
        d, m, h = c["drivetrain"], c["motion"], c["homing"]
        lim = c.limits
        spd = c.all_steps_per_deg()

        mask = 0
        for i, j in enumerate(JOINTS):
            if d["invert"][j]:
                mask |= 1 << i

        lines = [
            f"$100={spd['j1']:.4f}",   # steps per degree, J1 (X)
            f"$101={spd['j2']:.4f}",   # J2 (Y) -- A axis is cloned to Y
            f"$102={spd['j3']:.4f}",   # J3 (Z)
            f"$110={m['max_rate_deg_per_min']['j1']:.1f}",
            f"$111={m['max_rate_deg_per_min']['j2']:.1f}",
            f"$112={m['max_rate_deg_per_min']['j3']:.1f}",
            f"$120={m['accel_deg_per_sec2']['j1']:.1f}",
            f"$121={m['accel_deg_per_sec2']['j2']:.1f}",
            f"$122={m['accel_deg_per_sec2']['j3']:.1f}",
            f"$130={lim['j1']['max'] - lim['j1']['min']:.1f}",
            f"$131={lim['j2']['max'] - lim['j2']['min']:.1f}",
            f"$132={lim['j3']['max'] - lim['j3']['min']:.1f}",
            f"$3={mask}",              # direction invert mask
            "$4=0",                    # DRV8825 STEP/DIR are not inverted
            "$1=255",                  # keep steppers energised (holding torque)
            f"$22={1 if h['enabled'] else 0}",
            "$20=0",                   # soft limits off (we limit in software)
            f"$21={1 if h['enabled'] else 0}",
        ]
        if h["enabled"]:
            lines += [
                f"$24={h['seek_deg_per_min'] / 4:.1f}",
                f"$25={h['seek_deg_per_min']:.1f}",
                f"$27={h['pull_off_deg']:.2f}",
            ]
        return lines


# --------------------------------------------------------------------------
# Flask app
# --------------------------------------------------------------------------

def create_app() -> Flask:
    app = Flask(__name__)
    svc = ArmService()
    app.config["SVC"] = svc

    def body() -> dict:
        return request.get_json(silent=True) or {}

    # ---- page ---------------------------------------------------------
    @app.get("/")
    def index():
        return render_template("index.html")

    # ---- connection ---------------------------------------------------
    @app.get("/api/ports")
    def api_ports():
        return jsonify(ports=GrblController.list_ports())

    @app.post("/api/connect")
    def api_connect():
        d = body()
        port = d.get("port") or svc.cfg["serial"]["port"]
        baud = int(d.get("baud") or svc.cfg["serial"]["baud"])
        if not port:
            return jsonify(ok=False, message="No port selected"), 400
        ok, msg = svc.grbl.connect(port, baud)
        if ok:
            svc.cfg.update({"serial": {"port": port, "baud": baud}})
            svc.cfg.save()
        return jsonify(ok=ok, message=msg)

    @app.post("/api/disconnect")
    def api_disconnect():
        svc.stop_sequence()
        svc.grbl.disconnect()
        return jsonify(ok=True)

    # ---- status -------------------------------------------------------
    @app.get("/api/status")
    def api_status():
        angles = svc.current_angles()
        try:
            fk = kin.forward(**angles, geometry=svc.cfg.geometry)
        except Exception:  # noqa: BLE001
            fk = None
        since = int(request.args.get("since", 0))
        limits_ok, limits_msg = svc.check_limits(angles)
        return jsonify(
            grbl=svc.grbl.snapshot(),
            angles=angles,
            fk=fk,
            limits_ok=limits_ok,
            limits_msg=limits_msg,
            sequence=svc.sequence_status,
            logs=svc.logs_since(since),
        )

    # ---- motion -------------------------------------------------------
    @app.post("/api/jog")
    def api_jog():
        d = body()
        ok, msg = svc.jog(d["joint"], float(d["delta"]), d.get("feed"))
        return jsonify(ok=ok, message=msg), (200 if ok else 400)

    @app.post("/api/jog_cancel")
    def api_jog_cancel():
        svc.grbl.clear_queue()
        svc.grbl.jog_cancel()
        return jsonify(ok=True)

    @app.post("/api/goto_joints")
    def api_goto_joints():
        d = body()
        angles = {j: float(d[j]) for j in JOINTS if j in d}
        full = dict(svc.current_angles())
        full.update(angles)
        ok, msg = svc.goto_angles(full, d.get("feed"), rapid=bool(d.get("rapid")))
        return jsonify(ok=ok, message=msg), (200 if ok else 400)

    @app.post("/api/goto_xyz")
    def api_goto_xyz():
        d = body()
        ok, msg = svc.move_cartesian(
            float(d["x"]), float(d["y"]), float(d["z"]),
            linear=bool(d.get("linear")), feed=d.get("feed"),
        )
        return jsonify(ok=ok, message=msg), (200 if ok else 400)

    @app.post("/api/solve")
    def api_solve():
        """IK preview without moving anything."""
        d = body()
        try:
            sol = kin.inverse(
                float(d["x"]), float(d["y"]), float(d["z"]),
                svc.cfg.geometry,
                elbow_up=bool(d.get("elbow_up", svc.cfg["motion"]["elbow_up"])),
                limits=svc.cfg.limits,
            )
            return jsonify(ok=True, solution=sol)
        except kin.UnreachableError as exc:
            return jsonify(ok=False, message=str(exc))

    @app.post("/api/fk")
    def api_fk():
        d = body()
        return jsonify(
            ok=True,
            fk=kin.forward(float(d["j1"]), float(d["j2"]), float(d["j3"]),
                           svc.cfg.geometry),
        )

    @app.get("/api/workspace")
    def api_workspace():
        return jsonify(points=kin.workspace_profile(svc.cfg.geometry, svc.cfg.limits))

    # ---- machine control ----------------------------------------------
    @app.post("/api/estop")
    def api_estop():
        svc.stop_sequence()
        svc.grbl.soft_reset()
        return jsonify(ok=True)

    @app.post("/api/hold")
    def api_hold():
        svc.grbl.feed_hold()
        return jsonify(ok=True)

    @app.post("/api/resume")
    def api_resume():
        svc.grbl.resume()
        return jsonify(ok=True)

    @app.post("/api/unlock")
    def api_unlock():
        svc.grbl.unlock()
        return jsonify(ok=True)

    @app.post("/api/home")
    def api_home():
        h = svc.cfg["homing"]
        if h["enabled"]:
            svc.grbl.home()
            a = h["home_angles"]
            svc.grbl.send(f"G10 L20 P1 X{a['j1']:.3f} Y{a['j2']:.3f} Z{a['j3']:.3f}")
        else:
            return jsonify(
                ok=False,
                message="Homing is disabled. Fit limit switches and turn it on "
                        "in Settings, or use 'Set current as home'.",
            ), 400
        return jsonify(ok=True)

    @app.post("/api/set_home")
    def api_set_home():
        """Declare the arm's present physical pose to be the given angles."""
        d = body()
        a = d or svc.cfg["homing"]["home_angles"]
        svc.grbl.send(
            f"G10 L20 P1 X{float(a['j1']):.3f} Y{float(a['j2']):.3f} "
            f"Z{float(a['j3']):.3f}"
        )
        return jsonify(ok=True, message="Work zero set")

    @app.post("/api/park")
    def api_park():
        ok, msg = svc.goto_angles(svc.cfg["homing"]["park_angles"])
        return jsonify(ok=ok, message=msg), (200 if ok else 400)

    @app.post("/api/mdi")
    def api_mdi():
        line = (body().get("line") or "").strip()
        if not line:
            return jsonify(ok=False, message="empty"), 400
        svc.grbl.send(line)
        return jsonify(ok=True)

    @app.post("/api/push_settings")
    def api_push_settings():
        lines = svc.grbl_setting_lines()
        svc.grbl.send_many(lines)
        return jsonify(ok=True, lines=lines)

    @app.get("/api/settings_preview")
    def api_settings_preview():
        return jsonify(lines=svc.grbl_setting_lines())

    # ---- config -------------------------------------------------------
    @app.get("/api/config")
    def api_config():
        return jsonify(config=svc.cfg.to_dict())

    @app.post("/api/config")
    def api_config_post():
        svc.cfg.update(body())
        svc.cfg.save()
        return jsonify(ok=True, config=svc.cfg.to_dict())

    # ---- poses --------------------------------------------------------
    @app.post("/api/pose/save")
    def api_pose_save():
        d = body()
        name = (d.get("name") or "").strip()
        if not name:
            return jsonify(ok=False, message="Name required"), 400
        angles = d.get("angles") or svc.current_angles()
        svc.cfg["poses"][name] = {j: round(float(angles[j]), 3) for j in JOINTS}
        svc.cfg.save()
        return jsonify(ok=True, poses=svc.cfg["poses"])

    @app.post("/api/pose/delete")
    def api_pose_delete():
        svc.cfg["poses"].pop(body().get("name", ""), None)
        svc.cfg.save()
        return jsonify(ok=True, poses=svc.cfg["poses"])

    @app.post("/api/pose/goto")
    def api_pose_goto():
        d = body()
        pose = svc.cfg["poses"].get(d.get("name", ""))
        if not pose:
            return jsonify(ok=False, message="No such pose"), 404
        ok, msg = svc.goto_angles(pose, d.get("feed"))
        return jsonify(ok=ok, message=msg), (200 if ok else 400)

    # ---- sequences ----------------------------------------------------
    @app.post("/api/sequence/save")
    def api_seq_save():
        d = body()
        name = (d.get("name") or "").strip()
        if not name:
            return jsonify(ok=False, message="Name required"), 400
        svc.cfg["sequences"][name] = d.get("steps", [])
        svc.cfg.save()
        return jsonify(ok=True, sequences=svc.cfg["sequences"])

    @app.post("/api/sequence/delete")
    def api_seq_delete():
        svc.cfg["sequences"].pop(body().get("name", ""), None)
        svc.cfg.save()
        return jsonify(ok=True, sequences=svc.cfg["sequences"])

    @app.post("/api/sequence/run")
    def api_seq_run():
        d = body()
        name = d.get("name", "untitled")
        steps = d.get("steps") or svc.cfg["sequences"].get(name, [])
        if not steps:
            return jsonify(ok=False, message="Sequence is empty"), 400
        if not svc.grbl.connected:
            return jsonify(ok=False, message="Not connected"), 400
        svc.run_sequence(name, steps, loop=bool(d.get("loop")))
        return jsonify(ok=True)

    @app.post("/api/sequence/stop")
    def api_seq_stop():
        svc.stop_sequence()
        svc.grbl.clear_queue()
        return jsonify(ok=True)

    return app
