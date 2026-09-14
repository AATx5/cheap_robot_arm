#!/usr/bin/env python3
"""
Self-test: kinematics round-trips, limit handling, and a full API smoke test
against the built-in simulator. Run with `python test_arm.py` -- no hardware
and no pytest required.
"""

import json
import math
import random
import sys
import time

from arm.config import ArmConfig
from arm import kinematics as kin
from arm.server import create_app

PASS, FAIL = 0, 0


def check(name, cond, extra=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ok   {name}")
    else:
        FAIL += 1
        print(f"  FAIL {name} {extra}")


def near(a, b, tol=1e-6):
    return abs(a - b) < tol


# --------------------------------------------------------------------------
print("\n[1] Forward kinematics sanity")
cfg = ArmConfig()
G = cfg.geometry           # bh=120, bo=0, l1=200, l2=180

fk = kin.forward(0, 0, 0, G)
check("straight out along +X", near(fk["x"], 380) and near(fk["y"], 0) and near(fk["z"], 120),
      f"got {fk['x']:.3f},{fk['y']:.3f},{fk['z']:.3f}")

fk = kin.forward(90, 0, 0, G)
check("J1=90 puts the tip on +Y", near(fk["x"], 0, 1e-9) or abs(fk["x"]) < 1e-9,
      f"x={fk['x']:.6f}")
check("J1=90 y = full reach", near(fk["y"], 380, 1e-6), f"y={fk['y']:.4f}")

fk = kin.forward(0, 90, 0, G)
check("J2=90 straight up", near(fk["x"], 0, 1e-9) and near(fk["z"], 120 + 380, 1e-6),
      f"z={fk['z']:.3f}")

fk = kin.forward(0, 90, -90, G)
check("J2=90 J3=-90 folds forward", near(fk["x"], 180, 1e-6) and near(fk["z"], 320, 1e-6),
      f"x={fk['x']:.3f} z={fk['z']:.3f}")

check("elbow position is L1 from the shoulder",
      near(math.dist((fk["elbow"]["x"], fk["elbow"]["y"], fk["elbow"]["z"]),
                     (0, 0, 120)), 200, 1e-6))

# --------------------------------------------------------------------------
print("\n[2] IK -> FK round trip over the workspace")
random.seed(7)
tested = bad = 0
worst = 0.0
for _ in range(4000):
    j1 = random.uniform(-170, 170)
    j2 = random.uniform(-20, 130)
    j3 = random.uniform(-150, 10)
    p = kin.forward(j1, j2, j3, G)
    try:
        sol = kin.inverse(p["x"], p["y"], p["z"], G,
                          elbow_up=(j3 < 0), limits=cfg.limits)
    except kin.UnreachableError:
        continue
    back = kin.forward(sol["j1"], sol["j2"], sol["j3"], G)
    err = math.dist((p["x"], p["y"], p["z"]), (back["x"], back["y"], back["z"]))
    worst = max(worst, err)
    tested += 1
    if err > 1e-3:
        bad += 1

check(f"{tested} random poses round-trip to the same point", bad == 0,
      f"{bad} bad, worst error {worst:.6f} mm")
check("worst positional error under 1 micron", worst < 1e-3, f"{worst:.3e} mm")

# --------------------------------------------------------------------------
print("\n[3] Reach limits")
try:
    kin.inverse(1000, 0, 120, G)
    check("too-far target rejected", False)
except kin.UnreachableError as e:
    check("too-far target rejected", "Too far" in str(e))

try:
    kin.inverse(0, 0, 400, G)
    check("target on the J1 axis rejected", False)
except kin.UnreachableError as e:
    check("target on the J1 axis rejected", "rotation axis" in str(e))

try:
    kin.inverse(5, 0, 120, G)
    check("too-close target rejected", False)
except kin.UnreachableError as e:
    check("too-close target rejected", "Too close" in str(e))

sol_up = kin.inverse(300, 0, 200, G, elbow_up=True)
sol_dn = kin.inverse(300, 0, 200, G, elbow_up=False)
check("elbow up gives negative J3", sol_up["j3"] < 0, f"{sol_up['j3']:.2f}")
check("elbow down gives positive J3", sol_dn["j3"] > 0, f"{sol_dn['j3']:.2f}")
up_p = kin.forward(**{k: sol_up[k] for k in ('j1', 'j2', 'j3')}, geometry=G)
dn_p = kin.forward(**{k: sol_dn[k] for k in ('j1', 'j2', 'j3')}, geometry=G)
check("both branches reach the same point",
      math.dist((up_p["x"], up_p["y"], up_p["z"]), (dn_p["x"], dn_p["y"], dn_p["z"])) < 1e-3,
      f"{up_p['x']:.4f},{up_p['z']:.4f} vs {dn_p['x']:.4f},{dn_p['z']:.4f}")
check("both branches land on the requested point",
      near(up_p["x"], 300, 1e-3) and near(up_p["z"], 200, 1e-3),
      f"{up_p['x']:.4f},{up_p['z']:.4f}")

# limits force the other branch
tight = {"j1": {"min": -180, "max": 180},
         "j2": {"min": -90, "max": 20},      # forces elbow-down
         "j3": {"min": -180, "max": 180}}
s = kin.inverse(300, 0, 200, G, elbow_up=True, limits=tight)
check("falls back to the other elbow branch when limits block one", s["j3"] > 0,
      f"j3={s['j3']:.2f}")

# --------------------------------------------------------------------------
print("\n[4] Straight-line path")
pts = kin.line_waypoints((250, 0, 200), (250, 0, 100), G, cfg.limits,
                         elbow_up=True, segment_mm=5)
check("100 mm line at 5 mm/seg gives 20 waypoints", len(pts) == 20, f"{len(pts)}")
mid = kin.forward(pts[9]["j1"], pts[9]["j2"], pts[9]["j3"], G)
check("midpoint stays on the line", near(mid["x"], 250, 1e-3) and near(mid["z"], 150, 1e-3),
      f"x={mid['x']:.3f} z={mid['z']:.3f}")

pts = kin.line_waypoints((250, 60, 200), (250, -60, 200), G, cfg.limits, segment_mm=4)
jumps = [abs(pts[i]["j1"] - pts[i - 1]["j1"]) for i in range(1, len(pts))]
check("J1 never jumps more than 180 deg between segments", max(jumps) < 180,
      f"max {max(jumps):.1f}")

# --------------------------------------------------------------------------
print("\n[5] Config / steps-per-degree")
check("J1 direct drive = 8.889 steps/deg", near(cfg.steps_per_deg("j1"), 200 * 16 / 360, 1e-9),
      f"{cfg.steps_per_deg('j1'):.4f}")
check("J2 through 8:1 = 71.111 steps/deg", near(cfg.steps_per_deg("j2"), 200 * 16 * 8 / 360, 1e-9),
      f"{cfg.steps_per_deg('j2'):.4f}")
ok, why = cfg.in_limits({"j1": 0, "j2": 200, "j3": 0})
check("out-of-limit angle is caught", not ok and "J2" in why, why)
check("in-limit angle passes", cfg.in_limits({"j1": 0, "j2": 90, "j3": -45})[0])
check("clamp works", near(cfg.clamp("j2", 999), 130.0))

# --------------------------------------------------------------------------
print("\n[6] API smoke test against the simulator")
app = create_app()
c = app.test_client()

check("index page renders", c.get("/").status_code == 200)
check("config endpoint", c.get("/api/config").get_json()["config"]["geometry"]["l1"] == 200)
check("SIM appears in the port list",
      any(p["device"] == "SIM" for p in c.get("/api/ports").get_json()["ports"]))
check("workspace outline generated", len(c.get("/api/workspace").get_json()["points"]) > 100)

r = c.post("/api/connect", json={"port": "SIM"}).get_json()
check("connect to simulator", r["ok"], r.get("message", ""))
time.sleep(0.4)

st = c.get("/api/status").get_json()
check("status reports connected", st["grbl"]["connected"])
check("status includes FK", st["fk"] is not None)

r = c.post("/api/goto_joints", json={"j1": 30, "j2": 60, "j3": -40}).get_json()
check("joint move accepted", r["ok"], r.get("message", ""))
check("emits an absolute G1", "G90 G1" in r["message"] and "X30.000" in r["message"],
      r["message"])

r = c.post("/api/goto_joints", json={"j2": 500})
check("out-of-limit joint move refused", r.status_code == 400)

r = c.post("/api/jog", json={"joint": "j1", "delta": 5}).get_json()
check("jog emits $J=G91", r["ok"] and r["message"].startswith("$J=G91"), r.get("message"))

r = c.post("/api/solve", json={"x": 300, "y": 0, "z": 200}).get_json()
check("IK preview endpoint", r["ok"] and "j2" in r["solution"])

r = c.post("/api/solve", json={"x": 2000, "y": 0, "z": 200}).get_json()
check("IK preview reports unreachable", r["ok"] is False and "Too far" in r["message"])

time.sleep(2.0)     # let the simulated move finish
r = c.post("/api/goto_xyz", json={"x": 300, "y": 0, "z": 200, "linear": False}).get_json()
check("cartesian joint move", r["ok"], r.get("message", ""))
time.sleep(2.0)
r = c.post("/api/goto_xyz", json={"x": 280, "y": 0, "z": 220, "linear": True}).get_json()
check("cartesian linear move streams segments", r["ok"] and "segments" in r["message"],
      r.get("message", ""))

lines = c.get("/api/settings_preview").get_json()["lines"]
check("$100 = 200*16/360 steps per degree (J1 direct)",
      any(l.startswith("$100=8.8889") for l in lines),
      [l for l in lines if l.startswith("$100")])
check("$101 accounts for the 8:1 gearbox (8x $100)",
      any(l.startswith("$101=71.1111") for l in lines),
      [l for l in lines if l.startswith("$101")])
check("$101 == $102 (both geared 8:1)",
      [l for l in lines if l.startswith("$101")][0][5:]
      == [l for l in lines if l.startswith("$102")][0][5:])
check("steppers stay energised ($1=255)", "$1=255" in lines)

r = c.post("/api/pose/save", json={"name": "_test"}).get_json()
check("pose save", "_test" in r["poses"])
check("pose goto", c.post("/api/pose/goto", json={"name": "_test"}).get_json()["ok"])
r = c.post("/api/pose/delete", json={"name": "_test"}).get_json()
check("pose delete", "_test" not in r["poses"])

steps = [{"j1": 0, "j2": 80, "j3": -20, "dwell": 0.1},
         {"j1": 10, "j2": 70, "j3": -30, "dwell": 0.1}]
check("sequence run starts",
      c.post("/api/sequence/run", json={"name": "t", "steps": steps}).get_json()["ok"])
time.sleep(0.5)
check("sequence reports running", c.get("/api/status").get_json()["sequence"]["running"])
c.post("/api/sequence/stop", json={})
time.sleep(0.3)
check("sequence stops", not c.get("/api/status").get_json()["sequence"]["running"])

check("estop", c.post("/api/estop", json={}).get_json()["ok"])
check("hold/resume", c.post("/api/hold", json={}).get_json()["ok"]
      and c.post("/api/resume", json={}).get_json()["ok"])
check("homing refused when disabled", c.post("/api/home", json={}).status_code == 400)
check("mdi", c.post("/api/mdi", json={"line": "$$"}).get_json()["ok"])

logs = c.get("/api/status?since=0").get_json()["logs"]
check("console log is populated", len(logs) > 3, f"{len(logs)} entries")

c.post("/api/disconnect", json={})
check("disconnect", not c.get("/api/status").get_json()["grbl"]["connected"])

# --------------------------------------------------------------------------
print(f"\n{'=' * 50}\n  {PASS} passed, {FAIL} failed\n{'=' * 50}")
sys.exit(1 if FAIL else 0)
