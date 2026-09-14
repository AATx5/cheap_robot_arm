"""
Configuration for the 3-DOF stepper arm.

Machine layout (Arduino UNO + CNC Shield V3 + 4x DRV8825, GRBL 1.1):

    GRBL axis   Joint        Hardware
    ---------   ----------   -------------------------------------------------
    X           J1  base     1 NEMA stepper, direct/belt drive
    Y           J2  shoulder 2 NEMA steppers on a common axle, 8:1 planetary
                             (second motor plugged into the A slot, cloned to
                             Y with the shield's jumper pins)
    Z           J3  elbow     1 NEMA stepper, 8:1 planetary
    A           ---           clone of Y, no independent motion

Unit convention: GRBL thinks in "mm". We define 1 GRBL mm == 1 DEGREE of joint
output. That means $100/$101/$102 must be set to STEPS PER DEGREE, and every
feed rate is degrees/minute. All the math in this program is done in degrees
and millimetres of Cartesian space; nothing else needs to know about steps.

Everything in this file is overridable at runtime from the Settings tab, which
writes arm_config.json next to the project. Edit whichever is more convenient.
"""

from __future__ import annotations

import copy
import json
import os
from dataclasses import dataclass, field, asdict

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "arm_config.json"
)

# --------------------------------------------------------------------------
# Defaults -- MEASURE YOUR ARM AND CHANGE THESE.
# --------------------------------------------------------------------------

DEFAULTS = {
    # ---- serial -----------------------------------------------------------
    "serial": {
        "port": "",            # e.g. "COM3" or "/dev/ttyUSB0"; "" = pick in UI
        "baud": 115200,        # GRBL 1.1 default. GRBL 0.9 used 9600.
        "auto_connect": False,
    },

    # ---- drivetrain -------------------------------------------------------
    # steps_per_deg is computed from these unless you override it directly.
    "drivetrain": {
        "motor_steps_per_rev": 200,   # 1.8 deg NEMA. Use 400 for a 0.9 deg motor.
        "microsteps": 16,             # DRV8825 M0/M1/M2 jumpers (see README)
        "gear_ratio": {               # output revs per motor rev, as a divisor
            "j1": 8.0,                # base: 1.0 if direct drive, 4.0 for a 4:1 belt
            "j2": 8.0,                # 8:1 planetary
            "j3": 8.0,                # 8:1 planetary
        },
        # Flip if a joint runs backwards. Also settable with GRBL's $3 mask.
        "invert": {"j1": False, "j2": False, "j3": False},
    },

    # ---- geometry (millimetres) ------------------------------------------
    #
    #                              (tool tip)
    #                             /
    #                     L2     /
    #              o------------o          <- J3 elbow
    #             /
    #        L1  /
    #           o   <- J2 shoulder, at height BASE_HEIGHT, offset BASE_OFFSET
    #           |      radially from the J1 rotation axis
    #           |
    #      =====+=====  <- J1 base rotation axis, origin (0,0,0)
    #
    "geometry": {
        "base_height": 120.0,   # floor -> shoulder pivot, mm
        "base_offset": 0.0,     # radial offset of shoulder from the J1 axis, mm
        "l1": 200.0,            # shoulder pivot -> elbow pivot, mm
        "l2": 180.0,            # elbow pivot -> tool tip, mm
    },

    # ---- joint limits, degrees -------------------------------------------
    # Angle conventions:
    #   J1  0 = +X axis, positive = counter-clockwise seen from above
    #   J2  0 = upper arm horizontal (pointing out), +90 = straight up
    #   J3  0 = forearm in line with the upper arm (fully extended),
    #       negative = folding the forearm up/back toward the shoulder
    "limits": {
        "j1": {"min": -170.0, "max": 170.0},
        "j2": {"min": -20.0, "max": 130.0},
        "j3": {"min": -150.0, "max": 10.0},
    },

    # ---- motion -----------------------------------------------------------
    "motion": {
        "feed_deg_per_min": 1200.0,   # default G1 feed rate
        "jog_feed_deg_per_min": 900.0,
        "max_rate_deg_per_min": {     # -> GRBL $110/$111/$112
            "j1": 1800.0, "j2": 1800.0, "j3": 1800.0,
        },
        "accel_deg_per_sec2": {       # -> GRBL $120/$121/$122
            "j1": 1800.0, "j2": 120.0, "j3": 120.0,
        },
        "linear_segment_mm": 3.0,     # Cartesian straight-line path resolution
        "elbow_up": True,             # preferred IK branch
    },

    # ---- homing -----------------------------------------------------------
    "homing": {
        "enabled": False,             # True only if you fitted limit switches
        "seek_deg_per_min": 300.0,
        "pull_off_deg": 2.0,
        # Joint angles at the moment the switches trigger, used to set the
        # work coordinate offset after $H.
        "home_angles": {"j1": 0.0, "j2": 90.0, "j3": 0.0},
        # Where a "Go Home" command parks the arm.
        "park_angles": {"j1": 0.0, "j2": 90.0, "j3": -90.0},
    },

    # ---- saved poses / sequences (managed from the UI) --------------------
    "poses": {
        "park": {"j1": 0.0, "j2": 90.0, "j3": -90.0},
        "reach out": {"j1": 0.0, "j2": 20.0, "j3": -10.0},
    },
    "sequences": {},
}


def _deep_merge(base: dict, override: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


@dataclass
class ArmConfig:
    data: dict = field(default_factory=lambda: copy.deepcopy(DEFAULTS))

    # -- persistence -------------------------------------------------------
    @classmethod
    def load(cls, path: str = CONFIG_PATH) -> "ArmConfig":
        if os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8") as fh:
                    return cls(_deep_merge(DEFAULTS, json.load(fh)))
            except (OSError, ValueError) as exc:
                print(f"[config] could not read {path}: {exc}; using defaults")
        return cls()

    def save(self, path: str = CONFIG_PATH) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(self.data, fh, indent=2)

    def update(self, patch: dict) -> None:
        self.data = _deep_merge(self.data, patch)

    # -- convenience accessors --------------------------------------------
    def __getitem__(self, key):
        return self.data[key]

    @property
    def geometry(self) -> dict:
        return self.data["geometry"]

    @property
    def limits(self) -> dict:
        return self.data["limits"]

    def steps_per_deg(self, joint: str) -> float:
        d = self.data["drivetrain"]
        motor_steps = d["motor_steps_per_rev"] * d["microsteps"]
        return motor_steps * d["gear_ratio"][joint] / 360.0

    def all_steps_per_deg(self) -> dict:
        return {j: self.steps_per_deg(j) for j in ("j1", "j2", "j3")}

    def clamp(self, joint: str, angle: float) -> float:
        lim = self.limits[joint]
        return max(lim["min"], min(lim["max"], angle))

    def in_limits(self, angles: dict) -> tuple[bool, str]:
        for j, a in angles.items():
            lim = self.limits.get(j)
            if lim and not (lim["min"] - 1e-6 <= a <= lim["max"] + 1e-6):
                return False, (
                    f"{j.upper()} = {a:.2f} deg is outside its limit "
                    f"({lim['min']} .. {lim['max']})"
                )
        return True, ""

    def to_dict(self) -> dict:
        out = copy.deepcopy(self.data)
        out["_derived"] = {
            "steps_per_deg": self.all_steps_per_deg(),
            "max_reach": self.geometry["l1"] + self.geometry["l2"],
            "min_reach": abs(self.geometry["l1"] - self.geometry["l2"]),
        }
        return out


__all__ = ["ArmConfig", "DEFAULTS", "CONFIG_PATH", "asdict"]
