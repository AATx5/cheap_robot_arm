"""
Forward and inverse kinematics for the 3-DOF arm.

Frames and sign conventions
---------------------------
World origin sits on the floor at the J1 rotation axis.
  +Z is up, +X is "forward" when J1 = 0, +Y is to the left.

  J1  base yaw.   0 deg = tool pointing along +X.  Positive = CCW from above.
  J2  shoulder.   0 deg = upper arm horizontal, pointing radially out.
                  +90 deg = upper arm straight up.
  J3  elbow.      0 deg = forearm collinear with the upper arm (arm straight).
                  Negative = forearm folds back toward the shoulder.
                  This is a RELATIVE angle, which is what the elbow motor
                  actually measures since it is mounted on the upper arm.

The wrist/tool is a single point at the end of L2; there is no wrist pitch
joint on this arm, so the tool orientation is whatever J2 + J3 works out to.
"""

from __future__ import annotations

import math
from typing import Optional

EPS = 1e-9


class UnreachableError(ValueError):
    """Target is outside the arm's workspace."""


# --------------------------------------------------------------------------
# Forward kinematics
# --------------------------------------------------------------------------

def forward(j1: float, j2: float, j3: float, geometry: dict) -> dict:
    """Joint angles (degrees) -> tool tip position and elbow position."""
    l1 = geometry["l1"]
    l2 = geometry["l2"]
    bh = geometry["base_height"]
    bo = geometry["base_offset"]

    a1 = math.radians(j1)
    a2 = math.radians(j2)
    a23 = math.radians(j2 + j3)          # forearm angle from horizontal

    # Work in the vertical plane containing the arm, then rotate about Z.
    r_elbow = bo + l1 * math.cos(a2)
    z_elbow = bh + l1 * math.sin(a2)

    r_tip = r_elbow + l2 * math.cos(a23)
    z_tip = z_elbow + l2 * math.sin(a23)

    return {
        "x": r_tip * math.cos(a1),
        "y": r_tip * math.sin(a1),
        "z": z_tip,
        "r": r_tip,
        "elbow": {
            "x": r_elbow * math.cos(a1),
            "y": r_elbow * math.sin(a1),
            "z": z_elbow,
            "r": r_elbow,
        },
        "shoulder": {
            "x": bo * math.cos(a1),
            "y": bo * math.sin(a1),
            "z": bh,
            "r": bo,
        },
        # Tool pitch relative to horizontal, handy for readouts.
        "pitch": j2 + j3,
    }


# --------------------------------------------------------------------------
# Inverse kinematics
# --------------------------------------------------------------------------

def inverse(
    x: float,
    y: float,
    z: float,
    geometry: dict,
    elbow_up: bool = True,
    limits: Optional[dict] = None,
) -> dict:
    """Cartesian target (mm) -> joint angles (degrees).

    Raises UnreachableError if no solution exists. If `limits` is supplied and
    the preferred elbow branch violates them, the other branch is tried before
    giving up.
    """
    l1 = float(geometry["l1"])
    l2 = float(geometry["l2"])
    bh = float(geometry["base_height"])
    bo = float(geometry["base_offset"])

    # --- J1: point the arm at the target -------------------------------
    if abs(x) < EPS and abs(y) < EPS:
        raise UnreachableError(
            "Target is on the base rotation axis; J1 is undefined there."
        )
    j1 = math.degrees(math.atan2(y, x))

    # --- reduce to the 2D problem in the arm plane ----------------------
    r = math.hypot(x, y) - bo          # radial distance from shoulder pivot
    dz = z - bh                        # height above shoulder pivot
    d = math.hypot(r, dz)              # shoulder -> tip distance

    if d > l1 + l2 - EPS:
        raise UnreachableError(
            f"Too far: needs {d:.1f} mm of reach, arm has {l1 + l2:.1f} mm."
        )
    if d < abs(l1 - l2) + EPS:
        raise UnreachableError(
            f"Too close: {d:.1f} mm from the shoulder, minimum is "
            f"{abs(l1 - l2):.1f} mm."
        )

    # --- elbow interior angle via law of cosines ------------------------
    cos_elbow = (l1 * l1 + l2 * l2 - d * d) / (2.0 * l1 * l2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    interior = math.acos(cos_elbow)                 # 0 = folded, pi = straight

    # J3 measured from "straight" (0) folding negative.
    j3_mag = math.degrees(math.pi - interior)

    # --- shoulder angle -------------------------------------------------
    cos_sh = (d * d + l1 * l1 - l2 * l2) / (2.0 * l1 * d)
    cos_sh = max(-1.0, min(1.0, cos_sh))
    offset = math.degrees(math.acos(cos_sh))
    base_ang = math.degrees(math.atan2(dz, r))

    solutions = []
    # Elbow-up: shoulder lifted above the chord, forearm folds down (J3 < 0).
    solutions.append({"j1": j1, "j2": base_ang + offset, "j3": -j3_mag})
    # Elbow-down: mirror of the above.
    solutions.append({"j1": j1, "j2": base_ang - offset, "j3": +j3_mag})

    if not elbow_up:
        solutions.reverse()

    if limits is None:
        chosen = solutions[0]
    else:
        chosen = None
        for sol in solutions:
            if _within(sol, limits):
                chosen = sol
                break
        if chosen is None:
            bad = _first_violation(solutions[0], limits)
            raise UnreachableError(
                f"Reachable geometrically, but no elbow branch fits the joint "
                f"limits ({bad})."
            )

    chosen["elbow_up"] = chosen["j3"] < 0
    return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in chosen.items()}


def _within(sol: dict, limits: dict) -> bool:
    for j in ("j1", "j2", "j3"):
        lim = limits.get(j)
        if lim and not (lim["min"] - 1e-6 <= sol[j] <= lim["max"] + 1e-6):
            return False
    return True


def _first_violation(sol: dict, limits: dict) -> str:
    for j in ("j1", "j2", "j3"):
        lim = limits.get(j)
        if lim and not (lim["min"] - 1e-6 <= sol[j] <= lim["max"] + 1e-6):
            return f"{j.upper()} would need {sol[j]:.1f} deg"
    return "unknown"


# --------------------------------------------------------------------------
# Straight-line Cartesian paths
# --------------------------------------------------------------------------

def line_waypoints(
    start_xyz: tuple,
    end_xyz: tuple,
    geometry: dict,
    limits: dict,
    elbow_up: bool = True,
    segment_mm: float = 3.0,
) -> list:
    """Break a straight Cartesian move into joint-space waypoints.

    A plain G1 through GRBL interpolates the JOINTS linearly, which makes the
    tool tip swing along an arc. Chopping the line into short segments and
    solving IK at each one keeps the tip close to an actual straight line.
    """
    x0, y0, z0 = start_xyz
    x1, y1, z1 = end_xyz
    dist = math.dist((x0, y0, z0), (x1, y1, z1))
    n = max(1, int(math.ceil(dist / max(0.2, segment_mm))))

    points = []
    for i in range(1, n + 1):
        t = i / n
        p = inverse(
            x0 + (x1 - x0) * t,
            y0 + (y1 - y0) * t,
            z0 + (z1 - z0) * t,
            geometry,
            elbow_up=elbow_up,
            limits=limits,
        )
        points.append(p)

    # Guard against the base flipping 180 deg mid-path (happens when the line
    # passes near the J1 axis) -- unwrap so J1 moves continuously.
    for i in range(1, len(points)):
        while points[i]["j1"] - points[i - 1]["j1"] > 180.0:
            points[i]["j1"] -= 360.0
        while points[i]["j1"] - points[i - 1]["j1"] < -180.0:
            points[i]["j1"] += 360.0

    return points


def workspace_profile(geometry: dict, limits: dict, samples: int = 90) -> list:
    """Outline of the reachable area in the vertical (r, z) plane.

    Used by the UI to draw the reach envelope behind the arm graphic.
    """
    l1, l2 = geometry["l1"], geometry["l2"]
    bh, bo = geometry["base_height"], geometry["base_offset"]
    j2lim, j3lim = limits["j2"], limits["j3"]

    pts = []
    for i in range(samples + 1):
        j2 = j2lim["min"] + (j2lim["max"] - j2lim["min"]) * i / samples
        for j3 in (j3lim["min"], j3lim["max"]):
            a2, a23 = math.radians(j2), math.radians(j2 + j3)
            pts.append(
                [
                    bo + l1 * math.cos(a2) + l2 * math.cos(a23),
                    bh + l1 * math.sin(a2) + l2 * math.sin(a23),
                ]
            )
    return pts
