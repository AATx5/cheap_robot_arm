# Robotic Arm Control Panel

Browser-based control for a 3-DOF stepper arm running on an Arduino UNO +
DAOKI CNC Shield V3 + DRV8825 drivers, with GRBL 1.1 as the motion firmware.

- **J1 base** — 1 NEMA stepper, direct or belt drive → **X axis**
- **J2 shoulder** — 2 NEMA steppers on a shared axle through 8:1 planetaries → **Y axis + cloned A axis**
- **J3 elbow** — 1 NEMA stepper through an 8:1 planetary → **Z axis**

Jog panel, inverse kinematics, live 2D arm view, pose library, sequence
recorder/player, G-code console, and a settings page that computes and pushes
your GRBL `$` values. Includes a hardware-free simulator so you can try the
whole UI before you wire anything up.

---

## 1. Quick start

```bash
pip install -r requirements.txt
python run.py
```

A browser opens at `http://127.0.0.1:5000`. Pick **SIM** in the port dropdown
and hit Connect to drive the simulated arm — no Arduino needed.

To control it from your phone or a laptop on the same network:

```bash
python run.py --lan
```

Run the self-test any time (55 checks, no hardware required):

```bash
python test_arm.py
```

---

## 2. Firmware: flash GRBL

1. Download GRBL 1.1: https://github.com/gnea/grbl
2. In the Arduino IDE: **Sketch → Include Library → Add .ZIP Library**, pick
   the `grbl` folder inside the download.
3. **File → Examples → grbl → grblUpload**, select *Arduino Uno*, Upload.
4. Unplug the shield while flashing. Always power the shield's 12–24 V input
   **before** you rely on the motors holding position.

GRBL 1.1 talks at **115200 baud**. (0.9 used 9600 — if the console shows
garbage, that's your problem.)

---

## 3. Wiring the shield

### Driver sockets

| Socket | Joint | Notes |
|---|---|---|
| X | J1 base | one motor |
| Y | J2 shoulder | one of the two shoulder motors |
| Z | J3 elbow | one motor |
| A | J2 shoulder | second shoulder motor, **cloned to Y** |

### Cloning the A axis to Y — this is the important bit

Your two shoulder motors must receive identical step/dir signals. The CNC
Shield V3 has a small 3×2 jumper block next to the A driver socket. Bridge the
pair labelled **A-Y** (sometimes silkscreened as the middle pair of the
`X Y Z` clone header). Both motors then move as one axis and GRBL never knows
there are two.

If your shoulder motors are mounted **facing each other** on the axle (mirror
image), one of them will try to run backwards. Fix it in hardware — swap one
coil pair on that motor's connector (e.g. swap the two wires of coil A, so
`A1 A2 B1 B2` becomes `A2 A1 B1 B2`). Do **not** try to fix it in software;
GRBL's `$3` invert mask flips the whole axis, both motors together.

### DRV8825 microstepping jumpers

Under each driver there are three jumper positions, M0/M1/M2:

| M0 | M1 | M2 | Resolution |
|---|---|---|---|
| — | — | — | full step |
| ● | — | — | 1/2 |
| — | ● | — | 1/4 |
| ● | ● | — | 1/8 |
| — | — | ● | 1/16 |
| ● | — | ● | 1/32 |

**Use 1/16 on all four drivers** (jumper in the M2 position only). That's what
the default config assumes, and it's the sweet spot for smoothness vs. torque
and step rate. Put the same jumpers on Y and A — mismatched microstepping
between the two shoulder motors will tear the joint apart.

### Setting driver current (do this before the first move)

DRV8825 modules have a tiny trimpot. With the shield powered and motors
**unplugged**, measure DC volts between the trimpot wiper and GND:

```
Vref = motor_rated_current_amps / 2
```

A 1.5 A NEMA 17 wants Vref ≈ 0.75 V. Start ~20 % lower than that, and stick
the supplied heatsinks on the driver chips — the DRV8825 gets genuinely hot.
The two shoulder drivers should be set to the **same** Vref.

### Optional limit switches

X-/X+, Y-/Y+, Z+/Z- pins along the shield edge. NC (normally closed) switches
wired switch-to-GND are far less noise-prone than NO. If you fit them, tick
*limit switches fitted* in Settings and use **Home ($H)**. Without them, use
**Set current as home** instead (see below).

---

## 4. The unit trick: GRBL millimetres are degrees

GRBL is a CNC controller and thinks in mm. This program defines
**1 GRBL "mm" = 1 degree of joint output**. So `$100`–`$102` are set to
*steps per degree* and every feed rate is *degrees per minute*.

```
steps_per_degree = motor_steps_per_rev × microsteps × gear_ratio ÷ 360
```

With 1.8° motors at 1/16 microstepping:

| Joint | Gearing | steps/degree | Setting |
|---|---|---|---|
| J1 base | 1:1 direct | 200 × 16 × 1 / 360 = **8.8889** | `$100` |
| J2 shoulder | 8:1 planetary | 200 × 16 × 8 / 360 = **71.1111** | `$101` |
| J3 elbow | 8:1 planetary | 200 × 16 × 8 / 360 = **71.1111** | `$102` |

If your base joint has a belt reduction, set its ratio in Settings and the
numbers recompute themselves.

**You do not have to type these into a terminal.** Open the *Settings* tab,
enter your motor steps, microstepping and gear ratios, then press
**Write $ settings to GRBL**. It sends `$100`–`$102`, `$110`–`$112` (max rates),
`$120`–`$122` (acceleration), `$130`–`$132` (travel), the direction invert
mask `$3`, and `$1=255` so the steppers keep holding torque when idle. GRBL
stores them in EEPROM, so this is a one-time job.

---

## 5. First power-up checklist

1. Connect with motors powered but the arm supported — or with the couplers
   loose so nothing can crash.
2. Settings tab → enter your real link lengths and gear ratios → **Save config**
   → **Write $ settings to GRBL**.
3. Jog J1 by 10°. Did it move 10°? If it moved 80°, your gear ratio is wrong.
   If it moved the wrong way, tick the *invert* box for that joint and re-push.
4. Repeat for J2 and J3. Watch that **both shoulder motors turn the same way**
   — if they fight each other you'll hear it immediately. Kill power and swap
   a coil pair on the offending motor.
5. Set your joint limits in Settings to whatever the arm can physically do,
   with a few degrees of margin. The software refuses any move outside them.

---

## 6. Telling the arm where it is

Steppers have no idea where they are at power-on. Two options:

**With limit switches** — tick *limit switches fitted*, press **Home ($H)**.
GRBL seeks the switches and the program then writes the work offset using the
*home angles* you configured (the joint angles the arm is actually at when the
switches trip).

**Without limit switches** — move the arm by hand (or jog it) to a pose you can
eyeball reliably, such as upper arm vertical and forearm horizontal. Type those
angles into the J1/J2/J3 boxes in the *Zero & home* card and press
**Set current as home**. That sends `G10 L20 P1 ...` and the readouts jump to
match. Do this every time you power up.

---

## 7. Angle conventions

Everything the program shows and accepts is in these terms:

- **J1** 0° = arm pointing along +X. Positive = counter-clockwise viewed from above.
- **J2** 0° = upper arm horizontal, pointing out. +90° = straight up.
- **J3** 0° = forearm in line with the upper arm (arm fully extended).
  Negative = forearm folding back toward the shoulder. This is a *relative*
  angle, which is what the elbow motor physically measures since it rides on
  the upper arm.

Cartesian coordinates are millimetres, origin on the floor at the base
rotation axis, +Z up.

---

## 8. Using the panel

**Joint jog** — six buttons per joint at ×1/×3/×10 of the selected step size.
Coloured bars show each joint's position within its limits, turning red at the
end stops. Keyboard: `A`/`D` base, `W`/`S` shoulder, `Q`/`E` elbow, `1`–`6`
step size, `Space` = E-STOP.

**Cartesian target** — type X/Y/Z and the IK solution appears live before you
commit. *Elbow up* picks which of the two solutions to prefer; if joint limits
block it the solver quietly tries the other one.

- **Go — joint move** sends one `G1`. The motors interpolate together, so the
  tip travels along an arc. Fast and smooth; use this normally.
- **Go — straight line** chops the path into ~3 mm segments, solves IK at each
  one and streams them. The tip travels in a real straight line. Sends far more
  commands, so it's slower to start.

**Poses** — saves the current joint angles under a name. Click a name to go
there; `+seq` appends it to the sequence you're building.

**Sequences** — build a list of poses with dwell times, then Run (optionally
looping). Each step waits for GRBL to report Idle before the dwell starts, so
timing is real, not guessed. Stop halts after the current move.

**Console** — full `ok`/`error:`/`ALARM:` traffic with decoded messages, plus
an MDI box for raw G-code (`$$`, `$G`, `G90 G1 X10 F600`, …).

**E-STOP** — sends jog-cancel, feed-hold, then a soft reset (`Ctrl-X`). Motion
stops immediately but **position is lost** and you'll need to re-home. Use
*Pause* (feed hold) for a recoverable stop.

---

## 9. Files

```
run.py                    launcher (--lan, --port, --no-browser)
test_arm.py               55 self-checks, no hardware needed
arm_config.json           your settings, written by the Settings tab
arm/config.py             defaults, persistence, steps/degree maths
arm/kinematics.py         FK, IK, straight-line path generation
arm/grbl.py               threaded serial driver + simulator
arm/server.py             Flask API
arm/templates/index.html  the entire UI (one file, no build step)
```

---

## 10. Troubleshooting

| Symptom | Cause |
|---|---|
| Console shows garbage on connect | Wrong baud. GRBL 1.1 = 115200. |
| `error:9` on every command | GRBL is in alarm. Press **Unlock ($X)**. |
| Motor buzzes but doesn't turn | Vref too low, or a coil pair split across the two connector halves. Check coil continuity with a meter. |
| Joint moves the wrong distance | Gear ratio or microstepping in Settings doesn't match the hardware. |
| Shoulder motors fight each other | A-axis clone jumper is right but one motor is mirrored — swap one coil pair on that motor. |
| Shoulder only one motor moves | A-Y clone jumper not fitted, or the A driver's Vref is at zero. |
| Arm sags when idle | `$1` isn't 255. Re-push settings. |
| Drivers get very hot / skip steps | Vref too high. Heatsinks on, and back the trimpot off. |
| `error:15` when jogging | Jog exceeds `$130`–`$132` travel. Widen your joint limits and re-push. |
| Steps lost during fast moves | Lower `$110`–`$112` and `$120`–`$122` in Settings. Geared joints have a lot of reflected inertia. |

---

## 11. Safety

Keep hands out of the arm's envelope while it's powered — a stepper through an
8:1 planetary has enough torque to break fingers, and it will happily run into
its own frame at full speed. Test new sequences at a low feed rate first, keep
the E-STOP within reach, and put a physical power switch on the motor supply
that you can hit without going through a browser.
