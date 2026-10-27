# ApexPilot — IBM Racing League (TORCS) Capstone

**Author:** Eric Pukuta · IBM SkillsBuild AI Racing League
**Track:** Corkscrew · **Simulator:** TORCS with the SCR server

## Project description

ApexPilot is an autonomous TORCS driver written in Python. It connects
to the TORCS SCR server over UDP, reads the car's sensors (track
rangefinders, speed, wheel spin, position on track, damage) about every
10 ms of race time, and answers with steering, throttle, brake, clutch
and gear commands.

## The enhancement

The baseline driver provided with the course is a naive controller: it
steers toward the centre of the track, aims at one fixed target speed,
and brakes only once it is already too fast. On a technical track like
Corkscrew that means braking too late, spinning on corner exit, wall
contact — and damage, which the capstone rules forbid.

ApexPilot replaces that control logic with five improvements:

1. **Predictive target speed (curvature preview).** Three rangefinders
   (−30°, 0°, +30°) give three points on the track edge ahead. A circle
   fitted through those points estimates the radius R of the upcoming
   bend, and the target speed follows the car's lateral-acceleration
   limit, v = √(a·R). The car brakes *before* a hairpin and accelerates
   early out of fast corners instead of reacting inside them. When a
   rangefinder is saturated (open road), the controller falls back to an
   ahead-distance rule and ray asymmetry.
2. **Apex aiming.** In a bend, the lateral target shifts toward the
   inside of the corner (the apex), shortening the path and reducing
   steering demand; on straights the target is the centre line.
3. **Speed-adaptive steering.** The lateral-correction gain decreases
   with speed (responsive in hairpins, stable on straights) and the
   steering output is smoothed to prevent oscillation.
4. **ABS and traction control.** Wheel-spin sensors are compared with
   road speed: a locking rear wheel releases the brake (ABS), spinning
   driven wheels cut the throttle (TC). This keeps the car stable and
   protects the zero-damage requirement.
5. **Stuck recovery.** If the car is commanded to accelerate but its
   raced distance does not progress for two seconds (e.g. nose against
   a wall after a spin), it reverses briefly with opposite steering and
   resumes — instead of sitting against the wall for the whole lap.

Gear shifting uses RPM thresholds with hysteresis, and a brief clutch
phase gives a clean standing start.

The car's model dynamics and livery are **not** modified — all changes
are external control logic, as required by the capstone rules.

## Repository layout

```
src/
  pyclient.py    UDP client: identification + race loop
  msgParser.py   TORCS sensor message parser / action stringifier
  driver.py      the ApexPilot control logic (the enhancement lives here)
tests/
  test_driver.py logic tests on synthetic sensor data (no TORCS needed)
livery/
  PUT_LIVERY_HERE.md   where to drop the unchanged course livery files
SUBMISSION.md    ready-to-send submission pack (description + email)
```

## Running the driver

1. Install TORCS and enable the **scr_server** robot (see the course
   setup guide). Load the **Corkscrew** track, standing (grid) start.
2. Start the race in TORCS — the server listens on UDP port 3001.
3. In this repo: `cd src && python3 pyclient.py`
4. The client identifies itself, then drives until the server ends the
   race. Record the fastest clean lap (zero damage) for the submission
   video.

## Tests

```
python3 tests/test_driver.py
```

Seven logic tests cover: full-speed behaviour on an open straight,
braking and target speed into a hairpin, steering correction toward the
centre from both sides, traction-control and ABS triggers, the stuck
recovery, and the message parser round-trip. (Logic tests on synthetic
data — the TORCS run is the final validation.)

## Tuning knobs (top of `src/driver.py`)

| Parameter    | Default | Effect                                        |
|--------------|---------|-----------------------------------------------|
| `A_LAT_MAX`  | 12.0    | Lateral-g budget; lower = safer, slower laps  |
| `TOP_SPEED`  | 240     | km/h cap on straights                         |
| `APEX_OFFSET`| 0.30    | How far toward the apex the car aims          |
| `MIN_TARGET` | 42      | km/h floor for the slowest corners            |

If the car ever runs wide in a specific corner during testing, lower
`A_LAT_MAX` by 1.0 before touching anything else.
