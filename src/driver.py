"""
driver.py - ApexPilot: an improved TORCS driver (SCR interface).

Project: IBM Racing League (TORCS) Capstone
Author:  Eric Pukuta

The baseline driver provided with the course is a naive controller: it
steers proportionally toward the track centre, accelerates toward one
fixed target speed, brakes only when already too fast, and has no wheel
slip management. On a technical track such as Corkscrew that means late
braking, corner exits with a spinning car, wall contact (damage) and
slow laps.

ApexPilot keeps the same interface but replaces the control logic with
five improvements:

1. PREDICTIVE TARGET SPEED (curvature preview)
   Three track rangefinders (-30 deg, 0 deg, +30 deg) define three points
   on the track edge ahead of the car. A circle is fitted through those
   points; its radius R estimates the radius of the upcoming bend and
   the target speed follows from the lateral-acceleration limit of the
   car:  v = sqrt(A_LAT_MAX * R).  The car therefore brakes *before* a
   hairpin and accelerates early out of fast sweepers, instead of
   reacting once it is already in the corner.

2. APEX AIMING
   When a bend is detected, the lateral target position shifts toward
   the inside of the corner (the apex), which shortens the path and
   reduces the steering demand. On straights the target is the centre.

3. SPEED-ADAPTIVE STEERING
   The lateral-correction gain decreases with speed (stable at high
   speed, responsive at low speed) and the steering command is smoothed
   to avoid oscillation.

4. ABS + TRACTION CONTROL
   Wheel-spin sensors are compared with the car's speed. Under braking,
   if a rear wheel slows far below road speed the brake is released
   (ABS); under acceleration, if the driven wheels spin well above road
   speed the throttle is cut (TC). This keeps the car stable and the
   tyres gripping, which protects the zero-damage requirement.

5. STUCK RECOVERY
   If the car is commanded to accelerate but its raced distance barely
   changes for two seconds (nose against a wall after a spin), the
   driver reverses briefly with opposite steering, then resumes.

The car's model dynamics and livery are NOT modified - everything here
is external control logic talking to the standard SCR server.
"""

import math

# ----------------------------------------------------------------------
# Tunable parameters
# ----------------------------------------------------------------------
STEER_LOCK = 0.366519      # steering lock of the car, radians (~21 deg)
A_LAT_MAX = 12.0           # lateral acceleration budget, m/s^2
                           # (conservative on purpose: zero damage first)
TOP_SPEED = 240.0          # km/h, cap on long straights
MIN_TARGET = 42.0          # km/h, slowest corner target speed
APEX_OFFSET = 0.30         # how far toward the apex we aim (trackPos units)
WHEEL_RADIUS = 0.33        # m, for wheel-spin -> road speed conversion
SENSOR_ANGLES = [-90, -75, -60, -45, -30, -20, -15, -10, -5, 0,
                 5, 10, 15, 20, 30, 45, 60, 75, 90]
IDX_M30 = SENSOR_ANGLES.index(-30)   # rangefinder at -30 deg
IDX_0 = SENSOR_ANGLES.index(0)       # rangefinder straight ahead
IDX_P30 = SENSOR_ANGLES.index(30)    # rangefinder at +30 deg


def _clamp(value, low, high):
    return max(low, min(high, value))


def _circle_radius(p1, p2, p3):
    """Radius of the circle through three points; inf if collinear."""
    (x1, y1), (x2, y2), (x3, y3) = p1, p2, p3
    d = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if abs(d) < 1e-6:
        return float('inf'), 0.0
    ux = ((x1 ** 2 + y1 ** 2) * (y2 - y3) + (x2 ** 2 + y2 ** 2) * (y3 - y1)
          + (x3 ** 2 + y3 ** 2) * (y1 - y2)) / d
    uy = ((x1 ** 2 + y1 ** 2) * (x3 - x2) + (x2 ** 2 + y2 ** 2) * (x1 - x3)
          + (x3 ** 2 + y3 ** 2) * (x2 - x1)) / d
    radius = math.hypot(x1 - ux, y1 - uy)
    return radius, uy  # uy > 0 -> circle centre to the left -> left bend


class Driver(object):
    """Stateful TORCS driver implementing the ApexPilot logic."""

    def __init__(self):
        self.parser = None          # set by pyclient (msgParser instance)
        self.prev_steer = 0.0
        self.stuck_since = None     # lap time when immobility was noticed
        self.recovery_until = -1.0  # lap time until which we reverse
        self.last_dist = 0.0
        self.last_progress_time = 0.0

    # ------------------------------------------------------------------
    # 1. Predictive target speed from the curvature preview
    # ------------------------------------------------------------------
    def target_speed(self, S):
        track = S.get('track') or []
        if len(track) != 19:
            return 120.0, 0.0
        d_m30 = float(track[IDX_M30])
        d_0 = float(track[IDX_0])
        d_p30 = float(track[IDX_P30])

        # Ahead-distance term: the less road is visible straight ahead,
        # the slower we must be. This alone reproduces (and sharpens)
        # the classic baseline behaviour.
        target = min(TOP_SPEED, 35.0 + 2.6 * d_0)
        centre_y = 0.0

        if max(d_m30, d_0, d_p30) < 199.0:
            # All three preview rangefinders see a real edge (not the
            # 200 m saturation limit): fit the circle through the three
            # edge points to estimate the bend radius ahead.
            points = []
            for dist, ang in ((d_m30, -30.0), (d_0, 0.0), (d_p30, 30.0)):
                rad = math.radians(ang)
                # Sensor angles: positive points to the LEFT of the car.
                points.append((max(1.0, dist) * math.cos(rad),
                               max(1.0, dist) * math.sin(rad)))
            radius, centre_y = _circle_radius(*points)
            if not math.isinf(radius):
                curve_speed = math.sqrt(A_LAT_MAX * radius) * 3.6
                target = min(target, curve_speed)
        else:
            # A preview ray is saturated: no reliable circle fit.
            # Fall back to ray asymmetry for the bend direction only.
            if d_p30 - d_m30 > 15.0:
                centre_y = 5.0    # road opens to the left -> left bend
            elif d_m30 - d_p30 > 15.0:
                centre_y = -5.0   # road opens to the right -> right bend

        # Near the track edge, be conservative (protects zero damage).
        if abs(float(S.get('trackPos', 0.0))) > 0.85:
            target = min(target, 120.0)
        return _clamp(target, MIN_TARGET, TOP_SPEED), centre_y

    # ------------------------------------------------------------------
    # 2 + 3. Steering with apex aiming and speed-adaptive gain
    # ------------------------------------------------------------------
    def steering(self, S, centre_y):
        speed = float(S.get('speedX', 0.0))
        # Gain: firm at low speed, gentle at high speed.
        gain = _clamp(0.62 - 0.0011 * speed, 0.36, 0.62)
        # Apex aiming: shift the lateral target toward the inside.
        if centre_y > 2.0:
            target_pos = APEX_OFFSET          # left bend -> aim left
        elif centre_y < -2.0:
            target_pos = -APEX_OFFSET         # right bend -> aim right
        else:
            target_pos = 0.0                   # straight -> centre
        lateral_error = float(S.get('trackPos', 0.0)) - target_pos
        steer = (float(S.get('angle', 0.0)) - lateral_error * gain) / STEER_LOCK
        # Smoothing: avoid violent left/right oscillation.
        steer = 0.65 * steer + 0.35 * self.prev_steer
        self.prev_steer = steer
        return _clamp(steer, -1.0, 1.0)

    # ------------------------------------------------------------------
    # 4. ABS and traction control filters
    # ------------------------------------------------------------------
    def abs_filter(self, brake, S):
        if brake <= 0.05:
            return brake
        speed_ms = float(S.get('speedX', 0.0)) / 3.6
        wheels = S.get('wheelSpinVel') or []
        if speed_ms > 5.0 and len(wheels) == 4:
            rear_min = min(float(wheels[2]), float(wheels[3])) * WHEEL_RADIUS
            if rear_min < 0.78 * speed_ms:   # a rear wheel is locking up
                return brake * 0.45
        return brake

    def tc_filter(self, accel, S):
        if accel <= 0.05:
            return accel
        speed_ms = float(S.get('speedX', 0.0)) / 3.6
        wheels = S.get('wheelSpinVel') or []
        if speed_ms > 3.0 and len(wheels) == 4:
            rear_max = max(float(wheels[2]), float(wheels[3])) * WHEEL_RADIUS
            if rear_max > 1.30 * speed_ms:   # driven wheels are spinning
                return accel * 0.55
        return accel

    # ------------------------------------------------------------------
    # 5. Stuck detection / recovery
    # ------------------------------------------------------------------
    def update_stuck(self, S):
        now = float(S.get('curLapTime', 0.0))
        dist = float(S.get('distRaced', 0.0))
        if dist - self.last_dist > 1.0:
            self.last_dist = dist
            self.last_progress_time = now
            self.stuck_since = None
        if now < self.recovery_until:
            return True
        trying = float(S.get('speedX', 0.0)) < 3.0
        if trying and now - self.last_progress_time > 2.0:
            self.recovery_until = now + 1.2
            self.last_progress_time = now
            return True
        return False

    # ------------------------------------------------------------------
    # Main entry point, called once per server message
    # ------------------------------------------------------------------
    def drive(self, S):
        R = {'accel': 0.0, 'brake': 0.0, 'clutch': 0.0, 'gear': 1,
             'steer': 0.0, 'focus': 0, 'meta': 0}

        speed = float(S.get('speedX', 0.0))
        rpm = float(S.get('rpm', 0.0))
        gear = int(S.get('gear', 1))

        # --- stuck recovery overrides everything else -----------------
        if self.update_stuck(S):
            R['gear'] = -1
            R['accel'] = 0.65
            R['steer'] = _clamp(-self.prev_steer, -1.0, 1.0)
            return R

        # --- target speed + steering ---------------------------------
        target, centre_y = self.target_speed(S)
        R['steer'] = self.steering(S, centre_y)

        # --- longitudinal control -------------------------------------
        if speed < target - 1.0:
            # Accelerate; ease off as we approach the target.
            R['accel'] = _clamp((target - speed) / 25.0, 0.25, 1.0)
            R['brake'] = 0.0
        elif speed > target + 4.0:
            # Brake proportionally to the excess speed.
            R['brake'] = _clamp((speed - target) / 45.0, 0.15, 0.9)
            R['accel'] = 0.0
        else:
            R['accel'] = 0.35   # maintain speed through the corner
            R['brake'] = 0.0

        # --- gear shifting with hysteresis ----------------------------
        if gear > 0:
            if rpm > 8200 and gear < 6:
                gear += 1
            elif rpm < 4200 and gear > 1:
                gear -= 1
        if gear == 0:
            gear = 1
        R['gear'] = gear

        # Standing start: engage the clutch briefly for a clean launch.
        if float(S.get('distRaced', 0.0)) < 2.0 and speed < 12.0:
            R['clutch'] = 0.6

        # --- slip management ------------------------------------------
        R['brake'] = self.abs_filter(R['brake'], S)
        R['accel'] = self.tc_filter(R['accel'], S)

        return R
