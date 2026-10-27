"""
test_driver.py - logic tests for the ApexPilot driver.

These tests feed SYNTHETIC sensor states to the driver and check the
control decisions. They validate the internal logic (target-speed model,
steering direction, ABS/TC filters, stuck recovery) without needing a
running TORCS instance. Final validation is the TORCS run itself.

Run:  python3 -m pytest test_driver.py      (or: python3 test_driver.py)
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from driver import Driver  # noqa: E402


def make_state(**overrides):
    """A baseline 'straight road, mid-speed' sensor state."""
    state = {
        'angle': 0.0, 'curLapTime': 10.0, 'damage': 0.0,
        'distFromStart': 500.0, 'distRaced': 500.0, 'focus': 0.0,
        'fuel': 50.0, 'gear': 3, 'lastLapTime': 0.0, 'opponents': [],
        'racePos': 1, 'rpm': 6000.0, 'speedX': 100.0, 'speedY': 0.0,
        'speedZ': 0.0, 'track': [200.0] * 19, 'trackPos': 0.0,
        'wheelSpinVel': [84.0, 84.0, 84.0, 84.0], 'z': 0.4,
    }
    state.update(overrides)
    return state


def hairpin_track():
    """Rangefinders for a tight right-hand hairpin close ahead."""
    t = [200.0] * 19
    t[9] = 16.0    # 0 deg: wall of the corner is close
    t[8] = 14.0    # -5 deg
    t[10] = 22.0   # +5 deg
    t[7] = 20.0    # -10 deg
    t[11] = 34.0   # +10 deg
    t[6] = 30.0    # -15 deg
    t[12] = 48.0   # +15 deg
    return t


def test_straight_line_full_speed():
    d = Driver()
    target, _ = d.target_speed(make_state())
    assert target >= 230.0, target
    R = d.drive(make_state(speedX=100.0))
    assert R['accel'] >= 0.9 and R['brake'] == 0.0
    assert abs(R['steer']) < 0.05


def test_hairpin_target_speed_is_low_and_brakes():
    d = Driver()
    target, _ = d.target_speed(make_state(track=hairpin_track()))
    assert target < 110.0, target
    R = d.drive(make_state(track=hairpin_track(), speedX=170.0))
    assert R['brake'] > 0.3 and R['accel'] == 0.0


def test_steering_corrects_toward_centre():
    d = Driver()
    # Car left of centre (trackPos > 0): steer must be negative (right).
    R = d.drive(make_state(trackPos=0.6))
    assert R['steer'] < -0.05, R['steer']
    d2 = Driver()
    R2 = d2.drive(make_state(trackPos=-0.6))
    assert R2['steer'] > 0.05, R2['steer']


def test_traction_control_cuts_throttle():
    d = Driver()
    # Rear wheels spinning at ~2x road speed (100 km/h -> 27.8 m/s ->
    # ~84 rad/s at r=0.33; 170 rad/s is heavy wheelspin).
    spinning = make_state(wheelSpinVel=[84.0, 84.0, 170.0, 170.0])
    assert d.tc_filter(1.0, spinning) < 0.75
    assert d.tc_filter(1.0, make_state()) == 1.0


def test_abs_releases_brake_on_lockup():
    d = Driver()
    # Fast car (180 km/h -> 50 m/s) but rear wheels nearly stopped.
    locking = make_state(speedX=180.0,
                         wheelSpinVel=[150.0, 150.0, 40.0, 40.0])
    assert d.abs_filter(0.8, locking) < 0.5
    rolling = make_state(speedX=180.0,
                         wheelSpinVel=[150.0, 150.0, 148.0, 148.0])
    assert d.abs_filter(0.8, rolling) == 0.8


def test_stuck_recovery_reverses():
    d = Driver()
    # Lap time advances but the car does not move and does not race.
    for i in range(6):
        t = 10.0 + i * 0.6
        R = d.drive(make_state(curLapTime=t, distRaced=500.0,
                               speedX=0.0, gear=1))
    assert R['gear'] == -1 and R['accel'] > 0.5


def test_parser_round_trip():
    from msgParser import msgParser
    p = msgParser()
    state = p.parse('(angle 0.12)(speedX 87.5)(track 200 150 90)')
    assert state['angle'] == 0.12 and state['speedX'] == 87.5
    assert state['track'] == [200.0, 150.0, 90.0]
    s = p.stringify({'accel': 1.0, 'brake': 0.0, 'clutch': 0.0,
                     'gear': 3, 'steer': -0.1, 'focus': 0, 'meta': 0})
    assert '(gear 3)' in s and '(accel 1.0)' in s and '(steer -0.1)' in s


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items())
             if k.startswith('test_') and callable(v)]
    for fn in tests:
        fn()
        print('PASS', fn.__name__)
    print('All %d tests passed.' % len(tests))
