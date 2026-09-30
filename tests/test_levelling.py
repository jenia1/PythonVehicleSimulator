#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_levelling.py: checks for levelling in StrapdownINS - the complementary
filter, and the coarse alignment at the start of a run.

    Written as pytest tests, and also runnable directly while pytest is not
    installed:

        python3 tests/test_levelling.py

    Each check compares the filter with its exact discrete-time behaviour,
    for alpha = 1 - dt/tau:

      1. off by default  - no complementary_tau: identical to the unaided INS
      2. ideal IMU       - a resting dock stays exactly level
      3. gyro bias b     - pitch error settles at b (tau - dt),   ~ b tau
      4. accel bias b_a  - pitch error settles at atan(b_a / g),  ~ b_a / g
      5. initial error   - decays as e0 (1 - dt/tau)^n,          ~ e0 exp(-t/tau)

    and the coarse alignment, with accelerometer bias b_a:

      6. off by default  - no align: the estimate starts at the true attitude
      7. unaided         - starts at, and stays at, atan(b_a / g)
      8. levelled        - with gyro bias b too, exactly
                           atan(b_a / g) + b (tau - dt) (1 - alpha^n)

    and, without alignment, the case the proposal study runs:

      9. levelled        - from the true attitude, with both biases, exactly
                           (atan(b_a / g) + b (tau - dt)) (1 - alpha^n)

    Checks 2-4 and 6-9 run through simulate(), so they also exercise the path
    from a body's .nav_params to its StrapdownINS.

Author:     Jenia
"""
import sys

import numpy as np

from python_vehicle_simulator.lib import simulate
from python_vehicle_simulator.navigation import StrapdownINS
from python_vehicle_simulator.structures import Buoy, Dock
from python_vehicle_simulator.vehicles import DSRV

TAU = 1.0                   # complementary filter time constant (s)
DT = 0.02                   # sample time (s), alpha = 1 - DT/TAU = 0.98
G = 9.81                    # gravity (m/s^2), as in IMU and StrapdownINS
D2R = np.pi / 180


def runRestingDock(imu_params, nav_params, duration=20 * TAU):
    """
    log = runRestingDock(imu_params, nav_params, duration) steps a resting
    dock 10 m below a buoy through simulate(). 20 time constants is long
    enough for any start-up transient to fall below exp(-20) ~ 2e-9.
    """
    dock = Dock()
    dock.imu_params = imu_params
    dock.nav_params = nav_params

    N = int(round(duration / DT))
    return simulate(N, DT, DSRV("depthAutopilot", 10.0, 10.0),
                    [0, 0, 10.0, 0, 0, 0], "simple",
                    buoy=Buoy(), buoy_eta0=[0, 0, 0, 0, 0, 0],
                    dock=dock, dock_eta0=[0, 0, 10.0, 0, 0, 0])


def pitchError(log):
    """estimated minus true dock pitch at every sample (rad)"""
    return log["dock"]["eta_est"][:, 4] - log["dock"]["eta"][:, 4]


def finalPitchError(log):
    """estimated minus true dock pitch at the end of a run (rad)"""
    return pitchError(log)[-1]


###############################################################################
# Checks
###############################################################################
def test_off_by_default():
    """1. without complementary_tau the INS is exactly the unaided one"""
    rng = np.random.default_rng(0)
    unaided = StrapdownINS(np.zeros(6), np.zeros(6))
    explicitOff = StrapdownINS(np.zeros(6), np.zeros(6), complementary_tau=None)

    for _ in range(500):
        meas = {"f_b": rng.normal(0, 1, 3) + [0, 0, -G],
                "omega_b": rng.normal(0, 0.01, 3)}
        a, _ = unaided.update(meas, DT)
        b, _ = explicitOff.update(meas, DT)
        assert np.array_equal(a, b), "estimates differ with the filter off"


def test_ideal_imu_stays_level():
    """2. a resting dock with an ideal IMU stays exactly level"""
    log = runRestingDock({}, {"complementary_tau": TAU})
    error = log["dock"]["eta_est"][:, 3:6] - log["dock"]["eta"][:, 3:6]
    worst = np.abs(error).max()
    assert worst < 1e-12, "attitude drifted by %.3g rad" % worst


def test_gyro_bias_settles_at_b_tau():
    """3. a gyro bias b leaves a steady pitch error of b (tau - dt)"""
    b = 10.0 * D2R / 3600                           # 10 deg/hr, pitch axis
    log = runRestingDock({"gyro_bias": [0, b, 0]}, {"complementary_tau": TAU})

    error = finalPitchError(log)
    expected = b * (TAU - DT)
    assert abs(error - expected) < 1e-6 * expected, \
        "pitch error %.6g rad, expected b (tau - dt) = %.6g rad" % (
            error, expected)


def test_accel_bias_settles_at_ba_over_g():
    """4. an accelerometer bias b_a leaves a steady pitch error of atan(b_a/g)"""
    ba = 0.01                                       # ~1 mg, x axis
    log = runRestingDock({"accel_bias": [ba, 0, 0]}, {"complementary_tau": TAU})

    error = finalPitchError(log)
    expected = np.arctan2(ba, G)
    assert abs(error - expected) < 1e-6 * expected, \
        "pitch error %.6g rad, expected atan(b_a/g) = %.6g rad" % (
            error, expected)


def test_initial_error_decays_with_tau():
    """5. an initial pitch error decays as e0 (1 - dt/tau)^n"""
    e0 = 5.0 * D2R
    ins = StrapdownINS([0, 0, 0, 0, e0, 0], np.zeros(6), complementary_tau=TAU)
    atRest = {"f_b": np.array([0.0, 0.0, -G]), "omega_b": np.zeros(3)}

    n = int(round(TAU / DT))                        # one time constant
    for _ in range(n):
        eta, _ = ins.update(atRest, DT)

    expected = e0 * (1 - DT / TAU) ** n             # ~ e0 exp(-1)
    assert abs(eta[4] - expected) < 1e-12, \
        "pitch %.9g rad after one tau, expected %.9g rad" % (eta[4], expected)


def test_alignment_off_by_default():
    """6. without align the estimate starts at the true attitude"""
    ba = 0.01
    log = runRestingDock({"accel_bias": [ba, 0, 0]}, {})
    assert pitchError(log)[0] == 0.0, "estimate did not start at the truth"


def test_alignment_starts_at_ba_over_g():
    """7. aligned but unaided, an accel bias gives atan(b_a/g) throughout"""
    ba = 0.01
    log = runRestingDock({"accel_bias": [ba, 0, 0]}, {"align": True})

    error = pitchError(log)
    expected = np.arctan2(ba, G)
    worst = np.abs(error - expected).max()
    assert worst < 1e-12, \
        "pitch error off atan(b_a/g) = %.6g rad by up to %.3g rad" % (
            expected, worst)


def test_aligned_levelled_closed_form():
    """8. aligned and levelled: atan(b_a/g) + b (tau - dt)(1 - alpha^n)"""
    ba = 0.01
    b = 10.0 * D2R / 3600
    log = runRestingDock({"accel_bias": [ba, 0, 0], "gyro_bias": [0, b, 0]},
                         {"complementary_tau": TAU, "align": True})

    error = pitchError(log)
    n = np.arange(len(error))                       # updates since t = 0
    alpha = 1 - DT / TAU
    expected = np.arctan2(ba, G) + b * (TAU - DT) * (1 - alpha ** n)
    worst = np.abs(error - expected).max()
    assert worst < 1e-12, \
        "pitch error off the closed form by up to %.3g rad" % worst


def test_levelled_from_true_attitude_closed_form():
    """9. levelled from the true attitude: (atan(b_a/g) + b (tau - dt))(1 - alpha^n)"""
    ba = 0.01
    b = 10.0 * D2R / 3600
    log = runRestingDock({"accel_bias": [ba, 0, 0], "gyro_bias": [0, b, 0]},
                         {"complementary_tau": TAU})

    error = pitchError(log)
    n = np.arange(len(error))                       # updates since t = 0
    alpha = 1 - DT / TAU
    expected = (np.arctan2(ba, G) + b * (TAU - DT)) * (1 - alpha ** n)
    worst = np.abs(error - expected).max()
    assert worst < 1e-12, \
        "pitch error off the closed form by up to %.3g rad" % worst


###############################################################################
# Run directly, without pytest
###############################################################################
if __name__ == "__main__":
    tests = [test_off_by_default, test_ideal_imu_stays_level,
             test_gyro_bias_settles_at_b_tau,
             test_accel_bias_settles_at_ba_over_g,
             test_initial_error_decays_with_tau,
             test_alignment_off_by_default,
             test_alignment_starts_at_ba_over_g,
             test_aligned_levelled_closed_form,
             test_levelled_from_true_attitude_closed_form]

    failed = 0
    for test in tests:
        try:
            test()
            print("PASS  %s" % test.__doc__.strip())
        except AssertionError as error:
            failed += 1
            print("FAIL  %s\n      %s" % (test.__doc__.strip(), error))

    print("\n%d of %d checks passed" % (len(tests) - failed, len(tests)))
    sys.exit(1 if failed else 0)
