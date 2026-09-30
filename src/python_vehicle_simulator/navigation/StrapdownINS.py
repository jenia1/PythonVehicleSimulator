#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StrapdownINS.py:

    Strapdown inertial navigation: dead-reckons eta_est, nu_est forward in
    time from IMU measurements. With an ideal IMU this tracks the true eta, nu
    exactly (up to the discretization error already present in the truth
    model's own Euler integration); with the 'simple' IMU error model,
    estimates will drift as bias/noise are turned on.

    Optionally, roll and pitch are levelled against the accelerometer with a
    complementary filter (below), and can start from a coarse alignment
    instead of the true attitude (further below). Nothing else is aided.

    StrapdownINS(eta0, nu0, u_const=None, complementary_tau=None, align=False)
        eta0, nu0: initial estimate, normally the vehicle's true initial
        state (a navigation system has to be initialized somehow).
        u_const: if not None, surge (nu_est[0]) is held fixed at this value
        instead of being integrated from the accelerometer. Needed for
        vehicle models (e.g. DSRV) that hardcode a constant cruise speed
        without a matching Coriolis term in nu_dot[0] - for those, the
        accelerometer sees a nonzero specific force in x (from gravity
        resolved into the body frame as pitch changes) that doesn't
        correspond to any real surge acceleration, so integrating it would
        just drift the estimate away from the vehicle's own truth model.
        complementary_tau: time constant (s) of the complementary filter on
        roll and pitch. None, the default, leaves the INS unaided, exactly as
        it was before the filter existed.
        align: if True, the owner calls coarseAlign() once before the run, and
        roll and pitch start from the accelerometer instead of eta0. False,
        the default, starts from eta0 exactly.

Complementary filter:

    After the gyro has been integrated, roll and pitch are blended with the
    roll and pitch the accelerometer implies, taking the specific force to be
    gravity alone:

        rollAcc  = atan2(-f_y, -f_z)
        pitchAcc = atan2( f_x, sqrt(f_y^2 + f_z^2))

        angle = alpha * angleGyro + (1 - alpha) * angleAcc
        alpha = 1 - sampleTime / complementary_tau

    That is the textbook form: trust the gyro for changes faster than
    complementary_tau and the accelerometer for slower ones. At the
    simulation's sampleTime = 0.02 s, complementary_tau = 1 s gives the
    familiar alpha = 0.98. The time constant is the parameter rather than
    alpha because alpha depends on the sample rate - the same 0.98 would be a
    0.1 s filter at 500 Hz - while tau is a physical time.

    The blend is computed as

        angle = angleGyro + (1 - alpha) * ssa(angleAcc - angleGyro)

    which is the same equation, with the difference wrapped at +/-180 deg so
    that blending e.g. 179 deg and -179 deg does not average to ~0 deg.

    Yaw is not levelled: gravity carries no heading information, which is
    the magnetometer's job. Position and velocity are not aided either, so
    they still drift - more slowly (t^2 rather than t^3), but without bound.

    What to expect, with gyro bias b and accelerometer bias b_a:
        - gyro bias:  roll/pitch error settles at b * (tau - sampleTime),
                      i.e. about b * tau, instead of growing as b * t
        - accel bias: roll/pitch error settles at atan(b_a / g), about b_a/g
        - an initial error decays as (1 - sampleTime/tau)^n, about exp(-t/tau)

    Levelling assumes the accelerometer measures gravity only, i.e. that the
    body is not accelerating. It suits a resting dock. It is wrong for a
    manoeuvring vehicle, and for a swinging dock, whose accelerometer mostly
    measures the cable tension, along the cable.

Coarse alignment:

    A real navigation unit does not know its attitude at switch-on. Resting
    before the run, it takes roll and pitch from the accelerometer, with the
    same formulas as the filter above, so it starts with the accelerometer's
    error already in: atan(b_a / g) for a bias b_a. Starting from the true
    attitude instead is a perfect start no real unit gets, and it flatters
    the unaided INS against the levelled one, which cannot beat
    atan(b_a / g).

    Yaw is left at eta0: gravity carries no heading information.

    With alignment and levelling together, the filter starts at the
    accelerometer's steady state, so an accelerometer bias gives a constant
    error from the first sample. A gyro bias b then adds, exactly,

        roll/pitch error = atan(b_a / g) + b (tau - sampleTime) (1 - alpha^n)

Methods:

    eta_est, nu_est = update(imu_meas, sampleTime) propagates the estimate
    one step and returns it.

    coarseAlign(f_b) sets roll and pitch from an accelerometer reading taken
    at rest.

        Inputs:
            imu_meas: dict from IMU.measure(), { "f_b": (3,), "omega_b": (3,) }
            sampleTime

        Output:
            eta_est: (6,) estimated position/attitude
            nu_est:  (6,) estimated body-frame velocity

Author:     Jenia
"""
import math

import numpy as np
from python_vehicle_simulator.lib.gnc import Rzyx, attitudeEuler, ssa


def accelerometerAttitude(f_b):
    """
    roll, pitch = accelerometerAttitude(f_b) returns the roll and pitch (rad)
    an accelerometer reading implies if it measures gravity alone. At rest
    f_b = -R^T g_n, so f_b = [0, 0, -g] when level.
    """
    roll = math.atan2(-f_b[1], -f_b[2])
    pitch = math.atan2(f_b[0], math.hypot(f_b[1], f_b[2]))
    return roll, pitch


class StrapdownINS:
    def __init__(self, eta0, nu0, u_const=None, complementary_tau=None,
                 align=False):
        self.eta_est = np.array(eta0, float).copy()
        self.nu_est = np.array(nu0, float).copy()
        self.g = 9.81  # gravity (m/s^2), must match IMU.g
        self.u_const = u_const
        self.complementary_tau = complementary_tau
        self.align = align

    def update(self, imu_meas, sampleTime):

        f_b = imu_meas["f_b"]
        omega_b = imu_meas["omega_b"]

        # Gyro gives body angular rate directly (algebraic, not integrated)
        self.nu_est[3:6] = omega_b

        # Integrate specific force -> body-frame linear velocity
        # (inverse of the accelerometer model: f_b = v_dot_b - R^T @ g_n)
        R = Rzyx(self.eta_est[3], self.eta_est[4], self.eta_est[5])  # body -> NED
        g_n = np.array([0, 0, self.g])
        v_dot_b = f_b + R.T @ g_n
        self.nu_est[0:3] = self.nu_est[0:3] + sampleTime * v_dot_b

        if self.u_const is not None:
            self.nu_est[0] = self.u_const

        # Integrate 6-DOF kinematics (position + Euler angles). This is the
        # gyro step of the complementary filter: the angles it leaves in
        # eta_est are angleGyro.
        self.eta_est = attitudeEuler(self.eta_est, self.nu_est, sampleTime)

        # Complementary filter on roll and pitch, if enabled
        if self.complementary_tau is not None:
            self.level(f_b, sampleTime)

        return self.eta_est, self.nu_est

    def level(self, f_b, sampleTime):
        """
        level(f_b, sampleTime) blends the gyro-integrated roll and pitch in
        eta_est with the roll and pitch the accelerometer implies:

            angle = alpha * angleGyro + (1 - alpha) * angleAcc
            alpha = 1 - sampleTime / complementary_tau
        """
        if sampleTime >= self.complementary_tau:
            raise ValueError(
                "complementary_tau (%g s) must be longer than the sample time "
                "(%g s), or alpha = 1 - sampleTime/tau is not in (0, 1)"
                % (self.complementary_tau, sampleTime))

        alpha = 1.0 - sampleTime / self.complementary_tau

        # Roll and pitch implied by the accelerometer, if it sees only gravity
        rollAcc, pitchAcc = accelerometerAttitude(f_b)

        rollGyro, pitchGyro = self.eta_est[3], self.eta_est[4]

        # angle = alpha * angleGyro + (1 - alpha) * angleAcc, wrapped at +/-180
        self.eta_est[3] = rollGyro + (1 - alpha) * ssa(rollAcc - rollGyro)
        self.eta_est[4] = pitchGyro + (1 - alpha) * ssa(pitchAcc - pitchGyro)

    def coarseAlign(self, f_b):
        """
        coarseAlign(f_b) sets roll and pitch in eta_est from an accelerometer
        reading taken at rest, before the run. Yaw is left as it is.
        """
        self.eta_est[3], self.eta_est[4] = accelerometerAttitude(f_b)
