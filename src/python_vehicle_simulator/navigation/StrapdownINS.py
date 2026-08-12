#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StrapdownINS.py:

    Strapdown inertial navigation: dead-reckons eta_est, nu_est forward in
    time from IMU measurements only (no aiding/correction yet). With an
    ideal IMU this tracks the true eta, nu exactly (up to the discretization
    error already present in the truth model's own Euler integration); with
    the 'simple' IMU error model, estimates will drift as bias/noise are
    turned on.

    StrapdownINS(eta0, nu0, u_const=None)
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

Methods:

    eta_est, nu_est = update(imu_meas, sampleTime) propagates the estimate
    one step and returns it.

        Inputs:
            imu_meas: dict from IMU.measure(), { "f_b": (3,), "omega_b": (3,) }
            sampleTime

        Output:
            eta_est: (6,) estimated position/attitude
            nu_est:  (6,) estimated body-frame velocity

Author:     Jenia
"""
import numpy as np
from python_vehicle_simulator.lib.gnc import Rzyx, attitudeEuler


class StrapdownINS:
    def __init__(self, eta0, nu0, u_const=None):
        self.eta_est = np.array(eta0, float).copy()
        self.nu_est = np.array(nu0, float).copy()
        self.g = 9.81  # gravity (m/s^2), must match IMU.g
        self.u_const = u_const

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

        # Integrate 6-DOF kinematics (position + Euler angles)
        self.eta_est = attitudeEuler(self.eta_est, self.nu_est, sampleTime)

        return self.eta_est, self.nu_est
