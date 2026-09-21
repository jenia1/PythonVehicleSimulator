#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU.py:

    Strapdown IMU model with two modes, selected by the 'mode' flag:

    IMU('ideal')
        Direct kinematic readout of specific force and angular rate,
        no bias, scale factor, misalignment or noise.

    IMU('simple')
        Same kinematics passed through an error model:
            measured = M @ true + bias + noise
        where M = misalignment @ scale_factor. All error terms default to
        the ideal case (bias = 0, scale = 1, misalignment = identity,
        noise std = 0), so 'simple' behaves exactly like 'ideal' until error
        parameters are supplied:

            IMU('simple', gyro_bias=[0, 4.85e-5, 0])    # 10 deg/hr about y

        Bodies stepped by mainLoop.initBody() carry their own specification
        in an optional .imu_params dict, e.g.

            dock.imu_params = {"gyro_bias": [0, b, 0]}

        which is passed straight to this constructor.

Methods:

    imu_meas = measure(eta, nu, nu_dot) returns a dict:
        { "f_b":     (3,) specific force in body frame,  accelerometer (m/s^2)
          "omega_b": (3,) angular rate in body frame,     gyroscope (rad/s) }

    Inputs:
        eta:    (6,) position/attitude [x,y,z,phi,theta,psi], NED, ground truth
        nu:     (6,) body-frame velocity [u,v,w,p,q,r], ground truth
        nu_dot: (6,) body-frame acceleration [u_dot,...,r_dot], ground truth

Author:     Jenia
"""
import numpy as np
from python_vehicle_simulator.lib.gnc import Rzyx


class IMU:
    def __init__(self, mode="ideal",
                 accel_bias=None, accel_scale=None, accel_misalign=None,
                 accel_noise_std=None,
                 gyro_bias=None, gyro_scale=None, gyro_misalign=None,
                 gyro_noise_std=None):

        if mode not in ("ideal", "simple"):
            raise ValueError("IMU mode must be 'ideal' or 'simple'")

        self.mode = mode
        self.g = 9.81  # gravity (m/s^2)

        def vector(value, default):
            """3-vector error term, defaulting to the ideal case"""
            if value is None:
                return default
            value = np.asarray(value, float).reshape(3)
            return value

        def matrix(value, default):
            """3x3 error term, defaulting to the ideal case"""
            if value is None:
                return default
            return np.asarray(value, float).reshape(3, 3)

        # --- Accelerometer error model (identity/zero = ideal) ---
        self.accel_bias = vector(accel_bias, np.zeros(3))
        self.accel_scale = vector(accel_scale, np.ones(3))   # diagonal scale factors
        self.accel_misalign = matrix(accel_misalign, np.eye(3))
        self.accel_noise_std = vector(accel_noise_std, np.zeros(3))  # white noise std

        # --- Gyroscope error model (identity/zero = ideal) ---
        self.gyro_bias = vector(gyro_bias, np.zeros(3))
        self.gyro_scale = vector(gyro_scale, np.ones(3))
        self.gyro_misalign = matrix(gyro_misalign, np.eye(3))
        self.gyro_noise_std = vector(gyro_noise_std, np.zeros(3))

    def measure(self, eta, nu, nu_dot):
        """
        imu_meas = measure(eta, nu, nu_dot)
        """

        phi, theta, psi = eta[3], eta[4], eta[5]
        R = Rzyx(phi, theta, psi)  # body -> NED

        g_n = np.array([0, 0, self.g])  # NED, z positive down

        # True specific force and angular rate (body frame)
        f_b_true = nu_dot[0:3] - R.T @ g_n
        omega_b_true = nu[3:6]

        if self.mode == "ideal":
            f_b = f_b_true
            omega_b = omega_b_true

        else:  # "simple"
            f_b = self.accel_misalign @ (self.accel_scale * f_b_true) \
                + self.accel_bias \
                + np.random.normal(0.0, self.accel_noise_std)

            omega_b = self.gyro_misalign @ (self.gyro_scale * omega_b_true) \
                + self.gyro_bias \
                + np.random.normal(0.0, self.gyro_noise_std)

        return {"f_b": f_b, "omega_b": omega_b}
