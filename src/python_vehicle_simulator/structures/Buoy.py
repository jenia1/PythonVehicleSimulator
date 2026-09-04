#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Buoy.py:

    Class for the surface buoy the docking structure hangs from.

    Buoy()

    Static for now: the buoy holds the position it is initialized with by
    initBody(), and dynamics() returns zero velocity and acceleration. It
    still carries its own IMU and navigation filter, so it is stepped through
    the same initBody()/stepBody() path as the vehicle and the dock - a
    static body with an unaided INS is the cleanest read on dead-reckoning
    drift, since ground truth never moves and every deviation of the estimate
    is error.

    The initial position is passed to simulate() as buoy_eta0, the same way
    the vehicle's eta0 is, so it is not duplicated here.

Methods:

    [nu, u_actual, nu_dot] = dynamics(eta,nu,u_actual,u_control,sampleTime)
        returns zero nu and nu_dot, holding the buoy in place. The arguments
        are ignored; they exist so the buoy can be stepped through the same
        path as a vehicle.

Author:     Jenia
"""
import numpy as np


# Class Buoy
class Buoy:
    """
    Buoy()      Static anchor point at the top of the cable
    """

    def __init__(self):

        self.name = "Buoy (static anchor point)"
        self.L = 0.5                            # characteristic length (m)

        self.nu = np.zeros(6, float)            # velocity vector
        self.u_actual = np.array([], float)     # no actuators

        self.controls = []
        self.dimU = len(self.controls)

    def dynamics(self, eta, nu, u_actual, u_control, sampleTime):
        """
        [nu, u_actual, nu_dot] = dynamics(eta,nu,u_actual,u_control,sampleTime)
        holds the buoy in place: zero velocity, zero acceleration.
        """
        nu = np.zeros(6, float)
        nu_dot = np.zeros(6, float)

        return nu, u_actual, nu_dot
