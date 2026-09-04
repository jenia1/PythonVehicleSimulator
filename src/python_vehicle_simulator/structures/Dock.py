#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dock.py:

    Class for the docking head at the lower end of the cable - the unit the
    vehicle navigates to.

    Dock()

    Static for now: the dock holds the position it is initialized with by
    initBody(), and dynamics() returns zero velocity and acceleration, so the
    cable is not yet needed to hold it up. It carries its own IMU and
    navigation filter and is stepped through the same initBody()/stepBody()
    path as the vehicle and the buoy.

    The initial position is passed to simulate() as dock_eta0, the same way
    the vehicle's eta0 is, so it is not duplicated here.

    f_ext is the external force acting on the dock, set by simulate() from
    the cable before the dock is stepped (the role u_control plays for a
    vehicle). It is held here but not yet used: with static dynamics there is
    nothing for a force to act on. dynamics() reads it once the dock is given
    real 6-DOF dynamics.

Methods:

    [nu, u_actual, nu_dot] = dynamics(eta,nu,u_actual,u_control,sampleTime)
        returns zero nu and nu_dot, holding the dock in place. The arguments
        are ignored; they exist so the dock can be stepped through the same
        path as a vehicle.

Author:     Jenia
"""
import numpy as np


# Class Dock
class Dock:
    """
    Dock()      Docking head at the lower end of the cable
    """

    def __init__(self):

        self.name = "Dock (docking head)"
        self.L = 1.0                            # characteristic length (m)

        self.nu = np.zeros(6, float)            # velocity vector
        self.u_actual = np.array([], float)     # no actuators

        self.controls = []
        self.dimU = len(self.controls)

        # External force/moment, set from the cable by simulate() before the
        # dock is stepped. Not read yet, see the module docstring.
        self.f_ext = np.zeros(6, float)

    def dynamics(self, eta, nu, u_actual, u_control, sampleTime):
        """
        [nu, u_actual, nu_dot] = dynamics(eta,nu,u_actual,u_control,sampleTime)
        holds the dock in place: zero velocity, zero acceleration.
        """
        nu = np.zeros(6, float)
        nu_dot = np.zeros(6, float)

        return nu, u_actual, nu_dot
