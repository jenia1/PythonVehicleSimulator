#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cableGeometry.py:

    The cable pseudo-measurement of the dock position, as set out in the
    thesis proposal. The dock hangs from a source (buoy, ship or underwater
    base) whose position is known, on a cable of known length L. Rather than
    dead-reckoning the dock's position from its accelerometers, its position
    is reconstructed from its *attitude* and the cable geometry:

        l   = L * C^n_b u^b + eta                                       (3)
        p_D = p_S + l                                                   (2)

    where C^n_b is the rotation from the dock body frame to the navigation
    frame, u^b the unit vector along the cable in the dock body frame, and
    eta accounts for deviations from the model (sag, drag). This module
    assumes a taut, straight cable, eta = 0.

    Why it matters: a free inertial solution integrates acceleration twice,
    so its position error is unbounded. The pseudo-measurement uses only
    attitude, so its error is L * (attitude error) - linear in time rather
    than quadratic, and bounded by 2L however bad the attitude becomes.

    An inextensible cable also constrains the dock independently of the
    cable's shape,

        || p_D - p_S ||  <=  L                                          (4)

    and an estimate violating (4) can be projected back onto the feasible
    region, which for this constraint has the closed form

        p_D  <-  p_S + L (p_D - p_S) / || p_D - p_S ||                  (5)

Methods:

    l = cableVector(eta_dock, L, u_b) evaluates (3).

    p_D = dockPosition(p_source, eta_dock, L, u_b) evaluates (2) with (3).

    d = cableDistance(p_dock, p_source) is || p_D - p_S || from (4).

    p_D = projectToCable(p_dock, p_source, L) applies (5) where (4) is
    violated, and leaves feasible estimates untouched.

    Each accepts either a single sample or an (N, ...) stack of samples, and
    returns the matching shape.

Author:     Jenia
"""
import numpy as np

from python_vehicle_simulator.lib.gnc import Rzyx

# Unit vector along the cable in the dock body frame. The default points
# along the body z axis, i.e. the cable leaves the dock "upwards" towards the
# source when the dock is level, matching a dock hanging straight down.
U_B_DEFAULT = np.array([0.0, 0.0, 1.0])


def cableVector(eta_dock, L, u_b=None):
    """
    l = cableVector(eta_dock, L, u_b) is the cable vector of equation (3),
    l = L * C^n_b u^b, in the navigation frame.

    Inputs:
        eta_dock: (6,) or (N,6) dock position/attitude; only the attitude
                  columns [3:6] are used, since (3) depends on attitude alone
        L:        cable length (m)
        u_b:      (3,) unit vector along the cable in the dock body frame

    Output:
        (3,) or (N,3) cable vector
    """
    if u_b is None:
        u_b = U_B_DEFAULT
    u_b = np.asarray(u_b, float).reshape(3)

    eta = np.asarray(eta_dock, float)
    single = eta.ndim == 1
    eta = np.atleast_2d(eta)

    l = np.empty([eta.shape[0], 3], float)
    for k, row in enumerate(eta):
        l[k] = L * (Rzyx(row[3], row[4], row[5]) @ u_b)

    return l[0] if single else l


def dockPosition(p_source, eta_dock, L, u_b=None):
    """
    p_D = dockPosition(p_source, eta_dock, L, u_b) is the cable
    pseudo-measurement of the dock position, equation (2) with (3):
    p_D = p_S + L * C^n_b u^b.

    Inputs:
        p_source: (3,), (6,) or (N,3)/(N,6) source position; only the
                  position columns [0:3] are used
        eta_dock: (6,) or (N,6) dock position/attitude, attitude used
        L, u_b:   as for cableVector()

    Output:
        (3,) or (N,3) estimated dock position
    """
    p_S = np.asarray(p_source, float)
    single = p_S.ndim == 1
    p_S = np.atleast_2d(p_S)[:, 0:3]

    l = np.atleast_2d(cableVector(eta_dock, L, u_b))
    p_D = p_S + l

    return p_D[0] if (single and p_D.shape[0] == 1) else p_D


def cableDistance(p_dock, p_source):
    """
    d = cableDistance(p_dock, p_source) is || p_D - p_S ||, the quantity
    constrained by equation (4).

    Output:
        scalar, or (N,) for stacked input
    """
    p_D = np.atleast_2d(np.asarray(p_dock, float))[:, 0:3]
    p_S = np.atleast_2d(np.asarray(p_source, float))[:, 0:3]

    d = np.linalg.norm(p_D - p_S, axis=1)

    return d[0] if d.size == 1 else d


def projectToCable(p_dock, p_source, L):
    """
    p_D = projectToCable(p_dock, p_source, L) applies the Euclidean
    projection of equation (5) to any estimate violating the inextensible
    cable constraint (4), and returns feasible estimates unchanged.

    Output:
        (3,) or (N,3), matching the input
    """
    p_D = np.asarray(p_dock, float)
    single = p_D.ndim == 1

    p_D = np.atleast_2d(p_D)[:, 0:3].copy()
    p_S = np.atleast_2d(np.asarray(p_source, float))[:, 0:3]

    d = p_D - p_S
    n = np.linalg.norm(d, axis=1)

    # Only estimates outside the feasible sphere are moved. n == 0 cannot be
    # projected (no direction), and is already feasible, so it is left alone.
    outside = (n > L) & (n > 0)
    p_D[outside] = p_S[outside if p_S.shape[0] > 1 else 0] \
        + L * d[outside] / n[outside, None]

    return p_D[0] if single else p_D
