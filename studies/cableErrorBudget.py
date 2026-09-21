#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cableErrorBudget.py:

    Preliminary result for the thesis proposal: how well can the dock's
    position be known from its navigation unit, before any fusion filter
    exists?

    The dock position comes from the cable pseudo-measurement of eq (3): the
    dock's *attitude*, with the known source position and known cable length,

        p_D = p_S + L * C^n_b u^b

    so its error is L * (attitude error), bounded by 2L however bad the
    attitude becomes. The tether turns what would be an unbounded position
    problem into a bounded attitude problem.

    For reference only, the summary table also prints the free inertial
    solution - the dock's accelerometers integrated twice. It grows as t^3,
    because an attitude error makes the INS subtract gravity along the wrong
    direction and the leftover is integrated twice. It is deliberately not
    plotted: that an unaided INS diverges is textbook [Farrell, Groves], and
    not the point of the thesis.

    Figures written to studies/figures/ as PDF for inclusion in the proposal:

      fig1_cableSensitivity   eq (3): error amplification by cable length,
                              and which attitude angle actually matters
      fig2_errorGrowth        eq (3) in time, per IMU grade

Usage:

    python3 studies/cableErrorBudget.py

Assumptions, all stated in the proposal text:
    - taut, straight cable, eta = 0 in (3): no sag, no drag
    - dock attitude error is driven by gyro bias only, unaided, so it grows
      linearly. This is the worst case: a real dock filter would level
      roll/pitch against gravity and bound them. Quoting it as worst case is
      what motivates the filter design rather than overselling the problem.
    - the source position p_S is taken as known exactly

Author:     Jenia
"""
import os

import matplotlib
matplotlib.use("Agg")           # figures are written to file, never shown

import matplotlib.pyplot as plt
import numpy as np

from python_vehicle_simulator.vehicles import DSRV
from python_vehicle_simulator.structures import Buoy, Dock
from python_vehicle_simulator.lib import simulate
from python_vehicle_simulator.navigation import cableVector, dockPosition

### Study parameters ###
L = 10.0                        # cable length (m)
duration = 300.0                # terminal approach duration (s)
sampleTime = 0.02               # sample time (s)
captureRadius = 0.5             # magnetic latch capture radius (m)

# Gyro bias (deg/hr) and the name of its sensor grade. The dock unit is meant
# to be cheap, so the MEMS grades are the ones that matter. Figures and tables
# build their labels from these through gradeLabel(), so the number shown is
# always the number simulated: change a bias here and nothing else.
gyroGrades = [
    (1.0,   "tactical"),
    (10.0,  "industrial MEMS"),
    (100.0, "consumer MEMS"),
]


def gradeLabel(bias):
    """
    label = gradeLabel(bias) is the display label for a gyro bias in deg/hr,
    e.g. "industrial MEMS, 10 deg/hr", built from the number itself. A bias
    that is not listed in gyroGrades is shown by its number alone.
    """
    for gradeBias, name in gyroGrades:
        if gradeBias == bias:
            return "%s, %g deg/hr" % (name, bias)
    return "%g deg/hr" % bias

# Cable lengths for the sensitivity plot (m)
cableLengths = [5.0, 10.0, 20.0, 50.0]

D2R = np.pi / 180
DEG_PER_HOUR = D2R / 3600       # deg/hr -> rad/s

figureDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figures")

# Plot styling, kept readable at the size a thesis figure is printed
plt.rcParams.update({
    "font.size": 9,
    "axes.titlesize": 10,
    "axes.labelsize": 9,
    "legend.fontsize": 8,
    "figure.dpi": 150,
})


###############################################################################
# Simulation
###############################################################################
def runDock(gyroBias_degPerHour):
    """
    log = runDock(gyroBias_degPerHour) runs the simulation with the dock's
    navigation unit carrying a gyro bias about its y (pitch) axis, which
    swings the cable in the x direction.

    A single-axis bias is used so that the attitude error is exactly
    bias * t, which makes the simulated result directly comparable with the
    closed form 2L sin(dtheta/2) and validates both.

    The scene: the buoy and dock sit half-way along the vehicle's track, and
    the vehicle holds the dock's depth, so it passes the dock at about
    duration / 2. The vehicle has no sensing of the dock and ideal sensors -
    it only gives the scene its scale. Every error result is relative to the
    dock, so where the scene sits changes none of them.
    """
    bias = gyroBias_degPerHour * DEG_PER_HOUR

    vehicle = DSRV("depthAutopilot", L, 50.0)       # holds the dock's depth
    xDock = 0.5 * vehicle.U0 * duration             # half-way along the track

    buoy = Buoy()
    dock = Dock()
    dock.imu_params = {"gyro_bias": [0.0, bias, 0.0]}

    N = int(round(duration / sampleTime))

    return simulate(
        N, sampleTime, vehicle, [0, 0, 50.0, 0, 0, 0], "simple",
        buoy=buoy, buoy_eta0=[xDock, 0, 0.0, 0, 0, 0],
        dock=dock, dock_eta0=[xDock, 0, L, 0, 0, 0],
    )


def dockEstimates(log):
    """
    t, freeError, cableError, dTheta = dockEstimates(log) extracts the two
    position estimates from a run and their errors against ground truth.
    """
    t = log.time[:, 0]

    etaTrue = log["dock"]["eta"]
    etaEst = log["dock"]["eta_est"]         # the free inertial solution
    pSource = log["buoy"]["eta"][:, 0:3]    # known source position

    pTrue = etaTrue[:, 0:3]

    # Free INS: position integrated from the accelerometers
    freeError = np.linalg.norm(etaEst[:, 0:3] - pTrue, axis=1)

    # Cable pseudo-measurement (3): position from attitude alone
    pCable = dockPosition(pSource, etaEst, L)
    cableError = np.linalg.norm(pCable - pTrue, axis=1)

    # Attitude error actually achieved, for the closed-form overlay
    dTheta = etaEst[:, 4] - etaTrue[:, 4]

    return t, freeError, cableError, dTheta


###############################################################################
# Figure 1 - eq (3) sensitivity, pure geometry
###############################################################################
def figureSensitivity():

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.4))

    level = np.array([0, 0, 0, 0, 0, 0], float)

    # (a) error amplification by cable length
    attitudeError = np.linspace(0, 10, 200)         # deg
    for cableLength in cableLengths:
        reference = cableVector(level, cableLength)
        error = [np.linalg.norm(
                    cableVector(np.array([0, 0, 0, a * D2R, 0, 0]), cableLength)
                    - reference)
                 for a in attitudeError]
        ax1.plot(attitudeError, error, label="L = %g m" % cableLength)

    ax1.axhline(captureRadius, color="k", linestyle=":", linewidth=1)
    ax1.annotate("capture radius %.1f m" % captureRadius,
                 xy=(0.35, captureRadius), xytext=(0, 4),
                 textcoords="offset points", fontsize=7)
    ax1.set_xlabel("dock attitude error (deg)")
    ax1.set_ylabel("dock position error (m)")
    ax1.set_title("(a) eq (3): error is amplified by cable length")
    ax1.legend()
    ax1.grid(alpha=0.3)

    # (b) which attitude angle matters, as current slants the tether
    slant = np.linspace(0, 60, 200)                 # deg, tether from vertical
    oneDegree = 1.0 * D2R

    # Roll and pitch both swing a body-z cable by L*dtheta whatever the slant,
    # so their curves coincide exactly - drawn with different styles so that
    # both remain visible.
    styles = {3: dict(linestyle="-", linewidth=3, alpha=0.4),
              4: dict(linestyle="--", linewidth=1.5),
              5: dict(linestyle="-", linewidth=1.5)}

    for axisIndex, name in ((3, "roll error"), (4, "pitch error"), (5, "yaw error")):
        error = []
        for s in slant:
            nominal = np.array([0, 0, 0, 0, s * D2R, 0], float)
            perturbed = nominal.copy()
            perturbed[axisIndex] += oneDegree
            error.append(np.linalg.norm(
                cableVector(perturbed, L) - cableVector(nominal, L)))
        ax2.plot(slant, error, label=name, **styles[axisIndex])

    ax2.annotate("roll and pitch coincide: both swing\nthe cable by "
                 r"$L\,\delta\theta$ at any slant",
                 xy=(30, L * oneDegree), xytext=(-4, -28),
                 textcoords="offset points", fontsize=7)
    ax2.annotate("yaw does nothing until\ncurrent slants the tether",
                 xy=(8, 0.02), xytext=(0, 0),
                 textcoords="offset points", fontsize=7, color="C2")

    ax2.set_ylim(bottom=-0.008)
    ax2.set_xlabel("tether slant from vertical (deg)")
    ax2.set_ylabel("dock position error (m)")
    ax2.set_title("(b) 1 deg error in each angle, L = %g m" % L)
    ax2.legend()
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    save(fig, "fig1_cableSensitivity")


###############################################################################
# Figure 2 - eq (3) in time, simulated
###############################################################################
def figureErrorGrowth(runs):

    fig, ax = plt.subplots(figsize=(5.8, 3.6))

    colours = ["C0", "C1", "C2"]

    for (bias, _), colour in zip(gyroGrades, colours):
        t, _, cableError, _ = runs[bias]
        ax.plot(t, cableError, colour, label=gradeLabel(bias))

    # Closed form 2L sin(dtheta/2), overlaid on the worst grade to show the
    # simulation reproduces the algebra
    worstBias = gyroGrades[-1][0]
    t, _, _, dTheta = runs[worstBias]
    ax.plot(t, 2 * L * np.abs(np.sin(dTheta / 2)), "k--", linewidth=1,
            label=r"closed form $2L\sin(\delta\theta/2)$")

    ax.axhline(captureRadius, color="k", linestyle=":", linewidth=1)
    ax.annotate("capture radius %.1f m" % captureRadius,
                xy=(duration * 0.62, captureRadius), xytext=(0, 4),
                textcoords="offset points", fontsize=7)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("dock position error (m)")
    ax.set_title("cable pseudo-measurement, eq (3), L = %g m" % L)
    ax.legend(loc="upper left")
    ax.grid(alpha=0.3)

    fig.tight_layout()
    save(fig, "fig2_errorGrowth")


###############################################################################
# Reporting
###############################################################################
def save(fig, name):
    """save(fig, name) writes the figure as PDF and PNG into studies/figures/"""
    os.makedirs(figureDir, exist_ok=True)
    for extension in ("pdf", "png"):
        path = os.path.join(figureDir, "%s.%s" % (name, extension))
        fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print("  wrote %s.pdf / .png" % name)


def summaryTable(runs):
    """
    summaryTable(runs) prints the numbers to quote in the proposal text.
    """
    print()
    print("Dock position error after a %.0f s approach, L = %g m"
          % (duration, L))
    print("%-30s %12s %14s %15s" % (
        "gyro grade", "att err (deg)", "cable eq (3)", "free INS (*)"))
    print("-" * 75)
    for bias, _ in gyroGrades:
        t, freeError, cableError, dTheta = (
            runs[bias][k] for k in ("t", "free", "cable", "dTheta"))
        print("%-30s %12.2f %11.3f m %13.1f m %s" % (
            gradeLabel(bias), abs(dTheta[-1]) / D2R, cableError[-1],
            freeError[-1],
            "" if cableError[-1] > captureRadius else "  < capture radius"))
    print("-" * 75)
    print("(*) free inertial solution, for reference only - not plotted.")
    print("    %.0f x the cable estimate at the MEMS grade."
          % (runs[gyroGrades[1][0]]["free"][-1]
             / runs[gyroGrades[1][0]]["cable"][-1]))


###############################################################################
# Main program
###############################################################################
def main():
    print("Running dock simulations (%.0f s each)..." % duration)

    runs = {}
    for bias, _ in gyroGrades:
        log = runDock(bias)
        t, freeError, cableError, dTheta = dockEstimates(log)
        runs[bias] = {"log": log, "t": t, "free": freeError,
                      "cable": cableError, "dTheta": dTheta}
        print("  %-30s done" % gradeLabel(bias))

    print("Writing figures...")
    figureSensitivity()
    figureErrorGrowth({b: (r["t"], r["free"], r["cable"], r["dTheta"])
                       for b, r in runs.items()})

    summaryTable(runs)


if __name__ == "__main__":
    main()
