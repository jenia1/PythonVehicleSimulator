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

    Two ways of getting that attitude are compared, for three IMU grades:

      unaided   the gyro alone. The error grows with time, as b t.
      levelled  a complementary filter (StrapdownINS) levels roll and pitch
                against the accelerometer. The error is bounded, at about
                b tau + b_a / g, so the accelerometer then sets its size.

    Both start from the dock's true attitude: the alignment before the run
    is assumed perfect, since alignment is not the subject here.

    For reference only, the summary table also prints the free inertial
    solution - the dock's accelerometers integrated twice. It grows as t^3,
    because an attitude error makes the INS subtract gravity along the wrong
    direction and the leftover is integrated twice. It is deliberately not
    plotted: that an unaided INS diverges is textbook [Farrell, Groves], and
    not the point of the thesis.

    Figures written to studies/figures/ as PDF for inclusion in the proposal:

      fig1_cableSensitivity   eq (3): error amplification by cable length,
                              and which attitude angle actually matters
      fig2_errorGrowth        eq (3) in time, per IMU grade, unaided and
                              levelled, against the closed form

Usage:

    python3 studies/cableErrorBudget.py

Assumptions, all stated in the proposal text:
    - taut, straight cable, eta = 0 in (3): no sag, no drag
    - the dock starts with its true attitude: alignment before the run is
      assumed perfect
    - the dock hangs still, so its accelerometer measures gravity alone -
      which the levelling relies on
    - the dock's IMU errors are constant biases from Groves' table of
      typical values: a gyro bias about the pitch axis and an accelerometer
      bias along x, signed so that their effects add (the worst case).
      No noise.
    - the source position p_S is taken as known exactly

Author:     Jenia
"""
import os

import matplotlib
matplotlib.use("Agg")           # figures are written to file, never shown

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

from python_vehicle_simulator.vehicles import DSRV
from python_vehicle_simulator.structures import Buoy, Dock
from python_vehicle_simulator.lib import simulate
from python_vehicle_simulator.navigation import cableVector, dockPosition

### Study parameters ###
L = 10.0                        # cable length (m)
duration = 300.0                # terminal approach duration (s)
sampleTime = 0.02               # sample time (s)
captureRadius = 0.5             # magnetic latch capture radius (m)

complementaryTau = 1.0          # dock's complementary filter time constant (s)

# IMU grades, from Groves' table of typical biases: (gyro bias in deg/hr,
# accelerometer bias in mg, grade name). Each value is the middle of the
# table's range on a log scale. The consumer row gives only lower bounds
# (>100 deg/hr, >3 mg), so its values are one decade past them - an
# assumption, not a table value. Figures and tables build their labels from
# these through gradeLabel(), so the numbers shown are always the numbers
# simulated: change a grade here and nothing else.
imuGrades = [
    (0.1,    0.3,  "intermediate"),
    (10.0,   3.0,  "tactical"),
    (1000.0, 30.0, "consumer"),
]

# One colour per grade, in the order of imuGrades, used in every figure
gradeColours = ["C0", "C2", "C1"]


def gradeLabel(grade):
    """
    label = gradeLabel(grade) is the display label for an entry of imuGrades,
    e.g. "tactical, 10 deg/hr, 3 mg", built from its numbers.
    """
    gyroBias, accelBias, name = grade
    return "%s, %g deg/hr, %g mg" % (name, gyroBias, accelBias)

# Cable lengths for the sensitivity plot (m)
cableLengths = [5.0, 10.0, 20.0, 50.0]

D2R = np.pi / 180
DEG_PER_HOUR = D2R / 3600       # deg/hr -> rad/s
MILLI_G = 9.81e-3               # mg -> m/s^2, with g = 9.81 as in IMU.g

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
def runDock(grade, levelled):
    """
    log = runDock(grade, levelled) runs the simulation with the dock's
    navigation unit carrying the biases of one entry of imuGrades: a gyro
    bias about its y (pitch) axis and an accelerometer bias along its x axis.
    Both tilt the pitch estimate the same way, so the cable swings in the
    x direction and their effects add.

    The dock's INS starts from the true attitude (alignment assumed
    perfect). levelled: False leaves the attitude to the gyro alone, True
    adds the complementary filter.

    Single-axis biases are used so that the pitch error has an exact closed
    form (pitchErrorClosedForm), which makes the simulated result directly
    comparable with it and validates both.

    The scene: the buoy and dock sit half-way along the vehicle's track, and
    the vehicle holds the dock's depth, so it passes the dock at about
    duration / 2. The vehicle has no sensing of the dock and ideal sensors -
    it only gives the scene its scale. Every error result is relative to the
    dock, so where the scene sits changes none of them.
    """
    gyroBias, accelBias, _ = grade

    vehicle = DSRV("depthAutopilot", L, 50.0)       # holds the dock's depth
    xDock = 0.5 * vehicle.U0 * duration             # half-way along the track

    buoy = Buoy()
    dock = Dock()
    dock.imu_params = {"gyro_bias": [0.0, gyroBias * DEG_PER_HOUR, 0.0],
                       "accel_bias": [accelBias * MILLI_G, 0.0, 0.0]}
    if levelled:
        dock.nav_params = {"complementary_tau": complementaryTau}

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


def pitchErrorClosedForm(grade, levelled, t):
    """
    dTheta = pitchErrorClosedForm(grade, levelled, t) is the dock's pitch
    error (rad) at times t, worked out from the grade's biases alone. It
    starts at zero, from the true attitude. Then, with alpha = 1 - dt/tau and
    n = t / dt samples:

        unaided   b t
        levelled  (atan(b_a / g) + b (tau - dt)) (1 - alpha^n)

    Unaided, the accelerometer bias never enters the attitude. Levelled, the
    error rises within a few tau to a floor the accelerometer mostly sets.
    Both are exact for the discrete filter, not approximations - see
    tests/test_levelling.py.
    """
    gyroBias, accelBias, _ = grade
    b = gyroBias * DEG_PER_HOUR

    if not levelled:
        return b * t

    alpha = 1.0 - sampleTime / complementaryTau
    n = np.round(t / sampleTime)
    floor = np.arctan(accelBias * MILLI_G / 9.81) \
        + b * (complementaryTau - sampleTime)
    return floor * (1 - alpha ** n)


def cableErrorClosedForm(dTheta):
    """
    error = cableErrorClosedForm(dTheta) is the cable estimate's position
    error for a pitch error dTheta: the chord 2L sin(dTheta/2).
    """
    return 2 * L * np.abs(np.sin(dTheta / 2))


###############################################################################
# Figure 1 - eq (3) sensitivity, pure geometry
###############################################################################
def figureSensitivity():
    """
    figureSensitivity() draws the geometry of eq (3): how a dock attitude
    error becomes a dock position error, for several cable lengths, and the
    attitude error each length can tolerate before the estimate leaves the
    capture radius.

    Pure geometry, no simulation. The perturbation is applied in pitch, the
    axis the study uses; roll gives exactly the same curve, since either
    tilts a body-z cable by the same angle.

    A second panel, showing that roll and pitch cost L*dtheta alike and that
    yaw costs nothing at a vertical tether, was dropped (Jenia): a constant
    and a sine are better stated in a sentence than plotted.
    """
    fig, ax = plt.subplots(figsize=(5.6, 3.9))

    level = np.array([0, 0, 0, 0, 0, 0], float)
    attitudeError = np.linspace(0, 10, 200)         # deg

    for cableLength in cableLengths:
        reference = cableVector(level, cableLength)
        error = [np.linalg.norm(
                    cableVector(np.array([0, 0, 0, 0, a * D2R, 0]), cableLength)
                    - reference)
                 for a in attitudeError]

        # The attitude error this length tolerates, from 2L sin(eps/2) = r
        tolerated = 2 * np.arcsin(captureRadius / (2 * cableLength)) / D2R

        line, = ax.plot(attitudeError, error,
                        label="L = %g m  (%.1f deg)" % (cableLength, tolerated))
        ax.plot(tolerated, captureRadius, "o", color=line.get_color(),
                markersize=5, zorder=4)

    ax.axhline(captureRadius, color="k", linestyle=":", linewidth=1)
    # Label at the right, clear of the markers on the left
    ax.annotate("capture radius %.1f m" % captureRadius,
                xy=(attitudeError[-1], captureRadius), xytext=(-2, 4),
                textcoords="offset points", fontsize=7, ha="right")
    ax.set_xlim(0, attitudeError[-1])
    ax.set_ylim(bottom=0)
    ax.set_xlabel("dock attitude error (deg)")
    ax.set_ylabel("dock position error (m)")
    ax.set_title("eq (3): error is amplified by cable length")
    ax.legend(title="cable length (tolerable error)", fontsize=8,
              title_fontsize=8)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    save(fig, "fig1_cableSensitivity")


###############################################################################
# Figure 2 - eq (3) in time, simulated
###############################################################################
def figureErrorGrowth(runs):
    """
    figureErrorGrowth(runs) plots the cable estimate's error over time for
    every grade, unaided (dashed) and levelled (solid), with the closed form
    marked on each at intervals (markers rather than a line, so the curves'
    own styles stay readable).

    The axis spans twice the capture radius, the scale that matters for
    docking. A curve that leaves it is marked where it does, with its value
    at the end of the run.
    """
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    top = 2.0 * captureRadius
    markEvery = int(round(20.0 / sampleTime))      # a closed-form mark each 20 s

    ax.axhspan(0, captureRadius, color="0.85", alpha=0.7)
    ax.text(duration * 0.99, captureRadius, "capture radius %.1f m "
            % captureRadius, ha="right", va="bottom", fontsize=7)

    for grade, colour in zip(imuGrades, gradeColours):
        for levelled, style in ((False, "--"), (True, "-")):
            run = runs[grade[2]]["levelled" if levelled else "unaided"]
            t, cableError = run["t"], run["cable"]
            ax.plot(t, cableError, color=colour, linestyle=style)

            marks = slice(0, None, markEvery)
            ax.plot(t[marks], cableErrorClosedForm(
                        pitchErrorClosedForm(grade, levelled, t))[marks],
                    "o", color="k", markersize=3, markerfacecolor="none",
                    markeredgewidth=0.7)

            if cableError.max() > top:
                kOut = int(np.argmax(cableError > top))
                ax.annotate("%s, unaided:\n%.1f m at %.0f s"
                            % (grade[2], cableError[-1], t[-1]),
                            xy=(t[kOut], top), xytext=(14, -22),
                            textcoords="offset points", fontsize=7,
                            color=colour,
                            arrowprops=dict(arrowstyle="->", color=colour))

    # A little below zero, so that errors of a few millimetres are not hidden
    # under the axis line
    ax.set_ylim(-0.02 * top, top)
    ax.set_xlim(0, duration)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("dock position error (m)")
    ax.set_title("cable pseudo-measurement, eq (3), L = %g m" % L)
    ax.grid(alpha=0.3)

    handles = [Line2D([], [], color=colour, label=gradeLabel(grade))
               for grade, colour in zip(imuGrades, gradeColours)]
    handles += [
        Line2D([], [], color="k", linestyle="--", label="unaided (gyro alone)"),
        Line2D([], [], color="k",
               label=r"levelled, complementary filter, $\tau$ = %g s"
               % complementaryTau),
        Line2D([], [], marker="o", color="k", linestyle="None", markersize=3,
               markerfacecolor="none", markeredgewidth=0.7,
               label=r"closed form, $2L\sin(\delta\theta/2)$"),
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=7)

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
        # No creation date in the PDF, so re-running a script that changes
        # nothing leaves the file byte-identical and out of git status
        metadata = {"CreationDate": None} if extension == "pdf" else None
        fig.savefig(path, bbox_inches="tight", metadata=metadata)
    plt.close(fig)
    print("  wrote %s.pdf / .png" % name)


def summaryTable(runs):
    """
    summaryTable(runs) prints the numbers to quote in the proposal text, and
    how far the simulation is from the closed form.
    """
    print()
    print("Dock position error after a %.0f s approach, L = %g m, "
          "cable estimate eq (3)" % (duration, L))
    print("%-34s %19s %21s %19s" % (
        "", "pitch error (deg)", "cable eq (3) (mm)", "free INS (*) (m)"))
    print("%-34s %9s %9s %10s %10s %9s %9s" % (
        "IMU grade", "unaided", "levelled", "unaided", "levelled",
        "unaided", "levelled"))
    print("-" * 97)
    worstGap = 0.0
    for grade in imuGrades:
        u, v = runs[grade[2]]["unaided"], runs[grade[2]]["levelled"]
        print("%-34s %9.3f %9.3f %10.1f %10.1f %9.1f %9.1f" % (
            gradeLabel(grade),
            abs(u["dTheta"][-1]) / D2R, abs(v["dTheta"][-1]) / D2R,
            u["cable"][-1] * 1e3, v["cable"][-1] * 1e3,
            u["free"][-1], v["free"][-1]))
        for levelled, run in ((False, u), (True, v)):
            closed = cableErrorClosedForm(
                pitchErrorClosedForm(grade, levelled, run["t"]))
            worstGap = max(worstGap, np.abs(run["cable"] - closed).max())
    print("-" * 97)
    print("capture radius %.0f mm. Largest gap between simulation and closed "
          "form, over every run and sample: %.2g m."
          % (captureRadius * 1e3, worstGap))
    print("(*) free inertial solution, for reference only - not plotted.")


###############################################################################
# Main program
###############################################################################
def runAll():
    """
    runs = runAll() simulates every grade, unaided and levelled:
    runs[name]["unaided" | "levelled"] = {"log", "t", "free", "cable",
    "dTheta"}.
    """
    runs = {}
    for grade in imuGrades:
        runs[grade[2]] = {}
        for mode in ("unaided", "levelled"):
            log = runDock(grade, levelled=(mode == "levelled"))
            t, freeError, cableError, dTheta = dockEstimates(log)
            runs[grade[2]][mode] = {"log": log, "t": t, "free": freeError,
                                    "cable": cableError, "dTheta": dTheta}
            print("  %-34s %-9s done" % (gradeLabel(grade), mode))
    return runs


def main():
    print("Running dock simulations (%.0f s each)..." % duration)
    runs = runAll()

    print("Writing figures...")
    figureSensitivity()
    figureErrorGrowth(runs)

    summaryTable(runs)


if __name__ == "__main__":
    main()
