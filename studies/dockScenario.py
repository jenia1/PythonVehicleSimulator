#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dockScenario.py:

    Watch the scenario behind cableErrorBudget.py instead of reading it off
    error curves. One run of the same scene the proposal figures use:

      - the buoy at the surface, the dock hanging L below it on the cable
      - the vehicle making a straight pass at the dock's depth
      - the dock's navigation unit carrying a gyro bias about its pitch axis

    and the dock position estimated from that navigation unit through the
    cable, eq (3): the dock's attitude + known buoy position + known cable
    length,  p_D = p_S + L C^n_b u^b.

    The true dock never moves. Everything the estimate does is error.

    The free inertial solution (the dock's accelerometers integrated twice)
    is deliberately not shown: that an unaided INS diverges is textbook, and
    not the point of the thesis. cableErrorBudget.py prints it for reference.

    Outputs, in studies/figures/:

      anim_dockScenario.gif      the scene on the left, a close-up of the
                                 buoy and dock on the right
      fig5_dockScenarioSnapshot  one frame of it, at the moment the vehicle
                                 passes the dock (an animation cannot go in
                                 a PDF)
      fig4_dockNavigation_bias<b>
                                 one per gyro grade: true vs estimated dock
                                 position and pitch, and their errors, for
                                 the axes that move

Usage:

    python3 studies/dockScenario.py

    scenarioGrade picks the gyro bias. The default, 100 deg/hr, is chosen so
    that the cable estimate moves far enough to see on the close-up; the
    headline 10 deg/hr case moves ten times less and would look static.

Author:     Jenia
"""
import os

import matplotlib
matplotlib.use("Agg")           # the animation is written to file

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Rectangle
from matplotlib.ticker import FixedLocator, FuncFormatter, MultipleLocator

from cableErrorBudget import (
    runDock, save, figureDir, gyroGrades, gradeLabel, L, sampleTime,
    captureRadius, D2R
)
from python_vehicle_simulator.navigation import dockPosition

### Scenario parameters ###
scenarioGrade = 100.0           # dock gyro bias for this scenario (deg/hr)
frameEvery = 3.0                # simulated seconds between animation frames
fps = 15                        # animation frames per second
trailStride = 10                # samples between points on the drawn trails

# Close-up window around the buoy, as multiples of the cable length L
zoomHalfWidth = 1.4
zoomAbove = 0.35                # above the surface
zoomBelow = 1.25                # below the buoy, i.e. just past the dock

# One colour per quantity, used in every figure of this script
colour = {
    "true": "k",
    "cable": "C2",
    "pitch": "C1",
    "vehicle": "C4",
    "buoy": "C3",
}


###############################################################################
# Data
###############################################################################
def scenarioData(log):
    """
    d = scenarioData(log) collects everything the figures need from one run.
    Positions are (N+1, 3) with z positive down, angles in radians.
    """
    etaEst = log["dock"]["eta_est"]
    pSource = log["buoy"]["eta"][:, 0:3]

    return {
        "t": log.time[:, 0],
        "vehicle": log["vehicle"]["eta"][:, 0:3],
        "source": pSource,
        "true": log["dock"]["eta"][:, 0:3],
        "cable": dockPosition(pSource, etaEst, L),
        "thetaTrue": log["dock"]["eta"][:, 4],
        "thetaEst": etaEst[:, 4],
    }


def depthAxis(ax, step):
    """
    depthAxis(ax, step) labels the vertical axis in depth below the surface.
    The figures plot height (-z) so that deeper is lower on the page; the
    tick labels show it as a positive depth.
    """
    ax.yaxis.set_major_locator(MultipleLocator(step))
    ax.yaxis.set_major_formatter(
        FuncFormatter(lambda v, pos: "%g" % abs(v) if v <= 0 else ""))
    ax.set_ylabel("depth (m)")


###############################################################################
# Figure 4 - navigation, true vs estimated, for the axes that move
###############################################################################
def figureDockNavigation(d, bias):
    """
    figureDockNavigation(d, bias) plots one run: the dock's true position and
    pitch against the cable estimate of eq (3), and the errors. Only x, depth
    and pitch are shown - the pitch-axis bias leaves y, roll and yaw exact.

    The position panels are scaled to the estimate's excursion, so its
    motion is visible whatever the gyro grade.
    """
    t = d["t"]
    fig, axes = plt.subplots(2, 3, figsize=(11, 6.6), sharex=True)

    # --- Row 1: what the dock's navigation unit gives, against the truth ---
    # x is shown relative to the dock, so it reads as a displacement; depth
    # is shown as it is, since 10 m below the surface is itself meaningful.
    rows = ((0, "x", "x relative to the dock (m)", d["true"][0, 0]),
            (2, "depth z", "depth z (m)", 0.0))

    for col, (k, name, ylabel, offset) in enumerate(rows):
        ax = axes[0, col]
        centre = d["true"][0, k] - offset

        # Scaled to how far the estimate goes, but never tighter than twice
        # the capture radius, so the capture band is always in view
        excursion = np.abs(d["cable"][:, k] - d["true"][:, k]).max()
        halfRange = max(2.0 * excursion, 2.0 * captureRadius)

        ax.axhspan(centre - captureRadius, centre + captureRadius,
                   color="0.85", alpha=0.7,
                   label="capture radius, +/-%.1f m" % captureRadius)
        ax.plot(t, d["true"][:, k] - offset, color=colour["true"],
                linewidth=2.5, label="true dock")
        ax.plot(t, d["cable"][:, k] - offset, color=colour["cable"],
                label="cable estimate, eq (3)")

        ax.set_ylim(centre - halfRange, centre + halfRange)
        if k == 2:
            ax.invert_yaxis()           # deeper is lower on the page

        ax.set_ylabel(ylabel)
        ax.set_title("%s: true vs estimated" % name)
        ax.grid(alpha=0.3)

    axes[0, 0].legend(loc="lower left", fontsize=7)

    ax = axes[0, 2]
    ax.plot(t, d["thetaTrue"] / D2R, color=colour["true"], linewidth=2.5,
            label="true pitch")
    ax.plot(t, d["thetaEst"] / D2R, color=colour["pitch"],
            label="estimated pitch")
    ax.set_ylabel("pitch (deg)")
    ax.set_title("pitch: the error that drives the estimate")
    ax.legend(loc="upper left")
    ax.grid(alpha=0.3)

    # --- Row 2: the errors ---
    for col, (k, name) in enumerate(((0, "x"), (2, "depth z"))):
        ax = axes[1, col]
        cableError = np.abs(d["cable"][:, k] - d["true"][:, k])
        largest = cableError.max()

        ax.semilogy(t, np.maximum(cableError, 1e-9), color=colour["cable"])
        ax.axhline(captureRadius, color="k", linestyle=":", linewidth=1)
        ax.text(t[-1], captureRadius, "capture radius ", ha="right",
                va="bottom", fontsize=7)

        # Both limits set explicitly: with a zero-bias IMU every error sits
        # far below the axis, and a free upper limit would then invert it.
        # The floor drops below 1e-3 only as far as needed to keep the error
        # in view - in depth it is second order, L(1 - cos dtheta), only
        # micrometres for the tactical grade.
        bottom = max(1e-6, min(1e-3, 0.3 * largest))
        ax.set_ylim(bottom, max(1.0, 3.0 * largest))
        if largest < 1e-6:
            ax.text(0.5, 0.5, "no error:\nestimate = truth",
                    transform=ax.transAxes, ha="center", va="center",
                    fontsize=9, color="0.4")

        ax.set_xlabel("time (s)")
        ax.set_ylabel("|%s error| (m)" % name.split()[-1])
        ax.set_title("%s error of the cable estimate (log)" % name)
        ax.grid(alpha=0.3, which="both")

    ax = axes[1, 2]
    ax.plot(t, (d["thetaEst"] - d["thetaTrue"]) / D2R, color=colour["pitch"])
    ax.set_xlabel("time (s)")
    ax.set_ylabel("pitch error (deg)")
    ax.set_title("pitch error = gyro bias x time")
    ax.grid(alpha=0.3)

    # The one-line outcome of the run, so each figure stands on its own
    cableEnd = np.linalg.norm(d["cable"][-1] - d["true"][-1])
    fig.suptitle(
        "Dock navigation - %s gyro bias about the pitch axis\n"
        "After %.0f s the cable estimate is %.3f m off, %s the %.1f m "
        "capture radius"
        % (gradeLabel(bias), t[-1], cableEnd,
           "inside" if cableEnd <= captureRadius else "OUTSIDE",
           captureRadius),
        fontsize=10)
    fig.text(0.5, 0.005,
             "Only x, depth and pitch are shown: a pitch-axis bias leaves y, "
             "roll and yaw exact. The vehicle and buoy have ideal IMUs in "
             "this scenario, so their estimates equal the truth.",
             ha="center", va="bottom", fontsize=7, color="0.4")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    save(fig, "fig4_dockNavigation_bias%g" % bias)


###############################################################################
# Animation - the scene and a close-up of the dock
###############################################################################
def animateScenario(d, bias):

    t = d["t"]
    vehicle, true, cable = d["vehicle"], d["true"], d["cable"]

    xS, zS = d["source"][0, 0], d["source"][0, 2]       # buoy, fixed
    xD, zD = true[0, 0], true[0, 2]                     # true dock, fixed

    # Closest approach of the vehicle to the dock
    vehicleDistance = np.linalg.norm(vehicle - true, axis=1)
    kPass = int(vehicleDistance.argmin())

    # Trails are drawn from every trailStride-th sample, not every sample
    trailIndex = np.arange(0, len(t), trailStride)

    fig = plt.figure(figsize=(12.5, 6.0))
    grid = fig.add_gridspec(1, 2, width_ratios=[1.35, 1], wspace=0.2,
                            left=0.06, right=0.98, top=0.83, bottom=0.25)
    axScene = fig.add_subplot(grid[0])
    axZoom = fig.add_subplot(grid[1])

    # Close-up window, in plot coordinates (x, height = -z)
    zoomX = (xS - zoomHalfWidth * L, xS + zoomHalfWidth * L)
    zoomY = (-zS - zoomBelow * L, -zS + zoomAbove * L)

    # --- (a) the scene ---
    xEnd = vehicle[:, 0].max()
    depthEnd = max(vehicle[:, 2].max(), zD)
    axScene.set_xlim(-0.03 * xEnd, 1.03 * xEnd)
    axScene.set_ylim(-(depthEnd + 8), 6)
    depthAxis(axScene, 10)

    axScene.axhline(0, color="grey", linestyle="--", linewidth=1)
    axScene.text(0.01 * xEnd, 1, "surface", fontsize=7, color="grey")
    axScene.plot(xS, -zS, "o", color=colour["buoy"], markersize=8, zorder=5)
    axScene.plot([xS, xD], [-zS, -zD], color=colour["true"], linewidth=1)
    axScene.plot(xD, -zD, "s", color=colour["true"], markersize=7, zorder=5)
    axScene.add_patch(Rectangle((zoomX[0], zoomY[0]),
                                zoomX[1] - zoomX[0], zoomY[1] - zoomY[0],
                                fill=False, edgecolor="grey", linewidth=1))
    axScene.annotate("close-up (b)", xy=(zoomX[1], zoomY[0]),
                     xytext=(8, -14), textcoords="offset points",
                     fontsize=7, color="grey")

    # The vehicle is drawn beneath the buoy and dock markers, so that it does
    # not hide the dock at the moment it passes it
    vehicleTrail, = axScene.plot([], [], color=colour["vehicle"], linewidth=1.5)
    vehicleMark, = axScene.plot([], [], ">", color=colour["vehicle"],
                                markersize=9, zorder=4)
    sceneText = axScene.text(
        0.99, 0.04, "", transform=axScene.transAxes, ha="right", va="bottom",
        fontsize=8, family="monospace",
        bbox=dict(facecolor="white", edgecolor="grey", alpha=0.9))

    axScene.set_xlabel("x, distance along the track (m)")
    axScene.set_title(
        "(a) the scene: the vehicle holds the dock's depth and passes it at "
        "t = %.0f s\n(it has no sensing of the dock yet; it gives the scale)"
        % t[kPass], fontsize=9)

    # --- (b) the close-up ---
    axZoom.set_aspect("equal")
    axZoom.set_xlim(*zoomX)
    axZoom.set_ylim(*zoomY)
    depthAxis(axZoom, 5)

    # Ticks at whole multiples of 5 m from the buoy, rather than wherever the
    # absolute track position happens to put them
    axZoom.xaxis.set_major_locator(
        FixedLocator(xS + np.arange(-15, 16, 5)))
    axZoom.xaxis.set_major_formatter(
        FuncFormatter(lambda v, pos: "%+.0f" % (v - xS)
                      if abs(v - xS) > 0.5 else "0"))
    axZoom.set_xlabel("x relative to the buoy (m)")

    axZoom.axhline(0, color="grey", linestyle="--", linewidth=1)

    # Eq (4): the dock can only be within L of the buoy. Mathematically a
    # sphere, but shown only below the surface, where a dock can hang.
    region = Circle((xS, -zS), L, facecolor="0.85", alpha=0.45,
                    edgecolor="0.5", linestyle="--", linewidth=1)
    axZoom.add_patch(region)
    region.set_clip_path(Rectangle((zoomX[0], zoomY[0]),
                                   zoomX[1] - zoomX[0], 0 - zoomY[0],
                                   transform=axZoom.transData))
    axZoom.text(xS - 0.72 * L, -zS - 0.35 * L, "reachable\nregion, eq (4)",
                fontsize=7, color="0.35", ha="center")

    axZoom.plot([xS, xD], [-zS, -zD], color=colour["true"], linewidth=2)
    axZoom.plot(xS, -zS, "o", color=colour["buoy"], markersize=9, zorder=5)
    axZoom.plot(xD, -zD, "s", color=colour["true"], markersize=8, zorder=5)
    axZoom.add_patch(Circle((xD, -zD), captureRadius, fill=False,
                            edgecolor="k", linestyle=":", linewidth=1.2))

    cableTrail, = axZoom.plot([], [], color=colour["cable"], linewidth=1,
                              alpha=0.6)
    cableLine, = axZoom.plot([], [], "--", color=colour["cable"],
                             linewidth=1.5)
    cableMark, = axZoom.plot([], [], "o", color=colour["cable"],
                             markersize=8, zorder=6)
    vehicleZoom, = axZoom.plot([], [], ">", color=colour["vehicle"],
                               markersize=10, alpha=0.75, zorder=4)
    zoomText = axZoom.text(
        0.02, 0.98, "", transform=axZoom.transAxes, ha="left", va="top",
        fontsize=8, family="monospace",
        bbox=dict(facecolor="white", edgecolor="grey", alpha=0.9))

    axZoom.set_title("(b) close-up at the buoy and dock", fontsize=9)

    # --- one legend for both panels ---
    handles = [
        Line2D([], [], marker="o", color=colour["buoy"], linestyle="None",
               markersize=8, label="buoy: source, position known"),
        Line2D([], [], marker="s", color=colour["true"], linewidth=2,
               markersize=7, label="true dock and cable (never moves)"),
        Line2D([], [], marker="o", color=colour["cable"], linestyle="--",
               markersize=7,
               label=r"dock estimate from cable, eq (3): $p_S + L\,C^n_b u^b$"),
        Patch(facecolor="0.85", edgecolor="0.5", linestyle="--",
              label=r"reachable region, eq (4): $\|p_D - p_S\| \leq L$"),
        Line2D([], [], color="k", linestyle=":",
               label="capture radius %.1f m" % captureRadius),
        Line2D([], [], marker=">", color=colour["vehicle"], markersize=8,
               label="vehicle"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=3, fontsize=8,
               frameon=False, bbox_to_anchor=(0.5, 0.01))

    fig.suptitle(
        "Where is the dock? Its navigation unit has a %s gyro bias.\n"
        "Black is the truth. Green is the dock position estimated from that "
        "unit's attitude through the cable, eq (3)." % gradeLabel(bias),
        fontsize=11)

    def draw(k):
        """draw(k) puts every moving element at sample k"""
        trail = trailIndex[trailIndex <= k]

        # (a) scene
        vehicleTrail.set_data(vehicle[trail, 0], -vehicle[trail, 2])
        vehicleMark.set_data([vehicle[k, 0]], [-vehicle[k, 2]])
        sceneText.set_text("t = %5.0f s\nvehicle to dock: %7.1f m"
                           % (t[k], vehicleDistance[k]))

        # (b) cable estimate: always on the circle of radius L, eq (3)/(4)
        cableTrail.set_data(cable[trail, 0], -cable[trail, 2])
        cableLine.set_data([xS, cable[k, 0]], [-zS, -cable[k, 2]])
        cableMark.set_data([cable[k, 0]], [-cable[k, 2]])

        vehicleZoom.set_data([vehicle[k, 0]], [-vehicle[k, 2]])

        cableError = np.linalg.norm(cable[k] - true[k])
        pitchError = (d["thetaEst"][k] - d["thetaTrue"][k]) / D2R
        zoomText.set_text(
            "pitch error      %6.2f deg\n"
            "cable estimate   %6.3f m  %s"
            % (pitchError, cableError,
               "inside capture" if cableError <= captureRadius
               else "OUTSIDE capture"))

        return (vehicleTrail, vehicleMark, sceneText, cableTrail, cableLine,
                cableMark, vehicleZoom, zoomText)

    step = int(round(frameEvery / sampleTime))
    frames = list(range(0, len(t), step))
    if frames[-1] != len(t) - 1:
        frames.append(len(t) - 1)

    ani = animation.FuncAnimation(fig, lambda n: draw(frames[n]),
                                  frames=len(frames), blit=False)

    os.makedirs(figureDir, exist_ok=True)
    path = os.path.join(figureDir, "anim_dockScenario.gif")
    ani.save(path, writer=animation.PillowWriter(fps=fps), dpi=80)
    print("  wrote anim_dockScenario.gif (%d frames, %.0f s)"
          % (len(frames), len(frames) / fps))

    # One still frame, at the moment the vehicle passes the dock
    draw(kPass)
    save(fig, "fig5_dockScenarioSnapshot")


###############################################################################
# Main program
###############################################################################
def main():
    # One navigation figure per gyro grade in cableErrorBudget.gyroGrades.
    # Titles come from gradeLabel(), which builds them from the bias itself.
    runs = {}
    for bias, _ in gyroGrades:
        print("Running the dock scenario, %s..." % gradeLabel(bias))
        runs[bias] = scenarioData(runDock(bias))
        figureDockNavigation(runs[bias], bias)

    print("Animating the %g deg/hr run..." % scenarioGrade)
    if scenarioGrade not in runs:
        runs[scenarioGrade] = scenarioData(runDock(scenarioGrade))
    animateScenario(runs[scenarioGrade], scenarioGrade)


if __name__ == "__main__":
    main()
