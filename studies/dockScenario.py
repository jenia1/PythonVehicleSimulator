#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dockScenario.py:

    Watch the scenario behind cableErrorBudget.py instead of reading it off
    error curves. The same scene the proposal figures use:

      - the buoy at the surface, the dock hanging L below it on the cable
      - the vehicle making a straight pass at the dock's depth
      - the dock's navigation unit carrying the biases of an IMU grade, for
        each grade in cableErrorBudget.imuGrades

    and the dock position estimated from that navigation unit through the
    cable, eq (3): the dock's attitude + known buoy position + known cable
    length,  p_D = p_S + L C^n_b u^b. Each grade is run twice: with the
    attitude from the gyro alone (unaided), and levelled by the
    complementary filter.

    The true dock never moves. Everything the estimates do is error.

    The free inertial solution (the dock's accelerometers integrated twice)
    is deliberately not shown: that an unaided INS diverges is textbook, and
    not the point of the thesis. cableErrorBudget.py prints it for reference.

    Outputs, in studies/figures/:

      fig4_dockScene         (a) the scene and (b) a close-up of the buoy
                             and dock, at the moment the vehicle passes it
      fig5_dockNavigation    (a)-(c) the dock's true vs estimated x, depth
                             and pitch over the whole run
      anim_dockScenario.gif  the scene figure in motion

    The two were one figure until Jenia's supervisor asked for them apart.

Usage:

    python3 studies/dockScenario.py

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
    runAll, save, figureDir, imuGrades, gradeColours, gradeLabel, L,
    sampleTime, captureRadius, complementaryTau, D2R
)
from python_vehicle_simulator.navigation import dockPosition

### Scenario parameters ###
frameEvery = 3.0                # simulated seconds between animation frames
fps = 15                        # animation frames per second
trailStride = 10                # samples between points on the drawn trails

# Close-up window around the buoy, as multiples of the cable length L
zoomHalfWidth = 1.4
zoomAbove = 0.35                # above the surface
zoomBelow = 1.25                # below the buoy, i.e. just past the dock

# One colour per body. Each dock estimate takes its grade's colour from
# cableErrorBudget.gradeColours.
colour = {
    "true": "k",
    "vehicle": "C4",
    "buoy": "C3",
}

# The two attitude sources, and the line style each is drawn with. Unaided
# estimates also get hollow markers, levelled ones filled.
modes = ("unaided", "levelled")
lineStyle = {"unaided": "--", "levelled": "-"}

# The pitch error at which the cable estimate reaches the capture radius,
# from 2L sin(dtheta/2) = captureRadius. It is the pitch panel's shaded band.
capturePitch = 2 * np.arcsin(captureRadius / (2 * L))

# These figures are drawn at 12.5 in and printed at about 6.5 in: the
# proposal includes them wider than its text block, since article's default
# margins would otherwise hold them to 4.8 in. The page therefore shows them
# at 52%, and 15 pt here arrives as about 8 pt, the size of the caption
# beside it (Jenia's supervisor, 2026-10-19: "text in this figure is too
# small. increase font size and reduce the amount of text").
#
# 20 pt was tried, which prints at 10.5 pt, larger than the body text. It
# left the plots cramped and the box of numbers nearly touching the axes,
# so the type came back down and the panels returned to the proportions of
# the version in the proposal's V4 (Jenia, 2026-10-19).
plotStyle = {
    "font.size": 15,
    "axes.titlesize": 15,
    "axes.labelsize": 15,
    "legend.fontsize": 14,
    "xtick.labelsize": 14,
    "ytick.labelsize": 14,
    "lines.linewidth": 2.0,
}
insideSize = 12                 # labels drawn inside a panel

# This script's figures only: cableErrorBudget.py keeps its own smaller style
# for fig 1 and fig 2, which are printed at a smaller size.
plt.rcParams.update(plotStyle)


###############################################################################
# Data
###############################################################################
def scenarioData(log):
    """
    d = scenarioData(log) collects everything the figure needs from one run.
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
# Pieces both figures use
###############################################################################
def commonData(data):
    """
    c = commonData(data) collects what both figures need from
    data[grade name][mode]: the time vector, the vehicle track, the fixed
    buoy and dock positions, the vehicle's closest pass, and every run
    paired with the colour and style it is drawn in.

    The vehicle, the buoy and the true dock are identical in every run -
    only the dock's navigation unit differs - so they come from any one.
    """
    first = data[imuGrades[0][2]][modes[0]]
    vehicle, true = first["vehicle"], first["true"]
    distance = np.linalg.norm(vehicle - true, axis=1)

    return {
        "first": first, "t": first["t"], "vehicle": vehicle, "true": true,
        "xS": first["source"][0, 0], "zS": first["source"][0, 2],
        "xD": true[0, 0], "zD": true[0, 2],
        "distance": distance, "kPass": int(distance.argmin()),
        "runs": [(grade, gradeColour, mode, data[grade[2]][mode])
                 for grade, gradeColour in zip(imuGrades, gradeColours)
                 for mode in modes],
    }


def gradeHandles():
    """one legend entry per IMU grade, labelled by its numbers alone"""
    return [Line2D([], [], color=gradeColour, linewidth=3,
                   label=gradeLabel(grade))
            for grade, gradeColour in zip(imuGrades, gradeColours)]


def modeHandles():
    """
    the two attitude sources, in grey: the colour belongs to the grade, the
    line style and the marker to the mode
    """
    return [
        Line2D([], [], color="0.3", linestyle="--", marker="o", markersize=9,
               markerfacecolor="none", label="unaided"),
        Line2D([], [], color="0.3", marker="o", markersize=7,
               label=r"levelled, $\tau$ = %g s" % complementaryTau),
    ]


def figureTitle(fig):
    """
    the title, the same on both figures. The explanatory note that used to
    sit under them is gone: at the size these are printed it was unreadable,
    and the proposal's text and captions say the same thing at full size.
    """
    fig.suptitle("Dock position estimated through the cable, eq (3) "
                 "for three IMU grades", fontsize=17)


###############################################################################
# Figure 4 - the scene and the close-up, the figure the animation draws
###############################################################################
def sceneFigure(data):
    """
    fig, draw, kPass = sceneFigure(data) lays out (a) the scene and (b) the
    close-up at the buoy and dock.

    draw(k) puts the vehicle and every dock estimate at sample k, so the
    same figure serves as the still (drawn at the vehicle's closest pass,
    kPass) and as the animation.
    """
    c = commonData(data)
    t, vehicle, true, runs = c["t"], c["vehicle"], c["true"], c["runs"]
    xS, zS, xD, zD = c["xS"], c["zS"], c["xD"], c["zD"]
    kPass = c["kPass"]

    # Trails are drawn from every trailStride-th sample, not every sample
    trailIndex = np.arange(0, len(t), trailStride)

    fig = plt.figure(figsize=(12.5, 5.6))
    # The close-up gets the smaller share, but not a third: it is square-ish
    grid = fig.add_gridspec(1, 2, width_ratios=[1.55, 1], wspace=0.18,
                            left=0.06, right=0.98, top=0.87, bottom=0.21)
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
    axScene.text(0.01 * xEnd, 1.5, "surface", fontsize=insideSize, color="grey")
    axScene.plot(xS, -zS, "o", color=colour["buoy"], markersize=8, zorder=5)
    axScene.plot([xS, xD], [-zS, -zD], color=colour["true"], linewidth=1)
    axScene.plot(xD, -zD, "s", color=colour["true"], markersize=7, zorder=5)
    axScene.add_patch(Rectangle((zoomX[0], zoomY[0]),
                                zoomX[1] - zoomX[0], zoomY[1] - zoomY[0],
                                fill=False, edgecolor="grey", linewidth=1))
    # Beside the close-up rectangle, clear of the surface line above it and
    # of the box of numbers below
    axScene.annotate("close-up (b)", xy=(zoomX[0], -0.5 * (zS + zD)),
                     xytext=(-10, 0), textcoords="offset points",
                     ha="right", va="center", fontsize=insideSize,
                     color="grey")

    # The vehicle is drawn beneath the buoy and dock markers, so that it does
    # not hide the dock at the moment it passes it
    vehicleTrail, = axScene.plot([], [], color=colour["vehicle"], linewidth=1.5)
    vehicleMark, = axScene.plot([], [], ">", color=colour["vehicle"],
                                markersize=9, zorder=4)

    # The numbers at sample k, in the empty water below the vehicle's track.
    # It is a legend rather than a plain text box, so the coloured line beside
    # each row says which grade it is: the figure then needs no second place
    # to name the colours. The first row is the column header, carried by an
    # invisible handle so that it lines up with the rows under it.
    blank = Line2D([], [], linestyle="None")
    numberLegend = axScene.legend(
        handles=[blank] + gradeHandles(), labels=[""] * (1 + len(imuGrades)),
        loc="lower right", handlelength=1.6, labelspacing=0.3, borderpad=0.6,
        borderaxespad=0.9, prop={"family": "monospace", "size": 13})
    numberLegend.get_title().set_fontfamily("monospace")
    numberLegend.get_title().set_fontsize(13)
    numberLegend._legend_box.align = "left"

    axScene.set_xlabel("x, distance along the track (m)")
    axScene.set_title("(a) the scene: the AUV passes the dock at t = %.0f s"
                      % t[kPass], fontsize=14)

    # --- (b) the close-up ---
    # Equal aspect shrinks the axes inside its cell; anchoring it north keeps
    # its top edge level with the scene panel's
    axZoom.set_aspect("equal")
    axZoom.set_anchor("N")
    axZoom.set_xlim(*zoomX)
    axZoom.set_ylim(*zoomY)
    depthAxis(axZoom, 5)

    # Ticks at whole multiples of 5 m from the source, rather than wherever
    # the absolute track position happens to put them
    axZoom.xaxis.set_major_locator(
        FixedLocator(xS + np.arange(-10, 11, 10)))
    axZoom.xaxis.set_major_formatter(
        FuncFormatter(lambda v, pos: "%+.0f" % (v - xS)
                      if abs(v - xS) > 0.5 else "0"))
    axZoom.set_xlabel("x relative to the source (m)")

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
                fontsize=insideSize, color="0.35", ha="center")

    axZoom.plot([xS, xD], [-zS, -zD], color=colour["true"], linewidth=2)
    axZoom.plot(xS, -zS, "o", color=colour["buoy"], markersize=9, zorder=5)
    axZoom.plot(xD, -zD, "s", color=colour["true"], markersize=8, zorder=5)
    axZoom.add_patch(Circle((xD, -zD), captureRadius, fill=False,
                            edgecolor="k", linestyle=":", linewidth=1.2))

    # Each estimate: its trail, its cable from the buoy, and its marker.
    # Unaided markers are hollow and larger, so a levelled one on the same
    # spot still shows inside it.
    zoomArtists = []
    for grade, gradeColour, mode, d in runs:
        trailLine, = axZoom.plot([], [], color=gradeColour, linewidth=1,
                                 alpha=0.6)
        cableLine, = axZoom.plot([], [], color=gradeColour, linewidth=1.2,
                                 linestyle=lineStyle[mode])
        mark, = axZoom.plot(
            [], [], "o", color=gradeColour, zorder=6,
            markersize=10 if mode == "unaided" else 6,
            markerfacecolor="none" if mode == "unaided" else gradeColour,
            markeredgewidth=1.5)
        zoomArtists.append((trailLine, cableLine, mark))

    vehicleZoom, = axZoom.plot([], [], ">", color=colour["vehicle"],
                               markersize=10, alpha=0.75, zorder=4)
    axZoom.set_title("(b) close-up at the source and dock", fontsize=14)

    handles = [
        Line2D([], [], marker="o", color=colour["buoy"], linestyle="None",
               markersize=10, label="source"),
        Line2D([], [], marker="s", color=colour["true"], linewidth=2,
               markersize=9, label="true dock"),
        Line2D([], [], marker=">", color=colour["vehicle"], markersize=10,
               label="vehicle"),
    ] + modeHandles()
    fig.legend(handles=handles, loc="lower center", ncol=5,
               frameon=False, bbox_to_anchor=(0.5, 0.0))
    figureTitle(fig)

    def draw(k):
        """draw(k) puts every moving element at sample k"""
        trail = trailIndex[trailIndex <= k]

        # (a) the scene
        vehicleTrail.set_data(vehicle[trail, 0], -vehicle[trail, 2])
        vehicleMark.set_data([vehicle[k, 0]], [-vehicle[k, 2]])

        # (b) each estimate is always on the circle of radius L, eq (3)/(4)
        for (grade, gradeColour, mode, d), (trailLine, cableLine, mark) in \
                zip(runs, zoomArtists):
            est = d["cable"]
            trailLine.set_data(est[trail, 0], -est[trail, 2])
            cableLine.set_data([xS, est[k, 0]], [-zS, -est[k, 2]])
            mark.set_data([est[k, 0]], [-est[k, 2]])
        vehicleZoom.set_data([vehicle[k, 0]], [-vehicle[k, 2]])

        # The numbers, in the legend inside the scene panel
        numberLegend.set_title("t = %3.0f s, error (m)" % t[k])
        texts = numberLegend.get_texts()
        texts[0].set_text("%-12s %8s %8s" % ("", "unaided", "levelled"))
        for text, grade in zip(texts[1:], imuGrades):
            cells = []
            for mode in modes:
                error = np.linalg.norm(data[grade[2]][mode]["cable"][k]
                                       - true[k])
                cells.append("%7.3f%s" % (error,
                                          "*" if error > captureRadius
                                          else " "))
            text.set_text("%-12s %8s %8s" % (grade[2], cells[0], cells[1]))

    return fig, draw, kPass


###############################################################################
# Figure 5 - the time histories
###############################################################################
def navigationFigure(data):
    """
    fig = navigationFigure(data) plots the dock's x, depth and pitch over the
    whole run: the truth against every grade's estimate, unaided and levelled.

    Each panel: a shaded band for the capture radius and an axis twice as
    wide as the band, the scale that matters for docking. An estimate that
    leaves the axis is marked where it does. Only x, depth and pitch appear,
    because the biases leave y, roll and yaw exact.

    A cursor marks the moment of the scene figure, the vehicle's closest pass.
    """
    c = commonData(data)
    t, true, runs, kPass = c["t"], c["true"], c["runs"], c["kPass"]

    fig, axes = plt.subplots(1, 3, figsize=(12.5, 5.8), sharex=True)
    fig.subplots_adjust(left=0.07, right=0.98, top=0.87, bottom=0.24,
                        wspace=0.34)
    axX, axDepth, axPitch = axes

    panels = [
        dict(ax=axX, title="(a) x",
             ylabel=r"$x - x_D$ (m)", ystep=0.5,
             truth=true[:, 0] - c["xD"], centre=0.0, band=captureRadius,
             estimate=lambda d: d["cable"][:, 0] - c["xD"]),
        dict(ax=axDepth, title="(b) depth",
             ylabel="depth z (m)", ystep=0.5,
             truth=true[:, 2], centre=c["zD"], band=captureRadius,
             estimate=lambda d: d["cable"][:, 2]),
        dict(ax=axPitch, title="(c) pitch",
             ylabel="pitch (deg)", ystep=2.0,
             truth=c["first"]["thetaTrue"] / D2R, centre=0.0,
             band=capturePitch / D2R,
             estimate=lambda d: d["thetaEst"] / D2R),
    ]

    for panel in panels:
        ax, centre, band = panel["ax"], panel["centre"], panel["band"]
        low, high = centre - 2 * band, centre + 2 * band
        inverted = ax is axDepth        # deeper is lower on the page

        ax.axhspan(centre - band, centre + band, color="0.85", alpha=0.7)
        ax.plot(t, panel["truth"], color=colour["true"], linewidth=2.5)

        for grade, gradeColour, mode, d in runs:
            y = panel["estimate"](d)
            ax.plot(t, y, color=gradeColour, linestyle=lineStyle[mode])

            outside = (y < low) | (y > high)
            if outside.any():
                kOut = int(np.argmax(outside))
                edge = high if y[kOut] > high else low
                # The note goes inside the axis, below or above the edge
                # where the curve leaves - on the page, not in data values
                atTop = (edge == high) != inverted
                ax.annotate(
                    "%s,\n%s" % (grade[2], mode),
                    xy=(t[kOut], edge), xytext=(14, -22 if atTop else 22),
                    textcoords="offset points", fontsize=insideSize,
                    color=gradeColour,
                    va="center", arrowprops=dict(arrowstyle="->",
                                                 color=gradeColour))

        ax.axvline(t[kPass], color="0.5", linewidth=0.8)

        ax.set_xlim(0, t[-1])
        ax.set_ylim(low, high)
        ax.xaxis.set_major_locator(MultipleLocator(100))
        ax.yaxis.set_major_locator(MultipleLocator(panel["ystep"]))
        if inverted:
            ax.invert_yaxis()
        ax.set_xlabel("time (s)")
        ax.set_ylabel(panel["ylabel"])
        ax.set_title(panel["title"])
        ax.grid(alpha=0.3)

    handles = [
        Line2D([], [], color=colour["true"], linewidth=2.5,
               label="true dock"),
        Patch(facecolor="0.85",
              label="capture radius %.1f m" % captureRadius),
        Line2D([], [], color="0.5", linewidth=0.8,
               label="AUV passes, t = %.0f s" % t[kPass]),
    ] + gradeHandles() + modeHandles()
    fig.legend(handles=handles, loc="lower center", ncol=4,
               frameon=False, bbox_to_anchor=(0.5, 0.0))
    figureTitle(fig)

    return fig


def animate(fig, draw, nSamples):
    """animate(fig, draw, nSamples) writes the figure in motion as a GIF"""
    step = int(round(frameEvery / sampleTime))
    frames = list(range(0, nSamples, step))
    if frames[-1] != nSamples - 1:
        frames.append(nSamples - 1)

    ani = animation.FuncAnimation(fig, lambda n: draw(frames[n]),
                                  frames=len(frames), blit=False)

    os.makedirs(figureDir, exist_ok=True)
    path = os.path.join(figureDir, "anim_dockScenario.gif")
    ani.save(path, writer=animation.PillowWriter(fps=fps), dpi=80)
    print("  wrote anim_dockScenario.gif (%d frames, %.0f s)"
          % (len(frames), len(frames) / fps))


###############################################################################
# Main program
###############################################################################
def main():
    print("Running the dock scenario for every grade, unaided and levelled...")
    runs = runAll()
    data = {grade[2]: {mode: scenarioData(runs[grade[2]][mode]["log"])
                       for mode in modes}
            for grade in imuGrades}

    fig, draw, kPass = sceneFigure(data)

    print("Animating...")
    animate(fig, draw, len(data[imuGrades[0][2]][modes[0]]["t"]))

    # The still scene: the moment the vehicle passes the dock
    draw(kPass)
    save(fig, "fig4_dockScene")

    save(navigationFigure(data), "fig5_dockNavigation")


if __name__ == "__main__":
    main()
