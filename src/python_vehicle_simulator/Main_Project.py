#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main_Project.py: Entry point that runs only the DSRV simulation.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd edition, John Wiley & Sons, Chichester, UK.
URL: https://www.fossen.biz/wiley
"""
import matplotlib.pyplot as plt
from python_vehicle_simulator.vehicles import DSRV
from python_vehicle_simulator.structures import Buoy, Dock
from python_vehicle_simulator.lib import (
    printVehicleinfo, simulate, plotVehicleStates, plotControls, plot2D
)

### Simulation parameters ###
sampleTime = 0.02                   # sample time [seconds]
N = 10000                           # number of samples

# Initial state [x, y, z, phi, theta, psi], z positive downwards (m)
eta0 = [0, 0, 50.0, 0, 0, 0]
z_d = 0.0                       # desired depth (m)

imu_mode = "ideal"              # "ideal" or "simple" (bias/scale/misalignment/noise)

### Docking structure ###
# Both are static for now, so these positions are where they stay. The buoy
# sits at the surface half-way along the vehicle's track (the DSRV holds a
# constant surge speed U0, so the track is U0 * N * sampleTime long), with the
# dock hanging 2 m below it.
x_mid = eta0[0] + 0.5 * 4.11 * N * sampleTime

buoy_eta0 = [x_mid, 0, 0.0, 0, 0, 0]        # surface, mid-track
dock_eta0 = [x_mid, 0, 10.0, 0, 0, 0]        # 10 m below the buoy

# 2D plot and animation settings (DSRV only moves in the x-z plane)
numDataPoints = 50                  # number of 2D data points
FPS = 10                            # frames per second (animated GIF)
filename = '2D_animation.gif'       # data file for animated GIF
csvFilename = 'simdata.csv'         # data file for logged simulation data


def logDataToCSV(log, filename):
    """
    logDataToCSV(log, filename) writes the simulation time, the vehicle's
    6-DOF states and its control signals to a CSV file.

    The signals are listed explicitly rather than writing the whole log, so
    that adding a signal to the log (a new body, an IMU or navigation
    channel) does not silently change the columns of this file. Add to this
    list deliberately, and regenerate simdata_ref.csv when you do.
    """
    log.writeCSV(filename, signals=[
        ("vehicle", "eta"),
        ("vehicle", "nu"),
        ("vehicle", "u_control"),
        ("vehicle", "u_actual"),
        ("buoy", "eta"),
        ("dock", "eta"),
    ])


def runSimulation():
    """
    vehicle, log = runSimulation() builds the vehicle and the docking
    structure and runs the simulation with the parameters defined above.

    Kept separate from main() so that compare_simdata.py runs exactly this
    simulation without also plotting it.
    """
    vehicle = DSRV('depthAutopilot', z_d, eta0[2])

    # Docking structure. Both are static, so they only hold the positions
    # they are initialized with, but each carries its own IMU and navigation
    # filter and is stepped alongside the vehicle.
    buoy = Buoy()
    dock = Dock()

    log = simulate(N, sampleTime, vehicle, eta0, imu_mode,
                   buoy=buoy, buoy_eta0=buoy_eta0,
                   dock=dock, dock_eta0=dock_eta0)

    return vehicle, log


### Main program ###
def main():
    vehicle, log = runSimulation()
    printVehicleinfo(vehicle, sampleTime, N)

    # Log simulation data to CSV
    logDataToCSV(log, csvFilename)

    # 2D plots and animation. The plotting functions index simData by column
    # position, so they take the log's vehicle block in that layout.
    simTime = log.time
    simData = log.simData("vehicle")

    # Buoy and dock are static, so any sample gives their position
    markers = {
        "buoy": (log["buoy"]["eta"][0, 0], log["buoy"]["eta"][0, 2]),
        "dock": (log["dock"]["eta"][0, 0], log["dock"]["eta"][0, 2]),
    }

    plotVehicleStates(simTime, simData, 1)
    plotControls(simTime, simData, vehicle, 2)
    plot2D(simData, numDataPoints, FPS, filename, 3, markers=markers)

    plt.show()
    plt.close()

if __name__ == "__main__":
    main()
