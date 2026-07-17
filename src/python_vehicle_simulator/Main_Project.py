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
from python_vehicle_simulator.lib import (
    printVehicleinfo, simulate, plotVehicleStates, plotControls, plot2D
)
from python_vehicle_simulator.Logger import addToLog, writeLog, clearLog

### Simulation parameters ###
sampleTime = 0.02                   # sample time [seconds]
N = 10000                           # number of samples

# Initial state [x, y, z, phi, theta, psi], z positive downwards (m)
eta0 = [0, 0, 50.0, 0, 0, 0]
z_d = 60.0                          # desired depth (m)

# 2D plot and animation settings (DSRV only moves in the x-z plane)
numDataPoints = 50                  # number of 2D data points
FPS = 10                            # frames per second (animated GIF)
filename = '2D_animation.gif'       # data file for animated GIF
csvFilename = 'simdata.csv'         # data file for logged simulation data


def logDataToCSV(simTime, simData, vehicle, filename):
    """
    logDataToCSV(simTime, simData, vehicle, filename) records the simulation
    time, 6-DOF states and control signals with Logger.addToLog(), then
    writes them to a CSV file.
    """
    clearLog()
    for t, row in zip(simTime, simData):
        addToLog("time", t[0])
        addToLog("x", row[0])
        addToLog("y", row[1])
        addToLog("z", row[2])
        addToLog("phi", row[3])
        addToLog("theta", row[4])
        addToLog("psi", row[5])
        addToLog("u", row[6])
        addToLog("v", row[7])
        addToLog("w", row[8])
        addToLog("p", row[9])
        addToLog("q", row[10])
        addToLog("r", row[11])
        addToLog(vehicle.controls[0] + " (command)", row[12])
        addToLog(vehicle.controls[0] + " (actual)", row[13])

    writeLog(filename)


### Main program ###
def main():
    vehicle = DSRV('depthAutopilot', z_d, eta0[2])
    printVehicleinfo(vehicle, sampleTime, N)

    # Main simulation loop
    [simTime, simData] = simulate(N, sampleTime, vehicle, eta0)

    # Log simulation data to CSV
    logDataToCSV(simTime, simData, vehicle, csvFilename)

    # 2D plots and animation
    plotVehicleStates(simTime, simData, 1)
    plotControls(simTime, simData, vehicle, 2)
    plot2D(simData, numDataPoints, FPS, filename, 3)

    plt.show()
    plt.close()

if __name__ == "__main__":
    main()
