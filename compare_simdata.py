#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_simdata.py: Regression check for the Main_Project simulation.

    Runs exactly the simulation Main_Project.py runs (same vehicle, same
    parameters) and compares the logged output against simdata_ref.csv, the
    reference recorded before the buoy/cable/dock work started.

    Run this after every change. As long as the buoy/cable/dock additions
    leave the vehicle path untouched, the output must stay bit-identical to
    the reference - any difference means the change altered vehicle
    behaviour, whether or not that was intended.

Usage:

    python3 compare_simdata.py              exact comparison (default)
    python3 compare_simdata.py --tol 1e-9   allow a numerical tolerance

Exit code is 0 when the output matches the reference, 1 when it does not.

Author:     Jenia
"""
import argparse
import sys

import matplotlib
matplotlib.use("Agg")          # no display needed, this script never plots

import numpy as np

from python_vehicle_simulator import Main_Project as MP
from python_vehicle_simulator.vehicles import DSRV
from python_vehicle_simulator.lib import simulate

REF_FILE = "simdata_ref.csv"
OUT_FILE = "simdata.csv"


def readSimData(filename):
    """
    names, data = readSimData(filename) reads a Logger CSV file and returns
    the column names and the numeric data as an array.
    """
    names = np.genfromtxt(filename, delimiter=",", max_rows=1, dtype=str)
    data = np.genfromtxt(filename, delimiter=",", skip_header=1)
    return list(names), data


def runSimulation(filename):
    """
    runSimulation(filename) runs the Main_Project simulation and writes the
    logged data to filename, using the parameters defined in Main_Project.py
    so this check always follows the real configuration.
    """
    vehicle = DSRV("depthAutopilot", MP.z_d, MP.eta0[2])
    simTime, simData, imuData, navData = simulate(
        MP.N, MP.sampleTime, vehicle, MP.eta0, MP.imu_mode
    )
    MP.logDataToCSV(simTime, simData, vehicle, filename)


def compare(refFile, outFile, tol):
    """
    compare(refFile, outFile, tol) prints a per-column comparison and returns
    True when the output matches the reference.
    """
    refNames, refData = readSimData(refFile)
    outNames, outData = readSimData(outFile)

    if refNames != outNames:
        print("FAIL: column names differ")
        print("  reference: %s" % refNames)
        print("  output:    %s" % outNames)
        return False

    if refData.shape != outData.shape:
        print("FAIL: shape differs - reference %s, output %s"
              % (refData.shape, outData.shape))
        return False

    diff = np.abs(outData - refData)
    maxDiff = np.nanmax(diff, axis=0)

    failed = [name for name, d in zip(refNames, maxDiff) if d > tol]

    print("%-28s %s" % ("column", "max |output - reference|"))
    print("-" * 55)
    for name, d in zip(refNames, maxDiff):
        print("%-28s %.3e %s" % (name, d, "" if d <= tol else "  <-- DIFFERS"))
    print("-" * 55)

    if failed:
        print("FAIL: %d of %d columns differ (tolerance %g): %s"
              % (len(failed), len(refNames), tol, ", ".join(failed)))
        return False

    print("PASS: output matches %s (%d rows, tolerance %g)"
          % (refFile, refData.shape[0], tol))
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tol", type=float, default=0.0,
                        help="max allowed absolute difference (default: 0, exact)")
    args = parser.parse_args()

    runSimulation(OUT_FILE)
    sys.exit(0 if compare(REF_FILE, OUT_FILE, args.tol) else 1)


if __name__ == "__main__":
    main()
