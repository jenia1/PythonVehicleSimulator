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

REF_FILE = "simdata_ref.csv"
OUT_FILE = "simdata.csv"


def readSimData(filename):
    """
    names, data = readSimData(filename) reads a simulation CSV file and
    returns the column names and the numeric data as an array.
    """
    names = np.genfromtxt(filename, delimiter=",", max_rows=1, dtype=str)
    data = np.genfromtxt(filename, delimiter=",", skip_header=1)
    return [str(name) for name in names], data


def runSimulation(filename):
    """
    runSimulation(filename) runs the Main_Project simulation and writes the
    logged data to filename. The simulation is built by Main_Project itself,
    so this check always follows the real configuration.
    """
    vehicle, log = MP.runSimulation()
    MP.logDataToCSV(log, filename)


def compareColumns(refNames, refData, outNames, outData, tol):
    """
    compareColumns(...) prints the per-column comparison for every reference
    column and returns True when they all match the output within tol.
    """
    print("%-28s %s" % ("column", "max |output - reference|"))
    print("-" * 55)

    matched = True
    for k, name in enumerate(refNames):
        if name not in outNames:
            print("%-28s %s" % (name, "MISSING from the output"))
            matched = False
            continue

        d = np.nanmax(np.abs(outData[:, outNames.index(name)] - refData[:, k]))
        matched &= d <= tol
        print("%-28s %.3e %s" % (name, d, "" if d <= tol else "  <-- DIFFERS"))

    print("-" * 55)
    return matched


def compare(refFile, outFile, tol):
    """
    compare(refFile, outFile, tol) prints a per-column comparison and returns
    True only when the output matches the reference exactly - same columns,
    same values. A layout change is a failure like any other; it is up to
    Jenia to review it and regenerate the reference.
    """
    refNames, refData = readSimData(refFile)
    outNames, outData = readSimData(outFile)

    if refData.shape[0] != outData.shape[0]:
        print("FAIL: row count differs - reference %d, output %d"
              % (refData.shape[0], outData.shape[0]))
        return False

    valuesMatch = compareColumns(refNames, refData, outNames, outData, tol)

    added = [name for name in outNames if name not in refNames]
    missing = [name for name in refNames if name not in outNames]

    if added or missing:
        print()
        if missing:
            print("columns missing from the output: %s" % ", ".join(missing))
        if added:
            print("new columns not in the reference: %s" % ", ".join(added))
        print()
        print("FAIL: the column layout no longer matches %s" % refFile)

        if valuesMatch and not missing:
            print()
            print("      Every reference column still matches exactly, so this is")
            print("      the expected failure when new signals are added to the")
            print("      CSV. Review the new columns above, and if they are what")
            print("      you intended, regenerate the reference deliberately:")
            print()
            print("          cp %s %s" % (outFile, refFile))
        return False

    if not valuesMatch:
        print("FAIL: output differs from %s (tolerance %g)" % (refFile, tol))
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
