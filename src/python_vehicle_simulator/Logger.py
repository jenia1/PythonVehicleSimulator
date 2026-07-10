#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Logger.py: Lightweight named-variable data logger.

    addToLog("x", eta[0])   records one sample of the variable "x"

Call addToLog() as many times as you like, under as many names as you like
(e.g. once per state, per timestep). writeLog() then writes every logged
variable to a CSV file, one column per name.
"""
import csv

_log = {}   # variable name -> list of recorded values


def addToLog(name, value):
    """addToLog(name, value) appends value to the log entry for name."""
    _log.setdefault(name, []).append(value)


def clearLog():
    """clearLog() empties the log, e.g. before starting a new simulation run."""
    _log.clear()


def writeLog(filename):
    """
    writeLog(filename) writes all logged variables to a CSV file, one column
    per variable name (in the order first logged) and one row per sample.
    """
    names = list(_log.keys())

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(names)
        for row in zip(*(_log[name] for name in names)):
            writer.writerow(row)
