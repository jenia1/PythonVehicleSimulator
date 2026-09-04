#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simLog.py:

    SimLog(N, sampleTime)

    Named-column log for a multi-body simulation. Data is stored as one
    block per body ("vehicle", "buoy", "dock", "cable"), each holding named
    signals of fixed width, preallocated to N+1 samples:

        log = SimLog(N, sampleTime)
        log.log("vehicle", i, eta=vehicle.eta, nu=vehicle.nu)
        log.log("dock", i, eta=dock.eta, nu=dock.nu)

    Bodies are namespaced by their block name, so the dock's "x" cannot
    collide with the vehicle's "x", and every signal is written by row index
    into a preallocated array rather than being appended, so logging stays
    O(N) and every column is guaranteed the same length.

Methods:

    log(body, i, **signals) stores one sample of each named signal.

    setNames(body, signal, names) overrides the CSV column names for one
        signal, e.g. the per-vehicle control input names.

    simData(body) returns the eta | nu | u_control | u_actual array in the
        column layout the plotTimeSeries functions expect.

    writeCSV(filename, signals) writes the log to a CSV file, one column per
        signal component.

    Column names are resolved in this order:

        1. an explicit setNames() override
        2. DEFAULT_NAMES[signal], if the width matches
        3. the signal name itself, for a signal of width 1
        4. indexed fallback, "<signal>_0" ... "<signal>_<w-1>"

    Names are never guessed from the width alone: a 6-vector may be a
    position/attitude (x...psi) or a force/moment (fx...mz), and a wrong
    label that looks right is worse than an obviously generic one.

Author:     Jenia
"""
import csv

import numpy as np


# Default CSV column names, by signal name. Signals not listed here fall back
# to the signal name (width 1) or to indexed names (see columnNames).
DEFAULT_NAMES = {
    "eta":     ["x", "y", "z", "phi", "theta", "psi"],
    "nu":      ["u", "v", "w", "p", "q", "r"],
    "eta_est": ["x_est", "y_est", "z_est", "phi_est", "theta_est", "psi_est"],
    "nu_est":  ["u_est", "v_est", "w_est", "p_est", "q_est", "r_est"],
    "f_b":     ["f_x", "f_y", "f_z"],
    "omega_b": ["omega_x", "omega_y", "omega_z"],
    "f_ext":   ["fx_ext", "fy_ext", "fz_ext", "mx_ext", "my_ext", "mz_ext"],
}

# Block written without a name prefix, so its columns keep the plain names
# (x, y, z, ...) used by the reference data. Every other block is prefixed.
PRIMARY_BODY = "vehicle"


###############################################################################
# Class SimLog
###############################################################################
class SimLog:
    """
    SimLog(N, sampleTime) - see module docstring.
    """

    def __init__(self, N, sampleTime):
        self.N = N
        self.sampleTime = sampleTime

        # Bitwise identical to the arange(0, t_end + dt, dt) form previously
        # used by simulate(), without depending on the accumulated end time
        self.time = (np.arange(N + 1) * sampleTime)[:, None]

        self._blocks = {}       # body -> {signal -> (N+1, width) array}
        self._names = {}        # (body, signal) -> [column names]

    ###########################################################################
    # Logging
    ###########################################################################
    def log(self, body, i, **signals):
        """
        log(body, i, **signals) stores sample i of each named signal, e.g.

            log.log("vehicle", i, eta=vehicle.eta, nu=vehicle.nu)

        Arrays are allocated on a signal's first appearance and filled with
        NaN, so a sample that is never written stays visibly unset. Logging
        the same signal at a different width afterwards is an error.
        """
        if not 0 <= i <= self.N:
            raise IndexError(
                "sample index %d out of range for a log of %d samples"
                % (i, self.N + 1)
            )

        block = self._blocks.setdefault(body, {})

        for name, value in signals.items():
            value = np.atleast_1d(np.asarray(value, dtype=float))

            if value.ndim != 1:
                raise ValueError(
                    "signal '%s' of body '%s' must be a scalar or 1-D, got "
                    "shape %s" % (name, body, value.shape)
                )

            if name not in block:
                block[name] = np.full((self.N + 1, value.size), np.nan)
            elif value.size != block[name].shape[1]:
                raise ValueError(
                    "signal '%s' of body '%s' was first logged with width %d, "
                    "now %d" % (name, body, block[name].shape[1], value.size)
                )

            block[name][i] = value

    ###########################################################################
    # Access
    ###########################################################################
    def __getitem__(self, body):
        """log[body] returns the body's signals as a {name: array} dict."""
        return self._blocks[body]

    def __contains__(self, body):
        return body in self._blocks

    def bodies(self):
        """bodies() returns the logged body names, in the order first logged."""
        return list(self._blocks)

    def signals(self, body):
        """signals(body) returns the body's signal names, in log order."""
        return list(self._blocks[body])

    def simData(self, body=PRIMARY_BODY,
                signals=("eta", "nu", "u_control", "u_actual")):
        """
        simData(body) returns the signals horizontally stacked into a single
        array, by default in the eta | nu | u_control | u_actual layout that
        plotVehicleStates(), plotControls(), plot2D() and plot3D() index by
        column position.
        """
        block = self._blocks[body]
        return np.hstack([block[name] for name in signals])

    ###########################################################################
    # CSV output
    ###########################################################################
    def setNames(self, body, signal, names):
        """
        setNames(body, signal, names) overrides the CSV column names for one
        signal, e.g. the control inputs, whose names come from the vehicle:

            log.setNames("vehicle", "u_control",
                         [c + " (command)" for c in vehicle.controls])
        """
        self._names[(body, signal)] = list(names)

    def columnNames(self, body, signal):
        """
        columnNames(body, signal) returns the unprefixed column names for one
        signal, resolved as described in the module docstring.
        """
        width = self._blocks[body][signal].shape[1]

        names = self._names.get((body, signal), DEFAULT_NAMES.get(signal))

        if names is not None:
            if len(names) != width:
                raise ValueError(
                    "signal '%s' of body '%s' has width %d but %d column names "
                    "were given: %s"
                    % (signal, body, width, len(names), ", ".join(names))
                )
            return list(names)

        if width == 1:
            return [signal]

        return ["%s_%d" % (signal, k) for k in range(width)]

    def columnLabel(self, body, name):
        """
        columnLabel(body, name) prefixes a column name with its body, except
        for the primary body, whose columns keep their plain names.
        """
        if body == PRIMARY_BODY:
            return name
        return "%s_%s" % (body, name)

    def writeCSV(self, filename, signals=None, timeColumn="time"):
        """
        writeCSV(filename, signals) writes the log to a CSV file, one column
        per signal component and one row per sample.

        signals: optional [(body, signal), ...] selecting what to write and
            in what order. Defaults to every signal of every body, in the
            order they were first logged.
        timeColumn: name of the leading time column, or None to omit it.
        """
        if signals is None:
            signals = [(body, signal)
                       for body in self._blocks
                       for signal in self._blocks[body]]

        columns = []            # [(label, (N+1,) array), ...]

        if timeColumn is not None:
            columns.append((timeColumn, self.time[:, 0]))

        for body, signal in signals:
            data = self._blocks[body][signal]
            for k, name in enumerate(self.columnNames(body, signal)):
                columns.append((self.columnLabel(body, name), data[:, k]))

        labels = [label for label, _ in columns]
        duplicates = sorted({name for name in labels if labels.count(name) > 1})
        if duplicates:
            raise ValueError(
                "duplicate CSV column names: %s" % ", ".join(duplicates)
            )

        # Every column is a preallocated (N+1,) array, so this zip can never
        # truncate the output to a short column
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(labels)
            for row in zip(*(column for _, column in columns)):
                writer.writerow(row)
