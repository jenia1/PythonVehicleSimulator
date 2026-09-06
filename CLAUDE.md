# Robust Navigation to a Free-Floating Dock — simulation notes

Thesis simulation built on T. I. Fossen's PythonVehicleSimulator. Working
branch: `Jenia2Dsimulation`. The goal is to simulate an underwater vehicle
navigating to a dock hanging from a buoy on a cable, with realistic sensing —
each body carries its own IMU and navigation filter, and the controller only
ever sees estimates, never ground truth.

## How we work

- **Sketch before code.** For anything new, propose the interfaces and the
  design trade-offs first and settle them in discussion; write the code after.
  This has already changed the design twice and is worth the extra round trip.
- **Small steps, checked.** One reviewable change at a time, with
  `compare_simdata.py` run after each.
- **Claude writes the commit message, Jenia commits.** Reviewing the diff at
  commit time is how Jenia keeps track of what changed, so never commit.
  Write the message — including *why*, not just what — and hand it over.
- **Claude maintains this file.** Jenia does not track it. Keep it current as
  part of the work, not as an afterthought: fix anything that has gone stale
  in the same step that made it stale. A wrong note here is worse than none.
- **Say what was verified and what was assumed.** Prefer showing evidence
  (bitwise comparisons, rendered plots, physical sanity checks) over
  asserting that a change is safe.

## Rule: check every change against the reference

**After any change to the simulation, run:**

```bash
python3 compare_simdata.py
```

It runs exactly the simulation `Main_Project.py` runs — via
`Main_Project.runSimulation()`, so the check can never test a different
configuration than the one that actually runs — and compares the logged
output column-by-column against `simdata_ref.csv` (tracked in git). Exit
code 0 = match.

The comparison is **exact**: no tolerance, and the column names must match
too. `--tol 1e-9` allows a numerical tolerance if a change legitimately
reorders floating-point arithmetic.

**Never loosen the check to make a failure go away, and never regenerate the
reference.** Its strictness is the entire point. Some changes are *supposed*
to break it — adding CSV columns always will. That failure is the signal to
stop and review together: confirm the existing columns still match exactly,
look at what is new, and decide whether the new output is better. If it is,
Jenia regenerates the reference deliberately:

```bash
cp simdata.csv simdata_ref.csv    # Jenia only, after reviewing the diff
```

The reference has been regenerated twice so far, both times to take on new
columns. `compare_simdata.py` prints that guidance itself when the layout
changes but every reference column still matches.

**What the check cannot do:** it proves the output did not *change*, never
that it is *right*. A wrong physics term gets baked into the reference the
moment it is regenerated. Models with real behaviour — the `Cable` above all
— need their own unit tests asserting the physics directly.

## Layout

| Path | What it is |
|---|---|
| `src/python_vehicle_simulator/Main_Project.py` | Thesis entry point: DSRV + buoy + dock, depth autopilot, x-z plane, plots + CSV log |
| `src/python_vehicle_simulator/main.py` | Upstream Fossen entry point, interactive vehicle menu |
| `src/python_vehicle_simulator/lib/mainLoop.py` | `simulate()` and the body-stepping helpers |
| `src/python_vehicle_simulator/lib/simLog.py` | `SimLog`, the named-column multi-body log |
| `src/python_vehicle_simulator/lib/plotTimeSeries.py` | Plots, including `plotNavigation()` (estimate vs. truth) |
| `src/python_vehicle_simulator/structures/` | `Buoy`, `Dock` — the docking structure; `Cable` still to come |
| `src/python_vehicle_simulator/sensors/IMU.py` | IMU model, `"ideal"` or `"simple"` error model |
| `src/python_vehicle_simulator/navigation/StrapdownINS.py` | IMU-only dead reckoning, no aiding yet |
| `src/python_vehicle_simulator/vehicles/` | 10 standalone Fossen vehicle models, no shared base class |
| `compare_simdata.py` | The regression check above |
| `simdata_ref.csv` | Golden reference output (tracked) |
| `simdata.csv` | Current run output (regenerated, untracked) |

## How the loop is structured

`simulate()` is an orchestrator; per-body work lives in two helpers:

- `initBody(body, eta0, imu_mode)` — attaches `.eta`, `.imu`, `.nav`,
  `.eta_est`, `.nu_est` onto a vehicle-like object, alongside the `.nu` /
  `.u_actual` it already carries.
- `stepBody(body, u_control, sampleTime)` — dynamics → IMU measure → nav
  update → advance `eta`. Returns the IMU measurement for logging.

Ordering inside `stepBody` matters and must not be rearranged: `nu_dot` is
only valid at the **pre-update** `eta`, so the IMU is read before
`attitudeEuler()` advances `eta`.

State is attached onto the model object rather than held in a wrapper class.
A wrapper (`NavigatedBody`) was tried and removed — it duplicated state the
vehicle already owned.

Every body — vehicle, buoy, dock — goes through the same two calls. Each is
logged the same way too: true state and navigation estimate *before* the
step, IMU measurement *after* it.

## Design decisions

**Buoy** — the anchor point. Assumed anchored and static for now (may become
a dynamic body later), but it *does* carry its own IMU and navigation filter,
so it is stepped through the same `initBody`/`stepBody` path as every other
body. Being static, its `dynamics()` is trivial — it returns zero `nu` and
`nu_dot`, holding the buoy in place. That trivial method is what lets one
uniform stepping path serve all three bodies instead of a special case in
`simulate()`.

A static body with an IMU and an unaided INS is also the cleanest possible
read on dead-reckoning drift: ground truth never moves, so everything the
navigation estimate does is error.

**Cable** — couples buoy ↔ dock only, not the vehicle. Simplified first pass:
straight line between the fixed buoy point and the dock, zero force when
slack (`|d| < L0`), linear spring + damping beyond that. No segments, no
catenary sag yet. Complicate later as needed.

**Dock** — the unit at the cable end (the vehicle's docking head). Static
today; becomes a dynamic body with real 6-DOF dynamics when the cable lands.

**External forces** — the dock has no actuators, so the cable force is set on
the dock as state (`dock.f_ext`) before stepping, and the dock's `dynamics()`
reads it back off. This is the role `u_control` plays for a vehicle. Chosen
over threading an `f_ext` parameter through `stepBody` → `dynamics`, which
would mean touching all 10 vehicle files to accept an argument none of them
use.

**Out of scope for now** — the vehicle has no sensing of the buoy or dock
(no range/bearing). Communication and fusion of the bodies' navigation data
comes at a later stage. Until then the dock's nav estimate is computed and
logged but not consumed by anything.

## Logging

`simulate()` returns a `SimLog`: one named block per body, each holding named
signals of fixed width, preallocated to `N+1` samples.

```python
log = simulate(N, sampleTime, vehicle, eta0, imu_mode, buoy=..., dock=...)

log.time                     # (N+1, 1)
log["dock"]["eta_est"]       # (N+1, 6)
log.simData("vehicle")       # eta | nu | u_control | u_actual
```

Bodies are namespaced by block name, so the dock's `x` cannot collide with
the vehicle's. `simData()` is a compatibility view producing the column
layout the Fossen plotting functions index by position, so those were left
untouched.

CSV column names resolve as: a `setNames()` override → `DEFAULT_NAMES` →
the signal name for width 1 → indexed fallback (`f_ext_0`). Names are never
guessed from width alone: a 6-vector may be a pose or a force/moment, and a
wrong label that looks right is worse than an obviously generic one. The
`"vehicle"` block writes unprefixed (`x`), every other block is prefixed
(`dock_x`).

`logDataToCSV()` in `Main_Project.py` lists its columns **explicitly** rather
than writing the whole log, so adding a signal cannot silently change
`simdata.csv`. The CSV carries `eta`, `nu`, `eta_est` and `nu_est` for all
three bodies, so any body's estimation error is derivable from the file
alone. Raw IMU (`f_b`, `omega_b`) is logged but deliberately not written —
add it to that list if you want it on disk.

## Plots

`Main_Project` draws six figures: the Fossen vehicle-state, control and 2D
trajectory plots (1-3), then `plotNavigation(log, body, figNo)` for the
vehicle, buoy and dock (4-6). `plotNavigation` overlays each body's estimate
on its ground truth and plots position, attitude and velocity error.

`plot2D` takes an optional `markers={label: (x, z)}` argument, used to draw
the buoy and dock. They are told apart by a legend rather than labels beside
the points, which overlap at any sensible zoom.

Never call `plt.grid()` before the first `plt.subplot()`: with no current
axes it silently creates a default one spanning the whole figure, whose 0-1
ticks then show through and collide with every subplot's tick labels. That
bug was in `plotVehicleStates` and is fixed.

## Numerical notes

With `imu_mode = "ideal"` the navigation estimate tracks truth to ~1e-13, but
**not exactly**, and the residual is a straight ramp in `z` rather than
noise. It is not integration error — truth and INS run the identical Euler
recursion, and the `x` and `y` errors are exactly zero.

It is floating-point cancellation on gravity: the IMU computes
`f_b = nu_dot - Rᵀg` and the INS adds `Rᵀg` straight back, and that
excursion through ~9.81 destroys the low bits of a much smaller acceleration
(`eps × g ≈ 2e-15`). Only `w` is affected — `u` is pinned by `u_const`, and
`v` sees no gravity component at zero roll and yaw — and integrating that
persistent ~1e-15 velocity residual gives the `z` ramp, ~2e-13 m over 200 s.

Harmless (0.2 picometres), but worth recognising so it is not mistaken for a
real effect when hunting small errors later.

## Conventions

Match the surrounding Fossen style: `###`-banner section headers, docstrings
that open with the call signature, `camelCase` function names, physics
notation for variables (`eta`, `nu`, `nu_dot`, `tau`, `u_control`). New
files carry `Author: Jenia`; upstream files keep `Author: Thor I. Fossen`.

## Current state

`Buoy` and `Dock` are **static** bodies: fixed position, `dynamics()` returns
zero `nu` and `nu_dot`, each with its own IMU and `StrapdownINS`, stepped and
logged alongside the vehicle. `Main_Project` places the buoy at the surface
half-way along the vehicle's track and the dock 10 m below it.

Verified over a full run: both hold position exactly, both nav estimates show
zero drift with the ideal IMU, and both accelerometers read
`f_b = [0, 0, -9.81]` — the correct static-body signature with z positive
down.

`Cable` does not exist yet, so `cable` is still `None` and nothing couples
the buoy to the dock. These are the calls the models must satisfy:

```python
initBody(buoy, buoy_eta0, imu_mode)     # needs .nu, .u_actual
stepBody(buoy, None, sampleTime)        # needs .dynamics(...), trivial/static

cable.step(buoy.eta, dock.eta, dock.nu, sampleTime) -> force on dock

initBody(dock, dock_eta0, imu_mode)
stepBody(dock, None, sampleTime)        # .dynamics() reads self.f_ext
```

`initBody` requires `.nu` and `.u_actual` to already exist on the object, and
reads optional `.U0`. The cable takes `buoy.eta` explicitly rather than
storing the anchor internally, so the signature still holds when the buoy
becomes dynamic.

**Next:** the `Cable` model and real `Dock` dynamics. Plan is to write the
expected behaviours as tests first (rest length → zero force; slack → zero
force with no discontinuity at the transition; stretched 1 m → `k·1`; force
on the dock points toward the buoy; damping opposes relative velocity), then
the model. `pytest` needs installing first.

## Known issues

- **`simulate()` only works with the DSRV.** `stepBody()` unpacks three
  values from `dynamics()`, but only `DSRV.py` returns `nu_dot` — the other
  nine vehicles still return the upstream `nu, u_actual`, so they fail with
  `ValueError: not enough values to unpack (expected 3, got 2)`.
  Pre-existing since commit `f0e6977` (the IMU/nav work). Deliberately not
  fixed: the thesis uses only the DSRV, and fixing it means deriving
  body-frame accelerations in nine vehicle models. Revisit only if another
  vehicle is actually needed.
- `tests/test_simulate.py` exercises the tanker, otter and others, so it
  cannot pass until the above is fixed. Its calls were updated to the SimLog
  API but have not been run — `pytest` is not installed in `.venv`.
- `imu_mode="simple"` is currently identical to `"ideal"`: all the error
  parameters in `IMU.py` are still zero/identity, awaiting real sensor specs.
