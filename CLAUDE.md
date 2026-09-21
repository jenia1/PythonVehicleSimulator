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
| `src/python_vehicle_simulator/navigation/cableGeometry.py` | Proposal eq (2)–(5): the cable pseudo-measurement |
| `studies/` | Analyses that produce thesis figures, not part of the simulation |
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

## Thesis proposal studies

Two scripts in `studies/`, both run directly, both writing into
`studies/figures/`:

| script | outputs | purpose |
|---|---|---|
| `cableErrorBudget.py` | `fig1`, `fig2` (PDF + PNG), summary table | the Preliminary Results figures |
| `dockScenario.py` | `anim_dockScenario.gif`, `fig4_dockNavigation_bias<b>` (one per gyro grade), `fig5_dockScenarioSnapshot` | *watching* the same scenario, to understand it |

They share one scene, defined by `runDock()` in `cableErrorBudget.py`: a
**300 s** run, the buoy and dock half-way along the vehicle's track
(x ≈ 616 m), the dock `L` below the buoy, and the vehicle holding the dock's
depth so it passes the dock at t ≈ 151 s. The vehicle has no dock sensing and
ideal sensors — it only gives the scene its scale. Where the scene sits
changes none of the error numbers (verified), since every error is relative
to the dock.

**The free inertial solution is not plotted anywhere — a decision, not an
omission.** It diverges as `t³` (2 km at 10 °/hr over 300 s), which is
textbook and not the point of the thesis, and next to centimetre-level cable
errors it reads like a bug. It is still computed and printed in the
`cableErrorBudget.py` summary table, marked "for reference only". The old
fig 3 (the free estimate leaving the eq (4) sphere, and the eq (5)
projection) was dropped with it, since it had no subject left; the
animation's reachable region already illustrates eq (4). Do not re-add
the free INS to a figure without asking.

`dockScenario.py` imports `runDock` from `cableErrorBudget.py`, so there is a
single definition of the scenario. Its `scenarioGrade` defaults to
100 °/hr rather than the headline 10 °/hr, purely so the cable estimate moves
far enough to see on the close-up. The animation shows the thesis concept in
one picture: the true dock never moves, and the cable estimate (green) walks
*along* the circle of radius `L` — eq (3) respects eq (4) by construction.
`fig5` is the frame at the vehicle's closest pass, since an animation cannot
go in a PDF.

`fig4` is written **once per gyro grade** in `gyroGrades`
(`fig4_dockNavigation_bias1/10/100`), so the runs can be compared side by
side. It shows only x, depth and pitch, because the pitch-axis bias leaves
y, roll and yaw exact — plotting flat zeros is what makes `plotNavigation`
hard to read. Its position panels scale to the *cable* estimate (never
tighter than ±2 × capture radius, with the capture band shaded), because
on a fixed ±15 m axis a 0.03 m or 0.29 m error is invisible. The suptitle
states the run's outcome in one line. The log error axes drop below 1e-3
only as far as needed to keep the cable error in view (in depth it is second
order, tens of micrometres at the tactical grade).

**Changing a gyro grade means changing one number.** `gyroGrades` holds only
`(bias, grade name)`, and every label in figures, tables and the animation
is built by `gradeLabel(bias)` from the number itself — e.g.
`"industrial MEMS, 10 deg/hr"`. A label can therefore never disagree with
what was simulated. (It once did: a bias was set to 0 while its label still
read "1 deg/hr".) A bias not in the list is shown by its number alone.

Plotting conventions used there, worth reusing: height (`−z`) on the
vertical axis with the tick labels shown as positive depth, so deeper is
lower on the page; one colour per quantity across every figure (truth black,
cable green, pitch orange, vehicle purple, buoy red).

### The error budget

It answers: how well can the dock's position be known from its navigation
unit *before* any fusion filter exists? The dock position comes from the
**cable pseudo-measurement, eq (3)**: the dock's *attitude* with known `p_S`
and known `L`. Its error is `L · δθ`: linear in time, bounded by `2L`
however bad the attitude gets. The tether converts what would be an
unbounded position problem into a bounded attitude problem.

Headline numbers, `L = 10 m`, 300 s approach, 0.5 m capture radius, attitude
from the unaided gyro (worst case — see the filter plan below):

| gyro bias | attitude error | cable eq (3) | free INS, ref only |
|---|---|---|---|
| 1 °/hr tactical | 0.08° | **0.015 m** | 214 m |
| 10 °/hr industrial MEMS | 0.83° | **0.145 m** | 2 140 m |
| 100 °/hr consumer MEMS | 8.3° | 1.45 m | 21 391 m |

So a 10 °/hr MEMS unit holds the dock inside the capture radius over a
five-minute approach; 100 °/hr does not. Halving the run from 600 s halved
the cable error (linear) and cut the free INS by 8 (`t³`).

**Why the free INS diverges with a perfect accelerometer** — asked more than
once, so recorded: the accelerometer measures acceleration *minus gravity*
(9.81 m/s² up, at rest, and it read exactly that). The INS removes gravity
along the "down" given by its *attitude*; with the gyro-drifted attitude it
removes it along the wrong direction, and the leftover `g·sin(δθ)` looks like
a sideways push. Pitch error ∝ t → phantom acceleration ∝ t → velocity ∝ t²
→ position ∝ t³, and the simulation matches `g·b·t³/6` to four digits. It is
the `−(fⁿ×)ε` term in row 2 of `F` in the proposal's eq (8). The cable
estimate avoids it entirely: it multiplies the same pitch error by `L` once,
with no gravity subtraction and no integration. The simulated cable error is
overlaid with the closed form `2L·sin(δθ/2)` in fig 2, and they coincide —
each validates the other.

Geometry worth remembering: for a cable along the dock's body z axis, **roll
and pitch each swing it by `L·δθ` at any slant, while yaw contributes
exactly zero at a vertical tether** and only `L·sin(slant)·δψ` once current
slants it. That is the argument for when the magnetometer earns its place.

Stated assumptions, which belong in the proposal text: taut straight cable
(`η = 0`), `p_S` known exactly, the AUV's own navigation exact (so the
`Δp` error of eq (1) equals the `p_D` error — eq (21) with `P_A = 0`), and
attitude error driven by gyro bias alone so it grows linearly. In the study
the dock's *only* IMU error is a constant gyro bias on the pitch axis: no
noise anywhere, accelerometer perfect.

**Agreed next step: a complementary attitude filter on the dock** (sketched,
not yet built). Levelling roll/pitch against the accelerometer's gravity
reading bounds the attitude error at `b·τ` instead of `b·t`, and therefore
bounds the cable estimate — the linear growth above is the unaided worst
case. It cannot bound the INS *position* (a constant residual leak remains,
so that still grows as `t²`); only a position reference can, and for the dock
that reference is the cable itself. Option rejected along the way: giving
the INS the true attitude for gravity compensation — it is ground truth
inside the estimator, and it would make the free INS exactly perfect.

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

**Next, for the proposal:** the complementary attitude filter on the dock
(see *Thesis proposal studies*; sketched, awaiting Jenia's go-ahead).

**After that:** make the dock *move* on its tether. Parameterise
its position by cable angles rather than Cartesian coordinates, so
`‖p_D − p_S‖ = L` — constraint (4) — holds by construction and the
projection (5) is never needed in the simulation itself. Swing it as a
pendulum or drift it under current. That turns the error budget above into a
measurement against a moving truth, which is Objective 1 of the proposal.

**Then, thesis work proper:** the `Cable` force model and real `Dock`
dynamics. Plan is to write the expected behaviours as tests first (rest
length → zero force; slack → zero force with no discontinuity at the
transition; stretched 1 m → `k·1`; force on the dock points toward the buoy;
damping opposes relative velocity), then the model. `pytest` needs
installing first. Note this is a *different* cable model from
`cableGeometry.py`: that one is the navigation measurement, this one is the
force.

## Known issues

- **`simulate()` only works with the DSRV.** `stepBody()` unpacks three
  values from `dynamics()`, but only `DSRV.py` returns `nu_dot` — the other
  nine vehicles still return the upstream `nu, u_actual`, so they fail with
  `ValueError: not enough values to unpack (expected 3, got 2)`.
  Pre-existing since commit `f0e6977` (the IMU/nav work).
  **This is a one-line fix per vehicle, not real work** (an earlier note here
  wrongly called it hard): every model already computes `nu_dot` internally
  before its Euler step — e.g. `remus100.py:384` — and simply does not return
  it. Adding `, nu_dot` to the return statement is all that is needed.
  Relevant because `remus100` is a full 3-DOF-controlled AUV
  (rudder + stern plane + rpm, `depthHeadingAutopilot` with `ref_z`,
  `ref_psi`, `ref_n` settable per step), which the DSRV is not — the DSRV
  pins surge to `U0` and has a single stern plane, so it can control depth
  but cannot steer or change speed.
- `tests/test_simulate.py` exercises the tanker, otter and others, so it
  cannot pass until the above is fixed. Its calls were updated to the SimLog
  API but have not been run — `pytest` is not installed in `.venv`.
- `imu_mode="simple"` with **no parameters supplied** is still identical to
  `"ideal"` — the defaults are zero/identity, which is what keeps the
  reference stable. Supply a specification per body to make it bite:

  ```python
  dock.imu_params = {"gyro_bias": [0.0, 4.85e-5, 0.0]}   # 10 deg/hr about y
  log = simulate(..., imu_mode="simple", dock=dock, ...)
  ```

  `initBody()` picks `.imu_params` off the body via `getattr`, the same trick
  used for `.U0`, so it works for any body including the vehicle models
  without touching them.
