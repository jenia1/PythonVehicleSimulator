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
| `src/python_vehicle_simulator/navigation/StrapdownINS.py` | IMU dead reckoning, with an optional complementary filter on roll/pitch and an optional coarse alignment at the start |
| `src/python_vehicle_simulator/navigation/cableGeometry.py` | Proposal eq (2)–(5): the cable pseudo-measurement |
| `studies/` | Analyses that produce thesis figures, not part of the simulation |
| `src/python_vehicle_simulator/vehicles/` | 10 standalone Fossen vehicle models, no shared base class |
| `compare_simdata.py` | The regression check above |
| `tests/test_levelling.py` | Checks for the complementary filter and the coarse alignment; `python3 tests/test_levelling.py` (pytest-compatible, pytest not installed) |
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
| `dockScenario.py` | `fig4_dockScene`, `fig5_dockNavigation` (PDF + PNG), `anim_dockScenario.gif` | *watching* the same scenario, to understand it |

The gap in the numbering (no fig 3) is history, not a missing file. The
proposal numbers its own figures.

They share one scene, defined by `runDock(grade, levelled)` in
`cableErrorBudget.py`: a **300 s** run, the buoy and dock half-way along the
vehicle's track (x ≈ 616 m), the dock `L` below the buoy, and the vehicle
holding the dock's depth so it passes the dock at t ≈ 151 s. The vehicle has
no dock sensing and ideal sensors — it only gives the scene its scale. Where
the scene sits changes none of the error numbers (verified), since every
error is relative to the dock. `runAll()` runs every grade twice, unaided
and levelled (six runs, about 10 s in total). `dockScenario.py` imports
`runAll`, so there is a single definition of the scenario.

**The free inertial solution is not plotted anywhere — a decision, not an
omission.** It diverges as `t³` (2 km at the tactical grade over 300 s),
which is textbook and not the point of the thesis, and next to
centimetre-level cable errors it reads like a bug. It is still computed and
printed in the `cableErrorBudget.py` summary table, marked "for reference
only". The old fig 3 (the free estimate leaving the eq (4) sphere, and the
eq (5) projection) was dropped with it, since it had no subject left; the
close-up's reachable region already illustrates eq (4). Do not re-add the
free INS to a figure without asking.

### The IMU grades

From Groves' table *Typical Accelerometer and Gyro Biases for Different
Grades of IMU* (Jenia supplied it as an image, 2026-09-21). One entry per
row that matters for a dock, at the middle of each range on a log scale:

| grade | gyro bias | accel bias | source |
|---|---|---|---|
| intermediate | 0.1 °/hr | 0.3 mg | table: 0.1 °/hr, 0.1–1 mg |
| tactical | 10 °/hr | 3 mg | table: 1–100 °/hr, 1–10 mg |
| consumer | 1000 °/hr | 30 mg | table gives only **>100 °/hr, >3 mg**; one decade past those bounds is *our assumption* |

State the consumer extrapolation in the proposal. These replaced an earlier
gyro-only list (1 / 10 / 100 °/hr, named tactical / industrial MEMS /
consumer MEMS), whose names did not match Groves: by his table all three
are tactical.

**Changing a grade means changing one line.** `imuGrades` holds
`(gyro deg/hr, accel mg, name)`, and every label in figures, tables and the
animation is built by `gradeLabel(grade)` from those numbers, e.g.
`"tactical, 10 deg/hr, 3 mg"`. A label can therefore never disagree with
what was simulated. (It once did: a bias was set to 0 while its label still
read "1 deg/hr".) `mg` converts with g = 9.81 to match `IMU.g`, so `b_a/g`
is exactly `mg × 1e-3`.

### Unaided vs levelled, from the true attitude

Each grade is run twice. **Unaided** means the attitude comes from the gyro
alone. **Levelled** adds the complementary filter, τ = 1 s. **Both start
from the dock's true attitude**, so every curve starts at zero.

The decisions behind this, with the reasons (Jenia, 2026-09-21):
- **An accelerometer bias only matters with levelling.** Unaided, the
  attitude never looks at the accelerometer, so the accel bias does not
  touch the cable estimate at all.
- **Levelling trades one error for another.** It swaps the gyro's growing
  error `L·b·t` for a fixed floor, `L·(b_a/g + b·τ)`, which it reaches
  within a few τ. The accelerometer mostly sets that floor.
- **Alignment is assumed perfect.** Jenia: an alignment is done before the
  start, and it is not the subject here. So the study does not use the
  `align` option.

  It was tried first. A real alignment against a biased accelerometer
  would start both runs at the `atan(b_a/g)` tilt: at 30 mg, the consumer
  curves started 0.30 m off. That offset was unexplained in the figures
  and read as a puzzle.

Because the start is perfect, **the dashed and solid lines of a grade
cross**. The unaided error starts at zero and grows. The levelled error
jumps to its floor within seconds and stays there. So unaided is better
until it reaches the floor, roughly at `t ≈ (b_a/g)/b`:

| grade | levelled is better after | unaided leaves the capture radius |
|---|---|---|
| intermediate | 620 s (never, within 300 s) | never within 300 s |
| tactical | 63 s | 1031 s (never, within 300 s) |
| consumer | 7 s | 10.3 s |

At the intermediate grade the unaided gyro beats the levelled filter for
the whole run (1.5 mm against 3.0 mm). That is real, not a bug: a
complementary filter with a biased accelerometer converges to the biased
tilt, and this gyro drifts less than that in 300 s. Levelling earns its
place on the cheap units, which is what the dock is meant to be.

### The error budget

It answers: how well can the dock's position be known from its navigation
unit *before* any fusion filter exists? The dock position comes from the
**cable pseudo-measurement, eq (3)**: the dock's *attitude* with known `p_S`
and known `L`. Its error is `L · δθ`, bounded by `2L` however bad the
attitude gets. The tether converts what would be an unbounded position
problem into a bounded attitude problem.

Headline numbers, `L = 10 m`, 300 s approach, 0.5 m capture radius:

| grade | pitch error, unaided / levelled | cable eq (3), unaided | cable eq (3), levelled | free INS, ref only, unaided / levelled |
|---|---|---|---|---|
| intermediate | 0.008° / 0.017° | **1.5 mm** | 3.0 mm | 111 m / 0.7 m |
| tactical | 0.83° / 0.18° | 145 mm | **30.5 mm** | 816 m / 12 m |
| consumer | 83° / 2.0° | 13.3 m | **347 mm** | 191 km / 2.0 km |

What these numbers show:
- **Levelled, every grade holds the dock inside the capture radius,** the
  consumer grade included. The accelerometer sets the size: `L·b_a/g` is
  3, 30 and 300 mm, and the gyro adds only `L·b·τ` on top (0.005, 0.5 and
  47 mm).
- **Unaided, the consumer unit is outside the capture radius after 10.3 s**
  and ends 13.3 m off. Intermediate and tactical stay inside for the whole
  300 s.

(The free-INS columns now include the accelerometer bias integrated twice,
`b_a·t²/2`, on top of the gyro's gravity leak. That is why they differ from
the older gyro-only numbers above. They are for reference only.)

**The closed form is exact, and every run checks against it.**
`pitchErrorClosedForm()` gives the pitch error from the grade's numbers
alone:
- unaided: `b·t` (the accelerometer never enters)
- levelled: `(atan(b_a/g) + b(τ − dt))(1 − αⁿ)`

`cableErrorClosedForm()` turns it into the chord `2L·sin(δθ/2)`. The
simulation matches both to **2.9e-12 m** over every run and sample, and
`summaryTable()` prints that gap on every run. Fig 2 marks the closed form
on every curve.

**Why the free INS diverges with a perfect accelerometer** — asked more than
once, so recorded: the accelerometer measures acceleration *minus gravity*
(9.81 m/s² up, at rest, and it read exactly that). The INS removes gravity
along the "down" given by its *attitude*; with the gyro-drifted attitude it
removes it along the wrong direction, and the leftover `g·sin(δθ)` looks like
a sideways push. Pitch error ∝ t → phantom acceleration ∝ t → velocity ∝ t²
→ position ∝ t³, and the simulation matches `g·b·t³/6` to four digits. It is
the `−(fⁿ×)ε` term in row 2 of `F` in the proposal's eq (8). The cable
estimate avoids it entirely: it multiplies the same pitch error by `L` once,
with no gravity subtraction and no integration.

Geometry worth remembering: for a cable along the dock's body z axis, **roll
and pitch each swing it by `L·δθ` at any slant, while yaw contributes
exactly zero at a vertical tether** and only `L·sin(slant)·δψ` once current
slants it. That is the argument for when the magnetometer earns its place.

Stated assumptions, which belong in the proposal text:
- a taut, straight cable (`η = 0`)
- `p_S` known exactly
- the AUV's own navigation exact, so the `Δp` error of eq (1) equals the
  `p_D` error (eq (21) with `P_A = 0`)
- the dock starts with its true attitude: alignment before the run is
  assumed perfect
- the dock hangs still, which the levelling relies on
- the dock's IMU errors are constant biases only: a gyro bias about the
  pitch axis and an accelerometer bias along x, with no noise anywhere

Both biases tilt pitch the same way, so their effects add; with opposite
signs they would partly cancel, so adding is the worst case.

### The figures

- **fig 1:** pure eq (3) geometry, one panel: position error against
  attitude error for several cable lengths, with a marker where each
  length reaches the capture radius and that angle in the legend (5.7° at
  L = 5 m, 2.9° at 10 m, 1.4° at 20 m, 0.6° at 50 m). It doesn't depend on
  the grades. That tolerance is the panel's point, and the reason Jenia
  kept it.

  A second panel, showing roll, pitch and yaw against tether slant, was
  dropped (Jenia, 2026-09-23): it plotted a constant and a sine, which the
  proposal text states in a sentence instead. Its content is still in
  *Geometry worth remembering* above. The remaining panel now perturbs
  **pitch**, the axis the study uses; it perturbed roll before, which gives
  an identical curve but reads oddly next to a pitch-only study.
- **fig 2:** cable error against time, one colour per grade, dashed
  unaided, solid levelled. The closed form appears as hollow markers every
  20 s; markers rather than a line, because a dotted line over a dashed one
  reads as dash-dot. The axis spans 0 to 2× the capture radius, the scale
  that matters for docking. The consumer-unaided curve leaves it, so it is
  annotated with its end value. The floor sits slightly below zero so
  millimetre errors are not hidden under the axis line.
- **fig4_dockScene:** (a) the scene at the vehicle's closest pass, with a
  box of each run's error at that moment; (b) the close-up, where all six
  estimates walk *along* the circle of radius `L`, because eq (3) respects
  eq (4) by construction. Hollow markers are unaided, filled are levelled.

  **The box is a legend, not a text box** (Jenia, 2026-10-19). Each row
  carries the grade's coloured line beside its two numbers, so the box is
  also the colour key and the grades leave the figure legend at the bottom;
  only the grey mode markers stay there. The first row is the column
  header, carried by an invisible handle so that it lines up, and
  `_legend_box.align = "left"` keeps the title left-aligned with the rows.
  `draw(k)` fills the legend's texts, so the same box animates.
- **fig5_dockNavigation:** (a) x, (b) depth, (c) pitch against time, truth
  against every estimate. Each axis spans ±2× the capture radius with the
  band shaded; in pitch the band is `2·asin(r/2L)`, the pitch error that
  reaches the capture radius. A curve that leaves the axis gets a note, and
  a cursor marks the moment fig 4 shows. Only x, depth and pitch appear,
  because the biases leave y, roll and yaw exact.

  These two were **one** figure (`fig4_dockScenario`) until Jenia's
  supervisor asked for them apart (2026-09-30). Jenia had asked for them
  united two weeks earlier, so do not merge them back without asking.
  `commonData()`, `gradeHandles()`, `modeHandles()` and `figureTitle()` are
  shared, so the scene and the histories still agree on colours, styles and
  wording.

  Log-scale error panels were dropped (Jenia: "we don't need the log plots
  at this point").

  Its bottom legend is wider than its canvas, so `bbox_inches="tight"`
  saves it 13.5 in wide rather than 12.5 and the panels sit in the middle
  93%. Harmless while it is not in the proposal; shrink the legend before
  printing it.

- **Readable type, V4 proportions** (2026-10-19, the supervisor: "text in
  this figure is too small. increase font size and reduce the amount of
  text"). What a label measures on the page is
  `font pt x printed width / canvas width`, and nothing else. Both figures
  are drawn at 12.5 in; the proposal now prints fig 4 at 6.5 in (see **The
  page was the real limit** above), so the page shows it at 52%.
  `plotStyle` sets a 15 pt base font, which arrives as about 8 pt, the size
  of the caption beside it; `insideSize` = 12 pt is for labels drawn inside
  a panel, and the box of numbers is 13 pt monospace.

  Two settings were tried and rejected on the way, both worth not
  repeating:

  - **A smaller canvas** — 7.5 in with a 13 pt font. It printed at exactly
    the same size, but Jenia: "it now looks small and unimpressive", since
    the figure is also read on screen at its own size. Raise the font, keep
    the canvas.
  - **20 pt type**, which prints at 10.5 pt, bigger than the body text. It
    forced taller canvases (6.4 and 6.2 in) to make room, left the panels
    narrow, and grew the box of numbers until it nearly touched the axes
    around it — Jenia: "i dont like that the table inside almost toching
    the plot. can you make thouse plot the original size the way the were
    in V4".

  So the panels are back to the proportions of the figure in the proposal's
  V4: canvas 12.5 x 5.6 in, `width_ratios` 1.55:1, `wspace` 0.18,
  `zoomHalfWidth` 1.4. The one piece kept from the tall version is
  `set_anchor("N")` on the close-up, which keeps its top edge level with
  the scene's instead of floating in the middle of its cell. The box of
  numbers also carries `borderaxespad` so that it never sits against the
  frame.

  The room for that came from the page, not from the figure: printing at
  6.5 in rather than 4.8 means 15 pt reads as well as 20 pt did before.

  Text was cut to match: the explanatory note under the figures is gone,
  the title is one short line, legend labels are the grade's numbers alone
  ("consumer, 1000 deg/hr, 30 mg"), panel titles are short, and the
  close-up has three x ticks rather than seven. What the figures no longer
  say is in the proposal's captions and text.

Plotting conventions, worth reusing:
- **The figures call that node the source, the code calls it the buoy**
  (Jenia, 2026-10-20: "i called the bouy in the proposel \"Source\""). The
  proposal's eq (2) names it `p_S`, so every label a reader sees says
  source: the legend entry, the close-up's title and its x axis. In the
  simulation it stays the `Buoy` body, so `colour["buoy"]`, `log["buoy"]`
  and `buoy_eta0` are unchanged — renaming those would touch
  `Main_Project`, `mainLoop` and the CSV columns for a wording change.
- Height (`−z`) goes on the vertical axis, with tick labels shown as
  positive depth, so deeper is lower on the page.
- One colour per thing across every figure: truth black, vehicle purple,
  buoy red, and the grades (`gradeColours`) intermediate blue, tactical
  green, consumer orange.
- Unaided is dashed with hollow markers; levelled is solid with filled
  markers.
- `save()` writes PDFs without a creation date. An unchanged figure then
  stays byte-identical on a re-run and stays out of `git status` (it once
  showed fig 1 and fig 2 as modified when nothing had changed).
- **Check a figure at the size it is printed, not at the size it is drawn.**
  These are drawn at 12.5 in and printed at 4.8 in, so a 9 pt label arrives
  as 3.5 pt. Render the proposal page and look at it before calling a
  figure done.
- **The page was the real limit, not the figure** (2026-10-19). After the
  type grew, Jenia still saw little change on the page, and the measurement
  says why: `Proposel.tex` is a plain `article` with default margins, so the
  text block is 345 pt = 4.79 in on an 8.5 in page — 1.85 in of margin on
  each side, 44% of the paper. `width=\textwidth` therefore pinned fig 4 at
  4.79 in both before and after; only its height could grow (2.15 in → 2.47
  in, the taller canvas). The type did get bigger, from about 3.6 pt to 7.8
  pt printed, but the plot boxes could not.

  The fix is in the proposal, not in matplotlib: fig 4 is now included as

  ```latex
  \makebox[\textwidth][c]{\includegraphics[width=1.35\textwidth]{figures/fig4_dockScene.pdf}}
  ```

  which centres an oversized graphic on the text block and lets it run into
  the margins, 6.5 in wide with 1 in to spare on each side: +35% linear,
  +82% in area, and the labels land at 10.5 pt, the size of the body text.
  Nothing else on the page moves. The alternative, `geometry` with 1 in
  margins, would enlarge every figure and the tables too, but it repaginates
  the whole proposal and sets the lines to about 95 characters — worth
  raising with Jenia if the department does not prescribe the margins.

### The complementary filter and coarse alignment

`StrapdownINS(..., complementary_tau=τ)` levels roll and pitch against the
accelerometer after each gyro step. It is written in the textbook form Jenia
knows (xplicity.com's complementary-filter article):

```
angle = alpha * angleGyro + (1 - alpha) * angleAcc,   alpha = 1 - dt/τ
```

computed as `angleGyro + (1 − alpha)·ssa(angleAcc − angleGyro)` so the
difference wraps at ±180°. **τ = 1 s** was chosen (Jenia), which at
dt = 0.02 s is the familiar `alpha = 0.98`. τ is the parameter rather than
alpha because alpha depends on the sample rate; τ is a physical time. The
accelerometer angles, `roll = atan2(−f_y, −f_z)` and
`pitch = atan2(f_x, √(f_y² + f_z²))`, were verified against the IMU model to
3.6e-15°. They live in one function, `accelerometerAttitude()`, used by
both the filter and the alignment. Yaw is not levelled (gravity has no
heading information).

`StrapdownINS(..., align=True)` asks for a **coarse alignment**. It is
built and tested but **not used by the study**, which assumes a perfect
start (see *Unaided vs levelled* above). It is kept for when a realistic
start matters. `initBody`
then reads the IMU once with `nu_dot = 0` (the body is at rest before t = 0)
and calls `coarseAlign(f_b)`, which sets roll and pitch from the
accelerometer, errors included. Yaw stays at the true value. It lives in
`initBody` rather than in the first `update()` because it has to happen
before the t = 0 sample is logged.

**Both are opt-in per body**, like `imu_params`: `dock.nav_params =
{"complementary_tau": 1.0, "align": True}`, picked up by `initBody` via
`getattr`. Both are off by default, so every existing run and
`simdata_ref.csv` are unchanged (verified). Only the dock should get them:
both assume the body is not accelerating, which is wrong for the
manoeuvring vehicle.

`tests/test_levelling.py` has 9 checks against the exact discrete
behaviour:
- **Filter:** off by default; an ideal IMU stays level; a gyro bias settles
  at `b(τ − dt)`; an accel bias settles at `atan(b_a/g)`; an initial error
  decays as `(1 − dt/τ)ⁿ`.
- **Alignment:** off by default; aligned but unaided, the error is
  `atan(b_a/g)` throughout; aligned and levelled, the whole trajectory
  matches `atan(b_a/g) + b(τ − dt)(1 − αⁿ)`.
- **The study's case:** levelled from the true attitude with both biases,
  the whole trajectory matches the study's closed form.

Every check was shown to catch deliberately broken code (an alignment that
does nothing, a flipped sign, roll only). A sign flip in the accelerometer
pitch is caught *only* by the accel-bias check: with an ideal IMU or a pure
gyro bias the accelerometer pitch is zero, so its sign is invisible.

What levelling cannot do:
- **Bound the INS *position*.** A constant residual gravity leak remains,
  so it still grows as `t²`. Only a position reference can bound it, and
  for the dock that is the cable.
- **Level yaw.**
- **Work for a swinging dock.** Its accelerometer mostly measures cable
  tension along the cable, so it cannot see the swing angle eq (3) needs.
  That is likely the central research question for eq (3), and where this
  study continues once the dock moves.

Option rejected along the way: giving the INS the true attitude for gravity
compensation. That is ground truth inside the estimator, and it would make
the free INS exactly perfect.

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

**For the proposal:** the Preliminary Results figures are in place: fig 1,
fig 2 and `fig4_dockScenario`, with the Groves grades, unaided against
levelled, both from the true attitude.

A first draft of the proposal's *Preliminary Results* section exists
(2026-09-22). The proposal is a separate local git repo at
`../../Proposal/Proposel.tex`, built with `latexmk`. The draft covers the
simulation, the error-free baseline, and this study, and it states the
assumptions above.

The three figure PDFs are **copies** in `../../Proposal/figures/`. After
regenerating a figure here, copy it there again, or the proposal shows the
old one.

Open items in the draft:
- a `TODO(Jenia)` asks for the true source of the Groves bias table: the
  book, or possibly his tutorial paper
- the new references (Fossen handbook, PythonVehicleSimulator, Mahony 2008)
  should be verified
- fig 1 and fig 2 are still drawn in the old small-font style (9 pt on a
  6 in canvas) and are included at 0.65 and 0.75 `\textwidth`, so their
  labels print at about 5 pt against fig 4's 10.5; they need the same
  `plotStyle` treatment and the full width

The draft calls the complementary-filter weight `γ`, not `α`, because the
proposal already uses `α_k` for the robust EKF's adaptive factor. The code
keeps `alpha`.

**After that:** make the dock *move* on its tether. Parameterise
its position by cable angles rather than Cartesian coordinates, so
`‖p_D − p_S‖ = L` — constraint (4) — holds by construction and the
projection (5) is never needed in the simulation itself. Swing it as a
pendulum or drift it under current. That turns the error budget above into a
measurement against a moving truth, which is Objective 1 of the proposal. It
is also where levelling stops working: a swinging dock's accelerometer sees
cable tension, not gravity.

**Then, thesis work proper:** the `Cable` force model and real `Dock`
dynamics. Plan is to write the expected behaviours as tests first (rest
length → zero force; slack → zero force with no discontinuity at the
transition; stretched 1 m → `k·1`; force on the dock points toward the buoy;
damping opposes relative velocity), then the model. `pytest` needs
installing first. Note this is a *different* cable model from
`cableGeometry.py`: that one is the navigation measurement, this one is the
force.

## Keeping up with upstream

The fork tracks `cybergalactic/PythonVehicleSimulator`. Jenia syncs
`master` on GitHub, then `origin/master` is merged into
`Jenia2Dsimulation` (first done 2026-10-24, 15 commits).

**Nothing upstream has yet changed what this thesis simulates**, and that
is checked rather than assumed: `compare_simdata.py` passed unchanged
across the merge. The reason is that upstream's physics fixes are in code
the DSRV never calls.

What came in and what it means here:

| upstream change | where | effect on us |
|---|---|---|
| `crossFlowDrag` strip integration fixed (`dx = L/n`, strips at midpoints, `n` strips not `n+1`) | `lib/gnc.py` | none: only `otter`, `remus100` and `torpedo` call it. The DSRV does not |
| propeller advance speed uses relative speed `U_r`, not `U` | `vehicles/remus100.py` | none, `remus100` is unused here |
| rudder drag sign (`+ Xdd sin²δ`) | `vehicles/shipClarke83.py` | none |
| `plot3D` rewritten: an `AUVGeometry` class draws a real hull, with downsampling and start/end markers | `lib/plotTimeSeries.py` | available, unused. It needs `Poly3DCollection` from `mpl_toolkits` |
| start/end markers and axis labels in `plotVehicleStates` | `lib/plotTimeSeries.py` | **`Main_Project` figure 1 shows them** |
| `eta` default written as `np.zeros(6)` | `lib/mainLoop.py` | none, same value; our `eta0` argument supersedes it |
| `simData` reversed before plotting | `main.py` | none, that is upstream's demo entry point, not `Main_Project` |
| two new tests, `test_crossflow_drag.py` and `test_propeller_advance_speed.py` | `tests/` | they need pytest, which is still not installed |

Three files conflicted, all of them ones we had rewritten:

- **`mainLoop.py`** — ours kept whole. Upstream changed the scaffolding our
  `initBody` + `SimLog` rewrite had already replaced. Their *reason* was
  worth keeping, so it is now a comment where `eta0` is defaulted: a surface
  craft started at a nonzero depth is destabilized by the hydrostatic
  restoring force at t = 0.
- **`plotTimeSeries.py`** — both sides kept. Their line closes their new
  `plot3D`; ours is everything we added after it (`plot2D`,
  `plotNavigation`). Our fix for the stray `plt.grid()` before the first
  subplot survived the merge, which was worth checking, since upstream
  edited that very block.
- **`main.py`** — our three `SimLog` lines, then their flip, so the file
  stays as close to upstream as the new return type allows.

**An upstream bug to decide about:** in `plotVehicleStates`, the
North-East panel plots `plt.plot(y, x)` — East horizontal, North vertical,
the usual convention — but the new labels read `xlabel('North')`,
`ylabel('East')`, which is the wrong way round. It is upstream's, not ours,
and `Main_Project` draws that panel. Swapping the two strings fixes it;
left alone for now so the fork does not diverge without Jenia saying so.

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
  API but have not been run — `pytest` is not installed in `.venv`. That
  includes `test_otter_heading_autopilot_remains_finite`, which arrived in
  the upstream merge still unpacking the old `(simTime, simData)` tuple and
  was adapted to `simulate(...).simData("vehicle")` like the rest of the
  file. The two new upstream test files, `test_crossflow_drag.py` and
  `test_propeller_advance_speed.py`, are pure-pytest and untouched.
  `tests/test_levelling.py` sidesteps all of this by also running as a plain
  script; new checks should do the same until pytest is installed.
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
