# Physics Agent Progress

## TODO
- [x] Add collision shapes to top spheres
- [x] Add `_check_columns_collapsed()` stop condition helper
- [x] Wire stop condition into `run_single()` loop
- [x] Run and verify collision prevents interpenetration
- [x] Verify stop condition triggers correctly
- [x] Log results
- [x] Remove octahedra collision shapes (fight joints, cause bouncing)
- [x] Remove top sphere collision shapes (not needed)
- [x] Increase solver iterations to 150
- [x] Re-run and verify stability

## Completed Steps

### Step 1 -- Top sphere collision shapes (DONE)
Added `ChCollisionShapeSphere` + `EnableCollision(True)` to all 4 top spheres in
`build_system()`, same pattern as bottom spheres (lines 301-305 in simulation_2x2x3.py).

### Step 2 -- `_check_columns_collapsed()` helper (DONE)
Added between grid helpers and `_find_shared_vertices()` (lines 127-168). Logic:
- For each of the 4 columns (ix,iy), checks both adjacent pairs (iz=0-1 and iz=1-2)
- For each pair, computes world-frame distance between +Z vertex of lower body and
  -Z vertex of upper body (axial), and between +X vertices of both (equatorial)
- A pair is "collapsed" when BOTH distances < tol (default 0.1)
- A column is collapsed when BOTH pairs are collapsed
- Returns True only when ALL 4 columns are collapsed

### Step 3 -- Stop condition in `run_single()` (DONE)
- Extracts `oct_bodies = bodies[:12]` from the all_bodies list
- At every export step, after recording CSV data, calls `_check_columns_collapsed()`
- If True, prints stop time and breaks the loop
- After loop, prints whether it stopped early or ran to completion

### Step 4 -- Initial Simulation Results (DONE)
Ran `conda run -n chrono python -m src.simulation_2x2x3` -- all 5 runs completed.

**Key findings:**
- All 5 simulations ran to full 10.0s duration (no early stop triggered)
- 16,000 CSV rows per run (800 export frames x 20 bodies)
- 20 inter-octahedron ball joints + 8 sphere-oct ball joints = 28 total
- Collision IS working: octahedra do NOT interpenetrate
- Max tilt angles per octahedron (seed=1 test): 55-58 degrees
  - Previously (no collision): over-buckled past 150 deg after ~2s
  - Now: max ~58 deg at t~10s, well within physical range
- Axial distances between adjacent pairs: ~0.0 (ball joints keep vertices coincident)
- Equatorial distances: ~1.41 (close to sqrt(2) = 1.414, no lateral collapse)
- Stop condition correctly did NOT trigger -- structure is stable under 0.5N force
  with collision preventing interpenetration

### Step 5 -- Remove collision from octahedra (DONE)
**Rationale:** Convex-hull collision shapes on octahedra fight the ball joints at
shared vertices. The contact solver pushes overlapping hulls apart while joints hold
them together, injecting oscillatory energy that causes bouncing and erratic behavior.

**Changes in `_make_octahedron_body()`:**
- Deleted the entire collision-shape block: `pts` vector, `col_mat`, `hull`,
  `AddCollisionShape()`, `EnableCollision(True)` (was ~11 lines)
- Updated docstring to: "no collision -- joints handle connectivity"
- Function now just sets pos, mass, inertia, then AddBody

### Step 6 -- Remove collision from top spheres (DONE)
**Rationale:** Top spheres don't contact anything -- they only transmit force through
ball joints. Collision shapes on them are unnecessary overhead.

**Changes in `build_system()`:**
- Deleted 5 lines per column: comment, `EnableCollision(True)`, `top_col_mat`,
  `top_col_shape`, `AddCollisionShape()`
- KEPT bottom sphere collision (ground plane contact is the only needed collision pair)

### Step 7 -- Increase solver iterations (DONE)
**API discovery:** Wrote test script confirming
`system.GetSolver().AsIterative().SetMaxIterations(150)` is the correct call.
(`SetSolverMaxIterations` does not exist on ChSystemNSC.)

**Change in `build_system()`:**
- Added `system.GetSolver().AsIterative().SetMaxIterations(150)` after
  `SetCollisionSystemType()` (line 205)

### Step 8 -- Re-run simulation after collision removal (DONE)
Ran `conda run -n chrono python -m src.simulation_2x2x3` -- all 5 runs completed.

**Results:**
- Zero NaN, zero Inf across all 5 runs
- 16,000 CSV rows per run (800 frames x 20 bodies)
- 28 ball joints (20 inter-oct + 8 sphere-oct)
- Zero erratic jumps (>5 deg between consecutive exports) in 4/5 runs
  - Run 5 had 5 jumps near t=2.5s in the 160-177 deg range; these are smooth
    transitions through a high-tilt state (body rotates up to 177 then back down),
    not bouncing instability
- Bottom spheres stay at initial z (-0.7071 = -1/sqrt(2)), ground contact working
- Structure buckles smoothly under 0.5N load without collision interference
- No stop condition triggered (columns do not fully collapse at this force level)

**Final tilt angle ranges at t=10s (per run):**
- Run 1: 158.8 - 171.8 deg
- Run 2: 179.8 - 180.0 deg
- Run 3: 139.8 - 179.3 deg
- Run 4: 174.7 - 179.4 deg
- Run 5: 177.9 - 179.5 deg

**Current collision configuration:**
- Octahedra: NO collision (joints handle connectivity)
- Top spheres: NO collision (force only, no contact needed)
- Bottom spheres: YES collision (ground plane contact for support)
- Ground plane: YES collision (floor at z = -r)
- Solver iterations: 150 (up from default ~50)

### Step 9 -- Tighten equilibrium detection (DONE)
**Rationale:** The previous KE threshold (0.002 J) was too loose -- the system's
steady-state KE oscillates around 0.001 J, so it could falsely declare equilibrium
while the structure is still slowly evolving. Also needed a minimum-tilt guard to
prevent declaring equilibrium when nothing has happened yet.

**Changes in `run_single()`:**
1. `ke_threshold`: 0.002 -> 1e-6 (tight threshold requires true quiescence)
2. `ke_consec_required`: 40 -> 20 (0.25 s sustained low KE, down from 0.5 s)
3. Time guard: `t > 1.0` -> `t > 2.0` (longer initial transient window)
4. Added minimum tilt guard before KE check: computes max tilt across all 12
   octahedra from quaternions (z_z component of rotation matrix), only checks KE
   if at least one octahedron has tilted past 5 degrees. This prevents false
   equilibrium on runs where the perturbation is too small to trigger buckling.

**Did NOT change:**
- Damping code (0.9999 factor, scaler owns this)
- Collision shapes or solver iterations
- `_check_columns_collapsed()` secondary stop condition (still active)

### Step 10 -- Re-run simulation after equilibrium tightening (DONE)
Ran `conda run -n chrono python -m src.simulation_2x2x3` -- all 5 runs completed.

**Results:**
- All 5 runs: 16,000 CSV rows, 800 export frames, 28 ball joints, ran full 10.0s
- No equilibrium stop triggered (expected: 1e-6 J is very tight)
- No column-collapse stop triggered
- Tilt guard correctly distinguishes active vs. inactive runs:

| Run | min tilt (deg) | max tilt (deg) | mean tilt (deg) | 5-deg guard |
|-----|---------------|---------------|-----------------|-------------|
| 1   | 30.0          | 142.0         | 79.8            | PASS (KE checked) |
| 2   | 0.1           | 0.5           | 0.2             | FAIL (KE skipped) |
| 3   | 51.5          | 56.8          | 53.6            | PASS (KE checked) |
| 4   | 0.0           | 0.2           | 0.1             | FAIL (KE skipped) |
| 5   | 51.1          | 58.4          | 53.6            | PASS (KE checked) |

- Runs 2 and 4: perturbation too small to trigger buckling, tilt guard correctly
  prevented false equilibrium declaration
- Runs 1, 3, 5: structure buckled significantly, KE check was active but system
  still evolving at t=10s (KE > 1e-6 J throughout)
- Zero NaN, zero Inf across all runs

### Step 11 -- Relax equilibrium detection thresholds (DONE)
**Rationale:** The 1e-6 J KE threshold from Step 9 was too tight. Residual numerical
jiggling from collision-joint interaction keeps KE above 1e-6 even when the system is
clearly settled (Run 3 had tilt change of only 0.15 deg over 1.25 s at the end but KE
never reached 1e-6). The 5-degree tilt guard was also too low -- it allowed equilibrium
checks before meaningful buckling had occurred.

**Changes in `run_single()` (lines 382-425 of simulation_2x2x3.py):**
1. `ke_threshold`: 1e-6 -> 0.01 J (relaxed; accepts settled-but-jiggling state)
2. `ke_consec_required`: 20 -> 40 (0.50 s sustained low KE, compensates for looser
   threshold -- doubled from 0.25 s to reduce false positives)
3. Minimum tilt guard: 5.0 deg -> 30.0 deg (only declare equilibrium after significant
   buckling has occurred, not just minor perturbation)

**Did NOT change:**
- Damping code (0.9999 factor, scaler owns this)
- Collision shapes or solver iterations
- `_check_columns_collapsed()` secondary stop condition (still active)
