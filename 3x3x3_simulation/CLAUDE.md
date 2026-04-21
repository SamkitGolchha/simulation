# 3×3×3 Octahedral Array Simulation

## What This Project Is
Scales the validated 2×2×3 simulation to a 3×3×3 grid of 27 octahedra. Two
joint formulations coexist in this subproject behind a module-level
`JOINT_MODE` flag in `src/simulation_3x3x3.py`:

- **`JOINT_MODE = "spherical"`** — rigid `ChLinkLockSpherical`. This is the
  baseline on `main`, shipped by `setup333`, and it exposes the
  overconstraint failure: the oct-only subsystem has net DOF = 0, so all 27
  octahedra translate as a rigid column under load (max tilt 0.033°; see
  `progress/setup333.md`). Kept as empirical evidence for the DOF argument.
- **`JOINT_MODE = "bushing"`** — compliant `ChLoadBodyBodyBushingSpherical`,
  ported from the committed 2×2×3 refactor (2026-04-21). Bushings contribute
  zero rows to the constraint Jacobian Φ_q, so F_bushing = 6·B identically
  at any grid size. Owned by `joints333` on the `joints-3x3x3-bushings`
  branch. This is the path that unblocks 5×5×5.

The full DOF argument and cross-grid evidence live in
`../findings/ball_joint_scaling_analysis.docx` (2026-04-21).

## Reference
Read ../Sanity_Test/CLAUDE.md and ../2x2x3_simulation/CLAUDE.md for: code
conventions, PyChrono API patterns, CSV format, tilt angle computation, and
progress tracking rules. Everything there applies here.

## Architecture — Two Agents
1. **setup333** — owns the **rigid-spherical baseline on `main`**. Already
   shipped; the baseline run is the published evidence that the rigid
   formulation has no buckling mode at 3×3×3. Only re-activates if the
   committed 2×2×3 baseline changes and a re-sync is needed. Maintains
   src/simulation_3x3x3.py (with `JOINT_MODE = "spherical"`), src/visualizer.py,
   src/visualizer_corner.py, src/plot_tilts.py, run.py, tests/,
   different_angles/, and CLAUDE.md. Read .claude/agents/setup333.md.
2. **joints333** — owns the **bushing port** on branch
   `joints-3x3x3-bushings`. Ports `ChLoadBodyBodyBushingSpherical`, the
   `STIFFNESS_VARIANTS`/`CHANGE2_VARIANTS` sweeps, the `ChSolverADMM`
   swap, and the `ChLoadContainer` wiring verbatim from
   `origin/main:2x2x3_simulation/src/simulation_2x2x3.py`. Scoped strictly
   to 3x3x3_simulation/. Read .claude/agents/joints333.md for the
   port checklist and validation protocol.

Do NOT write test cases beyond what's already ported. Do NOT modify
../Sanity_Test/ or ../2x2x3_simulation/.

## Progress Tracking
Same rules as 2×2×3. Append to the relevant progress/*.md after every
completed task.

## Physical Setup

### 3×3×3 Lattice
27 octahedra arranged in a grid with 3 along X, 3 along Y, and 3 along Z. Edge
length 1.0. Center-to-center spacing is edge_length × sqrt(2) along each axis.

### Joint Rule
For every pair of octahedra in the grid, compute world-frame vertices and check
for coincidence. At every coincident vertex, create a shared-vertex coupling
between the two bodies. Coupling count is invariant across the two joint
modes: 54 inter-oct (x-edges 18 + y-edges 18 + z-edges 18) + 18 sphere-BC = **72
shared-vertex couplings total**.

The coupling implementation depends on `JOINT_MODE`:

- **`"spherical"` (rigid, baseline on `main`)**: each shared vertex gets a
  `ChLinkLockSpherical` — three kinematic translation constraints in Φ_q
  per coupling. At 3×3×3 this makes the oct-only subsystem rank-deficient
  on rigid modes (F_oct = 0) — empirically confirmed by the rigid-column
  behavior in `progress/setup333.md`.
- **`"bushing"` (compliant, `joints-3x3x3-bushings` branch)**: each shared
  vertex gets a `ChLoadBodyBodyBushingSpherical` — three isotropic
  translational springs (K = 1e4 N/m, C = 1e2 N·s/m per bushing triad,
  ζ = 0.5 under-damped), rotation free, **zero rows in Φ_q**. All 72
  couplings live in a per-system `ChLoadContainer`. Static deflection
  under 3 N/column load is F/K = 3·10⁻⁴ m, three orders inside the 0.1 m
  `_check_columns_collapsed` tolerance. Requires `ChSolverADMM` — PSOR
  cannot handle the K/C matrices the bushing contributes and raises
  `System descriptor includes stiffness or damping matrices` at runtime.

`STIFFNESS_VARIANTS` at the top of `simulation_3x3x3.py` parameterizes the
bushing sweep; `CHANGE2_VARIANTS` carries the contingency (F_top = 3 N/col,
perturb = 0.1 rad/s) used when the baseline forcing leaves tilts under the
10° gate (the trigger rule ported from 2×2×3).

### Boundary Conditions — Force-Based (No Kinematic Constraints)
All bodies are free to move in all directions. Loading is applied via forces,
not motors or fixed constraints. Ball joints keep the structure connected.

**9 bottom ghost spheres** — at the -Z vertex of each iz=0 octahedron:
- Ghost bodies: exist for joint connectivity but have no collision shapes.
- Octahedra rest on the ground directly via their own convex-hull collision.
- Connected to their octahedron via ball joint.

**9 top spheres** — at the +Z vertex of each iz=NZ-1 octahedron:
- NO motors. Constant 0.5 N downward force applied via persistent ChForce.
- As the structure tilts, the vertical force decomposes into axial + lateral
  components, driving cooperative buckling.
- Connected to their octahedron via ball joint.

**Ground plane implementation**:
- Ground body with a large box collision shape at the floor level.
- Octahedra have convex-hull collision shapes (rest directly on ground).
- Bottom spheres are ghost bodies (no collision) — joint-only connectivity.
- ChSystemNSC contact solver handles the reaction forces.

**DOF counts (two formulations):**

- **`JOINT_MODE = "spherical"` (rigid, baseline on `main`)**:
  - Global (all bodies): 45 × 6 − 72 × 3 = 54 DOFs — positive, healthy.
  - Oct-only subsystem (ignore sphere BCs): 27 × 6 − 54 × 3 = **0 DOFs** —
    exactly at the overconstraint boundary. No buckling mode accessible;
    structure translates as a rigid column under load. Verified empirically
    in `progress/setup333.md`.
- **`JOINT_MODE = "bushing"` (compliant, `joints-3x3x3-bushings` branch)**:
  - Bushings contribute zero rows to Φ_q; F = 6·B identically.
  - All 45 bodies × 6 = **270 DOFs retained**. Oct-only: 27 × 6 = **162 DOFs**.
  - Structural overconstraint is eliminated at 3×3×3 and at every larger
    grid size. The coupling that the rigid joint enforced as a constraint
    is now a stiff spring on the RHS of `M·v̇ = f`.

### Simulation Parameters
**Baseline (both modes)**: dt 5e-5, duration 10.0 s, export every 250 steps,
5 runs with random perturbations on an interior octahedron at (0,0,1)
(flat index 9), F_top 0.5 N per column, damping factor 0.9999 every 10 steps,
KE equilibrium threshold 0.01 J with 40 consecutive checks required.

**Solver / iterations**:
- `JOINT_MODE = "spherical"`: `ChSolverPSOR` (default), `SetMaxIterations(150)`.
- `JOINT_MODE = "bushing"`: `ChSolverADMM` with `SetMaxIterations(150)` —
  PSOR rejects the K/C matrices contributed by the bushing.

**`CHANGE2_VARIANTS` contingency (bushing mode only)**: if the baseline run
produces all tilts under the 10° gate and stabilises, keep the baseline MP4s,
add `output/change_2/<label>/`, and re-run there with F_top = 3.0 N/column,
perturb = 0.1 rad/s (K, C, dt, export_interval unchanged).

### Output
Same CSV format as 2×2×3. Body IDs: 0–26 octahedra, 27–35 bottom spheres,
36–44 top spheres. 45 bodies total.

MP4 video from each run CSV (front view + corner view) and tilt-angle PNG
from all 5 runs for the column at **(0,0)** — body IDs `[0, 9, 18]`. Per-run
`<stem>_tilt.png` plots with a dashed 10° gate line and annotated peak tilt
are produced alongside the aggregate overlay (mirrors the 2×2×3 2026-04-21
`plot_single_run` addition). If rendering exhausts Metal GPU memory on the
second CSV, use one subprocess per CSV (see
`../2x2x3_simulation/progress/gui-viz.md` Joint Update).

### run.py
Same pattern as 2×2×3: runs simulation, then visualizer on output CSVs,
then plotter on output directory.

### Apple Silicon note
On macOS arm64 the projectchrono pychrono build needs `libomp.dylib`
preloaded into the process before `import pychrono`, otherwise it fails
with `symbol not found in flat namespace '___kmpc_dispatch_init_4'`. Run via:
```
DYLD_INSERT_LIBRARIES=$CONDA_PREFIX/lib/libomp.dylib \
  $CONDA_PREFIX/bin/python run.py --sim
```
or set DYLD_INSERT_LIBRARIES once in your shell rc. The Python source files
in this project are kept as a clean 1:1 scale-up of the committed 2×2×3
codebase and do not include any bootstrap code.
