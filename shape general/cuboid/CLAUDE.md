# 3×3×3 Cuboid Array Simulation

## Hard Rules (non-negotiable)

1. **Clear output before every run.** Any time a code change is made and a
   simulation is re-run, wipe the target output directory first
   (e.g. `rm -rf output/bushing_K1e4/*`). No mixing old and new artifacts.
2. **Always maintain an agent progress report file** at
   `progress/<agent>.md` (e.g. `progress/physics_cuboid.md`,
   `progress/structure_cuboid.md`, `progress/gui_viz_cuboid.md`).
   This file is the human-readable record: decisions, tilt tables, verdicts,
   wall-clock. **No raw logs, no stdout dumps, no command transcripts** —
   those belong in `logs/` and are gitignored.
3. **Every sweep runs all 5 seeds in parallel and produces the default
   review set.** A "run" is not complete until all three exist in the
   output dir: (a) 5 CSVs (`sim_001..sim_005.csv`), (b) the aggregate
   tilt-angle plot (`tilt_angles.png`), and (c) 5 front_view MP4s.
   **Corner view is not part of the default review set** — render it only
   when the user explicitly asks. Parallel launch uses `OMP_NUM_THREADS=1`
   per process.

## What This Project Is

The shape-generalization of the validated 3×3×3 octahedral lattice. It swaps
the **octahedron** unit cell for a **cube** while keeping the rest of the
validated stack — bushing joints, `ChSolverADMM`, body-ID layout, the
CSV → MP4 → tilt-plot pipeline — as a clean 1:1 port of `3x3x3_simulation/`.

The unit cell is a **cube**: a single edge length `EDGE_LENGTH` (lx = ly = lz).
The joint formulation lives behind a module-level `JOINT_MODE` flag in
`src/simulation_cuboid.py`, identical in structure to `3x3x3_simulation/`:

- **`JOINT_MODE = "bushing"`** (default, the validated formulation) — compliant
  `ChLoadBodyBodyBushingSpherical`. Bushings contribute **zero rows to the
  constraint Jacobian Φ_q**, so the system can never be overconstrained no
  matter how many bushings share a cube face. This is the "scalable solution"
  validated on the 3×3×3 octahedron lattice (bushing buckling at F = 20 N).
- **`JOINT_MODE = "spherical"`** — rigid `ChLinkLockSpherical`. Kept in the
  `_add_ball_joint` factory for code-structure parity with `3x3x3_simulation/`,
  **but it must not be used on the cuboid lattice**: cubes share whole faces
  (4 coincident vertices per interface), so rigid spherical joints would be
  massively overconstrained. The flag exists only so the port is a faithful
  mirror of the octahedron codebase.

## Reference

Read `../../3x3x3_simulation/CLAUDE.md` for the two-mode architecture, the
bushing port checklist, and the DOF argument. Read
`../../Sanity_Test/CLAUDE.md` and `../../2x2x3_simulation/CLAUDE.md` for code
conventions, PyChrono API patterns, CSV format, tilt angle computation, and
progress tracking rules. Everything there applies here.

The full DOF / overconstraint argument lives in
`../../findings/ball_joint_scaling_analysis.docx`.

## Architecture — Three Agents

Work is split across three agents with **strict, non-overlapping file
ownership**. An agent must never edit a symbol or file owned by another. All
three are scoped strictly to `shape general/cuboid/` — none may touch
`3x3x3_simulation/`, `2x2x3_simulation/`, `Sanity_Test/`, or any other
subproject.

1. **physics_cuboid** — copies the shape-agnostic physics. Owns the solver
   setup, the `_add_ball_joint` bushing factory, `STIFFNESS_VARIANTS` /
   `CHANGE2_VARIANTS`, the KE-equilibrium detector, the time-stepping loop,
   CSV export, `run_all` / `run_sweep`, and `run.py`. Ports `tests/test_api.py`,
   `tests/test_force.py`, `tests/check_ke.py`. Leaves cube geometry and joint
   topology as named placeholder functions for `structure_cuboid` to fill.
   Read `.claude/agents/physics_cuboid.md`.
2. **structure_cuboid** — designs the cuboid structure and connects the
   joints, adapting the octahedron topology to a cube. Owns cube geometry
   (`cuboid_vertices`, `cuboid_inertia`, `_make_cuboid_body`), the grid
   constants (`EDGE_LENGTH`, `SPACING`, `NX/NY/NZ`), the shared-vertex scan
   (`_find_shared_vertices`), and the joint wiring inside `build_system`.
   Writes `docs/joint_topology_cuboid.md`, `tests/smoke_bushing.py`,
   `tests/rest_state_bushing.py`. Read `.claude/agents/structure_cuboid.md`.
3. **gui_viz_cuboid** — copies the GUI visualization. Owns `src/visualizer.py`
   (cube mesh: 8 vertices, 12 triangles), `src/plot_tilts.py`, and optionally
   `src/visualizer_corner.py`. Renders the MP4 + tilt-plot pipeline.
   Read `.claude/agents/gui_viz_cuboid.md`.

Agents run **sequentially**: structure_cuboid needs physics_cuboid's
scaffold; gui_viz_cuboid needs structure_cuboid's CSV output. They cannot be
parallelized on this pipeline.

## Progress Tracking

Same rules as `3x3x3_simulation/`. Append to the relevant `progress/*.md`
after every completed task. Decisions, parameter choices, joint counts,
validation verdicts, and wall-clock — never raw logs.

## Physical Setup

### 3×3×3 Cuboid Lattice

27 cubes arranged in a grid with 3 along X, 3 along Y, and 3 along Z. Edge
length `EDGE_LENGTH = 1.0`. Center-to-center spacing is `SPACING =
EDGE_LENGTH × sqrt(2)` along each axis — the **same open spacing as the
octahedron lattice**, leaving a 0.414·a gap between facing cube faces. Cubes do
**not** touch face-to-face: this is an **open strut frame**, not a solid block.

A face-to-face packing (`SPACING = EDGE_LENGTH`, every coincident corner
welded into 392 bushings) is a rigid block that **cannot buckle under any
load** — every run sat at ~0° tilt. That is the reason this lattice was
redesigned as an open frame.

### Joint Rule

The lattice is an **open strut frame** (`_axis_neighbor_couplings`): each cube
is tied to its 6 axis-neighbours by **one** bushing, anchored on the
centre-line at the midpoint of the inter-cube gap. This is the cube analogue of
the octahedron's single apex-to-apex joint — a single-point coupling leaves the
rotational freedom the frame needs to tilt and buckle (the old face-to-face scan
welded 4+ bushings per interface and could not move).

| Coupling | Count |
|---|---|
| Inter-cube struts: 18 along X + 18 along Y + 18 along Z | **54** |
| Sphere-BC: 9 bottom + 9 top spheres × 1 bushing | **18** |
| **Total** | **72** |

**Total: 72 couplings** (54 inter-cube + 18 sphere-BC) — identical to the
octahedron lattice. Full derivation in `docs/joint_topology_cuboid.md`.

Each coupling is a `ChLoadBodyBodyBushingSpherical`: three isotropic
translational springs (K = 1e4 N/m, C = 1e2 N·s/m per triad, ζ ≈ 0.5
under-damped), rotation free, **zero rows in Φ_q**. All 72 live in a per-system
`ChLoadContainer`. Requires `ChSolverADMM` — PSOR raises `System descriptor
includes stiffness or damping matrices` at runtime. (`JOINT_MODE = "spherical"`
is dead code kept only for structural parity with the octahedron file.)

`STIFFNESS_VARIANTS` at the top of `simulation_cuboid.py` parameterizes the
bushing sweep; `CHANGE2_VARIANTS` carries the contingency (F_top = 3 N/col,
perturb = 0.1 rad/s) used when the baseline forcing leaves tilts under the
10° gate.

### Why this cannot overconstrain

`ChLoadBodyBodyBushingSpherical` is a **force element**, not a kinematic
constraint. It contributes **zero rows to Φ_q**. Overconstraint is a property
of rigid `ChLink*` joints (too many constraint rows for the available
relative DOF) — forces cannot overconstrain. 72 bushings → 0 constraint
rows → the system retains its full 270 DOF. The open strut frame uses far fewer
bushings than the old face-to-face block (72 vs 464), so ADMM steps are much
cheaper too — runs reach buckling/equilibrium in a fraction of the wall-clock.

### Boundary Conditions — Force-Based (No Kinematic Constraints)

All bodies are free to move in all directions. Loading is applied via forces,
not motors or fixed constraints. Bushings keep the structure connected.

**9 bottom ghost spheres** — one per iz=0 cube, at the center of its bottom
(−Z) face:
- Ghost bodies: exist for joint connectivity but have no collision shapes.
- Cubes rest on the ground directly via their own box collision.
- Coupled to their cube via a single bushing at the bottom-face centre, like
  the octahedron's single apex coupling.

**9 top loading spheres** — one per iz=NZ-1 cube, at the center of its top
(+Z) face:
- NO motors. Constant 0.5 N downward force applied via persistent ChForce.
- As the structure tilts, the vertical force decomposes into axial + lateral
  components, driving cooperative buckling.
- Coupled to their cube via a single bushing at the top-face centre.

**Ground plane implementation**:
- Ground body with a large box collision shape at the floor level.
- Cubes have box collision shapes (rest directly on the ground).
- Bottom spheres are ghost bodies (no collision) — joint-only connectivity.
- ChSystemNSC contact solver handles the reaction forces.

### DOF Accounting

- Bodies: 27 cubes + 9 bottom spheres + 9 top spheres = **45 free bodies**.
- Global DOF: 45 × 6 = **270**, fully retained (bushings add zero Φ_q rows).
- Cube-only subsystem: 27 × 6 = **162 DOF** — buckling mode fully accessible.
- Structural overconstraint is eliminated at 3×3×3 and at every larger grid
  size, exactly as in the octahedron bushing formulation.

### Simulation Parameters

Ported verbatim from the `3x3x3_simulation/` bushing formulation:
- `dt = 1e-4`, export every 125 steps.
- 5 runs with random perturbations on an interior cube at `(0,0,1)` (flat
  index 9). Rows are streamed to the CSV as they are produced (no in-memory
  buffering), so a multi-hour run stays memory-bounded.
- **Stop conditions** — a run ends on whichever fires first: KE equilibrium,
  column collapse, or a **3 h per-process wall-clock cap**
  (`WALL_CLOCK_BUDGET_S = 10800 s`, plumbed through `run_single` / `run_all` /
  `run_sweep` and overridable via `scripts/run_seed.py --budget`). The
  simulated-time `duration` is a non-binding ceiling (default ~1e6 s) so the
  wall-clock cap and early-stops govern. With the 5 seeds launched in parallel
  (`scripts/launch_sweep.sh`) the per-process cap bounds the whole sweep to
  ~3 h of wall-clock.
- `F_top = 0.5 N` per column (baseline); `CHANGE2_VARIANTS` raises this to
  3.0 N/column with perturb 0.1 rad/s if the baseline leaves all tilts under
  the 10° gate.
- Damping factor 0.9999 applied every 10 steps.
- KE equilibrium threshold 0.01 J with 40 consecutive checks required.

**Solver**: `ChSolverADMM` with `SetMaxIterations(30)` (the octahedron's settled
value). The open frame has only 72 couplings — the same as the octahedron
lattice — so per-step cost is back in line with the octahedron baseline and runs
reach buckling/equilibrium well within the 3 h cap.

### Output

Same CSV format as `3x3x3_simulation/`. Body IDs: **0–26 cubes** (flat index
`ix + NX*iy + NX*NY*iz`), **27–35 bottom spheres**, **36–44 top spheres**.
45 bodies total.

MP4 video from each run CSV (front view) and a tilt-angle PNG from all 5 runs
for the column at **(0,0)** — body IDs `[0, 9, 18]`. Per-run `<stem>_tilt.png`
plots with a dashed 10° gate line and annotated peak tilt are produced
alongside the aggregate overlay. If rendering exhausts Metal GPU memory on the
second CSV, use one subprocess per CSV.

### run.py

Same pattern as `3x3x3_simulation/run.py`: `--sim` runs the simulation,
`--viz` renders MP4s on the output CSVs, `--plot` runs the tilt plotter,
`--all` (default) runs all three.

### Apple Silicon note

On macOS arm64 the projectchrono pychrono build needs `libomp.dylib`
preloaded into the process before `import pychrono`, otherwise it fails with
`symbol not found in flat namespace '___kmpc_dispatch_init_4'`. Run via:
```
DYLD_INSERT_LIBRARIES=$CONDA_PREFIX/lib/libomp.dylib \
  $CONDA_PREFIX/bin/python run.py --sim
```
or set `DYLD_INSERT_LIBRARIES` once in your shell rc. The Python source files
in this project are kept as a clean 1:1 port of the `3x3x3_simulation/`
codebase and **do not include any bootstrap code** — keep the port pure.
