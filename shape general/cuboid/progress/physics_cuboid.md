# physics_cuboid — progress log

Agent 1 of 3. Copies the shape-agnostic physics from the validated
3×3×3 octahedron bushing simulation into the cuboid subproject.

## 2026-05-20 — Scaffold delivered

### Task 1 — Port run.py
Ported `../../3x3x3_simulation/run.py` → `run.py`. Only changes: every
`simulation_3x3x3` import rewritten to `simulation_cuboid`; "Octahedral" →
"Cuboid" in the argparse description and STEP-1 wording. `--sim/--viz/--plot/
--all` pipeline structure and the per-variant `_variant_dirs()` logic are
unchanged.

### Task 2 — Write src/simulation_cuboid.py
Ported every shape-agnostic piece of physics verbatim from
`simulation_3x3x3.py`:
- imports, `JOINT_MODE: Literal[...] = "bushing"`, header comment with the
  body-ID layout (0–26 cubes, 27–35 bottom spheres, 36–44 top spheres).
- `STIFFNESS_VARIANTS` and `CHANGE2_VARIANTS` — verbatim.
- `_add_ball_joint` — verbatim, both `"spherical"` and `"bushing"` branches
  kept (spherical is dead code on the cuboid lattice, kept for structural
  parity).
- `_make_sphere_body`, `_compute_total_ke`, `_check_columns_collapsed`,
  `_grid_index_to_flat`, `_grid_center` — verbatim.
- `build_system` skeleton, `run_single`, `run_all`, `run_sweep`,
  `run_change2_sweep`, `__main__` — verbatim (seed loop, dt=1e-4, export
  every 125 steps, duration 10 s, F_top 0.5 N, damping 0.9999 every 10
  steps, KE threshold / 40 consecutive checks, ADMM solver).

Placeholders left for **structure_cuboid** (defined, not implemented — module
imports cleanly, each raises `NotImplementedError("owned by structure_cuboid")`):
- `cuboid_vertices`, `cuboid_inertia`, `_make_cuboid_body`,
  `_find_shared_vertices`.
- Grid constants `EDGE_LENGTH`, `SPACING`, `NX/NY/NZ` — declared with
  `# owned by structure_cuboid — placeholder value` comments and provisional
  values (`EDGE_LENGTH=1.0`, `SPACING=EDGE_LENGTH`, `NX=NY=NZ=3`) so the file
  parses. structure_cuboid sets the real values; note `SPACING = EDGE_LENGTH`
  for cubes (face-to-face), NOT `* sqrt(2)`.

### Task 3 — Port tests
- `tests/test_api.py` — ported verbatim (pure PyChrono API check, fully
  shape-agnostic, no body-ID references).
- `tests/test_force.py` — ported; body-ID range comments and variable names
  updated octahedron → cube (ranges 0–26 / 27–35 / 36–44 are identical
  layout). Logic unchanged.
- `tests/check_ke.py` — ported; `sys.path.insert` and `_check_columns_collapsed`
  body-ID comments updated for cubes.

### Parameter notes / deviations
- Simulation parameters: ported verbatim from the `3x3x3_simulation/` bushing
  formulation — `dt=1e-4`, export every 125 steps, `duration=10.0 s`, 5 runs,
  perturbed cube at flat index 9, F_top 0.5 N, CHANGE2 contingency 3.0 N /
  0.1 rad/s, damping 0.9999 every 10 steps, KE threshold 0.1 (bushing mode)
  with 40 consecutive checks. `ChSolverADMM` with `SetMaxIterations(150)` per
  CLAUDE.md / brief — note the 3×3×3 octahedron source file currently has
  `SetMaxIterations(30)` (a wall-clock concession); the brief and project
  CLAUDE.md both specify 150, so 150 is used here. This is the only solver
  parameter that differs from the octahedron source file's literal value.
- `_check_columns_collapsed`: octahedron used the axial vertex offset
  `r = a/sqrt(2)`; for a cube the top/bottom face-centre vertex is at
  `r = a/2`. This is a shape-driven correction (the function is otherwise
  shape-agnostic in structure); logged here because it is the one
  non-name change in an otherwise-verbatim helper. Adjusting it now keeps the
  early-stop column check meaningful once structure_cuboid fills the geometry.
- `tests/check_ke.py`: the 3×3×3 octahedron `check_ke.py` imports a symbol
  `_check_equilibrium` that does NOT exist in `simulation_3x3x3.py` (only
  `_compute_total_ke` does) — that octahedron test file is broken on import.
  To deliver a clean-importing cuboid scaffold, the cuboid `check_ke.py`
  imports `_compute_total_ke` (the real KE helper, which the script body
  already uses inline) instead of the non-existent name. No behaviour change;
  the script computes identical KE.

### Task 4 — Import verification
Command run (CONDA_PREFIX = /Users/samkit/anaconda3/envs/chrono):
```
DYLD_INSERT_LIBRARIES=$CONDA_PREFIX/lib/libomp.dylib \
  $CONDA_PREFIX/bin/python -c \
  "from src.simulation_cuboid import _add_ball_joint, run_all, run_sweep; print('import OK')"
```
Result: `import OK`.

Additional checks: all four placeholders raise
`NotImplementedError("owned by structure_cuboid")`; grid constants resolve
(1.0 / 1.0 / 3 / 3 / 3 / 27); `JOINT_MODE="bushing"`; `STIFFNESS_VARIANTS`
and `CHANGE2_VARIANTS` populated; `run.py` and all three test files parse.

### Deliverable status
`simulation_cuboid.py` imports cleanly with all physics wiring in place and
four named placeholder functions + grid constants for structure_cuboid to
fill. No simulation run (build_system raises `NotImplementedError` until the
placeholders are implemented — expected).

Files created:
- `run.py`
- `src/simulation_cuboid.py`
- `tests/test_api.py`, `tests/test_force.py`, `tests/check_ke.py`
- `progress/physics_cuboid.md`
