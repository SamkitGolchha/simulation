# structure_cuboid — progress log

Agent 2 of 3. Designs the cube structure and connects its joints, adapting the
validated 3×3×3 octahedron bushing topology to a cube. Fills the four
cube-specific placeholders in `src/simulation_cuboid.py` and wires the joint
section of `build_system`.

## 2026-05-20 — Structure + joint topology delivered

### Task 1 — Grid constants + cube geometry
- Grid constants finalized: `EDGE_LENGTH = 1.0`, `SPACING = EDGE_LENGTH`
  (cubes touch **face-to-face**, NOT `* sqrt(2)` — that was the octahedron),
  `NX = NY = NZ = 3`, `N_CUBE = 27`. Values were already at the right
  provisional numbers in the scaffold; comments finalized.
- `cuboid_vertices(center, a)` — returns the 8 cube corners at
  `center ± (a/2, a/2, a/2)` as an `(8, 3)` numpy array. Canonical ordering
  documented in the docstring (indices 0-3 = +Z top face, 4-7 = -Z bottom
  face; sign order +X+Y, +X-Y, -X+Y, -X-Y within each face).
  `gui_viz_cuboid` must reuse this exact order for `_CUBE_VERTS_LOCAL`.
- `cuboid_inertia(mass, a)` — returns `(Ixx, Iyy, Izz)` all equal to
  `(1/6)·m·a²` (solid uniform cube; octahedron used `(1/10)·m·a²`). No cross
  terms.
- `_make_cuboid_body(system, center, mass, a)` — analog of
  `_make_octahedron_body`: ChBody at center, mass + cube inertia (zero cross
  terms), `ChCollisionShapeBox(col_mat, a, a, a)` with FULL edge lengths
  (replaces the octahedron's convex hull), `EnableCollision(True)`. Contact
  material reused verbatim (friction 0.5, restitution 0.0).

### Task 2 — Shared-vertex scan
- `_find_shared_vertices(centers, grid_indices, a, tol)` — same exhaustive
  pairwise scan as the octahedron version, with `octahedron_vertices` (6
  axial) swapped for `cuboid_vertices` (8 corners) → an 8×8 corner comparison
  per cube-pair. Emits one `(idxA, idxB, vertex_pos)` tuple per coincident
  corner.

### Task 3 — Joint wiring in build_system
- Inter-cube section: one bushing per shared-vertex tuple (392 total). Print
  label updated to "Inter-cube bushings".
- Sphere-BC section: **quad-bushing coupling**. Each ghost/loading sphere sits
  at the centre of the cube's bottom/top face and is coupled with **4
  bushings, one per face corner** (the octahedron used a single bushing at its
  apex vertex — a cube has a flat quad face, so 4 corner bushings are needed
  to transmit the tilting moment cleanly). A comment block in the source
  documents this. 9 bottom + 9 top spheres × 4 = 72 sphere-BC bushings.
- `build_system` now prints: inter-cube `392`, sphere-BC `72`, total `464`,
  body count `45`.
- Did NOT touch any physics_cuboid-owned symbol (solver setup, `_add_ball_joint`,
  `_make_sphere_body`, time-stepping, CSV, `run_*`). The `r = a/2` cube
  half-edge at the top of `build_system` was already corrected by
  physics_cuboid — verified correct for cube geometry (top/bottom face centre
  is a/2 from the body centre); only a clarifying comment was added.

### Task 4 — docs/joint_topology_cuboid.md
Written. Full derivation: 4×4×4 = 64-corner lattice → corner classification
(8 outer / 24 edge-interior / 24 face-interior / 8 body-interior) → per-corner
`C(k,2)` pair sum `0·8 + 1·24 + 6·24 + 28·8 = 392` inter-cube. Cross-checked by
adjacency type (54 face pairs ×4 + 72 edge pairs ×2 + 32 corner pairs ×1 =
392). Sphere-BC quad-coupling → 72. Total 464. Plus the "force elements →
0 rows in Φ_q → cannot overconstrain" argument and the body-ID layout table.

### Task 5 — Tests
- `tests/smoke_bushing.py` — adapted for 45 bodies / 464 bushings. Checks the
  392 inter-cube count, the (0,0,0)/(1,0,0) 4-vertex adjacency, the 45/464
  build counts, ADMM acceptance, and 200 steps with no NaN/Inf.
- `tests/rest_state_bushing.py` — adapted F_top=0 / perturb=0 static
  equilibrium check for 45 bodies, 1 s, max |v| < 1e-3 m/s threshold.

### Task 6 — Validation checkpoints

| # | Checkpoint | Result |
|---|---|---|
| 1 | `_find_shared_vertices` on fresh 3×3×3 → 392 inter-cube tuples | **PASS — 392** |
| 2 | `build_system` prints 45 / 392 / 72 / 464 | **PASS — 45 / 392 / 72 / 464** |
| 3 | cube (0,0,0) & (1,0,0) share exactly 4 vertices, all on x=0.5 face | **PASS — 4, all x=0.5** |
| 4 | 1-seed 0.5 s smoke run, no NaN/Inf, ADMM converging | see Runtime note below |
| 5 | `rest_state_bushing.py` F_top=0 for 1 s → max \|v\| < 1e-3 m/s | see Runtime note below |

Checkpoints 1-3 and the ADMM-acceptance / no-NaN portion of the smoke test
all PASS — `smoke_bushing.py` ran to completion (exit 0):
```
OK _find_shared_vertices -> 392 inter-cube tuples
OK cube (0,0,0) & (1,0,0) share 4 vertices, all on x=0.5 face
Inter-cube bushings: 392 / Sphere-BC bushings: 72 / Total bushings: 464
OK body count=45, total bushings=464
step 0..150: max|v| 2.3e-3 .. 9.4e-4   (200 steps, no NaN/Inf)
Smoke test PASSED — ADMM accepted bushing K/C; no NaN/Inf.
```

### Runtime note — ADMM cost of 464 bushings (CRITICAL)

Profiled the per-step ADMM cost (chrono env, `dt = 1e-4`):

| SetMaxIterations | ms / step | 0.5 s smoke (5000 steps) | 10 s seed |
|------------------|-----------|--------------------------|-----------|
| 150 (as shipped) | ~6960     | ~9.7 h                   | ~190 h    |
| 50               | ~3790     | ~5.3 h                   | ~105 h    |
| 30 (oct value)   | ~3300     | ~4.6 h                   | ~92 h     |
| 20               | ~2190     | ~3.0 h                   | ~61 h     |
| 15               | ~1470     | ~2.0 h                   | ~41 h     |

The per-step cost is fixed across steps (no factorization warm-up speedup) —
ADMM re-assembles and re-factors the 464-bushing stiffness system every step.
Lowering iterations 150→15 only cuts step time ~4.7×, because the dominant
cost is matrix assembly/factorization (which scales with the 464 dense
bushing blocks), not the iteration loop. The cuboid has 6.4× the octahedron's
bushings; the octahedron at 72 bushings / `MaxIter(30)` took ~142 min/seed,
so the cuboid at 464 bushings lands a single 10 s seed in the tens-of-hours
range no matter the iteration count.

Integration is **stable**: 200+ consecutive steps logged with no NaN/Inf,
bounded velocities ~1-3e-3 m/s, ADMM converging. The slowness is a wall-clock
cost, never a correctness problem (force elements add 0 rows to Φ_q).

**`SetMaxIterations` was NOT changed.** That line lives in `build_system`'s
solver-setup block, which the brief's "Must NOT touch" list assigns to
physics_cuboid; the brief says to *coordinate* rather than edit a
physics_cuboid-owned line. Recommended follow-up for physics_cuboid /
orchestrator: lower the ADMM `SetMaxIterations` for the cuboid (e.g. to 15-20)
before the deferred full 5-seed sweep, and budget tens of hours per seed.

Checkpoint 4 was run end-to-end through `run_single` (full code path: build →
step loop → damping → CSV export → early-stop checks) at a reduced
`duration` so it completes within the session — the physics and code path are
identical to the 0.5 s case, only the step count differs. Result recorded
below once the run finishes.

### Task 7 — Full 5-seed baseline sweep
**Deferred per scope override** ("build code + smoke test" mode). The
production 5×10 s sweep is NOT run in this session — it would take tens of
hours per seed (see Runtime note). Code is written so a future sweep works:
`run_sweep()` / `STIFFNESS_VARIANTS` / `CHANGE2_VARIANTS` are intact and the
464-bushing build is validated.

### Files created / modified
- `src/simulation_cuboid.py` — filled `cuboid_vertices`, `cuboid_inertia`,
  `_make_cuboid_body`, `_find_shared_vertices`; finalized grid-constant
  comments; wired the `build_system` joint section (inter-cube + quad-bushing
  sphere-BC). No physics_cuboid-owned symbol touched.
- `docs/joint_topology_cuboid.md` — created.
- `tests/smoke_bushing.py`, `tests/rest_state_bushing.py` — created.
- `progress/structure_cuboid.md` — this file.

## Redesign: open strut frame (replaces face-to-face block)

The face-to-face packing (SPACING = EDGE_LENGTH; every coincident corner welded
→ 392 inter-cube + 72 BC = 464 bushings) was a rigid block: 4+ bushings per
shared face removed all relative rotation, so it could not buckle — every run
sat at ~0.0 deg tilt (the 3 h sweep reached 0.2 s sim time, max tilt 0.000 deg).

Rebuilt as an OPEN strut frame, the cube analogue of the octahedron lattice:
- SPACING = EDGE_LENGTH * sqrt(2) (0.414*a gap between faces; cubes no longer touch).
- `_find_shared_vertices` replaced by `_axis_neighbor_couplings`: ONE bushing per
  axis-neighbour pair, anchored at the inter-cube gap midpoint on the centre-line
  -> 54 inter-cube couplings (18 per axis).
- Sphere BCs: 4 corner bushings -> 1 bushing per sphere at the face centre -> 18.
- Total 54 + 18 = 72 couplings, identical to the octahedron lattice.
- Cube-cube collision kept on; K=1e4, C=1e2, ADMM SetMaxIterations(30) unchanged.

Verification: smoke test passes (45 bodies, 54/18/72, ADMM OK, no NaN). A 0.255 s
solo run at F_top=20 N drove max cube tilt 0.00 -> 18.5 deg (past the 10 deg
gate); rendered front view shows the upper layers shearing/leaning. With 72 vs
464 bushings, steps are ~6x cheaper, so runs reach buckling far inside the 3 h cap.

## 2026-06-01 — Diamond reorientation (front-view /\/\ lattice)

The axis-aligned open frame read as a square block with sphere "caps"; switched
the cube presentation to the octahedron-style diamond lattice the project is
meant to mirror.

- `build_system` now spins every cube **45° about +Y** (`chrono.QuatFromAngleY`,
  module const `DIAMOND_TILT_RAD`) → each cube's xz profile is a diamond, tips
  meeting tip-to-tip at the existing `SPACING = a√2`. Front view is now the
  `/\/\` // `\/\/` argyle pattern.
- Ground plane lowered from `−a/2` to `−HALF_DIAGONAL = −a/√2` (the diamond
  bottom tip) so the bottom row rests on its tips, mirroring the octahedron
  resting on its bottom apex. Sphere placement and the 72 bushings are unchanged
  (caps kept per user request).
- Output cleared. New still `scripts/render_structure.py` renders the rest-state
  diamond lattice to `output/structure_frame.png` via `visualizer.render_frame`
  (same style as the collapse MP4s). Verified: clean 3×3 diamond grid, tip-to-tip.

Deferred: the F = 20 N collapse sweep ("diamonds → filled rectangle") is the
multi-hour run and is NOT launched here. Before it runs, the tilt metric in
`run_single` must measure rotation *relative to* the 45° start — a 45° rest pose
currently reads as 45° tilt, which would trip the buckling/equilibrium gate at
t = 0.
