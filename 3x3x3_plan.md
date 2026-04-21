# Plan: 3×3×3 octahedral lattice simulation

**Purpose**: stand up a `3x3x3_simulation/` subproject and use it to (i) publish the empirical evidence that the **rigid `ChLinkLockSpherical` formulation fails at 3×3×3** — oct-only subsystem DOF hits zero, all 27 octahedra translate as a rigid column under load — and (ii) validate the **compliant spherical bushing formulation** (`ChLoadBodyBodyBushingSpherical`) that was proven in `2x2x3_simulation/` on 2026-04-21. Bushings contribute zero rows to the constraint Jacobian Φ_q, so F_bushing = 6B identically at any grid size; this is the path that unblocks 5×5×5.

See `findings/ball_joint_scaling_analysis.docx` (2026-04-21) for the DOF accounting, the rigid-joint failure table across 2×2×3 / 3×3×3 / 4×4×4 / 5×5×5, and the 2×2×3 change_2 evidence that bushings carry load (12 mm F/K settle) and admit rotational freedom (55° peaks on 4/5 seeds).

Two agents execute the plan:
- `setup333` — preserves the **rigid-spherical baseline on `main`** (already shipped; the "rigid column, max tilt 0.033°" result is the published evidence for F_oct = 0).
- `joints333` — on a dedicated branch, **ports the committed 2×2×3 bushing codebase to 3×3×3** behind a module-level `JOINT_MODE` flag and runs the validation protocol in §4.3. **The formulation is no longer open-ended: it is the `ChLoadBodyBodyBushingSpherical` formulation committed in `origin/main:2x2x3_simulation/src/simulation_2x2x3.py`.**

**Constraints**:
- Do NOT touch `2x2x3_simulation/`, `Sanity_Test/`, or `5x5x5_simulation/` (once created).
- All work for the new joint experiment happens on a git branch — do not commit to `main`.
- Use `conda run -n chrono python …`. Apple M5 Pro requires libomp preload (see `scale_plan.md` §1) — ship `src/_omp_bootstrap.py` and import it first in every entry point.

---

## 1. Scaling numbers — 3×3×3 vs 2×2×3

| Quantity | 2×2×3 | **3×3×3** | 5×5×5 |
|---|---|---|---|
| Octahedra | 12 | **27** | 125 |
| Columns (bottom grid) | 4 | **9** | 25 |
| Layers | 3 | **3** | 5 |
| Bottom ghost spheres | 4 | 9 | 25 |
| Top loading spheres | 4 | 9 | 25 |
| **Total free bodies** | 20 | **45** | 175 |
| Grid-edge inter-oct couplings | 20 | **54** | 300 |
| Sphere↔oct couplings | 8 | 18 | 50 |
| **Total shared-vertex couplings** | 28 | **~72** | ~350 |
| Oct-only DOFs under rigid `ChLinkLockSpherical` (6·N_oct − 3·J_inter) | **+12** | **0** (boundary, no buckling mode) | **−150** (deep overconstraint, solver cannot close) |
| Oct-only DOFs under bushing `ChLoadBodyBodyBushingSpherical` (6·N_oct) | +72 | **+162** | +750 |

**Why 3×3×3 is the decisive testbed**: the rigid formulation's oct-only DOF count crosses zero exactly here. `setup333` ran the rigid baseline on 2026-04-12 and observed max tilt of 0.033° across all 27 octahedra (`progress/setup333.md`) — direct empirical confirmation that the rigid joint has no buckling mode left at this grid size. Any formulation that removes rank from Φ_q fixes this at every scale: the bushing contributes zero rows to Φ_q, so F_bushing = 6·B holds identically at 3×3×3, 4×4×4, and 5×5×5.

The 2×2×3 change_2 sweep (F_top = 3 N/col, perturb = 0.1 rad/s, K = 1e4 N/m, C = 1e2 N·s/m) produced peak tilts of 55.2–55.5° on 4/5 seeds with the expected 12 mm F/K static settle, confirming the bushing both carries load and admits rotation. Full memo: `findings/ball_joint_scaling_analysis.docx`.

Grid-edge count details: `x_edges = (NX−1)·NY·NZ = 18`, `y_edges = 18`, `z_edges = 18` → 54 interior pairs, each contributing exactly one shared vertex and thus one ball joint under the existing formulation.

Runtime budget (rough): ~2–3× the 2×2×3 wall-clock per run; rendering scales with body count in PyVista. Expect 5 full runs + MP4s + tilt plot in ~15–25 min total.

---

## 2. Environment

- `chrono` conda env is already correct on this Mac. No env.yml changes.
- Ship `src/_omp_bootstrap.py` (ctypes preload of `$CONDA_PREFIX/lib/libomp.dylib`, Darwin-only, no-op elsewhere).
- First line of every entry point (`run.py`, `src/simulation_3x3x3.py`, tests): `from src import _omp_bootstrap  # noqa` (or `from . import _omp_bootstrap` inside the package).
- Verify with: `conda run -n chrono python -c "from src import _omp_bootstrap; import pychrono as c; print(c.ChSystemNSC())"`.

---

## 3. Scaffolding work (pre-branch, runs on `main`)

A single agent (call it `setup333`) creates the subproject skeleton so the joint-experiment branch starts from a working baseline.

### 3.1 Port `simulation_2x2x3.py` → `simulation_3x3x3.py` with minimal changes

Parameterize grid dimensions and generalize the helpers:

- Module constants: `NX = NY = NZ = 3`.
- `_grid_index_to_flat(ix, iy, iz)` → `ix + NX*iy + NX*NY*iz`.
- Column list: `[(ix, iy) for iy in range(NY) for ix in range(NX)]` (9 columns, flat-row order).
- Perturbation target: interior octahedron `(0,0,1)`, flat index `0 + 3·0 + 9·1 = 9` (matches the shipped baseline in `src/simulation_3x3x3.py` and keeps the perturbed body on the plotted (0,0) column).
- `_check_columns_collapsed`: iterate over all `NX·NY` columns × `(NZ−1)` vertical pairs = 9×2 = 18 pair checks.
- Body-ID layout for CSVs (document in header):
  - `0..26` → octahedra (`oct_ixiyiz`)
  - `27..35` → bottom ghost spheres (`bot_sphere_ixiy`)
  - `36..44` → top loading spheres (`top_sphere_ixiy`)

### 3.2 Solver / integration parameters (baseline — joint branch may override)

Mid-point between 2×2×3 and 5×5×5 — the system is bigger but not overconstrained, so stay close to 2×2×3:

- `SetMaxIterations(200)` (up from 150).
- `dt = 5e-5` (unchanged from 2×2×3).
- `duration = 10.0 s` (unchanged).
- `export_interval = 400` (gives ~500 frames per run; CSV row count ≈ 45·500 = 22.5k, manageable).
- `F_top = 0.5 N` per column (unchanged — load scales naturally with 9 columns).
- Damping factor `0.9999` every 10 steps (unchanged).
- KE equilibrium threshold: `0.02 J` (mid between 2×2×3's 0.01 and 5×5×5's 0.05); `ke_consec_required = 40`.

### 3.3 Viz + plot ports

- `src/visualizer.py`: generalize `_OCT_COLORS` to a procedural 3-layer palette (9 octahedra per layer). Everything else (frame loop, camera reset, ffmpeg stitch) already drives off `body_kinds` and needs no changes.
- `src/plot_tilts.py`: target column = **(0,0)** (edge column, matching the shipped 2×2×3 and 3×3×3 baseline). 3 subplots (one per layer). Body IDs in column (0,0): `0+3·0+9·iz = 0, 9, 18`. Also port the `plot_single_run(csv_path, ...)` helper added in the 2×2×3 2026-04-21 update that writes per-run `<stem>_tilt.png` next to each CSV with a dashed 10° gate line and annotated peak tilt.
- `src/visualizer_corner.py` + `different_angles/corner_view.py`: port with identical palette changes.

### 3.4 `run.py`

Clone `2x2x3_simulation/run.py` exactly; only change module import (`src.simulation_3x3x3`) and update the "Pipeline finished" banner text.

### 3.5 Smoke test before handoff to the joint-experiment branch

- `conda run -n chrono python run.py --sim` with `n_runs=1`, `duration=2.0 s` → confirm build_system prints `Inter-octahedron ball joints: 54` and `Total ball joints (inter-oct + sphere-oct): 72`. Restore full params after.
- `conda run -n chrono python run.py --all` with full params → verify 5 CSVs + 5 MP4s + `tilt_angles.png` all land in `output/`.
- Commit scaffold to `main`. Tag: `3x3x3-baseline`.

---

## 4. Bushing-port branch (the execution path for the new vision)

Once `3x3x3-baseline` is tagged, `joints333` branches off:

```bash
git checkout -b joints-3x3x3-bushings
```

The goal of this branch is **not** open-ended experimentation — it is a direct port of the committed 2×2×3 bushing codebase to 3×3×3, hidden behind a `JOINT_MODE` flag so the rigid baseline stays reproducible in the same file.

### 4.1 Where the joint seam lives

Joint creation is concentrated in **three places** inside `src/simulation_3x3x3.py`. The port reuses all three:

1. **`_add_ball_joint(...)`** — the low-level factory. In the committed 3×3×3 baseline it returns `ChLinkLockSpherical`. The port replaces its body with the bushing factory from `origin/main:2x2x3_simulation/src/simulation_2x2x3.py:129–150` — takes a `ChLoadContainer`, `bodyA`, `bodyB`, `joint_pos`, `bushing_k`, `bushing_c`, and appends a `ChLoadBodyBodyBushingSpherical` (three isotropic translational springs, rotation free). Gate the replacement on `JOINT_MODE`.
2. **`_find_shared_vertices(...)`** — O(N²·36) naive scan returning `(idxA, idxB, vertex_pos)` triples. Unchanged — the bushing formulation couples at the same shared vertices as the rigid formulation. Topology is invariant (54 inter-oct + 18 sphere-BC = 72 couplings on 3×3×3).
3. **`build_system(...)`** — loops over shared-vertex output and calls `_add_ball_joint`. In `JOINT_MODE="bushing"` it must also: (a) construct a module-level `ChLoadContainer` and `system.Add(load_container)`; (b) pass the container + the active `(K, C)` from `STIFFNESS_VARIANTS` into each `_add_ball_joint` call; (c) swap `ChSolverPSOR` → **`ChSolverADMM`** — PSOR cannot handle the K/C matrices the bushing contributes and raises `System descriptor includes stiffness or damping matrices` at runtime.

Add a module-level `JOINT_MODE: Literal["spherical", "bushing"] = "bushing"` constant at the top of `simulation_3x3x3.py`. The constant is the single switch; `setup333`'s baseline behavior is reproduced by flipping it to `"spherical"` without any other edit.

### 4.2 The joint formulation — ChLoadBodyBodyBushingSpherical

Port verbatim from `origin/main:2x2x3_simulation/src/simulation_2x2x3.py` (bushing refactor committed 2026-04-21):

- **`STIFFNESS_VARIANTS`** (module scope): `[("bushing_K1e4", 1.0e4, 1.0e2)]` as the default single-variant sweep. `K = 1e4 N/m`, `C = 1e2 N·s/m` per bushing triad. Damping ratio `ζ = C / (2·√(mK)) = 0.5` (under-damped; quenches solver chatter, leaves buckling mode free). Static deflection `F/K = 3·10⁻⁴ m` under 3 N — three orders inside the 0.1 m `_check_columns_collapsed` tolerance, so bushing compression never spuriously triggers the stop.
- **`CHANGE2_VARIANTS`** (module scope): `[("K1e4_F3_P0_1", 1.0e4, 1.0e2, 3.0, 0.1)]`. Contingency sweep — same K, C, but F_top = 3 N/column and perturb = 0.1 rad/s. **Trigger rule** (verbatim from the 2×2×3 standing instruction): if the baseline run produces all tilts under the 10° gate and stabilising, keep the baseline MP4s, add `output/change_2/<label>/`, and re-run there. Expected behaviour at change_2 on 2×2×3: 4/5 seeds buckle to 55°, 1 stable basin (legitimate, not a solver bug).
- **Solver**: `ChSolverADMM` with `SetMaxIterations(150)` (unchanged from 2×2×3 change_2; raise to 200 only if ADMM reports convergence failures).

### 4.3 Validation protocol (all must pass before merge)

Run the branch on 3×3×3 in `JOINT_MODE="bushing"` and confirm ALL of these:

1. **Topology sanity**: `build_system` prints `body count = 45`, `inter-oct couplings = 54`, `sphere-BC couplings = 18`, `total shared-vertex couplings = 72`. Matches the spherical baseline on main; topology is invariant across the refactor.
2. **Rest state**: run with `F_top = 0` for 1 second → max body velocity < `1e-3 m/s` (structure in static equilibrium under gravity + ground contact; bushing holds the lattice together as a stiff spring network).
3. **Baseline parity on the easy case**: pick seed=1, compare the column (0,0) perturbed octahedron's tilt angle at `t = 2.0 s` against the `ChLinkLockSpherical` baseline from `setup333`'s main-branch run. Within 5° counts as "same regime" on the early transient.
4. **Buckling emerges**: at least **3 of 5 seeds** show the perturbed-octahedron tilt **> 25°** by `t = 5.0 s`, tracked and plotted on the column (0,0) body IDs `[0, 9, 18]`. If the baseline `F_top = 0.5 N` / `perturb = 0.02 rad/s` run produces all tilts under the 10° gate (as it did on 2×2×3 before change_2), fall back to `CHANGE2_VARIANTS` per the trigger rule in §4.2 and validate there.
5. **Solver health**: `ChSolverADMM` converges on every seed; no `System descriptor includes stiffness or damping matrices` error, no NaN/Inf in any CSV; wall-clock per run within 3× of the `setup333` rigid baseline.
6. **Viz regression**: `tilt_angles.png` (aggregate overlay, column (0,0)) renders without NaNs; `<stem>_tilt.png` per-run plots render with peak-tilt annotation; MP4s play end-to-end in both front_view and corner_view. If rendering exhausts Metal GPU memory on the second CSV (PyVista + Metal leak observed on 2×2×3), use the **one-subprocess-per-CSV** pattern documented in `2x2x3_simulation/progress/gui-viz.md` (2026-04-21 Joint Update).

Results logged to `progress/joints333.md` with before/after tilt plots side by side — the "rigid column, 0.033° max tilt" vs the bushing-formulation result.

### 4.4 What the bushing-port agent MUST NOT do

- Do not modify `_grid_index_to_flat`, the column list, the flat index layout, the perturbed octahedron index, or the CSV schema — downstream tools (plot_tilts, visualizer) depend on these being identical to the rigid baseline.
- Do not tune K, C, F_top, perturb, dt, or damping away from the values ported from the committed 2×2×3 codebase without logging the reason and the before/after tilt plots in `progress/joints333.md`.
- Do not reduce `duration` or `n_runs` for "faster iteration" in committed code — use a local untracked `run_smoke.py` if needed.
- Do not touch `2x2x3_simulation/`, `Sanity_Test/`, `5x5x5_simulation/`, or `environment.yml`.
- Do not rebase or squash history on the branch until validation is green (preserve clean bisection if something breaks).

---

## 5. File structure — `3x3x3_simulation/`

```
Research/Simulation/
└── 3x3x3_simulation/                        # NEW — mirrors 2x2x3 layout
    ├── CLAUDE.md                            # project overview, joint-experiment note, DOF math
    ├── run.py                               # --sim / --viz / --plot / --all
    ├── .claude/
    │   └── agents/
    │       ├── setup333.md                  # scaffold agent (baseline port, scoped to main)
    │       └── joints333.md                 # experiment agent (scoped to joint branch only)
    ├── src/
    │   ├── __init__.py
    │   ├── _omp_bootstrap.py                # ctypes libomp preload, darwin-only
    │   ├── simulation_3x3x3.py              # baseline on main; joint seam here
    │   ├── visualizer.py                    # 3-layer palette, 27 octahedra
    │   ├── visualizer_corner.py
    │   └── plot_tilts.py                    # center column (1,1), 3 subplots
    ├── different_angles/
    │   └── corner_view.py
    ├── tests/
    │   ├── test_api.py
    │   ├── test_force.py
    │   ├── check_ke.py
    │   └── check_ke2.py
    ├── view_change/
    ├── progress/
    │   ├── setup333.md                      # baseline port log
    │   └── joints_experiment.md             # joint branch log (validation results)
    └── output/                              # gitignored
        ├── sim_001.csv … sim_005.csv        # 45 bodies × ~500 timesteps
        ├── front_view/
        │   └── collapse_001.mp4 … collapse_005.mp4
        ├── corner_view/
        │   └── collapse_001.mp4 … collapse_005.mp4
        └── tilt_angles.png                  # 3 subplots, center column (1,1)
```

---

## 6. Sequencing

1. **Baseline already on `main`**: `setup333` scaffolded the tree, ran `--all`, and logged the rigid-column result (max tilt 0.033°). Tag: `3x3x3-baseline`. No further work required from `setup333` unless the committed 2×2×3 codebase changes and a re-sync is needed.
2. **Branch off**: `git checkout -b joints-3x3x3-bushings`.
3. **Bushing port**: `joints333` adds `JOINT_MODE` switch, ports `_add_ball_joint` / `STIFFNESS_VARIANTS` / `CHANGE2_VARIANTS` / ADMM solver swap from the committed 2×2×3 file, runs the §4.3 validation protocol (baseline first; fall back to change_2 per the trigger rule if tilts stay under the 10° gate), logs results in `progress/joints333.md`.
4. **Merge decision**: user reviews `progress/joints333.md` and the before/after artifacts. On green, the branch merges and `main` now ships `JOINT_MODE = "bushing"` as the default for 3×3×3.

---

## 7. Open questions for the user

1. ~~Which alternative joint formulation is the experiment targeting?~~ **Resolved 2026-04-21**: `ChLoadBodyBodyBushingSpherical`, port verbatim from `origin/main:2x2x3_simulation/src/simulation_2x2x3.py`. See `findings/ball_joint_scaling_analysis.docx` for the DOF argument and the 2×2×3 evidence.
2. Branch name: `joints-3x3x3-bushings`.
3. ~~Should `setup333` backport the 3×3×3 formulation as a grid-size parameter into a shared module?~~ **Resolved: fully self-contained.** `simulation_3x3x3.py` stays a straight port of `simulation_2x2x3.py` with `NX=NY=NZ=3` and no cross-subproject imports.
4. ~~Do you want the bushing branch to auto-apply to `5x5x5_simulation/`?~~ **Resolved: no.** See §8 for the successor-path gate — 5×5×5 activates only after §4.3 is green.

---

## 8. Successor path — 5×5×5 after `joints-3x3x3-bushings` merges

The findings memo explicitly recommends scaling **directly to 5×5×5** once 3×3×3 shows buckling on ≥3/5 seeds; the intermediate 4×4×4 step is no longer needed as a structural gate, because the bushing formulation removes the rank deficiency at every grid size (F_bushing = 6·B is identity, not an asymptote).

- **Gate**: `joints-3x3x3-bushings` must pass §4.3 and be merged to `main` before any 5×5×5 work begins.
- **Handoff**: `scale_plan.md` (untouched by this pass) becomes live. The `scaler555` agent inherits `JOINT_MODE = "bushing"` as the default formulation, `STIFFNESS_VARIANTS = [("bushing_K1e4", 1.0e4, 1.0e2)]`, and `ChSolverADMM` with `SetMaxIterations(300)`. `CHANGE2_VARIANTS` carries over with the same trigger rule. `scale_plan.md` will need an editorial pass to replace its rigid-joint language and rewrite the DOF table — that pass is explicitly **out of scope for this document** and happens after the 3×3×3 merge.
- **What does NOT need to change in `scale_plan.md`**: the libomp bootstrap, the grid parameterization, the body-ID layout, the render pipeline (including the one-subprocess-per-CSV pattern, which already matches the fix documented in `2x2x3_simulation/progress/gui-viz.md`).

Until §4.3 is green, no 5×5×5 work happens. On green, the 5×5×5 subproject is unblocked.
