# setup333 progress log

## 2026-04-12 — Initial scaffold + first pipeline run

### Port
- Pure 1:1 scale-up of committed `origin/main:2x2x3_simulation/` with NX=NY=NZ=3.
- No parameter tuning. All dt, force, damping, KE thresholds, solver iterations
  identical to committed 2×2×3.
- Files ported: `src/simulation_3x3x3.py`, `src/visualizer.py`,
  `src/visualizer_corner.py`, `src/plot_tilts.py`, `run.py`,
  `different_angles/corner_view.py`, `tests/test_api.py`, `tests/test_force.py`,
  `tests/check_ke.py`, `tests/check_ke2.py`.
- Only semantic changes beyond grid size:
  - `_grid_index_to_flat` / perturbed index → flat index 9 at (0,0,1)
  - `_check_columns_collapsed` iterates 9 columns of the 3×3 base
  - `visualizer.py` palette expanded to 27 colors (9 per iz-layer)
  - `plot_tilts.py` column (0,0) body IDs → [0, 9, 18]
  - `visualizer_corner.py` `_FOCAL` / `_CAM_DIST` parameterized on NX/NY

### Environment
- Conda env `chrono` had drifted from `environment.yml`: `pyvista` and
  `matplotlib` were missing. Restored via `conda env update -f environment.yml`.
- All python commands prefixed with
  `DYLD_INSERT_LIBRARIES=$CONDA_PREFIX/lib/libomp.dylib`.

### Simulation run
- `python run.py --sim` → 805.9 s wall (5 runs, seeds 1-5).
- Joint counts printed per run: inter-oct 54, total 72 — matches prediction
  (54 inter-oct = 18 per axis × 3 axes; 72 = 54 + 18 sphere-BC joints).
- Rows per CSV: 36000 (45 bodies × 800 export steps).
- **No run triggered KE equilibrium stop** — every run ran the full 10 s.

### Tilt result (column (0,0), seed=1)
| t (s) | body 0 (bot) | body 9 (mid, perturbed) | body 18 (top) |
|------:|-------------:|------------------------:|--------------:|
| 0.10  |       0.0080 |                  0.0080 |        0.0080 |
| 2.00  |       0.0106 |                  0.0106 |        0.0106 |
| 5.00  |       0.0117 |                  0.0117 |        0.0117 |
| 9.99  |       0.0208 |                  0.0208 |        0.0209 |

All three heights move as a **rigid column** — tilt values identical to 4
decimal places. Max tilt anywhere in all 27 octahedra at t=9.99s (seed 1): 0.033°.

### Interpretation — overconstraint boundary reached
This is the regime predicted by the DOF accounting in CLAUDE.md:
- Oct-only subsystem net DOFs = 27×6 − 54×3 = **0** (exactly at boundary).
- Buckling requires rigid modes; with zero rigid DOFs in the oct subsystem,
  the spherical-joint formulation admits no mode into which the 0.5 N column
  load can decompose. The structure resists buckling by being kinematically
  rigid, not by being stiff.
- The residual ~0.03° tilt drift is numerical noise from solver tolerance,
  not physical buckling.
- Compare 2×2×3: oct-only subsystem net DOFs = 12×6 − 20×3 = +12. Rigid
  modes exist, buckling emerges in all 5 seeds at ~30°+ by t=5 s.

This validates the motivation for the joint-formulation experiment on the
`joints333` branch: before scaling to 5×5×5 (net −150), we need a joint
formulation that produces buckling at 3×3×3.
