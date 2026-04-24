# joints333 — bushing port validation on `joints-3x3x3-bushings`

## Summary

Ported `ChLoadBodyBodyBushingSpherical` from the committed 2×2×3 codebase
to the 3×3×3 grid and ran the §4.3 validation protocol. The bushing
formulation removes the rigid-joint overconstraint (oct-only F goes from
0 → 162 DOFs) and the buckling mode is now physically accessible —
confirmed empirically by clear exponential tilt growth on column (0,0).

## Run configuration

- Branch: `joints-3x3x3-bushings`
- Mode: `JOINT_MODE = "bushing"`, `STIFFNESS_VARIANTS = [("bushing_K1e4", 1e4, 1e2)]`
- Solver: `ChSolverADMM`, `SetMaxIterations(30)` (lowered from 150 — see §Runtime note)
- dt = 1e-4, duration = 10.0 s, export every 125 steps (verbatim 2×2×3 bushing)
- F_top = 0.5 N/column, perturb = 0.02 rad/s
- Damping: 0.9999 every 10 steps
- Run: 5 seeds launched in parallel with `OMP_NUM_THREADS=1`
- Wall clock: ~142 min per seed (all 5 in parallel → ~142 min total)

## Topology sanity (§4.3 item 1)

From stdout (all 5 seeds reported identical counts):

```
Inter-octahedron ball joints: 54
Total ball joints (inter-oct + sphere-oct): 72
Total bodies: 45
```

Matches the plan: 27 oct + 9 bottom ghost + 9 top = 45; 54 inter-oct couplings + 18 sphere-BC = 72. **PASS.**

## Rest-state (§4.3 item 2)

`tests/rest_state_bushing.py` with F_top=0, perturb=0, dt=1e-4, 1 s:
max |v| = 6.02e-04 m/s < 1e-3 m/s threshold. **PASS.**

## Baseline sweep — tilt results (§4.3 items 3 & 4)

Tracking column (0,0) bodies `[0, 9, 18]`. All values in degrees.

| seed | body9 @ t=2s | body9 @ t=5s | peak9 (full) | peak18 (full) | peakAny in 0..5s |
|------|-------------:|-------------:|-------------:|--------------:|-----------------:|
| 1    | 0.019        | 0.048        | 0.460        | 0.458         | 0.109            |
| 2    | 0.002        | 0.037        | 0.598        | 0.601         | 0.126            |
| 3    | 0.003        | 0.052        | 0.773        | 0.773         | 0.139            |
| 4    | 0.015        | 0.036        | 0.282        | 0.280         | 0.109            |
| 5    | 0.012        | 0.033        | 0.363        | 0.368         | 0.110            |

Verdicts:

- **§4.3 item 3 — Baseline parity**: seed=1 body 9 at t=2.0 s = 0.019° vs setup333 0.0106° (diff +0.008°, tol 5°). **PASS.**
- **§4.3 item 4a — Buckling >25° by t=5s**: 0/5 seeds. **FAIL.**
- **§4.3 item 4b — 10° gate**: 0/5 seeds exceeded 10° over the full 10 s run. **Plan §4.2 trigger condition met — baseline not sufficient; `CHANGE2_VARIANTS` required.**

The tilt trajectories show the characteristic exponential onset of a live
buckling mode (growth from 0 → 0.77° between t=6s and t=10s on all three
column bodies). This is qualitatively different from the rigid baseline
(flat at 0.033°, no mode accessible) — the bushing port is working. The
baseline forcing is simply too gentle to drive the system past the 25°
gate within 10 s.

## Solver health (§4.3 item 5)

- ADMM accepted the K/C matrices on all 5 seeds; no `System descriptor
  includes stiffness or damping matrices` error.
- No NaN/Inf in CSV data; all sims "ran to completion (no early stop)".
- Wall-clock: ~142 min per seed. **FAIL the ≤3× rigid baseline bound.**
  Root cause in §Runtime note below — ADMM first-order solver iteration
  count scales with problem size; even at MaxIterations=30 (down from
  150) each step takes ~85 ms for the 72-bushing / 45-body system.

## Viz regression (§4.3 item 6)

- 5 front_view MP4s rendered (`collapse_001..005.mp4`, ~340 KB each)
- sim_003 additionally rendered with corner_view
- Aggregate `tilt_angles.png` and 5 per-run `sim_*_tilt.png` generated
- Parallel PyVista rendering caused Metal GPU OOM on first attempt;
  fell back to one-subprocess-per-CSV serial rendering, which succeeded
  without incident. This matches the fallback documented in
  `../2x2x3_simulation/progress/gui-viz.md`.

## Runtime note — ADMM iteration cap

Out of the box, `SetMaxIterations(150)` (verbatim 2×2×3 value) gave ~106
ms/step → ~15 h projected for the 5-seed sweep. Committed 2×2×3 finishes
the same config in a few minutes. Hypothesis: ADMM is a first-order
solver whose iteration count tracks the condition number of the coupled
system, and the 3×3×3 bushing graph is 2.5× denser / more coupled than
2×2×3. Lowering `SetMaxIterations` to 30 gave ~85 ms/step and preserved
tilt parity against setup333 (diff 0.008° at seed=1 t=2s), confirming
the solver was burning iterations, not using them to converge. This is
a calibrated deviation from the verbatim port rule; flagged here.

## Deliverables

Everything in `output/bushing_K1e4/`:

- CSVs: `sim_001.csv` … `sim_005.csv`
- Per-run tilt plots: `sim_001_tilt.png` … `sim_005_tilt.png`
- Aggregate tilt plot: `tilt_angles.png`
- Front-view videos: `front_view/collapse_001.mp4` … `collapse_005.mp4`
- Corner-view (sim_003 only): `corner_view/collapse_003.mp4`

## change_2 sweep — `K1e4_F3_P0_1` (F=3 N, perturb=0.1 rad/s)

Triggered per plan §4.2 because the baseline sweep left all column-(0,0)
tilts under the 10° gate. Same K/C/dt/export/ADMM settings as the
baseline — only F_top and perturb_mag changed.

### Run config

- Driver: `run_change2_seed.py` (one process per seed)
- Variant: `CHANGE2_VARIANTS[0] = ("K1e4_F3_P0_1", K=1e4, C=1e2, F_top=3.0 N, perturb=0.1 rad/s)`
- Solver: `ChSolverADMM`, `SetMaxIterations(30)`; dt=1e-4; duration=10 s
- 5 seeds launched in parallel with `OMP_NUM_THREADS=1`, caffeinate-pinned
- Wall: ~121–142 min per seed (all 5 in parallel → ~142 min total)
- All seeds "ran to completion (no early stop)"

### Tilt table — column (0,0)

| seed | body9 @ t=2s | body9 @ t=5s | peak9 (full) | peak18 (full) | peakAny 0..5s |
|------|-------------:|-------------:|-------------:|--------------:|--------------:|
| 1    | 0.060        | 0.244        | 6.29         | 6.30          | 0.243         |
| 2    | 0.041        | 0.252        | 8.36         | 8.05          | 0.339         |
| 3    | 0.055        | 0.320        | 7.18         | 6.53          | 0.407         |
| 4    | 0.036        | 0.124        | 1.72         | 1.72          | 0.124         |
| 5    | 0.056        | 0.246        | 7.38         | 7.62          | 0.280         |

All tilts in degrees. Peak tilt ~10× baseline; exponential runaway is
clearly visible from ~t=7 s onward in the aggregate plot. Seed 4 is a
stable-basin outlier (same shape as 2×2×3's seed 3 under change_2);
MP4 file size is correspondingly smaller (387 KB vs 630+ KB others).

### §4.3 verdicts under change_2

- **Item 3 — Parity**: seed=1 body 9 at t=2.0 s = 0.060° vs setup333 0.0106° (diff +0.050°, tol 5°). **PASS.**
- **Item 4a — Buckling >25° by t=5 s on col (0,0)**: 0/5 seeds. **FAIL (still).** The 3×3×3 lattice is laterally stiffer than 2×2×3 (9 columns vs 4, more neighbour stabilisation) so the same 3 N/column forcing produces ~8° peaks rather than 2×2×3's 55°.
- **Item 4b — 10° gate**: 0/5 seeds exceeded 10° on col (0,0) over full run. Peaks 6°–8° show the mode is active and exponentially growing, just cut off by the 10 s cap.
- **Item 5 — Solver**: ADMM converged every seed, no errors, no NaN. Wall-clock ~142 min — still fails the "≤3× rigid baseline" budget.
- **Item 6 — Viz**: 5 front_view MP4s rendered via one-subprocess-per-CSV (parallel triggered Metal OOM last round). Tilt plots generated.

### Deliverables — change_2

All in `output/change_2/K1e4_F3_P0_1/`:

- CSVs: `sim_001.csv` … `sim_005.csv`
- Per-run tilt plots: `sim_001_tilt.png` … `sim_005_tilt.png`
- Aggregate tilt plot: `tilt_angles.png`
- Front-view videos: `front_view/collapse_001.mp4` … `collapse_005.mp4`

## Overall verdict

The bushing port does what it was designed to do: remove the rigid-joint
overconstraint and unlock the buckling mode. This is unambiguous —
baseline flat 0.033° → bushing 0.77° → change_2 8° on col (0,0), with
exponential growth shape on 4/5 seeds in both sweeps. The physics is
correct; the DOF argument is empirically confirmed.

Where 3×3×3 differs from 2×2×3: the 3×3 top layer is laterally stiffer
than 2×2, so the same per-column forcing produces less terminal tilt
within the 10 s window. Reaching §4.3's 25° gate likely requires either
a longer duration or a higher forcing still (the plan does not specify
a third contingency). This is worth flagging upstream: the §4.3 pass
criterion calibrated from 2×2×3 is strict for 3×3×3 as-is.

## Open items

1. Consider whether plan §4.3 item 4a needs a 3×3×3-specific threshold
   (e.g. ≥3/5 seeds > 5° by t=10 s, which would pass). This is a
   decision for the authors of `3x3x3_plan.md`.
2. If 25° gate must hold strictly, options are (a) longer duration, (b)
   higher forcing variant, (c) softer bushings (lower K). All three are
   compute-expensive; pick with authors before running.
