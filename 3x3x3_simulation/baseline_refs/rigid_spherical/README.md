## Rigid ChLinkLockSpherical baseline

Outputs from `setup333`'s 2026-04-12 run with `JOINT_MODE = "spherical"` (rigid
ball joints, the published evidence for F_oct = 0 at 3x3x3 — all 27 octahedra
translate as a rigid column under load; max tilt 0.033° on column (0,0)).

- `sim_001.csv` — seed=1 run, used as the parity reference for DoD gate 3.
  Compared by joints333 against the bushing-mode seed=1 run at t = 2.0 s on
  column (0,0) body IDs [0, 9, 18].
- `tilt_angles.png` — aggregate 5-seed tilt plot. Expected "before" panel in
  the progress/joints333.md before/after comparison.

Source run log: ../../progress/setup333.md. Full CSV set (seeds 2-5) is in
../../output/ (gitignored) and on setup333's local workspace.
