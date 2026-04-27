"""Change_3 single-seed driver: F_top=20 N, perturb=0.1 rad/s.

Force escalated from 6 N → 20 N because at F=6 N the 3x3x3 lattice
plateaued at 10-13 deg tilt (change_2 hit 8 deg at F=3 N). Linear
scaling suggested F~20 N to clear the 45 deg gate. KE early-stop
threshold raised to 45 deg in simulation_3x3x3.py so the sim only
declares "settled" after a real collapse.

Output: output/Change_3/20N_force/sim_<seed>.csv

Usage: python scripts/run_change3_seed.py --seed <int>
"""

from __future__ import annotations

import argparse
import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

LABEL = "20N_force"
K = 1.0e4
C = 1.0e2
F_TOP = 20.0
PERTURB = 0.1
DURATION = 15.0  # s; with 20 N collapse should onset by t~3 s


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()

    sys.path.insert(0, PROJECT_ROOT)
    from src.simulation_3x3x3 import run_single  # noqa: E402

    out_dir = os.path.join(PROJECT_ROOT, "output", "Change_3", LABEL)
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, f"sim_{args.seed:03d}.csv")

    print(
        f"[seed={args.seed}] Change_3 {LABEL}: K={K} C={C} F={F_TOP}N "
        f"perturb={PERTURB}rad/s duration={DURATION}s -> {csv_path}",
        flush=True,
    )
    t0 = time.time()
    result_path, joint_count = run_single(
        seed=args.seed,
        csv_path=csv_path,
        bushing_k=K,
        bushing_c=C,
        top_force_n=F_TOP,
        perturb_mag=PERTURB,
        duration=DURATION,
    )
    wall = time.time() - t0
    print(
        f"[seed={args.seed}] done in {wall:.1f}s ({wall/60:.1f} min); "
        f"joints={joint_count}; csv={result_path}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
