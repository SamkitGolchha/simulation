"""Single-seed driver for the cuboid 20 N regime.

Reads STIFFNESS_VARIANTS[0] (the validated config) and runs run_single() on
the requested seed. Output: output/<label>/sim_<seed>.csv.

Usage:
    OMP_NUM_THREADS=1 \
      DYLD_INSERT_LIBRARIES=$CONDA_PREFIX/lib/libomp.dylib \
      $CONDA_PREFIX/bin/python scripts/run_seed.py --seed N
"""
from __future__ import annotations

import argparse
import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument(
        "--budget",
        type=float,
        default=None,
        help="Per-process wall-clock cap in seconds (default: module "
        "WALL_CLOCK_BUDGET_S = 3 h). Run ends on KE equilibrium, column "
        "collapse, or this budget, whichever fires first.",
    )
    args = parser.parse_args()

    sys.path.insert(0, PROJECT_ROOT)
    from src.simulation_cuboid import (  # noqa: E402
        STIFFNESS_VARIANTS,
        WALL_CLOCK_BUDGET_S,
        run_single,
    )

    budget = args.budget if args.budget is not None else WALL_CLOCK_BUDGET_S
    label, k, c, f_top, p_mag = STIFFNESS_VARIANTS[0]
    out_dir = os.path.join(PROJECT_ROOT, "output", label)
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, f"sim_{args.seed:03d}.csv")

    print(
        f"[seed={args.seed}] {label}: K={k} C={c} F={f_top}N "
        f"perturb={p_mag}rad/s budget={budget / 3600:.2f}h -> {csv_path}",
        flush=True,
    )
    t0 = time.time()
    result_path, joint_count = run_single(
        seed=args.seed,
        csv_path=csv_path,
        bushing_k=k,
        bushing_c=c,
        top_force_n=f_top,
        perturb_mag=p_mag,
        wall_clock_budget_s=budget,
    )
    wall = time.time() - t0
    print(
        f"[seed={args.seed}] done in {wall:.1f}s ({wall / 3600:.2f} h); "
        f"joints={joint_count}; csv={result_path}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
