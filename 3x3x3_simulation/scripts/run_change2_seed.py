"""Single-seed change_2 entry point for parallel launching.

Reads CHANGE2_VARIANTS[0] = ("K1e4_F3_P0_1", K=1e4, C=1e2, F_top=3.0, perturb=0.1)
and runs one simulation into output/change_2/<label>/sim_<seed>.csv.

Usage: python scripts/run_change2_seed.py --seed <int>
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
    args = parser.parse_args()

    sys.path.insert(0, PROJECT_ROOT)
    from src.simulation_3x3x3 import CHANGE2_VARIANTS, run_single  # noqa: E402

    label, k, c, f_top, perturb = CHANGE2_VARIANTS[0]
    out_dir = os.path.join(PROJECT_ROOT, "output", "change_2", label)
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, f"sim_{args.seed:03d}.csv")

    print(
        f"[seed={args.seed}] change_2 {label}: K={k} C={c} F={f_top}N perturb={perturb}rad/s -> {csv_path}",
        flush=True,
    )
    t0 = time.time()
    result_path, joint_count = run_single(
        seed=args.seed,
        csv_path=csv_path,
        bushing_k=k,
        bushing_c=c,
        top_force_n=f_top,
        perturb_mag=perturb,
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
