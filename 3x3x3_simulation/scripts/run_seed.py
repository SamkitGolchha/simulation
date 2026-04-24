"""Single-seed bushing-sweep entry point for parallel launching.

Invoked once per seed (1..5) by the parallel driver. Reads the active
STIFFNESS_VARIANTS[0] triple and runs one simulation into
output/bushing_K1e4/sim_<seed>.csv.

Usage: python run_seed.py --seed <int>
"""

from __future__ import annotations

import argparse
import os
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument(
        "--out-dir",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "output", "bushing_K1e4"),
    )
    args = parser.parse_args()

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from src.simulation_3x3x3 import STIFFNESS_VARIANTS, run_single  # noqa: E402

    label, k, c = STIFFNESS_VARIANTS[0]
    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, f"sim_{args.seed:03d}.csv")

    print(
        f"[seed={args.seed}] {label}: K={k} C={c} -> {csv_path}",
        flush=True,
    )
    t0 = time.time()
    result_path, joint_count = run_single(
        seed=args.seed,
        csv_path=csv_path,
        bushing_k=k,
        bushing_c=c,
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
