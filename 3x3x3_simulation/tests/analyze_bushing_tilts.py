"""Analyze bushing-mode CSVs for joints333 §4.3 items 3 & 4.

Given the CSV directory (default: 3x3x3_simulation/output/bushing_K1e4), for
each sim_*.csv:
  - Compute tilt angle (deg) = acos(1 - 2(q1^2 + q2^2)) in degrees for every
    row, for the column (0,0) body IDs [0, 9, 18].
  - Report, per seed: tilt at t=2.0 s on the perturbed octahedron (body 9),
    peak tilt on body 9 and body 18 over 0..5 s, peak tilt over full run.
  - Summary: §4.3 verdicts.

Usage:
    conda run -n chrono python tests/analyze_bushing_tilts.py [csv_dir]

Defaults to output/bushing_K1e4 under the project root.
"""

from __future__ import annotations

import csv
import math
import os
import sys
from collections import defaultdict

COLUMN_00_BODY_IDS = [0, 9, 18]  # bot/mid(perturbed)/top octahedra on col (0,0)
SETUP333_SEED1_T2 = 0.0106       # deg, from progress/setup333.md
PARITY_TOL_DEG = 5.0
BUCKLING_GATE_DEG = 25.0          # §4.3 item 4
TILT_10DEG_GATE = 10.0           # §4.2 change_2 trigger gate
T_PARITY = 2.0
T_BUCKLING = 5.0


def _tilt_deg_from_quat(q1: float, q2: float) -> float:
    z_z = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    z_z = max(-1.0, min(1.0, z_z))
    return math.degrees(math.acos(z_z))


def _load_csv(path: str):
    """Return dict body_id -> list of (t, tilt_deg) sorted by time."""
    series: dict[int, list[tuple[float, float]]] = defaultdict(list)
    with open(path) as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            bid = int(row["body_id"])
            if bid not in COLUMN_00_BODY_IDS:
                continue
            t = float(row["time"])
            q1 = float(row["q1"])
            q2 = float(row["q2"])
            series[bid].append((t, _tilt_deg_from_quat(q1, q2)))
    for bid in series:
        series[bid].sort()
    return series


def _nearest(series: list[tuple[float, float]], target_t: float) -> float:
    if not series:
        return float("nan")
    best = min(series, key=lambda x: abs(x[0] - target_t))
    return best[1]


def _peak_within(series: list[tuple[float, float]], t_end: float) -> float:
    vals = [v for t, v in series if t <= t_end]
    return max(vals) if vals else float("nan")


def analyze_dir(csv_dir: str) -> int:
    csvs = sorted(p for p in os.listdir(csv_dir) if p.startswith("sim_") and p.endswith(".csv"))
    if not csvs:
        print(f"No sim_*.csv files in {csv_dir}")
        return 2

    print(f"Analyzing {len(csvs)} CSVs in {csv_dir}\n")
    print(f"{'seed':>5} {'body9@2s':>10} {'body9@5s':>10} {'peak9(full)':>12} "
          f"{'peak18(full)':>13} {'peakAny(5s)':>12}")
    print("-" * 72)

    seeds_buckle_25 = 0
    seeds_over_10 = 0
    rows_for_seeds: dict[int, dict] = {}
    for name in csvs:
        seed = int(name.split("_")[1].split(".")[0])
        path = os.path.join(csv_dir, name)
        series = _load_csv(path)

        b9_t2 = _nearest(series.get(9, []), T_PARITY)
        b9_t5 = _nearest(series.get(9, []), T_BUCKLING)
        peak9_full = _peak_within(series.get(9, []), float("inf"))
        peak18_full = _peak_within(series.get(18, []), float("inf"))
        peak_any_5s = max(
            _peak_within(series.get(bid, []), T_BUCKLING) for bid in COLUMN_00_BODY_IDS
        )

        rows_for_seeds[seed] = {
            "b9_t2": b9_t2, "b9_t5": b9_t5,
            "peak9_full": peak9_full, "peak18_full": peak18_full,
            "peak_any_5s": peak_any_5s,
        }
        if peak_any_5s > BUCKLING_GATE_DEG:
            seeds_buckle_25 += 1
        if peak9_full > TILT_10DEG_GATE or peak18_full > TILT_10DEG_GATE:
            seeds_over_10 += 1

        print(f"{seed:>5} {b9_t2:>10.3f} {b9_t5:>10.3f} {peak9_full:>12.3f} "
              f"{peak18_full:>13.3f} {peak_any_5s:>12.3f}")

    print()
    print("§4.3 verdicts (column (0,0), body IDs [0, 9, 18]):")

    seed1 = rows_for_seeds.get(1, {})
    if seed1:
        diff = seed1["b9_t2"] - SETUP333_SEED1_T2
        parity_ok = abs(diff) <= PARITY_TOL_DEG
        print(f"  3. Baseline parity: seed=1 body 9 at t=2.0 s = {seed1['b9_t2']:.3f}° vs setup333 0.0106°"
              f" (diff {diff:+.3f}°) -> {'PASS' if parity_ok else 'FAIL'} (tol {PARITY_TOL_DEG}°)")

    buckling_ok = seeds_buckle_25 >= 3
    print(f"  4a. Buckling (>{BUCKLING_GATE_DEG}° by t={T_BUCKLING} s on col (0,0)):"
          f" {seeds_buckle_25}/{len(csvs)} seeds -> {'PASS' if buckling_ok else 'FAIL — trigger change_2'}")
    print(f"  4b. 10° gate trigger check (peak body 9 or 18 over full run):"
          f" {seeds_over_10}/{len(csvs)} seeds exceeded 10° — if 0, change_2 contingency is required")

    return 0 if buckling_ok else 1


def main() -> int:
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_dir = os.path.join(project_root, "output", "bushing_K1e4")
    csv_dir = sys.argv[1] if len(sys.argv) > 1 else default_dir
    return analyze_dir(csv_dir)


if __name__ == "__main__":
    raise SystemExit(main())
