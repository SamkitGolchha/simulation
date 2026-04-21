# Reads all sim_*.csv files from a directory and computes octahedron tilt angles.
# Plots tilt angles for the (0,0) column only: body_id 0 (iz=0), 9 (iz=1), 18 (iz=2).
# Output is saved as tilt_angles.png at 200 DPI in the output directory.

from __future__ import annotations

import csv
import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------------------------
# Column (0,0) body IDs and subplot titles for the 3x3x3 grid
# ---------------------------------------------------------------------------
_COLUMN_00_BODY_IDS: List[int] = [0, 9, 18]

_OCT_TITLES: Dict[int, str] = {
    0: "Octahedron (0,0,0) \u2014 bottom",
    9: "Octahedron (0,0,1) \u2014 middle",
    18: "Octahedron (0,0,2) \u2014 top",
}

# ---------------------------------------------------------------------------
# CSV parsing
# ---------------------------------------------------------------------------

RunData = Dict[int, Tuple[List[float], List[float]]]
# body_id -> (times, tilt_angles_deg)


def compute_tilt_angle(q0: float, q1: float, q2: float, q3: float) -> float:
    # Return the tilt angle in degrees from a quaternion using the Z-axis method.
    rot = Rotation.from_quat([q1, q2, q3, q0])  # scipy scalar-last: [qx,qy,qz,qw]
    z_world = rot.apply([0.0, 0.0, 1.0])
    cos_angle = float(np.clip(z_world[2], -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_angle)))


def parse_run_csv(csv_path: str) -> RunData:
    # Parse one sim CSV and return per-body tilt angle data for the (0,0) column only.
    target_ids = set(_COLUMN_00_BODY_IDS)
    raw: Dict[int, List[Tuple[float, float, float, float, float]]] = {}

    with open(csv_path, newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            bid = int(row["body_id"])
            if bid not in target_ids:
                continue
            t = float(row["time"])
            q0 = float(row["q0"])
            q1 = float(row["q1"])
            q2 = float(row["q2"])
            q3 = float(row["q3"])
            if bid not in raw:
                raw[bid] = []
            raw[bid].append((t, q0, q1, q2, q3))

    result: RunData = {}
    for bid, entries in raw.items():
        entries.sort(key=lambda e: e[0])
        times = [e[0] for e in entries]
        angles = [compute_tilt_angle(e[1], e[2], e[3], e[4]) for e in entries]
        result[bid] = (times, angles)

    return result


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------


def load_all_runs(output_dir: str) -> List[Tuple[str, RunData]]:
    # Glob sim_*.csv files in output_dir, sort by name, and parse each one.
    csv_files = sorted(Path(output_dir).glob("sim_*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No sim_*.csv files found in {output_dir!r}")
    runs: List[Tuple[str, RunData]] = []
    for path in csv_files:
        label = f"Run {len(runs) + 1}"
        print(f"  Parsing {path.name} as {label} ...")
        runs.append((label, parse_run_csv(str(path))))
    return runs


def make_colormap_colors(n: int) -> List[tuple]:
    # Generate n evenly-spaced colors from the 'tab10' colormap.
    cmap = plt.get_cmap("tab10")
    return [cmap(i / max(n - 1, 1)) for i in range(n)]


def plot_single_run(csv_path: str, out_path: str | None = None) -> str:
    # Plot tilt angles for the (0,0) column from one CSV; save alongside the CSV
    # as <stem>_tilt.png with a dashed 10° gate line and peak-tilt annotation.
    csv_path_abs = os.path.abspath(csv_path)
    run_data = parse_run_csv(csv_path_abs)

    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(12, 10), sharex=True)
    for subplot_idx, body_id in enumerate(_COLUMN_00_BODY_IDS):
        ax = axes[subplot_idx]
        ax.set_title(_OCT_TITLES[body_id], fontsize=11)
        ax.set_ylabel("Tilt Angle (\u00b0)", fontsize=10)
        ax.grid(True, linestyle="--", alpha=0.5)
        if body_id in run_data:
            times, angles = run_data[body_id]
            ax.plot(times, angles, color="tab:blue", linewidth=1.5)
            peak = max(angles) if angles else 0.0
            ax.axhline(10.0, color="tab:red", linestyle=":", linewidth=1.0, alpha=0.6)
            ax.text(
                0.99, 0.95, f"peak {peak:.2f}\u00b0",
                transform=ax.transAxes, ha="right", va="top", fontsize=9,
            )
    axes[-1].set_xlabel("Time (s)", fontsize=10)
    fig.suptitle(Path(csv_path_abs).stem, fontsize=12)
    fig.tight_layout()

    if out_path is None:
        out_path = os.path.splitext(csv_path_abs)[0] + "_tilt.png"
    fig.savefig(out_path, dpi=200)
    plt.close(fig)
    print(f"Saved {out_path}")
    return out_path


def plot_tilt_angles(output_dir: str) -> None:
    # Load all runs, write one per-run tilt PNG per CSV, then the aggregate overlay.
    output_dir_abs = os.path.abspath(output_dir)
    print(f"Loading CSVs from {output_dir_abs} ...")
    csv_files = sorted(Path(output_dir_abs).glob("sim_*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No sim_*.csv files found in {output_dir_abs!r}")

    runs: List[Tuple[str, RunData]] = []
    for path in csv_files:
        label = f"Run {len(runs) + 1}"
        print(f"  Parsing {path.name} as {label} ...")
        run_data = parse_run_csv(str(path))
        runs.append((label, run_data))
        plot_single_run(str(path))

    colors = make_colormap_colors(len(runs))

    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        figsize=(12, 10),
        sharex=True,
    )

    for subplot_idx, body_id in enumerate(_COLUMN_00_BODY_IDS):
        ax = axes[subplot_idx]
        ax.set_title(_OCT_TITLES[body_id], fontsize=11)
        ax.set_ylabel("Tilt Angle (\u00b0)", fontsize=10)
        ax.grid(True, linestyle="--", alpha=0.5)

        for run_idx, (label, run_data) in enumerate(runs):
            if body_id not in run_data:
                continue
            times, angles = run_data[body_id]
            ax.plot(
                times,
                angles,
                label=label,
                color=colors[run_idx],
                linewidth=1.5,
                alpha=0.85,
            )

        ax.legend(loc="upper right", fontsize=8, ncol=2)

    axes[-1].set_xlabel("Time (s)", fontsize=10)

    fig.tight_layout()

    out_path = os.path.join(output_dir_abs, "tilt_angles.png")
    fig.savefig(out_path, dpi=200)
    plt.close(fig)
    print(f"Saved {out_path}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python -m src.plot_tilts <output_directory>", file=sys.stderr)
        sys.exit(1)
    plot_tilt_angles(sys.argv[1])
