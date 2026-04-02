# Orchestrator script for the 2x2x3 octahedral tilt simulation pipeline.
# Runs simulation, renders MP4 videos, and generates tilt angle plots.
# Usage: conda run -n chrono python run.py [--sim] [--viz] [--plot] [--all]

from __future__ import annotations

import argparse
import os
import sys
import time


def run_simulation() -> None:
    # Run all 5 simulation runs and export CSVs.
    print("=" * 60)
    print("STEP 1: Running simulation (5 runs) ...")
    print("=" * 60)
    from src.simulation_2x2x3 import run_all

    t0 = time.time()
    paths = run_all()
    elapsed = time.time() - t0
    print(f"\nSimulation complete in {elapsed:.1f}s.")
    for p in paths:
        print(f"  -> {p}")


def run_visualization() -> None:
    # Render MP4 videos from all CSVs in output/.
    print("\n" + "=" * 60)
    print("STEP 2: Rendering MP4 videos ...")
    print("=" * 60)
    from src.visualizer import visualize_all

    t0 = time.time()
    visualize_all("output")
    elapsed = time.time() - t0
    print(f"\nVisualization complete in {elapsed:.1f}s.")


def run_plotting() -> None:
    # Generate tilt angle plots from all CSVs in output/.
    print("\n" + "=" * 60)
    print("STEP 3: Generating tilt angle plot ...")
    print("=" * 60)
    from src.plot_tilts import plot_tilt_angles

    t0 = time.time()
    plot_tilt_angles("output")
    elapsed = time.time() - t0
    print(f"\nPlotting complete in {elapsed:.1f}s.")


def main() -> None:
    # Parse arguments and run requested pipeline stages.
    parser = argparse.ArgumentParser(
        description="2x2x3 Octahedral Tilt Simulation Pipeline"
    )
    parser.add_argument("--sim", action="store_true", help="Run simulation only")
    parser.add_argument("--viz", action="store_true", help="Render MP4 videos only")
    parser.add_argument("--plot", action="store_true", help="Generate tilt plots only")
    parser.add_argument("--all", action="store_true", help="Run full pipeline (default)")
    args = parser.parse_args()

    run_all_steps = args.all or not (args.sim or args.viz or args.plot)

    if run_all_steps or args.sim:
        run_simulation()

    if run_all_steps or args.viz:
        run_visualization()

    if run_all_steps or args.plot:
        run_plotting()

    print("\n" + "=" * 60)
    print("Pipeline finished. Outputs in output/")
    print("=" * 60)


if __name__ == "__main__":
    main()
