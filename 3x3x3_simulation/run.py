# Orchestrator script for the 3x3x3 octahedral tilt simulation pipeline.
# Runs simulation, renders MP4 videos, and generates tilt angle plots.
# Usage: conda run -n chrono python run.py [--sim] [--viz] [--plot] [--all]

from __future__ import annotations

import argparse
import os
import sys
import time


def _variant_dirs() -> list[tuple[str, str]]:
    # Resolve output subdirs for each stiffness variant, relative to this file.
    # In JOINT_MODE == "bushing" this drives per-variant viz/plot calls; in
    # "spherical" mode a single [("spherical", "output")] entry falls out so
    # the setup333 rigid baseline layout is preserved when the flag is flipped.
    from src.simulation_3x3x3 import JOINT_MODE, STIFFNESS_VARIANTS
    project_root = os.path.dirname(os.path.abspath(__file__))
    output_root = os.path.join(project_root, "output")
    if JOINT_MODE == "bushing":
        return [(label, os.path.join(output_root, label)) for label, _k, _c in STIFFNESS_VARIANTS]
    return [("spherical", output_root)]


def run_simulation() -> None:
    # Run simulation — mode-dependent:
    #   bushing: sweep over STIFFNESS_VARIANTS into output/<label>/.
    #   spherical: 5 seeds into output/.
    print("=" * 60)
    print("STEP 1: Running simulation ...")
    print("=" * 60)
    from src.simulation_3x3x3 import JOINT_MODE, run_all, run_sweep

    t0 = time.time()
    if JOINT_MODE == "bushing":
        results = run_sweep()
        elapsed = time.time() - t0
        print(f"\nSweep complete in {elapsed:.1f}s.")
        for label, out_dir in results:
            print(f"  {label} -> {out_dir}")
    else:
        paths = run_all()
        elapsed = time.time() - t0
        print(f"\nSimulation complete in {elapsed:.1f}s.")
        for p in paths:
            print(f"  -> {p}")


def run_visualization() -> None:
    # Render MP4 videos per variant into output/<label>/front_view/ (bushing)
    # or output/front_view/ (spherical).
    print("\n" + "=" * 60)
    print("STEP 2: Rendering MP4 videos ...")
    print("=" * 60)
    from src.visualizer import visualize_all

    t0 = time.time()
    for label, variant_dir in _variant_dirs():
        print(f"\n--- Visualizing {label} ---")
        visualize_all(variant_dir)
    elapsed = time.time() - t0
    print(f"\nVisualization complete in {elapsed:.1f}s.")


def run_plotting() -> None:
    # Generate tilt-angle plot per variant.
    print("\n" + "=" * 60)
    print("STEP 3: Generating tilt angle plot ...")
    print("=" * 60)
    from src.plot_tilts import plot_tilt_angles

    t0 = time.time()
    for label, variant_dir in _variant_dirs():
        print(f"\n--- Plotting {label} ---")
        plot_tilt_angles(variant_dir)
    elapsed = time.time() - t0
    print(f"\nPlotting complete in {elapsed:.1f}s.")


def main() -> None:
    # Parse arguments and run requested pipeline stages.
    parser = argparse.ArgumentParser(
        description="3x3x3 Octahedral Tilt Simulation Pipeline"
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
    from src.simulation_3x3x3 import JOINT_MODE
    if JOINT_MODE == "bushing":
        print("Pipeline finished. Per-variant outputs under output/<label>/:")
        for label, variant_dir in _variant_dirs():
            print(f"  {label}/")
            print(f"    CSVs:       {variant_dir}/sim_*.csv")
            print(f"    Front view: {variant_dir}/front_view/collapse_*.mp4")
            print(f"    Tilt plot:  {variant_dir}/tilt_angles.png")
    else:
        print("Pipeline finished. Outputs in output/")
        print("  CSVs:       output/sim_*.csv")
        print("  Front view: output/front_view/collapse_*.mp4")
        print("  Tilt plot:  output/tilt_angles.png")
    print("=" * 60)


if __name__ == "__main__":
    main()
