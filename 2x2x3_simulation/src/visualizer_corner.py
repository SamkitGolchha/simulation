# Corner tower-view renderer for the 2x2x3 octahedral simulation.
# Reads CSVs from output/, renders MP4s to view_change/.
# Camera is positioned at 1.2x the structure height, looking from the (+X,+Y)
# corner to show top and both side planes simultaneously.

from __future__ import annotations

import glob
import math
import os
import shutil
import sys
from typing import List

import numpy as np
import pyvista as pv

from src.visualizer import (
    BodyKindMap,
    SimData,
    _OCT_COLORS,
    build_octahedron_mesh,
    build_sphere_mesh,
    derive_mp4_name,
    parse_csv,
    stitch_frames_to_mp4,
)

# ---------------------------------------------------------------------------
# Camera constants
# ---------------------------------------------------------------------------
_EDGE_LENGTH = 1.0
_R = _EDGE_LENGTH / math.sqrt(2.0)
_SPACING = _EDGE_LENGTH * math.sqrt(2.0)

# Structure extents
_Z_BOT = -_R                          # bottom vertex of iz=0 octahedra
_Z_TOP = 2.0 * _SPACING + _R         # top vertex of iz=2 octahedra
_STRUCT_HEIGHT = _Z_TOP - _Z_BOT     # ~4.24

# Focal point: center of the structure
_FOCAL = (
    0.5 * _SPACING,                   # center X
    0.5 * _SPACING,                   # center Y
    0.5 * (_Z_BOT + _Z_TOP),         # center Z
)

# Camera: corner tower view at 1.2x structure height
_CAM_Z = 1.2 * _STRUCT_HEIGHT
_CAM_DIST = 6.0  # horizontal distance from focal point along the 45-deg diagonal
_CAM_POS = (
    _FOCAL[0] + _CAM_DIST / math.sqrt(2.0),
    _FOCAL[1] + _CAM_DIST / math.sqrt(2.0),
    _CAM_Z,
)
_CAM_UP = (0.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# Frame rendering (corner camera)
# ---------------------------------------------------------------------------

def render_frame_corner(
    frame_idx: int,
    sim_data: SimData,
    time: float,
    frames_dir: str,
    body_kinds: BodyKindMap,
) -> None:
    """Render one timestep to PNG with the corner tower camera."""
    pl = pv.Plotter(off_screen=True, window_size=[1920, 1088])
    pl.set_background("white")

    oct_ids = sorted(bid for bid, kind in body_kinds.items() if kind == "oct")
    sphere_ids = sorted(bid for bid, kind in body_kinds.items() if kind == "sphere")

    for idx, body_id in enumerate(oct_ids):
        if body_id not in sim_data or time not in sim_data[body_id]:
            continue
        state = sim_data[body_id][time]
        mesh = build_octahedron_mesh(state[:3], state[3:])
        color = _OCT_COLORS[idx % len(_OCT_COLORS)]
        pl.add_mesh(
            mesh,
            color=color,
            opacity=0.6,
            show_edges=True,
            edge_color="black",
            line_width=1,
        )

    for body_id in sphere_ids:
        if body_id not in sim_data or time not in sim_data[body_id]:
            continue
        state = sim_data[body_id][time]
        sphere = build_sphere_mesh(state[:3])
        pl.add_mesh(sphere, color="red")

    # Corner tower camera
    pl.camera_position = [_CAM_POS, _FOCAL, _CAM_UP]

    frame_path = os.path.join(frames_dir, f"frame_{frame_idx:04d}.png")
    pl.screenshot(frame_path)
    pl.close()


# ---------------------------------------------------------------------------
# Pipeline
# ---------------------------------------------------------------------------

def visualize_corner(csv_path: str, output_dir: str, output_filename: str) -> None:
    """Parse CSV, render all frames with corner camera, stitch MP4."""
    csv_path_abs = os.path.abspath(csv_path)
    os.makedirs(output_dir, exist_ok=True)
    frames_dir = os.path.join(output_dir, "frames")
    mp4_path = os.path.join(output_dir, output_filename)
    os.makedirs(frames_dir, exist_ok=True)

    print(f"Parsing {csv_path_abs} ...")
    sim_data, times, body_kinds = parse_csv(csv_path_abs)
    total_frames = len(times)
    n_oct = sum(1 for k in body_kinds.values() if k == "oct")
    n_sph = sum(1 for k in body_kinds.values() if k == "sphere")
    print(f"  Found {total_frames} timesteps, {len(sim_data)} bodies ({n_oct} octahedra, {n_sph} spheres).")

    for idx, t in enumerate(times):
        render_frame_corner(idx, sim_data, t, frames_dir, body_kinds)
        if (idx + 1) % 20 == 0 or (idx + 1) == total_frames:
            print(f"  Rendered frame {idx + 1}/{total_frames} (t={t:.4f}s)")

    print("Stitching frames into MP4 ...")
    stitch_frames_to_mp4(frames_dir, mp4_path)
    print(f"  Saved {mp4_path}")

    print("Cleaning up frames ...")
    shutil.rmtree(frames_dir, ignore_errors=True)
    print("Done.")


def visualize_corner_all(input_dir: str, output_dir: str) -> None:
    """Find all sim_*.csv in input_dir, render corner-view MP4s to output_dir."""
    input_dir_abs = os.path.abspath(input_dir)
    output_dir_abs = os.path.abspath(output_dir)
    pattern = os.path.join(input_dir_abs, "sim_*.csv")
    csv_files = sorted(glob.glob(pattern))

    if not csv_files:
        print(f"No sim_*.csv files found in {input_dir_abs}", file=sys.stderr)
        sys.exit(1)

    print(f"Found {len(csv_files)} CSV file(s). Rendering corner view to {output_dir_abs}.")
    for csv_file in csv_files:
        mp4_name = derive_mp4_name(csv_file)
        print(f"\n--- Processing {os.path.basename(csv_file)} -> {mp4_name} ---")
        visualize_corner(csv_file, output_dir_abs, mp4_name)


if __name__ == "__main__":
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    input_dir = os.path.join(project_root, "output")
    output_dir = os.path.join(project_root, "output", "corner_view")
    visualize_corner_all(input_dir, output_dir)
