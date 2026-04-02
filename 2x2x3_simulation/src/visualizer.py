# Renders each simulation timestep as a 3D PyVista frame and stitches into MP4.
# Reads body positions/quaternions from a CSV, builds octahedra and spheres for the 2x2x3 grid.
# Uses imageio_ffmpeg to locate the ffmpeg binary for video encoding.

from __future__ import annotations

import csv
import glob
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pyvista as pv
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------------------------
# Geometry constants for a regular octahedron with edge length a = 1.0
# ---------------------------------------------------------------------------
_A: float = 1.0                          # edge length
_R: float = _A / math.sqrt(2.0)         # circumradius to axial vertex = sqrt(2)/2

# Local-frame vertices (centered at origin, Z-axis is the tilt axis)
_OCT_VERTS_LOCAL: np.ndarray = np.array([
    [_R,   0.0,  0.0],  # 0: +X
    [-_R,  0.0,  0.0],  # 1: -X
    [0.0,  _R,   0.0],  # 2: +Y
    [0.0, -_R,   0.0],  # 3: -Y
    [0.0,  0.0,  _R],   # 4: +Z (top)
    [0.0,  0.0, -_R],   # 5: -Z (bottom)
], dtype=np.float64)

# 8 triangular faces (each row: 3 vertex indices)
_OCT_FACES: np.ndarray = np.array([
    [4, 0, 2],
    [4, 2, 1],
    [4, 1, 3],
    [4, 3, 0],
    [5, 2, 0],
    [5, 1, 2],
    [5, 3, 1],
    [5, 0, 3],
], dtype=np.int_)

# Color palette for up to 12 octahedra, cycling by Z-layer (4 per layer)
# Layer 0 (iz=0): blues, Layer 1 (iz=1): greens, Layer 2 (iz=2): purples
_OCT_COLORS: List[str] = [
    # iz=0 layer (body_id 0-3)
    "steelblue",
    "dodgerblue",
    "cornflowerblue",
    "royalblue",
    # iz=1 layer (body_id 4-7)
    "mediumseagreen",
    "seagreen",
    "limegreen",
    "forestgreen",
    # iz=2 layer (body_id 8-11)
    "mediumpurple",
    "slateblue",
    "darkorchid",
    "mediumorchid",
]


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

BodyState = Tuple[float, float, float, float, float, float, float]
# (x, y, z, q0, q1, q2, q3)

SimData = Dict[int, Dict[float, BodyState]]
# body_id -> { time -> (x,y,z,q0,q1,q2,q3) }

# Maps body_id to its kind ("oct" or "sphere"), populated during parse_csv.
BodyKindMap = Dict[int, str]


def parse_csv(csv_path: str) -> Tuple[SimData, List[float], BodyKindMap]:
    # Read the simulation CSV and return per-body state dicts, sorted timesteps, and body kind map.
    data: SimData = {}
    times_seen: List[float] = []
    time_set: set = set()
    body_kinds: BodyKindMap = {}

    with open(csv_path, newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            t = float(row["time"])
            bid = int(row["body_id"])
            bname = row["body_name"]
            state: BodyState = (
                float(row["x"]),
                float(row["y"]),
                float(row["z"]),
                float(row["q0"]),
                float(row["q1"]),
                float(row["q2"]),
                float(row["q3"]),
            )
            if bid not in data:
                data[bid] = {}
            data[bid][t] = state
            if t not in time_set:
                time_set.add(t)
                times_seen.append(t)
            # Classify body kind from name (only need to do once per body_id)
            if bid not in body_kinds:
                if bname.startswith("oct"):
                    body_kinds[bid] = "oct"
                elif "sphere" in bname:
                    body_kinds[bid] = "sphere"
                else:
                    body_kinds[bid] = "unknown"

    times_seen.sort()
    return data, times_seen, body_kinds


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------


def build_octahedron_mesh(
    position: Tuple[float, float, float],
    quaternion: Tuple[float, float, float, float],
) -> pv.PolyData:
    # Rotate local octahedron vertices by the body quaternion, translate to world position.
    x, y, z, q0, q1, q2, q3 = (*position, *quaternion)
    rot = Rotation.from_quat([q1, q2, q3, q0])  # scipy scalar-last: [qx,qy,qz,qw]
    world_verts = rot.apply(_OCT_VERTS_LOCAL) + np.array([x, y, z])

    # PyVista PolyData face format: [n_pts, i0, i1, i2, ...]
    faces_flat: List[int] = []
    for face in _OCT_FACES:
        faces_flat.extend([3, face[0], face[1], face[2]])
    faces_arr = np.array(faces_flat, dtype=np.int_)

    mesh = pv.PolyData(world_verts, faces_arr)
    return mesh


def build_sphere_mesh(
    position: Tuple[float, float, float],
) -> pv.PolyData:
    # Create a small sphere at the given world-space position.
    x, y, z = position
    return pv.Sphere(radius=0.05, center=(x, y, z))


# ---------------------------------------------------------------------------
# Frame rendering
# ---------------------------------------------------------------------------


def render_frame(
    frame_idx: int,
    sim_data: SimData,
    time: float,
    frames_dir: str,
    body_kinds: BodyKindMap,
) -> None:
    # Render one timestep to a PNG file in frames_dir using off-screen PyVista.
    pl = pv.Plotter(off_screen=True, window_size=[1920, 1088])
    pl.set_background("white")

    # Collect octahedron body_ids and sphere body_ids from the kind map
    oct_ids = sorted(bid for bid, kind in body_kinds.items() if kind == "oct")
    sphere_ids = sorted(bid for bid, kind in body_kinds.items() if kind == "sphere")

    # Add octahedra
    for idx, body_id in enumerate(oct_ids):
        if body_id not in sim_data or time not in sim_data[body_id]:
            continue
        state = sim_data[body_id][time]
        pos = state[:3]
        quat = state[3:]
        mesh = build_octahedron_mesh(pos, quat)
        color = _OCT_COLORS[idx % len(_OCT_COLORS)]
        pl.add_mesh(
            mesh,
            color=color,
            opacity=0.6,
            show_edges=True,
            edge_color="black",
            line_width=1,
        )

    # Add spheres (all rendered as small red spheres)
    for body_id in sphere_ids:
        if body_id not in sim_data or time not in sim_data[body_id]:
            continue
        state = sim_data[body_id][time]
        pos = state[:3]
        sphere = build_sphere_mesh(pos)
        pl.add_mesh(sphere, color="red")

    # Camera: side view, auto-fit to the larger 2x2x3 grid
    pl.camera_position = "xz"
    pl.camera.azimuth = 0.0
    pl.camera.elevation = 0.0
    pl.reset_camera()

    frame_path = os.path.join(frames_dir, f"frame_{frame_idx:04d}.png")
    pl.screenshot(frame_path)
    pl.close()


# ---------------------------------------------------------------------------
# Video stitching
# ---------------------------------------------------------------------------


def stitch_frames_to_mp4(frames_dir: str, output_path: str) -> None:
    # Invoke ffmpeg via imageio_ffmpeg to encode PNGs into an H.264 MP4.
    import imageio_ffmpeg

    ffmpeg_exe = imageio_ffmpeg.get_ffmpeg_exe()
    input_pattern = os.path.join(frames_dir, "frame_%04d.png")
    cmd = [
        ffmpeg_exe,
        "-y",
        "-framerate", "30",
        "-i", input_pattern,
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        output_path,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"ffmpeg failed (exit {result.returncode}):\n{result.stderr}"
        )


# ---------------------------------------------------------------------------
# Filename derivation
# ---------------------------------------------------------------------------


def derive_mp4_name(csv_path: str) -> str:
    # Map sim_NNN.csv to collapse_NNN.mp4; fall back to collapse.mp4 for other names.
    stem = Path(csv_path).stem          # e.g. "sim_001"
    if stem.startswith("sim_"):
        suffix = stem[len("sim_"):]     # e.g. "001"
        return f"collapse_{suffix}.mp4"
    return "collapse.mp4"


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------


def visualize(csv_path: str, output_filename: Optional[str] = None) -> None:
    # Full pipeline: parse CSV, render all frames, stitch MP4, clean up frames.
    csv_path_abs = os.path.abspath(csv_path)
    output_dir = str(Path(csv_path_abs).parent)

    mp4_name = output_filename if output_filename is not None else "collapse.mp4"
    frames_dir = os.path.join(output_dir, "frames")
    mp4_path = os.path.join(output_dir, mp4_name)

    os.makedirs(frames_dir, exist_ok=True)

    print(f"Parsing {csv_path_abs} ...")
    sim_data, times, body_kinds = parse_csv(csv_path_abs)
    total_frames = len(times)
    n_oct = sum(1 for k in body_kinds.values() if k == "oct")
    n_sph = sum(1 for k in body_kinds.values() if k == "sphere")
    print(f"  Found {total_frames} timesteps, {len(sim_data)} bodies ({n_oct} octahedra, {n_sph} spheres).")

    for idx, t in enumerate(times):
        render_frame(idx, sim_data, t, frames_dir, body_kinds)
        if (idx + 1) % 20 == 0 or (idx + 1) == total_frames:
            print(f"  Rendered frame {idx + 1}/{total_frames} (t={t:.4f}s)")

    print("Stitching frames into MP4 ...")
    stitch_frames_to_mp4(frames_dir, mp4_path)
    print(f"  Saved {mp4_path}")

    print("Cleaning up frames ...")
    shutil.rmtree(frames_dir, ignore_errors=True)
    print("Done.")


def visualize_all(output_dir: str) -> None:
    # Find all sim_*.csv files in output_dir and produce one collapse_NNN.mp4 per CSV.
    output_dir_abs = os.path.abspath(output_dir)
    pattern = os.path.join(output_dir_abs, "sim_*.csv")
    csv_files = sorted(glob.glob(pattern))

    if not csv_files:
        print(f"No sim_*.csv files found in {output_dir_abs}", file=sys.stderr)
        sys.exit(1)

    print(f"Found {len(csv_files)} CSV file(s) in {output_dir_abs}.")
    for csv_file in csv_files:
        mp4_name = derive_mp4_name(csv_file)
        print(f"\n--- Processing {os.path.basename(csv_file)} -> {mp4_name} ---")
        visualize(csv_file, output_filename=mp4_name)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(
            "Usage:\n"
            "  python -m src.visualizer <path/to/sim_NNN.csv>   # single run\n"
            "  python -m src.visualizer <output_dir/>            # all runs",
            file=sys.stderr,
        )
        sys.exit(1)

    arg = sys.argv[1]
    if os.path.isdir(arg):
        visualize_all(arg)
    else:
        # Single-file backward-compatible path: always writes collapse.mp4
        visualize(arg, output_filename="collapse.mp4")
