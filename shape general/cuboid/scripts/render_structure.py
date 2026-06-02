"""Render a single still frame of the rest-state cuboid diamond structure.

Builds the bushing system (F_top = 0, perturb = 0), reads each body's initial
pose, and renders one front-view PNG via ``src.visualizer.render_frame`` — the
same renderer used for the collapse MP4s, so the still matches the video style.
Output: ``output/structure_frame.png``.

Run with:

    DYLD_INSERT_LIBRARIES=$CONDA_PREFIX/lib/libomp.dylib \
        $CONDA_PREFIX/bin/python scripts/render_structure.py
"""

from __future__ import annotations

import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.simulation_cuboid import STIFFNESS_VARIANTS, build_system  # noqa: E402
from src.visualizer import render_frame  # noqa: E402


def main() -> int:
    label, k, c, _f, _p = STIFFNESS_VARIANTS[0]
    print(f"Building rest-state structure: {label} K={k} C={c}, F_top=0, perturb=0")

    system, bodies, names, _jc, _top, _lc = build_system(
        1, bushing_k=k, bushing_c=c, top_force_n=0.0, perturb_mag=0.0
    )

    # Assemble the visualizer's per-body state dict at t = 0 (initial diamond pose).
    sim_data: dict[int, dict[float, tuple]] = {}
    body_kinds: dict[int, str] = {}
    for bid, (body, name) in enumerate(zip(bodies, names)):
        p = body.GetPos()
        q = body.GetRot()
        sim_data[bid] = {0.0: (p.x, p.y, p.z, q.e0, q.e1, q.e2, q.e3)}
        if name.startswith("cube"):
            body_kinds[bid] = "cube"
        elif "sphere" in name:
            body_kinds[bid] = "sphere"
        else:
            body_kinds[bid] = "unknown"

    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_dir = os.path.join(project_root, "output")
    tmp_dir = os.path.join(out_dir, "_frame_tmp")
    os.makedirs(tmp_dir, exist_ok=True)

    render_frame(0, sim_data, 0.0, tmp_dir, body_kinds)

    dst_png = os.path.join(out_dir, "structure_frame.png")
    shutil.move(os.path.join(tmp_dir, "frame_0000.png"), dst_png)
    shutil.rmtree(tmp_dir, ignore_errors=True)
    print(f"Saved {dst_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
