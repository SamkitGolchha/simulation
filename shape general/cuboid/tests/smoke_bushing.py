"""Bushing-mode smoke test for the 3x3x3 cuboid OPEN STRUT FRAME.

Verifies:
  1. Topology counts: body count = 45, inter-cube couplings = 54
     (one bushing per axis-neighbour pair), sphere-BC bushings = 18
     (one per sphere), total = 72 — matching the octahedron lattice.
  2. Adjacency sanity: cube (0,0,0) and cube (1,0,0) are spaced SPACING apart
     with a gap between their faces (NO shared face) and are linked by exactly
     ONE inter-cube coupling.
  3. The ChSolverADMM solver accepts the bushing K/C matrices (no
     `System descriptor includes stiffness or damping matrices` error).
  4. A handful of integration steps produce no NaN/Inf.

Run with:
    DYLD_INSERT_LIBRARIES=$CONDA_PREFIX/lib/libomp.dylib \\
      $CONDA_PREFIX/bin/python tests/smoke_bushing.py
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.simulation_cuboid import (
    EDGE_LENGTH,
    JOINT_MODE,
    SPACING,
    build_system,
    _axis_neighbor_couplings,
    _grid_center,
)


def main() -> int:
    print("Smoke test: open strut frame, K=10000.0 C=100.0")
    K = 1.0e4
    C = 1.0e2

    # ---- 1. Topology counts via the coupling enumerator ----
    print("Checking topology counts ...")
    centers = []
    grid_indices = []
    for iz in range(3):
        for iy in range(3):
            for ix in range(3):
                grid_indices.append((ix, iy, iz))
                centers.append(_grid_center(ix, iy, iz))

    couplings = _axis_neighbor_couplings(centers, grid_indices)
    n_inter = len(couplings)
    print(f"  OK _axis_neighbor_couplings -> {n_inter} inter-cube couplings")

    # cube (0,0,0)=flat 0 and (1,0,0)=flat 1: exactly ONE coupling, with a gap.
    n_01 = sum(1 for (a, b, _) in couplings if {a, b} == {0, 1})
    gap = SPACING - EDGE_LENGTH
    print(f"  cube (0,0,0)&(1,0,0): {n_01} coupling(s), face gap = {gap:.3f}")
    assert n_01 == 1, f"expected exactly 1 coupling between (0,0,0)&(1,0,0), got {n_01}"
    assert gap > 1e-6, f"expected a positive gap between cube faces, got {gap}"

    # ---- 2. Build full system ----
    print("Building full system ...")
    system, bodies, names, joint_count, top_spheres, load_container = build_system(
        perturbation_seed=1, bushing_k=K, bushing_c=C,
    )
    n_bodies = len(bodies)
    n_sphere_bc = joint_count - n_inter
    print(f"  Inter-cube bushings: {n_inter}")
    print(f"  Sphere-BC bushings (1 per sphere): {n_sphere_bc}")
    print(f"  Total bushings (inter-cube + sphere-BC): {joint_count}")
    print(f"  Total bodies: {n_bodies}")

    assert n_bodies == 45, f"expected 45 bodies, got {n_bodies}"
    assert n_inter == 54, f"expected 54 inter-cube, got {n_inter}"
    assert n_sphere_bc == 18, f"expected 18 sphere-BC, got {n_sphere_bc}"
    assert joint_count == 72, f"expected 72 total, got {joint_count}"
    print(f"  OK body count={n_bodies}, total bushings={joint_count}")

    # ---- 3 & 4. Integration steps: ADMM accepts K/C, no NaN/Inf ----
    print("Running a few integration steps ...")
    for _ in range(51):
        system.DoStepDynamics(1e-4)
    for b in bodies:
        p = b.GetPos()
        if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)):
            print("  NaN/Inf detected!", file=sys.stderr)
            return 1

    print("  Smoke test PASSED — 54+18=72 couplings; ADMM accepted K/C; no NaN/Inf.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
