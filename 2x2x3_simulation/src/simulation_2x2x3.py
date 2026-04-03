# PyChrono rigid-body simulation of a 2x2x3 lattice of 12 octahedra connected by ball joints.
# Force-based BCs: collision ground plane supports bottom spheres, ChForce loads top spheres downward.
# Runs 5 independent simulations with random perturbations on an interior octahedron, exports CSV.

import csv
import math
import os
import sys
from typing import Any

import numpy as np
import pychrono as chrono

# ---------------------------------------------------------------------------
# Import geometry helpers from Sanity_Test via importlib to avoid src namespace clash
# ---------------------------------------------------------------------------
import importlib.util as _ilu

_sanity_sim_path = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "..", "..", "Sanity_Test", "src", "simulation.py")
)
_spec = _ilu.spec_from_file_location("sanity_simulation", _sanity_sim_path)
_sanity_mod = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(_sanity_mod)

octahedron_vertices = _sanity_mod.octahedron_vertices
octahedron_faces = _sanity_mod.octahedron_faces
octahedron_inertia = _sanity_mod.octahedron_inertia


# ---------------------------------------------------------------------------
# Lattice constants
# ---------------------------------------------------------------------------
EDGE_LENGTH: float = 1.0
SPACING: float = EDGE_LENGTH * math.sqrt(2.0)  # centre-to-centre distance


# ---------------------------------------------------------------------------
# Body creation helpers
# ---------------------------------------------------------------------------

def _make_octahedron_body(
    system: chrono.ChSystemNSC,
    center: tuple[float, float, float],
    mass: float = 1.0,
    a: float = EDGE_LENGTH,
) -> chrono.ChBody:
    """Create a ChBody representing a regular octahedron, add it to system, and return it."""
    body = chrono.ChBody()
    body.SetPos(chrono.ChVector3d(center[0], center[1], center[2]))
    body.SetMass(mass)
    Ixx, Iyy, Izz = octahedron_inertia(mass, a)
    body.SetInertiaXX(chrono.ChVector3d(Ixx, Iyy, Izz))
    body.SetInertiaXY(chrono.ChVector3d(0.0, 0.0, 0.0))
    system.AddBody(body)
    return body


def _make_sphere_body(
    system: chrono.ChSystemNSC,
    pos: tuple[float, float, float],
    radius: float = 0.05,
    mass: float = 0.001,
) -> chrono.ChBody:
    """Create a small sphere ChBody at pos, add it to system, and return it."""
    body = chrono.ChBody()
    body.SetPos(chrono.ChVector3d(pos[0], pos[1], pos[2]))
    body.SetMass(mass)
    I_sphere = (2.0 / 5.0) * mass * radius * radius
    body.SetInertiaXX(chrono.ChVector3d(I_sphere, I_sphere, I_sphere))
    body.SetInertiaXY(chrono.ChVector3d(0.0, 0.0, 0.0))
    system.AddBody(body)
    return body


def _add_ball_joint(
    system: chrono.ChSystemNSC,
    bodyA: chrono.ChBody,
    bodyB: chrono.ChBody,
    joint_pos: tuple[float, float, float],
) -> chrono.ChLinkLockSpherical:
    """Create and add a spherical (ball) joint between bodyA and bodyB at joint_pos."""
    joint = chrono.ChLinkLockSpherical()
    frame = chrono.ChFramed(chrono.ChVector3d(joint_pos[0], joint_pos[1], joint_pos[2]))
    joint.Initialize(bodyA, bodyB, frame)
    system.AddLink(joint)
    return joint


# ---------------------------------------------------------------------------
# Grid helpers
# ---------------------------------------------------------------------------

def _grid_index_to_flat(ix: int, iy: int, iz: int) -> int:
    """Convert (ix, iy, iz) grid indices to flat body index (ix + 2*iy + 4*iz)."""
    return ix + 2 * iy + 4 * iz


def _grid_center(ix: int, iy: int, iz: int) -> tuple[float, float, float]:
    """Return the world-frame centre of the octahedron at grid position (ix, iy, iz)."""
    return (ix * SPACING, iy * SPACING, iz * SPACING)


def _body_name_oct(ix: int, iy: int, iz: int) -> str:
    """Return a human-readable name for the octahedron at (ix,iy,iz)."""
    return f"oct_{ix}{iy}{iz}"


# ---------------------------------------------------------------------------
# Shared vertex detection
# ---------------------------------------------------------------------------

def _find_shared_vertices(
    centers: list[tuple[float, float, float]],
    grid_indices: list[tuple[int, int, int]],
    a: float = EDGE_LENGTH,
    tol: float = 1e-9,
) -> list[tuple[int, int, tuple[float, float, float]]]:
    """Find all pairs of octahedra sharing a vertex, return (flat_idxA, flat_idxB, vertex_pos)."""
    n = len(centers)
    # Pre-compute all world-frame vertices
    all_verts: list[np.ndarray] = []
    for c in centers:
        all_verts.append(octahedron_vertices(c, a))

    shared: list[tuple[int, int, tuple[float, float, float]]] = []
    for i in range(n):
        for j in range(i + 1, n):
            vi = all_verts[i]  # (6, 3)
            vj = all_verts[j]  # (6, 3)
            # Check all 6x6 pairs for coincidence
            for a_idx in range(6):
                for b_idx in range(6):
                    if np.linalg.norm(vi[a_idx] - vj[b_idx]) < tol:
                        pos = tuple(float(x) for x in vi[a_idx])
                        shared.append((i, j, pos))
    return shared


# ---------------------------------------------------------------------------
# Build the full 2x2x3 system
# ---------------------------------------------------------------------------

def build_system(
    perturbation_seed: int,
) -> tuple[chrono.ChSystemNSC, list[chrono.ChBody], list[str], int, list[chrono.ChBody]]:
    """Build the 2x2x3 lattice system with ball joints, collision ground, and top forces."""
    a = EDGE_LENGTH
    r = a / math.sqrt(2.0)

    # Create system with Bullet collision solver
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Ground body with collision floor at Z = -r (initial bottom sphere Z)
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(True)
    floor_z = -r
    ground_mat = chrono.ChContactMaterialNSC()
    ground_shape = chrono.ChCollisionShapeBox(ground_mat, 100.0, 100.0, 0.1)
    ground.AddCollisionShape(
        ground_shape,
        chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, floor_z - 0.05)),
    )
    system.AddBody(ground)

    # -------------------------------------------------------------------
    # Create 12 octahedra on a 2x2x3 grid
    # -------------------------------------------------------------------
    grid_indices: list[tuple[int, int, int]] = []
    centers: list[tuple[float, float, float]] = []
    oct_bodies: list[chrono.ChBody] = []
    oct_names: list[str] = []

    for iz in range(3):
        for iy in range(2):
            for ix in range(2):
                grid_indices.append((ix, iy, iz))
                c = _grid_center(ix, iy, iz)
                centers.append(c)
                body = _make_octahedron_body(system, c, mass=1.0, a=a)
                oct_bodies.append(body)
                oct_names.append(_body_name_oct(ix, iy, iz))

    # -------------------------------------------------------------------
    # Detect shared vertices and create inter-octahedron ball joints
    # -------------------------------------------------------------------
    shared_pairs = _find_shared_vertices(centers, grid_indices, a)
    joint_count = 0

    for (idxA, idxB, vpos) in shared_pairs:
        _add_ball_joint(system, oct_bodies[idxA], oct_bodies[idxB], vpos)
        joint_count += 1

    print(f"  Inter-octahedron ball joints: {joint_count}")

    # -------------------------------------------------------------------
    # Per-column boundary conditions: 4 bottom spheres + 4 top spheres
    # -------------------------------------------------------------------
    bot_spheres: list[chrono.ChBody] = []
    top_spheres: list[chrono.ChBody] = []
    bot_names: list[str] = []
    top_names: list[str] = []

    # Column ordering: (0,0), (1,0), (0,1), (1,1)
    columns = [(0, 0), (1, 0), (0, 1), (1, 1)]

    for col_idx, (cix, ciy) in enumerate(columns):
        # --- Bottom sphere: -Z vertex of iz=0 octahedron in this column ---
        flat_bot = _grid_index_to_flat(cix, ciy, 0)
        bot_center = centers[flat_bot]
        bot_vertex = (bot_center[0], bot_center[1], bot_center[2] - r)

        bot_sph = _make_sphere_body(system, bot_vertex)
        bot_spheres.append(bot_sph)
        bot_names.append(f"bot_sphere_{cix}{ciy}")

        # Enable collision on bottom sphere (supported by ground plane contact)
        bot_sph.EnableCollision(True)
        bot_col_mat = chrono.ChContactMaterialNSC()
        bot_col_shape = chrono.ChCollisionShapeSphere(bot_col_mat, 0.05)
        bot_sph.AddCollisionShape(bot_col_shape)

        # Ball joint: bottom sphere <-> iz=0 octahedron
        _add_ball_joint(system, bot_sph, oct_bodies[flat_bot], bot_vertex)
        joint_count += 1

        # --- Top sphere: +Z vertex of iz=2 octahedron in this column ---
        flat_top = _grid_index_to_flat(cix, ciy, 2)
        top_center = centers[flat_top]
        top_vertex = (top_center[0], top_center[1], top_center[2] + r)

        top_sph = _make_sphere_body(system, top_vertex)
        top_spheres.append(top_sph)
        top_names.append(f"top_sphere_{cix}{ciy}")

        # Ball joint: iz=2 octahedron <-> top sphere
        _add_ball_joint(system, oct_bodies[flat_top], top_sph, top_vertex)
        joint_count += 1

        # Constant downward force on top sphere (ChForce must be added before configuring)
        top_force = chrono.ChForce()
        top_sph.AddForce(top_force)
        top_force.SetMode(chrono.ChForce.FORCE)
        top_force.SetAlign(chrono.ChForce.WORLD_DIR)
        top_force.SetDir(chrono.ChVector3d(0.0, 0.0, -1.0))
        top_force.SetMforce(5.0)  # 5 N downward, tunable

    print(f"  Total ball joints (inter-oct + sphere-oct): {joint_count}")

    # -------------------------------------------------------------------
    # Assemble body list and name list in canonical order
    # -------------------------------------------------------------------
    # Body IDs: 0-11 octahedra, 12-15 bottom spheres, 16-19 top spheres
    all_bodies = oct_bodies + bot_spheres + top_spheres
    all_names = oct_names + bot_names + top_names

    # -------------------------------------------------------------------
    # Perturbation: random angular velocity on an interior octahedron (iz=1)
    # -------------------------------------------------------------------
    # Pick octahedron (0, 0, 1) = flat index 4 as the perturbed body
    perturb_flat = _grid_index_to_flat(0, 0, 1)
    rng = np.random.default_rng(perturbation_seed)
    perturb_dir = rng.standard_normal(3)
    perturb_dir /= np.linalg.norm(perturb_dir)
    perturb_mag = 0.01  # rad/s
    omega = perturb_mag * perturb_dir
    oct_bodies[perturb_flat].SetAngVelParent(
        chrono.ChVector3d(float(omega[0]), float(omega[1]), float(omega[2]))
    )

    return system, all_bodies, all_names, joint_count, top_spheres


# ---------------------------------------------------------------------------
# Simulation runner
# ---------------------------------------------------------------------------

def run_single(
    seed: int,
    csv_path: str,
    dt: float = 1e-4,
    duration: float = 10.0,
    export_interval: int = 500,
) -> tuple[str, int]:
    """Build a fresh system, step it for duration seconds, write body states to csv_path."""
    system, bodies, body_names, joint_count, _top_spheres = build_system(seed)

    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    n_steps = int(duration / dt)
    rows: list[list[Any]] = []

    for step_idx in range(n_steps):
        system.DoStepDynamics(dt)

        if step_idx % export_interval == 0:
            t = system.GetChTime()
            for body_id, (body, name) in enumerate(zip(bodies, body_names)):
                p = body.GetPos()
                q = body.GetRot()
                rows.append([
                    f"{t:.6f}",
                    body_id,
                    name,
                    f"{p.x:.8f}",
                    f"{p.y:.8f}",
                    f"{p.z:.8f}",
                    f"{q.e0:.8f}",
                    f"{q.e1:.8f}",
                    f"{q.e2:.8f}",
                    f"{q.e3:.8f}",
                ])

    with open(csv_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["time", "body_id", "body_name", "x", "y", "z", "q0", "q1", "q2", "q3"])
        writer.writerows(rows)

    return csv_path, joint_count


def run_all(n_runs: int = 5) -> list[str]:
    """Run n_runs simulations with seeds 1..n_runs and return list of CSV output paths."""
    output_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "output",
    )
    paths: list[str] = []
    for i in range(1, n_runs + 1):
        csv_path = os.path.join(output_dir, f"sim_{i:03d}.csv")
        print(f"Running simulation {i}/{n_runs} (seed={i}) -> {csv_path}")
        result_path, joint_count = run_single(seed=i, csv_path=csv_path)
        paths.append(result_path)
        row_count = sum(1 for _ in open(csv_path)) - 1
        print(f"  Done. Rows written: {row_count}")
    return paths


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    paths = run_all(n_runs=5)
    print("\nOutput CSV files:")
    for p in paths:
        print(f"  {p}")
