# PyChrono rigid-body simulation of a 3x3x3 lattice of 27 cubes connected by ball joints.
# Ghost Spheres — bottom spheres remain as bodies but their collision is removed.
# Cubes rest on the ground directly via their own box collision shapes.
# ChForce loads top spheres downward.
# Runs 5 independent simulations with random perturbations on an interior cube, exports CSV.
# Body IDs: 0-26 cubes, 27-35 bottom ghost spheres, 36-44 top loading spheres.

import csv
import math
import os
import sys
import time
from typing import Any, Literal

import numpy as np
import pychrono as chrono

# ---------------------------------------------------------------------------
# Joint formulation switch
# ---------------------------------------------------------------------------
# "spherical": rigid ChLinkLockSpherical — kept for code-structure parity with
#   the 3x3x3 octahedron codebase. It MUST NOT be used on the cuboid lattice:
#   cubes share whole faces (4 coincident vertices per interface), so rigid
#   spherical joints would be massively overconstrained.
# "bushing":   compliant ChLoadBodyBodyBushingSpherical — the validated
#   scalable formulation. Zero rows in the constraint Jacobian, so the system
#   can never be overconstrained no matter how many bushings share a cube
#   face. Requires ChSolverADMM (PSOR rejects the K/C matrices at runtime).
JOINT_MODE: Literal["spherical", "bushing"] = "bushing"

# ---------------------------------------------------------------------------
# Cube geometry helpers — owned by structure_cuboid (placeholders here)
# ---------------------------------------------------------------------------

def cuboid_vertices(center: tuple[float, float, float], a: float = 1.0) -> np.ndarray:
    """Return an (8, 3) array of the cube's corner coordinates centred at center.

    A cube has 8 corners at ``center ± (a/2, a/2, a/2)``. The ordering below is
    FIXED and CANONICAL — gui_viz_cuboid's visual mesh (`_CUBE_VERTS_LOCAL`)
    MUST reuse this exact index order so the rendered solid matches the
    physics body frame.

    Ordering (index : signs of the (x, y, z) offset from center):

        0 : (+, +, +)   top    +X +Y corner
        1 : (+, -, +)   top    +X -Y corner
        2 : (-, +, +)   top    -X +Y corner
        3 : (-, -, +)   top    -X -Y corner
        4 : (+, +, -)   bottom +X +Y corner
        5 : (+, -, -)   bottom +X -Y corner
        6 : (-, +, -)   bottom -X +Y corner
        7 : (-, -, -)   bottom -X -Y corner

    Indices 0-3 are the +Z (top) face, 4-7 are the -Z (bottom) face. Within a
    face the order is row-major over (x desc, y desc) sign — i.e. z varies
    slowest, then x, then y.
    """
    cx, cy, cz = center
    h = a / 2.0
    return np.array([
        [cx + h, cy + h, cz + h],  # 0: top    +X +Y
        [cx + h, cy - h, cz + h],  # 1: top    +X -Y
        [cx - h, cy + h, cz + h],  # 2: top    -X +Y
        [cx - h, cy - h, cz + h],  # 3: top    -X -Y
        [cx + h, cy + h, cz - h],  # 4: bottom +X +Y
        [cx + h, cy - h, cz - h],  # 5: bottom +X -Y
        [cx - h, cy + h, cz - h],  # 6: bottom -X +Y
        [cx - h, cy - h, cz - h],  # 7: bottom -X -Y
    ], dtype=float)


def cuboid_inertia(mass: float, a: float = 1.0) -> tuple[float, float, float]:
    """Return (Ixx, Iyy, Izz) for a uniform solid cube; all equal by symmetry.

    For a solid uniform box of mass m with edge lengths (d1, d2, d3) the
    principal inertia about each axis is ``I = (1/12)·m·(dj² + dk²)`` over the
    two perpendicular edges. For a cube (d1 = d2 = d3 = a):

        I_xx = I_yy = I_zz = (1/12)·m·(a² + a²) = (1/6)·m·a²

    (The octahedron used (1/10)·m·a².) No cross terms — SetInertiaXY(0,0,0).
    """
    I = (1.0 / 6.0) * mass * a * a
    return (I, I, I)


# ---------------------------------------------------------------------------
# Lattice constants — owned by structure_cuboid
# ---------------------------------------------------------------------------
# Open strut-frame lattice: cubes are spaced a·√2 apart (mirroring the octahedron
# lattice) so neighbours do NOT share faces — there is a 0.414·a gap between
# facing faces. Each axis-neighbour pair is tied by ONE compliant bushing on the
# centre-line (see _axis_neighbor_couplings), leaving the rotational freedom the
# structure needs to buckle. A face-to-face packing (SPACING = EDGE_LENGTH, every
# coincident corner welded) is a rigid block that cannot buckle under any load.
EDGE_LENGTH: float = 1.0  # cube edge length a (lx = ly = lz)
SPACING: float = EDGE_LENGTH * math.sqrt(2.0)  # centre-to-centre; leaves a 0.414·a gap
NX: int = 3  # grid extent along X
NY: int = 3  # grid extent along Y
NZ: int = 3  # grid extent along Z
N_CUBE: int = NX * NY * NZ  # 27


# ---------------------------------------------------------------------------
# Diamond orientation — owned by structure_cuboid
# ---------------------------------------------------------------------------
# Every cube is spun 45° about the depth (Y) axis so its front-view (xz) profile
# is a diamond, mirroring the octahedron lattice. The cube's inertia is isotropic
# (Ixx = Iyy = Izz), so this rotation changes nothing dynamically — it only
# re-presents the square cross-section as a diamond. With SPACING = a·√2 the
# diamond tips (centre→tip = a/√2 ≈ 0.707) meet tip-to-tip along X and Z → the
# /\/\ lattice that collapses into a filled block under load. A diamond's lowest
# point is its bottom tip at −a/√2 (not the −a/2 face centre), so the ground
# plane drops to that level in build_system and the bottom row rests on its tips,
# exactly as the octahedron rests on its bottom apex.
DIAMOND_TILT_RAD: float = math.radians(45.0)
HALF_DIAGONAL: float = EDGE_LENGTH / math.sqrt(2.0)  # cube centre → diamond tip


# ---------------------------------------------------------------------------
# Bushing sweep — ships the 20 N validated regime as the only default
# ---------------------------------------------------------------------------
# Each shared-vertex coupling is a ChLoadBodyBodyBushingSpherical (force-based
# 3-axis spring, no rows in the constraint Jacobian). Damping is tuned so
# ζ = C / (2·sqrt(m·K)) ≈ 0.5 (m = 1 kg per cube). Each variant is a 5-tuple
# (label, K, C, F_top, perturb) — the source now self-describes the entire
# config, no hidden globals.
#
# The default is the 20 N regime that produced full buckling (55–61° tilt,
# all 5 seeds) on the 3×3×3 octahedron lattice — the only force level
# empirically validated to collapse the structure. Lower-force exploratory
# variants (0.5 N baseline, 3 N change_2) live in tests/force_changes.py
# and are NOT expected to buckle.
STIFFNESS_VARIANTS: list[tuple[str, float, float, float, float]] = [
    # (label, K, C, F_top[N], perturb[rad/s])
    ("20N_force", 1.0e4, 1.0e2, 20.0, 0.1),
]

# Per-process wall-clock cap. A run ends on whichever fires first: KE
# equilibrium, column collapse, or this budget. The 5 seeds are launched in
# parallel (scripts/launch_sweep.sh), so a 3 h per-process cap bounds the whole
# sweep to ~3 h of wall-clock.
WALL_CLOCK_BUDGET_S: float = 3 * 3600  # 10800 s = 3 hours


# ---------------------------------------------------------------------------
# Body creation helpers
# ---------------------------------------------------------------------------

def _make_cuboid_body(
    system: chrono.ChSystemNSC,
    center: tuple[float, float, float],
    mass: float = 1.0,
    a: float = EDGE_LENGTH,
) -> chrono.ChBody:
    """Create a ChBody for a solid cube with box collision for face-face contact stops.

    Analog of the octahedron's `_make_octahedron_body`: builds a ChBody at
    `center`, sets mass and the uniform-cube inertia (zero cross terms), and
    attaches a `ChCollisionShapeBox` so cubes stop on the ground and on each
    other once the lattice buckles. Contact-material settings (friction 0.5,
    restitution 0.0) are reused verbatim from the octahedron version.
    """
    body = chrono.ChBody()
    body.SetPos(chrono.ChVector3d(center[0], center[1], center[2]))
    body.SetMass(mass)
    Ixx, Iyy, Izz = cuboid_inertia(mass, a)
    body.SetInertiaXX(chrono.ChVector3d(Ixx, Iyy, Izz))
    body.SetInertiaXY(chrono.ChVector3d(0.0, 0.0, 0.0))

    # Box collision shape for face-face / edge contact stops during tilting.
    # ChCollisionShapeBox takes the FULL edge lengths (a, a, a) — not half-extents.
    col_mat = chrono.ChContactMaterialNSC()
    col_mat.SetFriction(0.5)
    col_mat.SetRestitution(0.0)
    box = chrono.ChCollisionShapeBox(col_mat, a, a, a)
    body.AddCollisionShape(box)
    body.EnableCollision(True)

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
    container: Any,
    bodyA: chrono.ChBody,
    bodyB: chrono.ChBody,
    joint_pos: tuple[float, float, float],
    bushing_k: float | None = None,
    bushing_c: float | None = None,
) -> Any:
    """Couple bodyA and bodyB at joint_pos — formulation selected by JOINT_MODE.

    JOINT_MODE == "spherical":
        `container` must be the ChSystemNSC. Creates a rigid ChLinkLockSpherical
        (three kinematic translation constraints in Φ_q) and adds it via
        system.AddLink(). This branch is dead code on the cuboid lattice (cubes
        would be overconstrained by rigid joints) but is kept so the file is a
        faithful structural mirror of the octahedron codebase.

    JOINT_MODE == "bushing":
        `container` must be the ChLoadContainer, and bushing_k / bushing_c
        must be supplied. Builds a ChLoadBodyBodyBushingSpherical with three
        isotropic translational springs (stiffness bushing_k, damping
        bushing_c) at the shared vertex, rotation free, zero rows in Φ_q.
    """
    if JOINT_MODE == "spherical":
        joint = chrono.ChLinkLockSpherical()
        frame = chrono.ChFramed(chrono.ChVector3d(joint_pos[0], joint_pos[1], joint_pos[2]))
        joint.Initialize(bodyA, bodyB, frame)
        container.AddLink(joint)
        return joint

    if JOINT_MODE == "bushing":
        if bushing_k is None or bushing_c is None:
            raise ValueError(
                "_add_ball_joint: bushing mode requires bushing_k and bushing_c"
            )
        frame = chrono.ChFramed(chrono.ChVector3d(joint_pos[0], joint_pos[1], joint_pos[2]))
        stiffness = chrono.ChVector3d(bushing_k, bushing_k, bushing_k)
        damping = chrono.ChVector3d(bushing_c, bushing_c, bushing_c)
        bushing = chrono.ChLoadBodyBodyBushingSpherical(
            bodyA, bodyB, frame, stiffness, damping
        )
        container.Add(bushing)
        return bushing

    raise ValueError(f"Unknown JOINT_MODE: {JOINT_MODE!r}")


# ---------------------------------------------------------------------------
# Grid helpers
# ---------------------------------------------------------------------------

def _grid_index_to_flat(ix: int, iy: int, iz: int) -> int:
    """Convert (ix, iy, iz) grid indices to flat body index (ix + NX*iy + NX*NY*iz)."""
    return ix + NX * iy + NX * NY * iz


def _grid_center(ix: int, iy: int, iz: int) -> tuple[float, float, float]:
    """Return the world-frame centre of the cube at grid position (ix, iy, iz)."""
    return (ix * SPACING, iy * SPACING, iz * SPACING)


def _body_name_cube(ix: int, iy: int, iz: int) -> str:
    """Return a human-readable name for the cube at (ix,iy,iz)."""
    return f"cube_{ix}{iy}{iz}"


# ---------------------------------------------------------------------------
# Kinetic energy equilibrium detector
# ---------------------------------------------------------------------------

def _compute_total_ke(bodies: list[chrono.ChBody]) -> float:
    """Compute total kinetic energy (translational + rotational) of all bodies."""
    total_ke = 0.0
    for b in bodies:
        v = b.GetPosDt()
        w = b.GetAngVelLocal()
        m = b.GetMass()
        total_ke += 0.5 * m * (v.x**2 + v.y**2 + v.z**2)
        I = b.GetInertiaXX()
        total_ke += 0.5 * (I.x * w.x**2 + I.y * w.y**2 + I.z * w.z**2)
    return total_ke


# ---------------------------------------------------------------------------
# Tilt metric — measured relative to the cube's initial diamond pose
# ---------------------------------------------------------------------------
# Cubes start spun 45° about +Y (see DIAMOND_TILT_RAD), so their local +Z axis
# already sits 45° from world up at rest. Tilt is therefore the angle between a
# cube's CURRENT +Z (in world) and that initial diamond +Z = (sin45, 0, cos45),
# so a cube at rest in the diamond pose reads 0° tilt — not 45°. This keeps the
# buckling gate and the tilt plots meaningful for the rotated lattice.
_REF_UP: tuple[float, float, float] = (
    math.sin(DIAMOND_TILT_RAD),
    0.0,
    math.cos(DIAMOND_TILT_RAD),
)


def _cube_tilt_rad(e0: float, e1: float, e2: float, e3: float) -> float:
    """Angle (rad) between a cube's current +Z axis and its initial diamond +Z."""
    # Body +Z axis in world = third column of the rotation matrix R(q).
    bzx = 2.0 * (e1 * e3 + e0 * e2)
    bzy = 2.0 * (e2 * e3 - e0 * e1)
    bzz = 1.0 - 2.0 * (e1 * e1 + e2 * e2)
    dot = bzx * _REF_UP[0] + bzy * _REF_UP[1] + bzz * _REF_UP[2]
    return math.acos(max(-1.0, min(1.0, dot)))


# ---------------------------------------------------------------------------
# Column-collapse stop condition
# ---------------------------------------------------------------------------

def _check_columns_collapsed(
    cube_bodies: list[chrono.ChBody],
    a: float = EDGE_LENGTH,
    tol: float = 0.1,
) -> bool:
    """Return True when all 9 columns have all adjacent pairs collapsed (vertices touching)."""
    r = a / 2.0
    columns = [(ix, iy) for iy in range(NY) for ix in range(NX)]

    for (ix, iy) in columns:
        column_collapsed = True
        for iz in range(NZ - 1):  # pairs (iz, iz+1)
            lower = _grid_index_to_flat(ix, iy, iz)
            upper = _grid_index_to_flat(ix, iy, iz + 1)

            # +Z face-centre vertex of lower body in world frame
            lower_top = cube_bodies[lower].TransformPointLocalToParent(
                chrono.ChVector3d(0, 0, r)
            )
            # -Z face-centre vertex of upper body in world frame
            upper_bot = cube_bodies[upper].TransformPointLocalToParent(
                chrono.ChVector3d(0, 0, -r)
            )
            axial_dist = (lower_top - upper_bot).Length()

            # +X face-centre vertex of both bodies in world frame
            lower_x = cube_bodies[lower].TransformPointLocalToParent(
                chrono.ChVector3d(r, 0, 0)
            )
            upper_x = cube_bodies[upper].TransformPointLocalToParent(
                chrono.ChVector3d(r, 0, 0)
            )
            equat_dist = (lower_x - upper_x).Length()

            if not (axial_dist < tol and equat_dist < tol):
                column_collapsed = False
                break

        if not column_collapsed:
            return False

    return True


# ---------------------------------------------------------------------------
# Shared vertex detection — owned by structure_cuboid (placeholder here)
# ---------------------------------------------------------------------------

def _axis_neighbor_couplings(
    centers: list[tuple[float, float, float]],
    grid_indices: list[tuple[int, int, int]],
) -> list[tuple[int, int, tuple[float, float, float]]]:
    """Return ONE coupling per axis-neighbour cube pair (open strut frame).

    For each pair of cubes adjacent along +X, +Y, or +Z in the grid, emit a
    single ``(flat_idxA, flat_idxB, anchor_world)`` tuple. ``anchor_world`` is
    the midpoint of the inter-cube gap on the line joining the two cube centres
    — the cube analogue of the octahedron's on-axis apex-to-apex joint. One
    bushing per pair (not a welded face) leaves the rotational freedom the
    lattice needs to buckle.

    On a 3×3×3 grid this returns 18 + 18 + 18 = 54 inter-cube couplings,
    exactly matching the octahedron lattice.
    """
    flat_of: dict[tuple[int, int, int], int] = {
        gi: i for i, gi in enumerate(grid_indices)
    }
    couplings: list[tuple[int, int, tuple[float, float, float]]] = []
    for i, (ix, iy, iz) in enumerate(grid_indices):
        for dx, dy, dz in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
            j = flat_of.get((ix + dx, iy + dy, iz + dz))
            if j is None:
                continue
            ca, cb = centers[i], centers[j]
            anchor = (
                0.5 * (ca[0] + cb[0]),
                0.5 * (ca[1] + cb[1]),
                0.5 * (ca[2] + cb[2]),
            )
            couplings.append((i, j, anchor))
    return couplings


# ---------------------------------------------------------------------------
# Build the full 3x3x3 system
# ---------------------------------------------------------------------------

def build_system(
    perturbation_seed: int,
    bushing_k: float | None = None,
    bushing_c: float | None = None,
    top_force_n: float = 0.5,
    perturb_mag: float = 0.02,
) -> tuple[
    chrono.ChSystemNSC,
    list[chrono.ChBody],
    list[str],
    int,
    list[chrono.ChBody],
    Any,
]:
    """Build the 3x3x3 lattice system with ball joints, collision ground, and top forces.

    Returns (system, all_bodies, all_names, joint_count, top_spheres, load_container).
    `load_container` is None in JOINT_MODE == "spherical" (the rigid baseline did
    not need one); in "bushing" mode it is the ChLoadContainer that owns all
    bushings and MUST be kept referenced by the caller for the duration of the
    sim run so Python GC does not reclaim it.
    """
    a = EDGE_LENGTH
    # Cube half-edge: the +Z/-Z face centre sits a/2 above/below the body
    # centre (the octahedron used a/sqrt(2) for its axial vertex).
    r = a / 2.0

    # Create system with Bullet collision solver
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    load_container: Any = None
    if JOINT_MODE == "spherical":
        # Keep the rigid baseline path byte-for-byte.
        system.GetSolver().AsIterative().SetMaxIterations(150)
    elif JOINT_MODE == "bushing":
        # ADMM instead of default PSOR: PSOR cannot handle the stiffness/damping
        # matrices contributed by ChLoadBodyBodyBushingSpherical, ADMM can.
        if bushing_k is None or bushing_c is None:
            raise ValueError(
                "build_system: JOINT_MODE='bushing' requires bushing_k and bushing_c"
            )
        solver = chrono.ChSolverADMM()
        # Matches the octahedron's tuned value (3x3x3_simulation: 30). With
        # 464 cuboid bushings vs 72 octahedron bushings the per-step ADMM cost
        # is already 6.4×; keeping MaxIter at the octahedron's settled point
        # avoids inflating wall-clock unnecessarily.
        solver.SetMaxIterations(30)
        system.SetSolver(solver)

        # Load container owns all bushing couplings; kept referenced by
        # build_system's caller so GC doesn't reclaim the Python handle during
        # the sim run.
        load_container = chrono.ChLoadContainer()
        system.Add(load_container)
    else:
        raise ValueError(f"Unknown JOINT_MODE: {JOINT_MODE!r}")

    # Ground body with collision floor at the diamond's bottom tip (Z = -a/√2).
    # The cubes are spun 45° about Y, so their lowest feature is the bottom tip at
    # -HALF_DIAGONAL (not the -r = -a/2 face centre); the bottom row rests on its
    # tips here, exactly as the octahedron rests on its bottom apex.
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(True)
    floor_z = -HALF_DIAGONAL
    ground_mat = chrono.ChContactMaterialNSC()
    ground_shape = chrono.ChCollisionShapeBox(ground_mat, 100.0, 100.0, 0.1)
    ground.AddCollisionShape(
        ground_shape,
        chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, floor_z - 0.05)),
    )
    system.AddBody(ground)

    # -------------------------------------------------------------------
    # Create 27 cubes on a 3x3x3 grid
    # -------------------------------------------------------------------
    grid_indices: list[tuple[int, int, int]] = []
    centers: list[tuple[float, float, float]] = []
    cube_bodies: list[chrono.ChBody] = []
    cube_names: list[str] = []

    # Diamond orientation: spin every cube 45° about +Y so its xz profile is a
    # diamond (see DIAMOND_TILT_RAD). Computed once, reused for all 27 cubes.
    diamond_rot = chrono.QuatFromAngleY(DIAMOND_TILT_RAD)

    for iz in range(NZ):
        for iy in range(NY):
            for ix in range(NX):
                grid_indices.append((ix, iy, iz))
                c = _grid_center(ix, iy, iz)
                centers.append(c)
                body = _make_cuboid_body(system, c, mass=1.0, a=a)
                body.SetRot(diamond_rot)
                cube_bodies.append(body)
                cube_names.append(_body_name_cube(ix, iy, iz))

    # -------------------------------------------------------------------
    # Detect shared vertices and create inter-cube bushing couplings
    # -------------------------------------------------------------------
    # Open strut frame: ONE bushing per axis-neighbour cube pair, anchored on the
    # centre-line at the midpoint of the inter-cube gap. This is the cube analogue
    # of the octahedron's single apex-to-apex joint — a force element with ZERO
    # rows in Φ_q, leaving the rotational freedom the lattice needs to buckle.
    # 18 pairs along each of X/Y/Z = 54 inter-cube couplings. See CLAUDE.md
    # "Joint Rule" and docs/joint_topology_cuboid.md.
    shared_pairs = _axis_neighbor_couplings(centers, grid_indices)
    inter_cube_count = 0

    # Container passed to _add_ball_joint depends on JOINT_MODE: the system for
    # rigid spherical joints (AddLink), the load_container for bushings (Add).
    coupling_container = load_container if JOINT_MODE == "bushing" else system

    for (idxA, idxB, vpos) in shared_pairs:
        _add_ball_joint(
            coupling_container,
            cube_bodies[idxA],
            cube_bodies[idxB],
            vpos,
            bushing_k,
            bushing_c,
        )
        inter_cube_count += 1

    joint_count = inter_cube_count
    print(f"  Inter-cube bushings: {inter_cube_count}")

    # -------------------------------------------------------------------
    # Per-column boundary conditions: 9 bottom spheres + 9 top spheres
    # -------------------------------------------------------------------
    # SINGLE-POINT SPHERE COUPLING
    # ----------------------------
    # Mirroring the octahedron lattice: each ghost / loading sphere sits at the
    # centre of the cube's bottom / top face and is tied to the cube by ONE
    # bushing at that point. A single point coupling leaves rotational freedom so
    # the loaded column can tilt and buckle (four corner bushings would weld the
    # sphere to the face and stiffen the column). The bushing is a force element
    # — 0 rows in Φ_q — and the ChForce on the loading sphere is transmitted
    # through it to the cube.
    #
    # Count: 9 bottom + 9 top spheres × 1 bushing = 18 sphere-BC bushings.
    bot_spheres: list[chrono.ChBody] = []
    top_spheres: list[chrono.ChBody] = []
    bot_names: list[str] = []
    top_names: list[str] = []
    sphere_bc_count = 0

    # Column ordering: row-major over (ix, iy), 9 columns total
    columns = [(ix, iy) for iy in range(NY) for ix in range(NX)]

    for col_idx, (cix, ciy) in enumerate(columns):
        # --- Bottom sphere: centre of bottom (-Z) face of iz=0 cube ---
        flat_bot = _grid_index_to_flat(cix, ciy, 0)
        bot_center = centers[flat_bot]
        bot_vertex = (bot_center[0], bot_center[1], bot_center[2] - r)

        bot_sph = _make_sphere_body(system, bot_vertex)
        bot_spheres.append(bot_sph)
        bot_names.append(f"bot_sphere_{cix}{ciy}")

        # Ghost sphere: no collision — cubes rest on ground via their own boxes

        # Single bushing: bottom sphere <-> iz=0 cube at the bottom-face centre.
        _add_ball_joint(
            coupling_container,
            bot_sph,
            cube_bodies[flat_bot],
            bot_vertex,
            bushing_k,
            bushing_c,
        )
        sphere_bc_count += 1

        # --- Top sphere: centre of top (+Z) face of iz=NZ-1 cube ---
        flat_top = _grid_index_to_flat(cix, ciy, NZ - 1)
        top_center = centers[flat_top]
        top_vertex = (top_center[0], top_center[1], top_center[2] + r)

        top_sph = _make_sphere_body(system, top_vertex)
        top_spheres.append(top_sph)
        top_names.append(f"top_sphere_{cix}{ciy}")

        # Single bushing: iz=NZ-1 cube <-> top sphere at the top-face centre.
        _add_ball_joint(
            coupling_container,
            cube_bodies[flat_top],
            top_sph,
            top_vertex,
            bushing_k,
            bushing_c,
        )
        sphere_bc_count += 1

        # Constant downward force on top sphere (ChForce must be added before configuring)
        top_force = chrono.ChForce()
        top_sph.AddForce(top_force)
        top_force.SetMode(chrono.ChForce.FORCE)
        top_force.SetAlign(chrono.ChForce.WORLD_DIR)
        top_force.SetDir(chrono.ChVector3d(0.0, 0.0, -1.0))
        top_force.SetMforce(top_force_n)

    joint_count = inter_cube_count + sphere_bc_count
    print(f"  Sphere-BC bushings (1 per sphere): {sphere_bc_count}")
    print(f"  Total bushings (inter-cube + sphere-BC): {joint_count}")

    # -------------------------------------------------------------------
    # Assemble body list and name list in canonical order
    # -------------------------------------------------------------------
    # Body IDs: 0-26 cubes, 27-35 bottom spheres, 36-44 top spheres
    all_bodies = cube_bodies + bot_spheres + top_spheres
    all_names = cube_names + bot_names + top_names
    print(f"  Total bodies: {len(all_bodies)}")

    # -------------------------------------------------------------------
    # Perturbation: random angular velocity on an interior cube (iz=1)
    # -------------------------------------------------------------------
    # Pick cube (0, 0, 1) = flat index 9 as the perturbed body
    perturb_flat = _grid_index_to_flat(0, 0, 1)
    rng = np.random.default_rng(perturbation_seed)
    perturb_dir = rng.standard_normal(3)
    perturb_dir /= np.linalg.norm(perturb_dir)
    omega = perturb_mag * perturb_dir  # rad/s; magnitude from parameter
    cube_bodies[perturb_flat].SetAngVelParent(
        chrono.ChVector3d(float(omega[0]), float(omega[1]), float(omega[2]))
    )

    return system, all_bodies, all_names, joint_count, top_spheres, load_container


# ---------------------------------------------------------------------------
# Simulation runner
# ---------------------------------------------------------------------------

def run_single(
    seed: int,
    csv_path: str,
    bushing_k: float | None = None,
    bushing_c: float | None = None,
    top_force_n: float = 0.5,
    perturb_mag: float = 0.02,
    dt: float | None = None,
    duration: float = 1.0e6,
    export_interval: int | None = None,
    wall_clock_budget_s: float = WALL_CLOCK_BUDGET_S,
) -> tuple[str, int]:
    """Build a fresh system, step it, and stream body states to csv_path.

    The step loop ends on whichever fires first: KE equilibrium, column
    collapse, or ``wall_clock_budget_s`` of real time. ``duration`` is a
    non-binding simulated-time ceiling (default ~1e6 s) so the wall-clock
    budget and early-stops govern; lower it only to force a short fixed run.

    In JOINT_MODE == "bushing", bushing_k / bushing_c must be supplied (the
    driver functions pass them from the active STIFFNESS_VARIANTS tuple).
    In "spherical" mode they are ignored.

    dt / export_interval default to the mode-appropriate values: bushing uses
    dt=1e-4, export_interval=125; spherical uses dt=5e-5, export_interval=250.
    """
    if JOINT_MODE == "bushing":
        if dt is None:
            dt = 1e-4
        if export_interval is None:
            export_interval = 125
    else:
        if dt is None:
            dt = 5e-5
        if export_interval is None:
            export_interval = 250

    system, bodies, body_names, joint_count, _top_spheres, _load_container = build_system(
        seed,
        bushing_k=bushing_k,
        bushing_c=bushing_c,
        top_force_n=top_force_n,
        perturb_mag=perturb_mag,
    )
    cube_bodies = bodies[:N_CUBE]  # first 27 bodies are cubes

    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    n_steps = int(duration / dt)
    stopped_early = False

    # Stream rows to disk as they are produced. A multi-hour wall-clock run would
    # otherwise accumulate millions of rows in memory; writing each export batch
    # immediately keeps memory bounded and the CSV byte-identical.
    csv_fh = open(csv_path, "w", newline="")
    writer = csv.writer(csv_fh)
    writer.writerow(["time", "body_id", "body_name", "x", "y", "z", "q0", "q1", "q2", "q3"])

    # Equilibrium detector: sustained low KE for N consecutive export checks.
    if JOINT_MODE == "bushing":
        # Bushing springs keep the assembly lightly oscillating at a KE floor
        # ~1e-2; threshold is raised to 0.1 with headroom above it. Tilt is now
        # measured RELATIVE to the 45° diamond start (see _cube_tilt_rad), so the
        # gate requires 30° of real buckling away from the rest pose before
        # equilibrium is declared.
        ke_threshold = 0.1
        min_tilt_deg = 30.0
    else:
        # Rigid spherical baseline.
        ke_threshold = 0.01
        min_tilt_deg = 30.0
    ke_consec_required = 40  # 40 * export_interval * dt = 0.50 s of sustained low KE
    ke_consec_count = 0

    # Wall-clock budget: cap the run at wall_clock_budget_s of real time, checked
    # only at export points (avoids a time.time() call every step).
    t_wall_start = time.time()

    for step_idx in range(n_steps):
        system.DoStepDynamics(dt)

        # Manual velocity damping (PyChrono lacks SetLinearDamping/SetAngularDamping)
        # Scale linear and angular velocities by 0.9999 every 10 steps (~0.01% per step)
        if step_idx % 10 == 0:
            for b in bodies:
                v = b.GetPosDt()
                b.SetPosDt(chrono.ChVector3d(v.x*0.9999, v.y*0.9999, v.z*0.9999))
                w = b.GetAngVelLocal()
                b.SetAngVelLocal(chrono.ChVector3d(w.x*0.9999, w.y*0.9999, w.z*0.9999))

        if step_idx % export_interval == 0:
            t = system.GetChTime()
            batch: list[list[Any]] = []
            for body_id, (body, name) in enumerate(zip(bodies, body_names)):
                p = body.GetPos()
                q = body.GetRot()
                batch.append([
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
            writer.writerows(batch)

            # Wall-clock budget stop
            elapsed = time.time() - t_wall_start
            if elapsed >= wall_clock_budget_s:
                print(f"  Wall-clock budget reached at t = {t:.6f} s (step {step_idx})"
                      f" — {elapsed / 3600:.2f} h >= {wall_clock_budget_s / 3600:.2f} h budget")
                stopped_early = True
                break

            # Check equilibrium: sustained low kinetic energy after initial transient
            if t > 2.0:
                # Require minimum tilt before checking equilibrium
                max_tilt = 0.0
                for cb in cube_bodies:
                    q = cb.GetRot()
                    tilt = _cube_tilt_rad(q.e0, q.e1, q.e2, q.e3)
                    max_tilt = max(max_tilt, tilt)
                if max_tilt > math.radians(min_tilt_deg):
                    total_ke = _compute_total_ke(bodies)
                    if total_ke < ke_threshold:
                        ke_consec_count += 1
                    else:
                        ke_consec_count = 0
                    if ke_consec_count >= ke_consec_required:
                        print(f"  Equilibrium reached at t = {t:.6f} s (step {step_idx})"
                              f" — KE < {ke_threshold} for {ke_consec_required} consecutive checks"
                              f" — max tilt = {math.degrees(max_tilt):.1f} deg")
                        stopped_early = True
                        break

            # Check column-collapse stop condition
            if _check_columns_collapsed(cube_bodies):
                print(f"  All columns collapsed at t = {t:.6f} s (step {step_idx})")
                stopped_early = True
                break

    csv_fh.close()

    if stopped_early:
        print("  Simulation stopped early.")
    else:
        print("  Simulation ran to completion (no early stop).")

    return csv_path, joint_count


def _default_output_root() -> str:
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "output",
    )


def run_all(
    output_dir: str | None = None,
    bushing_k: float | None = None,
    bushing_c: float | None = None,
    n_runs: int = 5,
    top_force_n: float = 0.5,
    perturb_mag: float = 0.02,
    wall_clock_budget_s: float = WALL_CLOCK_BUDGET_S,
) -> list[str]:
    """Run n_runs simulations (seeds 1..n_runs) into output_dir.

    In JOINT_MODE == "bushing", bushing_k / bushing_c must be supplied (the
    sweep drivers pass them from the active STIFFNESS_VARIANTS tuple). In
    "spherical" mode these are ignored and the rigid-baseline behavior is
    preserved.
    """
    if output_dir is None:
        output_dir = _default_output_root()
    os.makedirs(output_dir, exist_ok=True)
    paths: list[str] = []
    for i in range(1, n_runs + 1):
        csv_path = os.path.join(output_dir, f"sim_{i:03d}.csv")
        if JOINT_MODE == "bushing":
            print(
                f"Running simulation {i}/{n_runs} (seed={i}, K={bushing_k:.3g},"
                f" C={bushing_c:.3g}, F_top={top_force_n} N,"
                f" perturb={perturb_mag} rad/s) -> {csv_path}"
            )
        else:
            print(f"Running simulation {i}/{n_runs} (seed={i}) -> {csv_path}")
        result_path, _joint_count = run_single(
            seed=i,
            csv_path=csv_path,
            bushing_k=bushing_k,
            bushing_c=bushing_c,
            top_force_n=top_force_n,
            perturb_mag=perturb_mag,
            wall_clock_budget_s=wall_clock_budget_s,
        )
        paths.append(result_path)
        row_count = sum(1 for _ in open(csv_path)) - 1
        print(f"  Done. Rows written: {row_count}")
    return paths


def run_sweep(
    variants: list[tuple[str, float, float, float, float]] | None = None,
    output_root: str | None = None,
    n_runs: int = 5,
    wall_clock_budget_s: float = WALL_CLOCK_BUDGET_S,
) -> list[tuple[str, str]]:
    """Run the stiffness sweep: for each (label, K, C, F_top, perturb), run
    n_runs sims into output_root/<label>/. Returns a list of (label, output_dir)
    for downstream viz / plotting.
    """
    if variants is None:
        variants = STIFFNESS_VARIANTS
    if output_root is None:
        output_root = _default_output_root()

    results: list[tuple[str, str]] = []
    for label, k, c, f_top, p_mag in variants:
        variant_dir = os.path.join(output_root, label)
        print(
            f"\n=== Variant {label}: K={k:.3g} N/m, C={c:.3g} N·s/m,"
            f" F_top={f_top} N, perturb={p_mag} rad/s ==="
        )
        run_all(
            output_dir=variant_dir,
            bushing_k=k,
            bushing_c=c,
            n_runs=n_runs,
            top_force_n=f_top,
            perturb_mag=p_mag,
            wall_clock_budget_s=wall_clock_budget_s,
        )
        results.append((label, variant_dir))
    return results


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if JOINT_MODE == "bushing":
        results = run_sweep()
        print("\nStiffness sweep complete. Variant output directories:")
        for label, out_dir in results:
            print(f"  {label} -> {out_dir}")
    else:
        paths = run_all()
        print("\nOutput CSV files:")
        for p in paths:
            print(f"  {p}")
