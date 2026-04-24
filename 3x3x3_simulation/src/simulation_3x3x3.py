# PyChrono rigid-body simulation of a 3x3x3 lattice of 27 octahedra connected by ball joints.
# Ghost Spheres — bottom spheres remain as bodies but their collision is removed.
# Octahedra rest on the ground directly via their own convex-hull collision shapes.
# ChForce loads top spheres downward.
# Runs 5 independent simulations with random perturbations on an interior octahedron, exports CSV.
# Body IDs: 0-26 octahedra, 27-35 bottom ghost spheres, 36-44 top loading spheres.

import csv
import math
import os
import sys
from typing import Any, Literal

import numpy as np
import pychrono as chrono

# ---------------------------------------------------------------------------
# Joint formulation switch
# ---------------------------------------------------------------------------
# "spherical": rigid ChLinkLockSpherical — setup333 baseline on main; at 3x3x3
#   the oct-only subsystem is rank-deficient on rigid modes (F_oct = 0), which
#   is the point of preserving this path as empirical evidence.
# "bushing":   compliant ChLoadBodyBodyBushingSpherical — ported from the
#   committed 2x2x3 refactor (2026-04-21). Zero rows in the constraint
#   Jacobian, so F_bushing = 6·B identically at any grid size. Requires
#   ChSolverADMM (PSOR rejects the K/C matrices at runtime).
JOINT_MODE: Literal["spherical", "bushing"] = "bushing"

# ---------------------------------------------------------------------------
# Geometry helpers (inlined from Sanity_Test/src/simulation.py)
# ---------------------------------------------------------------------------

def octahedron_vertices(center: tuple[float, float, float], a: float = 1.0) -> np.ndarray:
    """Return 6x3 array of vertex coordinates for a regular octahedron centred at center."""
    cx, cy, cz = center
    r = a / math.sqrt(2.0)
    return np.array([
        [cx + r,  cy,       cz      ],  # 0: +X equatorial
        [cx - r,  cy,       cz      ],  # 1: -X equatorial
        [cx,      cy + r,   cz      ],  # 2: +Y equatorial
        [cx,      cy - r,   cz      ],  # 3: -Y equatorial
        [cx,      cy,       cz + r  ],  # 4: top vertex (+Z)
        [cx,      cy,       cz - r  ],  # 5: bottom vertex (-Z)
    ], dtype=float)


def octahedron_faces() -> list[tuple[int, int, int]]:
    """Return the 8 triangular face index tuples for a regular octahedron (vertex indices)."""
    return [
        (0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4),  # upper 4 faces
        (0, 3, 5), (3, 1, 5), (1, 2, 5), (2, 0, 5),  # lower 4 faces
    ]


def octahedron_inertia(mass: float, a: float = 1.0) -> tuple[float, float, float]:
    """Return (Ixx, Iyy, Izz) for a uniform solid regular octahedron; all equal by symmetry."""
    I = (1.0 / 10.0) * mass * a * a
    return (I, I, I)


# ---------------------------------------------------------------------------
# Lattice constants
# ---------------------------------------------------------------------------
EDGE_LENGTH: float = 1.0
SPACING: float = EDGE_LENGTH * math.sqrt(2.0)  # centre-to-centre distance
NX: int = 3
NY: int = 3
NZ: int = 3
N_OCT: int = NX * NY * NZ  # 27


# ---------------------------------------------------------------------------
# Bushing stiffness sweep (JOINT_MODE = "bushing" only)
# ---------------------------------------------------------------------------
# Each shared-vertex coupling is a ChLoadBodyBodyBushingSpherical (force-based
# 3-axis spring, no rows in the constraint Jacobian). The sweep runs each
# (label, K, C) tuple as an independent batch of 5 seeds. Damping is tuned so
# ζ = C / (2·sqrt(m·K)) ≈ 0.5 (m = 1 kg per octahedron).
STIFFNESS_VARIANTS: list[tuple[str, float, float]] = [
    ("bushing_K1e4", 1.0e4, 1.0e2),
]

# change_2 variants: same bushing formulation, but with the top load and initial
# perturbation cranked up so the structure actually buckles. Used when the
# baseline run leaves tilts under the 10° gate and stabilises (the 2×2×3
# contingency trigger rule — see 3x3x3_plan.md §4.2). Output lives under
# output/change_2/<label>/.
CHANGE2_VARIANTS: list[tuple[str, float, float, float, float]] = [
    # (label, K, C, F_top[N], perturb[rad/s])
    ("K1e4_F3_P0_1", 1.0e4, 1.0e2, 3.0, 0.1),
]


# ---------------------------------------------------------------------------
# Body creation helpers
# ---------------------------------------------------------------------------

def _make_octahedron_body(
    system: chrono.ChSystemNSC,
    center: tuple[float, float, float],
    mass: float = 1.0,
    a: float = EDGE_LENGTH,
) -> chrono.ChBody:
    """Create a ChBody for a regular octahedron with convex-hull collision for edge-edge contact stops."""
    body = chrono.ChBody()
    body.SetPos(chrono.ChVector3d(center[0], center[1], center[2]))
    body.SetMass(mass)
    Ixx, Iyy, Izz = octahedron_inertia(mass, a)
    body.SetInertiaXX(chrono.ChVector3d(Ixx, Iyy, Izz))
    body.SetInertiaXY(chrono.ChVector3d(0.0, 0.0, 0.0))

    # Convex-hull collision shape for edge-edge contact stops during tilting
    r = a / math.sqrt(2.0)
    pts = chrono.vector_ChVector3d()
    for lx, ly, lz in [(r,0,0),(-r,0,0),(0,r,0),(0,-r,0),(0,0,r),(0,0,-r)]:
        pts.push_back(chrono.ChVector3d(lx, ly, lz))
    col_mat = chrono.ChContactMaterialNSC()
    col_mat.SetFriction(0.5)
    col_mat.SetRestitution(0.0)
    hull = chrono.ChCollisionShapeConvexHull(col_mat, pts)
    body.AddCollisionShape(hull)
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
        system.AddLink(). This reproduces setup333's main-branch behavior
        byte-for-byte — flipping JOINT_MODE back to "spherical" must be the
        only edit required to recover that baseline.

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
    """Return the world-frame centre of the octahedron at grid position (ix, iy, iz)."""
    return (ix * SPACING, iy * SPACING, iz * SPACING)


def _body_name_oct(ix: int, iy: int, iz: int) -> str:
    """Return a human-readable name for the octahedron at (ix,iy,iz)."""
    return f"oct_{ix}{iy}{iz}"


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
# Column-collapse stop condition
# ---------------------------------------------------------------------------

def _check_columns_collapsed(
    oct_bodies: list[chrono.ChBody],
    a: float = EDGE_LENGTH,
    tol: float = 0.1,
) -> bool:
    """Return True when all 9 columns have all adjacent pairs collapsed (vertices touching)."""
    r = a / math.sqrt(2.0)
    columns = [(ix, iy) for iy in range(NY) for ix in range(NX)]

    for (ix, iy) in columns:
        column_collapsed = True
        for iz in range(NZ - 1):  # pairs (iz, iz+1)
            lower = _grid_index_to_flat(ix, iy, iz)
            upper = _grid_index_to_flat(ix, iy, iz + 1)

            # +Z vertex of lower body in world frame
            lower_top = oct_bodies[lower].TransformPointLocalToParent(
                chrono.ChVector3d(0, 0, r)
            )
            # -Z vertex of upper body in world frame
            upper_bot = oct_bodies[upper].TransformPointLocalToParent(
                chrono.ChVector3d(0, 0, -r)
            )
            axial_dist = (lower_top - upper_bot).Length()

            # +X vertex of both bodies in world frame
            lower_x = oct_bodies[lower].TransformPointLocalToParent(
                chrono.ChVector3d(r, 0, 0)
            )
            upper_x = oct_bodies[upper].TransformPointLocalToParent(
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
    r = a / math.sqrt(2.0)

    # Create system with Bullet collision solver
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    load_container: Any = None
    if JOINT_MODE == "spherical":
        # Keep the setup333 baseline path byte-for-byte.
        system.GetSolver().AsIterative().SetMaxIterations(150)
    elif JOINT_MODE == "bushing":
        # ADMM instead of default PSOR: PSOR cannot handle the stiffness/damping
        # matrices contributed by ChLoadBodyBodyBushingSpherical, ADMM can.
        if bushing_k is None or bushing_c is None:
            raise ValueError(
                "build_system: JOINT_MODE='bushing' requires bushing_k and bushing_c"
            )
        solver = chrono.ChSolverADMM()
        solver.SetMaxIterations(30)
        system.SetSolver(solver)

        # Load container owns all bushing couplings; kept referenced by
        # build_system's caller so GC doesn't reclaim the Python handle during
        # the sim run.
        load_container = chrono.ChLoadContainer()
        system.Add(load_container)
    else:
        raise ValueError(f"Unknown JOINT_MODE: {JOINT_MODE!r}")

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
    # Create 27 octahedra on a 3x3x3 grid
    # -------------------------------------------------------------------
    grid_indices: list[tuple[int, int, int]] = []
    centers: list[tuple[float, float, float]] = []
    oct_bodies: list[chrono.ChBody] = []
    oct_names: list[str] = []

    for iz in range(NZ):
        for iy in range(NY):
            for ix in range(NX):
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

    # Container passed to _add_ball_joint depends on JOINT_MODE: the system for
    # rigid spherical joints (AddLink), the load_container for bushings (Add).
    coupling_container = load_container if JOINT_MODE == "bushing" else system

    for (idxA, idxB, vpos) in shared_pairs:
        _add_ball_joint(
            coupling_container,
            oct_bodies[idxA],
            oct_bodies[idxB],
            vpos,
            bushing_k,
            bushing_c,
        )
        joint_count += 1

    print(f"  Inter-octahedron ball joints: {joint_count}")

    # -------------------------------------------------------------------
    # Per-column boundary conditions: 9 bottom spheres + 9 top spheres
    # -------------------------------------------------------------------
    bot_spheres: list[chrono.ChBody] = []
    top_spheres: list[chrono.ChBody] = []
    bot_names: list[str] = []
    top_names: list[str] = []

    # Column ordering: row-major over (ix, iy), 9 columns total
    columns = [(ix, iy) for iy in range(NY) for ix in range(NX)]

    for col_idx, (cix, ciy) in enumerate(columns):
        # --- Bottom sphere: -Z vertex of iz=0 octahedron in this column ---
        flat_bot = _grid_index_to_flat(cix, ciy, 0)
        bot_center = centers[flat_bot]
        bot_vertex = (bot_center[0], bot_center[1], bot_center[2] - r)

        bot_sph = _make_sphere_body(system, bot_vertex)
        bot_spheres.append(bot_sph)
        bot_names.append(f"bot_sphere_{cix}{ciy}")

        # Ghost sphere: no collision — octahedra rest on ground via their own hulls

        # Ball joint: bottom sphere <-> iz=0 octahedron
        _add_ball_joint(
            coupling_container,
            bot_sph,
            oct_bodies[flat_bot],
            bot_vertex,
            bushing_k,
            bushing_c,
        )
        joint_count += 1

        # --- Top sphere: +Z vertex of iz=NZ-1 octahedron in this column ---
        flat_top = _grid_index_to_flat(cix, ciy, NZ - 1)
        top_center = centers[flat_top]
        top_vertex = (top_center[0], top_center[1], top_center[2] + r)

        top_sph = _make_sphere_body(system, top_vertex)
        top_spheres.append(top_sph)
        top_names.append(f"top_sphere_{cix}{ciy}")

        # Ball joint: iz=NZ-1 octahedron <-> top sphere
        _add_ball_joint(
            coupling_container,
            oct_bodies[flat_top],
            top_sph,
            top_vertex,
            bushing_k,
            bushing_c,
        )
        joint_count += 1

        # Constant downward force on top sphere (ChForce must be added before configuring)
        top_force = chrono.ChForce()
        top_sph.AddForce(top_force)
        top_force.SetMode(chrono.ChForce.FORCE)
        top_force.SetAlign(chrono.ChForce.WORLD_DIR)
        top_force.SetDir(chrono.ChVector3d(0.0, 0.0, -1.0))
        top_force.SetMforce(top_force_n)

    print(f"  Total ball joints (inter-oct + sphere-oct): {joint_count}")

    # -------------------------------------------------------------------
    # Assemble body list and name list in canonical order
    # -------------------------------------------------------------------
    # Body IDs: 0-26 octahedra, 27-35 bottom spheres, 36-44 top spheres
    all_bodies = oct_bodies + bot_spheres + top_spheres
    all_names = oct_names + bot_names + top_names
    print(f"  Total bodies: {len(all_bodies)}")

    # -------------------------------------------------------------------
    # Perturbation: random angular velocity on an interior octahedron (iz=1)
    # -------------------------------------------------------------------
    # Pick octahedron (0, 0, 1) = flat index 9 as the perturbed body
    perturb_flat = _grid_index_to_flat(0, 0, 1)
    rng = np.random.default_rng(perturbation_seed)
    perturb_dir = rng.standard_normal(3)
    perturb_dir /= np.linalg.norm(perturb_dir)
    omega = perturb_mag * perturb_dir  # rad/s; magnitude from parameter
    oct_bodies[perturb_flat].SetAngVelParent(
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
    duration: float = 10.0,
    export_interval: int | None = None,
) -> tuple[str, int]:
    """Build a fresh system, step it for duration seconds, write body states to csv_path.

    In JOINT_MODE == "bushing", bushing_k / bushing_c must be supplied (the
    driver functions pass them from the active STIFFNESS_VARIANTS /
    CHANGE2_VARIANTS tuple). In "spherical" mode they are ignored.

    dt / export_interval default to the mode-appropriate values ported verbatim
    from the 2x2x3 baselines: bushing uses dt=1e-4, export_interval=125 (matches
    committed 2x2x3 bushing refactor); spherical uses dt=5e-5,
    export_interval=250 (matches setup333 rigid baseline).
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
    oct_bodies = bodies[:N_OCT]  # first 27 bodies are octahedra

    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    n_steps = int(duration / dt)
    rows: list[list[Any]] = []
    stopped_early = False

    # Equilibrium detector: sustained low KE for N consecutive export checks.
    # Values are mode-dependent and ported verbatim from the respective 2x2x3
    # baselines — see hard constraint in joints333 brief: do not tune KE
    # thresholds away from the ported 2x2x3 values.
    if JOINT_MODE == "bushing":
        # Bushing springs keep the assembly lightly oscillating at a KE floor
        # ~1e-2; threshold is raised to 0.1 with headroom above it. Tilt gate
        # dropped to 10° (matches the regime where bushing collapses stabilise).
        ke_threshold = 0.1
        min_tilt_deg = 10.0
    else:
        # Rigid spherical baseline — unchanged from setup333's main-branch run.
        ke_threshold = 0.01
        min_tilt_deg = 30.0
    ke_consec_required = 40  # 40 * export_interval * dt = 0.50 s of sustained low KE
    ke_consec_count = 0

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

            # Check equilibrium: sustained low kinetic energy after initial transient
            if t > 2.0:
                # Require minimum tilt before checking equilibrium
                max_tilt = 0.0
                for ob in oct_bodies:
                    q = ob.GetRot()
                    z_z = 1.0 - 2.0 * (q.e1**2 + q.e2**2)
                    tilt = math.acos(max(-1.0, min(1.0, z_z)))
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
            if _check_columns_collapsed(oct_bodies):
                print(f"  All columns collapsed at t = {t:.6f} s (step {step_idx})")
                stopped_early = True
                break

    if stopped_early:
        print("  Simulation stopped early.")
    else:
        print("  Simulation ran to completion (no early stop).")

    with open(csv_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["time", "body_id", "body_name", "x", "y", "z", "q0", "q1", "q2", "q3"])
        writer.writerows(rows)

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
) -> list[str]:
    """Run n_runs simulations (seeds 1..n_runs) into output_dir.

    In JOINT_MODE == "bushing", bushing_k / bushing_c must be supplied (the
    sweep drivers pass them from the active STIFFNESS_VARIANTS /
    CHANGE2_VARIANTS tuple). In "spherical" mode these are ignored and the
    rigid-baseline behavior is preserved.
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
        )
        paths.append(result_path)
        row_count = sum(1 for _ in open(csv_path)) - 1
        print(f"  Done. Rows written: {row_count}")
    return paths


def run_sweep(
    variants: list[tuple[str, float, float]] | None = None,
    output_root: str | None = None,
    n_runs: int = 5,
) -> list[tuple[str, str]]:
    """Run the stiffness sweep: for each (label, K, C), run n_runs sims into
    output_root/<label>/. Returns a list of (label, output_dir) for downstream
    viz / plotting.
    """
    if variants is None:
        variants = STIFFNESS_VARIANTS
    if output_root is None:
        output_root = _default_output_root()

    results: list[tuple[str, str]] = []
    for label, k, c in variants:
        variant_dir = os.path.join(output_root, label)
        print(f"\n=== Variant {label}: K={k:.3g} N/m, C={c:.3g} N·s/m ===")
        run_all(output_dir=variant_dir, bushing_k=k, bushing_c=c, n_runs=n_runs)
        results.append((label, variant_dir))
    return results


def run_change2_sweep(
    variants: list[tuple[str, float, float, float, float]] | None = None,
    output_root: str | None = None,
    n_runs: int = 5,
) -> list[tuple[str, str]]:
    """Run the change_2 sweep: 5-tuple variants (label, K, C, F_top, perturb)
    into output_root/<label>/. Output root defaults to
    <project>/output/change_2. Used when the baseline STIFFNESS_VARIANTS run
    leaves tilts under the 10° gate and stabilises — the contingency trigger
    rule ported from 2×2×3 (see 3x3x3_plan.md §4.2).
    """
    if variants is None:
        variants = CHANGE2_VARIANTS
    if output_root is None:
        output_root = os.path.join(_default_output_root(), "change_2")

    results: list[tuple[str, str]] = []
    for label, k, c, f_top, p_mag in variants:
        variant_dir = os.path.join(output_root, label)
        print(
            f"\n=== change_2 variant {label}: K={k:.3g} N/m, C={c:.3g} N·s/m,"
            f" F_top={f_top} N, perturb={p_mag} rad/s ==="
        )
        run_all(
            output_dir=variant_dir,
            bushing_k=k,
            bushing_c=c,
            n_runs=n_runs,
            top_force_n=f_top,
            perturb_mag=p_mag,
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
