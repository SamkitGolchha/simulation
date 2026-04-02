# PyChrono rigid-body simulation of 3 stacked octahedra connected by spherical joints.
# Applies a downward linear motor on the top marker sphere and grounds the bottom sphere.
# Runs 5 independent simulations with random middle-octahedron perturbations, exports CSV.

import csv
import os
import math
import numpy as np
import pychrono as chrono

# ---------------------------------------------------------------------------
# Geometry helpers
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
# Body creation helpers
# ---------------------------------------------------------------------------

def _make_octahedron_body(
    sys: chrono.ChSystemNSC,
    center: tuple[float, float, float],
    mass: float = 1.0,
    a: float = 1.0,
) -> chrono.ChBody:
    """Create a ChBody representing a regular octahedron, add it to sys, and return it."""
    body = chrono.ChBody()
    body.SetPos(chrono.ChVector3d(center[0], center[1], center[2]))
    body.SetMass(mass)
    Ixx, Iyy, Izz = octahedron_inertia(mass, a)
    body.SetInertiaXX(chrono.ChVector3d(Ixx, Iyy, Izz))
    body.SetInertiaXY(chrono.ChVector3d(0.0, 0.0, 0.0))
    sys.AddBody(body)
    return body


def _make_sphere_body(
    sys: chrono.ChSystemNSC,
    pos: tuple[float, float, float],
    radius: float = 0.05,
    mass: float = 0.001,
) -> chrono.ChBody:
    """Create a small sphere ChBody at pos, add it to sys, and return it."""
    body = chrono.ChBody()
    body.SetPos(chrono.ChVector3d(pos[0], pos[1], pos[2]))
    body.SetMass(mass)
    I_sphere = (2.0 / 5.0) * mass * radius * radius
    body.SetInertiaXX(chrono.ChVector3d(I_sphere, I_sphere, I_sphere))
    body.SetInertiaXY(chrono.ChVector3d(0.0, 0.0, 0.0))
    sys.AddBody(body)
    return body


def _add_ball_joint(
    sys: chrono.ChSystemNSC,
    bodyA: chrono.ChBody,
    bodyB: chrono.ChBody,
    joint_pos: tuple[float, float, float],
) -> chrono.ChLinkLockSpherical:
    """Create and add a spherical (ball) joint between bodyA and bodyB at joint_pos."""
    joint = chrono.ChLinkLockSpherical()
    frame = chrono.ChFramed(chrono.ChVector3d(joint_pos[0], joint_pos[1], joint_pos[2]))
    joint.Initialize(bodyA, bodyB, frame)
    sys.AddLink(joint)
    return joint


# ---------------------------------------------------------------------------
# Main build function
# ---------------------------------------------------------------------------

def build_system(
    perturbation_seed: int,
) -> tuple[chrono.ChSystemNSC, list[chrono.ChBody]]:
    """Build the ChSystemNSC with 3 octahedra, 2 marker spheres, joints, and motor."""
    a = 1.0
    r = a / math.sqrt(2.0)          # vertex offset from centre along each axis
    spacing = a * math.sqrt(2.0)    # centre-to-centre gap = 2r

    # Octahedra centres stacked along Z
    c1 = (0.0, 0.0, 0.0 * spacing)
    c2 = (0.0, 0.0, 1.0 * spacing)
    c3 = (0.0, 0.0, 2.0 * spacing)

    # Shared vertex positions
    v1_top = (0.0, 0.0, c1[2] + r)   # top of oct1 == bottom of oct2
    v2_top = (0.0, 0.0, c2[2] + r)   # top of oct2 == bottom of oct3

    # Bottom vertex of oct1 (where bottom sphere sits)
    v1_bot = (0.0, 0.0, c1[2] - r)
    # Top vertex of oct3 (where top sphere sits)
    v3_top = (0.0, 0.0, c3[2] + r)

    # -----------------------------------------------------------------------
    # Create system
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))

    # Ground body (fixed anchor)
    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    # Octahedra bodies
    oct1 = _make_octahedron_body(sys, c1, mass=1.0, a=a)
    oct2 = _make_octahedron_body(sys, c2, mass=1.0, a=a)
    oct3 = _make_octahedron_body(sys, c3, mass=1.0, a=a)

    # Marker spheres
    bot_sphere = _make_sphere_body(sys, v1_bot)
    top_sphere = _make_sphere_body(sys, v3_top)

    # -----------------------------------------------------------------------
    # Spherical joints at shared vertices between adjacent octahedra
    _add_ball_joint(sys, oct1, oct2, v1_top)
    _add_ball_joint(sys, oct2, oct3, v2_top)

    # Spherical joints connecting marker spheres to their adjacent octahedra
    _add_ball_joint(sys, bot_sphere, oct1, v1_bot)
    _add_ball_joint(sys, oct3, top_sphere, v3_top)

    # -----------------------------------------------------------------------
    # Ground the bottom sphere via ChLinkMateFix
    fix_link = chrono.ChLinkMateFix()
    fix_link.Initialize(bot_sphere, ground)
    sys.AddLink(fix_link)

    # -----------------------------------------------------------------------
    # Drive the top sphere downward via ChLinkMotorLinearSpeed
    # Motor frame at the top sphere position; default Z-axis = global Z
    motor = chrono.ChLinkMotorLinearSpeed()
    motor_frame = chrono.ChFramed(
        chrono.ChVector3d(v3_top[0], v3_top[1], v3_top[2])
    )
    motor.Initialize(top_sphere, ground, motor_frame)
    motor.SetSpeedFunction(chrono.ChFunctionConst(-0.05))
    sys.AddLink(motor)

    # -----------------------------------------------------------------------
    # Apply random angular velocity perturbation to the middle octahedron
    rng = np.random.default_rng(perturbation_seed)
    perturb_dir = rng.standard_normal(3)
    perturb_dir /= np.linalg.norm(perturb_dir)
    perturb_mag = 0.01  # rad/s
    omega = perturb_mag * perturb_dir
    oct2.SetAngVelParent(chrono.ChVector3d(float(omega[0]), float(omega[1]), float(omega[2])))

    return sys, [oct1, oct2, oct3, bot_sphere, top_sphere]


# ---------------------------------------------------------------------------
# Simulation runner
# ---------------------------------------------------------------------------

BODY_NAMES = ["oct1", "oct2", "oct3", "bot_sphere", "top_sphere"]


def run_single(
    seed: int,
    csv_path: str,
    dt: float = 1e-4,
    duration: float = 2.0,
    export_interval: int = 100,
) -> str:
    """Build a fresh system, step it for duration seconds, write body states to csv_path."""
    sys, bodies = build_system(seed)

    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    n_steps = int(duration / dt)
    rows: list[list] = []

    for step_idx in range(n_steps):
        sys.DoStepDynamics(dt)

        if step_idx % export_interval == 0:
            t = sys.GetChTime()
            for body_id, (body, name) in enumerate(zip(bodies, BODY_NAMES)):
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

    return csv_path


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
        run_single(seed=i, csv_path=csv_path)
        paths.append(csv_path)
        print(f"  Done. Rows written: {sum(1 for _ in open(csv_path)) - 1}")
    return paths


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    paths = run_all(n_runs=5)
    print("\nOutput CSV files:")
    for p in paths:
        print(f"  {p}")
