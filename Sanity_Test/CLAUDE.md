# Octahedral Tilt Simulation

## What This Project Is
A PyChrono rigid-body simulation of 3 octahedra stacked vertically, connected by
spherical (ball) joints at shared vertices, with small marker spheres at the top and
bottom vertices. The bottom sphere is fixed to the ground (acts as the floor). The top
sphere has a constant-velocity downward vertical displacement applied to it.
The simulation is a single continuous script that runs 5 times with different random
perturbations, producing: (1) an MP4 video of one run, and (2) a matplotlib graph
of tilt angle vs. time for each octahedron across all 5 runs.

## Tech Stack
- **Physics engine:** PyChrono (pychrono) via conda. Use `ChSystemNSC` with default solver.
- **Visualization:** PyVista for offline 3D rendering → ffmpeg for MP4.
- **Plotting:** Matplotlib for tilt angle graphs.
- **Math:** NumPy + SciPy (`scipy.spatial.transform.Rotation`) for quaternion-to-Euler.

## Architecture
Two subagents work in parallel:
1. `physics-engine` — builds `src/simulation.py` (the single complete simulation file).
2. `gui-viz` — builds `src/visualizer.py` and `src/plot_tilts.py`.

Do NOT write test cases. The user will evaluate the output and decide on tests later.

## Code Conventions
- Every `.py` file: exactly 3-line comment at top describing the file's purpose.
- Every function: exactly 1-line comment at top of the function body.
- Use type hints on all function signatures.
- No classes unless genuinely needed. Prefer pure functions.

## Physical Setup

### Bodies
- **3 regular octahedra** stacked along the Z-axis, edge length `a = 1.0`.
- Center-to-center spacing: `a * sqrt(2)` ≈ 1.414 along Z.
- The top vertex of octahedron N coincides with the bottom vertex of octahedron N+1.
- **Bottom marker sphere** (radius = 0.05, mass ≈ 0.001): placed at the bottom vertex
  of octahedron 1 (the lowest point of the entire stack). This sphere is **fixed to
  ground** via `ChLinkMateFix` — it is the floor anchor for the entire system.
- **Top marker sphere** (radius = 0.05, mass ≈ 0.001): placed at the top vertex of
  octahedron 3 (the highest point of the entire stack). This sphere is **driven
  downward** via a linear motor.

### Constraints & Loading
- At each shared vertex between adjacent octahedra, a `ChLinkLockSpherical` ball joint
  connects the two octahedra.
- A `ChLinkLockSpherical` also connects the bottom sphere to octahedron 1 at their
  shared vertex, and the top sphere to octahedron 3 at their shared vertex.
- The **bottom sphere is grounded**: fixed to the world frame via `ChLinkMateFix`
  to a ground body. This is the only fixed constraint. The octahedra themselves are
  NOT directly fixed to ground — they hang off the bottom sphere through the joint chain.
- The **top sphere is driven downward**: attach a `ChLinkMotorLinearSpeed` between
  the top sphere and the ground body, oriented along the negative Z-axis. Set a
  constant descent speed of 0.05 units/second. This prescribes vertical translation
  on the top vertex — the rest of the chain must accommodate through the joints.

### Simulation Parameters
- Timestep: `dt = 1e-4` seconds.
- Duration: `T = 2.0` seconds.
- Export interval: every 100 steps (every 0.01s → 200 frames per run).
- **5 runs total**, each with a different small random angular velocity perturbation
  on the middle octahedron (magnitude ~ 0.01 rad/s, random direction) to break
  symmetry.
- Each run rebuilds the system from scratch (clean state).

### Output Format
- `output/sim_NNN.csv` (NNN = 001 to 005): columns = `time,body_id,body_name,x,y,z,q0,q1,q2,q3`
  - body_id 0,1,2 = octahedra bottom to top
  - body_id 3 = bottom sphere
  - body_id 4 = top sphere
- `output/collapse.mp4`: 30fps video from run 001.
- `output/tilt_angles.png`: 3 vertically stacked subplots (one per octahedron), each
  showing tilt angle (degrees) vs time (seconds) with 5 overlaid lines (one per run).

## Key PyChrono API Notes
- `chrono.ChSystemNSC()` — open chain, no over-constraint.
- Spherical joint: `joint = chrono.ChLinkLockSpherical(); joint.Initialize(bodyA, bodyB, chrono.ChFramed(chrono.ChVector3d(x,y,z)))`.
- Linear motor for displacement: create `ChLinkMotorLinearSpeed` between top sphere
  and ground. Orient the motor frame so its Z-axis points along global Z. Use
  `motor.SetSpeedFunction(chrono.ChFunctionConst(-0.5))` for constant downward speed.
- Get quaternion: `q = body.GetRot()` → `q.e0, q.e1, q.e2, q.e3`.
- Get position: `p = body.GetPos()` → `p.x, p.y, p.z`.
- Tilt angle from quaternion: `angle = arccos(clip(R.apply([0,0,1])[2], -1, 1))`
  where `R = Rotation.from_quat([q1, q2, q3, q0])` (scipy scalar-last convention).

- Follow existing code conventions
- For every file write three lines summarising what you are doing in the file
- For every function write one line to summarise what the function does
