# 2×2×3 Octahedral Array Simulation

## What This Project Is
Scales the validated Sanity_Test to a 2×2×3 grid of 12 octahedra. The physics,
code conventions, and output format are identical to the Sanity_Test — the only
difference is that all vertices shared between any pair of octahedra in the 3D grid
get a ball joint, not just vertical stack neighbors.

## Reference
Read ../Sanity_Test/CLAUDE.md for: code conventions, PyChrono API patterns, CSV format,
tilt angle computation, and progress tracking rules. Everything there applies here.

## Architecture — Three Agents
1. physics — owns the constraint/motor fix in src/simulation_2x2x3.py.
2. scaler — owns lattice geometry and shared-vertex detection in src/simulation_2x2x3.py.
3. gui-viz — owns src/visualizer.py and src/plot_tilts.py only.

Do NOT write test cases. Do NOT modify anything in ../Sanity_Test/.

## Progress Tracking
Same rules as Sanity_Test. Single progress.md at this project's root. Append after
every completed task.

## Physical Setup

### 2×2×3 Lattice
12 octahedra arranged in a grid with 2 along X, 2 along Y, and 3 along Z. Edge
length 1.0. Center-to-center spacing is edge_length × sqrt(2) along each axis.

### Joint Rule — The Key Difference
For every pair of octahedra in the grid, compute their world-frame vertices and check
for coincidence. At every coincident vertex, create a
spherical ball joint connecting those two bodies. This produces significantly more
joints than the sanity test — log the exact count in progress.md.

### Boundary Conditions — Force-Based (No Kinematic Constraints)
All bodies are free to move in all directions. Loading is applied via forces, not motors
or fixed constraints. Ball joints keep the structure connected.

**4 bottom spheres** — placed at the -Z vertex of each iz=0 octahedron:
- NO `ChLinkMateFix`. Instead, a **collision ground plane** at Z = initial bottom sphere Z
  prevents penetration below the floor. The contact solver provides upward reaction
  automatically. Bottom spheres are free to slide in X,Y and lift in Z.
- Connected to their octahedron via ball joint.

**4 top spheres** — placed at the +Z vertex of each iz=2 octahedron:
- NO motors. Instead, a **constant downward force** (~5 N, tunable) is applied each
  timestep via `body.Accumulate_force(chrono.ChVector3d(0, 0, -F_top), False)`.
- As the structure tilts, the vertical force naturally decomposes into axial + lateral
  components, driving cooperative buckling.
- Connected to their octahedron via ball joint.

**Ground plane implementation** (PyChrono collision):
- Ground body with a large box collision shape at the floor level
- Bottom sphere bodies have sphere collision shapes enabled
- ChSystemNSC contact solver handles the reaction forces

**DOF count:**
- 20 free bodies × 6 DOFs = 120
- 20 inter-oct ball joints + 8 sphere ball joints = 84 constraints
- Ground plane: contact-based (only activates on penetration, not a DOF lock)
- **Net DOFs = 36** — all layers free to tilt

### Simulation Parameters
dt 1e-4, duration 10.0s, export every 500 steps (200 frames), 5 runs with random
perturbations on an interior octahedron. Motor speed: 0.01 units/s (slower than
Sanity_Test to reduce solver stress with the larger constraint system).

### Output
Same CSV format. Octahedra are body_id 0 through 11. Bottom spheres are body_id
12–15, top spheres are 16–19 (20 bodies total).


MP4 video from all run csv files and tilt-angle PNG from all 5 runs for the column on 0,0.

### run.py
Same pattern as Sanity_Test: runs simulation, then visualizer on first CSV, then
plotter on output directory.