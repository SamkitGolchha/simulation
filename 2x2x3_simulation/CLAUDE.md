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
1. physics — owns collision shapes, stop condition, and contact logic in src/simulation_2x2x3.py.
2. scaler — owns parameter tuning (force, dt, export interval, perturbation) in src/simulation_2x2x3.py.
3. gui-viz — owns src/visualizer.py and src/plot_tilts.py, and runs the viz/plot pipeline.

## Current Task — Collision + Stop Condition (run agents in order: scaler, physics, gui-viz)

### What's wrong
Octahedra have no collision geometry — they pass through each other. The structure
over-buckles past 150 deg after ~2s. The first ~2s of behavior look correct.

### What needs to happen
1. **scaler**: Reduce F_top from 5.0N to 0.5N, dt from 1e-4 to 5e-5, export_interval
   from 500 to 250, perturbation from 0.01 to 0.005 rad/s.
2. **physics**: Add collision shapes to top spheres. Add `_check_columns_collapsed()`
   stop condition — when adjacent octahedra in ALL 4 columns have both their axial (+Z/-Z)
   and equatorial (+X) vertices within tolerance (0.1), stop the simulation early.
3. **gui-viz**: After new CSVs are generated, re-render MP4 videos and tilt angle plots.

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

**4 bottom ghost spheres** — placed at the -Z vertex of each iz=0 octahedron:
- Ghost bodies: exist for joint connectivity but have **no collision shapes**.
- Octahedra rest on the ground directly via their own convex-hull collision shapes.
- Connected to their octahedron via ball joint.

**4 top spheres** — placed at the +Z vertex of each iz=2 octahedron:
- NO motors. Instead, a **constant downward force** (0.5 N per sphere) is applied
  via persistent `ChForce` objects.
- As the structure tilts, the vertical force naturally decomposes into axial + lateral
  components, driving cooperative buckling.
- Connected to their octahedron via ball joint.

**Ground plane implementation** (PyChrono collision):
- Ground body with a large box collision shape at the floor level
- Octahedra have convex-hull collision shapes (rest directly on ground)
- Bottom spheres are ghost bodies (no collision) — joint-only connectivity
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