# 2×2×3 Octahedral Array Simulation

## What This Project Is
Scales the validated Sanity_Test to a 2×2×3 grid of 12 octahedra. The physics,
code conventions, and output format are identical to the Sanity_Test — the only
difference is that all vertices shared between any pair of octahedra in the 3D grid
get a ball joint, not just vertical stack neighbors.

## Reference
Read ../Sanity_Test/CLAUDE.md for: code conventions, PyChrono API patterns, CSV format,
tilt angle computation, and progress tracking rules. Everything there applies here.

## Architecture — Two Agents
1. scaler — owns src/simulation_2x2x3.py only.
2. gui-viz — owns src/visualizer.py and src/plot_tilts.py only.

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

### Boundary Conditions — Per-Column Anchors and Drivers
Each of the 4 columns (one per (ix, iy) pair) gets its own bottom and top marker sphere:
- **4 bottom spheres** — placed at the bottom vertex (-Z) of each iz=0 octahedron,
  each grounded via `ChLinkMateFix` to the ground body.
- **4 top spheres** — placed at the top vertex (+Z) of each iz=2 octahedron,
  each driven downward via `ChLinkMotorLinearSpeed` at 0.01 units/s.
All 4 motors apply the same constant speed so the grid is compressed uniformly.
Each marker sphere is connected to its nearest octahedron via a ball joint.

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