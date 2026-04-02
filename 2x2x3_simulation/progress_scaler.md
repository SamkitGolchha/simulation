# Scaler Agent Progress

## TODO
- [x] Read Sanity_Test reference implementation and CLAUDE.md specs
- [x] Build src/simulation_2x2x3.py with 2x2x3 lattice, joints, BCs, CSV output
- [x] Run 5 simulations successfully
- [x] Verify 5 CSV files in output/
- [x] Log joint count and results

## Results

### Joint count
- Inter-octahedron ball joints: **20**
  - 6 X-adjacent pairs (1 shared vertex each) = 6
  - 6 Y-adjacent pairs (1 shared vertex each) = 6
  - 8 Z-adjacent pairs (1 shared vertex each) = 8
- Sphere-octahedron ball joints: **8** (4 bottom + 4 top)
- **Total ball joints: 28**

### Simulation runs
- All 5 runs completed successfully (seeds 1-5)
- 4000 rows per CSV (200 frames x 20 bodies)
- Output files: output/sim_001.csv through output/sim_005.csv
- dt=1e-4, duration=2.0s, export every 100 steps
- Motor speed: 0.01 units/s downward (all 4 columns)
- Perturbation: 0.01 rad/s random angular velocity on octahedron (0,0,1)

### Body ID mapping
- 0-11: Octahedra (ix + 2*iy + 4*iz ordering)
- 12-15: Bottom spheres (columns (0,0), (1,0), (0,1), (1,1))
- 16-19: Top spheres (same column ordering)
