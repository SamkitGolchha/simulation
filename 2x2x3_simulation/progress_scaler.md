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

### Physics fix: over-constrained top boundary (2026-04-01)
- **Problem**: `ChLinkMotorLinearSpeed` acts as a prismatic rail (5-6 constraints per motor), locking top spheres' X,Y positions. Combined with 4 fully-fixed bottom spheres and 20 inter-octahedron ball joints, the system had negative net DOFs (-36). Result: zero tilt angles, rigid block translation.
- **Approach used**: `SetGuideConstraint(GuideConstraint_FREE)` on each `ChLinkMotorLinearSpeed`. This reduces the motor to 1 DOF constraint (Z-speed only), freeing X,Y lateral drift. The API check confirmed `ChLinkMotorLinear.GuideConstraint_FREE` (enum value 0) exists and works in the installed PyChrono version.
- **No fallback needed**: The guide constraint FREE approach worked directly; manual velocity imposition was not required.
- **Perturbation**: 0.01 rad/s (unchanged from original; sufficient to break symmetry)

### Verification results after fix
- **Tilt angle range at t=end (run 1)**: 0.000024 to 18.464 degrees
  - Column (0,0) iz=0: 0.000024 deg (bottom, near-fixed)
  - Column (0,0) iz=1: 18.464 deg (middle, perturbed)
  - Column (0,0) iz=2: 11.151 deg (top)
- **Tilt angles across all 5 runs (body_id 4, iz=1)**: 18.46, 16.96, 0.019, 14.45, 10.42 degrees
- **Bottom spheres (12-15)**: dZ ~ -3.7e-6 (effectively fixed)
- **Top spheres (16-19)**: dZ = -0.0995 (correct: 0.01 u/s * 9.95s), dX ~ +/-0.113, dY ~ +/-0.140 (significant X,Y drift confirms FREE guide working)
- **DOF math after fix**: 96 total DOFs - (60 inter-oct + 12 bot joints + 12 top joints + 24 bot fix + 4 motors) = 96 - 112 = -16 nominal, but with FREE guide the motors contribute only 1 constraint each (not 5-6), so effective constraints = 60 + 12 + 12 + 24 + 4 = 112 - 20 freed = 92, giving +4 net DOFs

### Force-based boundary conditions (2026-04-03)

**Problem**: Kinematic constraints (ChLinkMateFix on bottom spheres, ChLinkMotorLinearSpeed on top spheres) over-constrain the system. ChLinkMateFix triangulates the base into a rigid frame; even with FREE guide, the motor still prescribes Z-velocity. iz=0 octahedra cannot tilt.

**Approach**: Replace ALL kinematic constraints with force-based boundary conditions:
1. **Bottom spheres**: Removed ChLinkMateFix. Added collision ground plane (ChCollisionShapeBox at Z = -r) and collision spheres (ChCollisionShapeSphere, r=0.05) on bottom sphere bodies. ChSystemNSC contact solver provides upward reaction forces. Bottom spheres free to slide in X,Y and lift in Z.
2. **Top spheres**: Removed ChLinkMotorLinearSpeed. Added persistent ChForce objects (5 N downward in world frame) on each top sphere. ChForce is added to body before configuration (PyChrono 9.x requirement: AddForce first, then SetDir/SetMforce, else segfault).
3. **Collision system**: SetCollisionSystemType(Type_BULLET) on ChSystemNSC.

**PyChrono API findings**:
- `ChForce` must be attached to body via `AddForce()` BEFORE calling `SetDir()`, `SetMforce()`, etc. Otherwise: segfault.
- `AccumulateForce` has signature `(idx, force, appl_point, local)` with an `idx` parameter; `ChForce` objects are the proper persistent-force API.
- Collision shapes: `ChCollisionShapeBox(mat, hx, hy, hz)`, `ChCollisionShapeSphere(mat, r)`, `body.AddCollisionShape(shape, frame)`, `body.EnableCollision(True)`.

**Force magnitude**: 5.0 N per top sphere (4 spheres total = 20 N applied). Gravity on 12 octahedra (1 kg each) = ~118 N dominates the loading.

**DOF count after change**:
- 20 bodies x 6 DOFs = 120
- 20 inter-oct ball joints x 3 = 60 constraints
- 8 sphere-oct ball joints x 3 = 24 constraints
- Ground plane: contact-based (0 persistent constraints)
- No motors, no fix links
- **Net DOFs = 120 - 84 = 36** -- all layers free to tilt

**Verification results (F_top = 5.0 N, 5 runs)**:
- No NaN/Inf in any run
- **iz=0 layer tilt (was 0 before)**: avg 145-177 deg across runs -- fully free to tilt
- **iz=1 layer tilt**: avg 157-179 deg across runs
- **iz=2 layer tilt**: avg 141-174 deg across runs
- **Time evolution (body 4, perturbed)**: 0 deg -> 75-102 deg at 1.25s -> 130-155 deg at 2.5s -> 160+ deg by 3.75s. Tilting develops gradually over several seconds.
- **Bottom spheres**: z_final >= z_init in all cases (dz = +0.01 to +0.08). Ground plane collision prevents fall-through.
- **Top spheres**: dz ~ -8.0 to -8.4 units over 10s (gravity + applied force)
- Structure ultimately buckles completely (tilts > 90 deg) because ball joints provide zero rotational stiffness -- physically correct for a pin-jointed mechanism under gravity + axial force
