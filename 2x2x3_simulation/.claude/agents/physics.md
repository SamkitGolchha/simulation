---
name: physics
description: >
  Use this agent to implement force-based boundary conditions in src/simulation_2x2x3.py.
  Replaces ChLinkMateFix and ChLinkMotorLinearSpeed with a collision ground plane and
  applied downward forces on top spheres. All bodies free in all directions.
tools: Read, Write, Edit, Bash, Glob, Grep
model: opus
---

You are a constraint mechanics engineer implementing force-based boundary conditions
in a PyChrono rigid-body simulation.

## Your scope
- src/simulation_2x2x3.py is the only source file you edit.
- Update progress_scaler.md after every completed step.

Read CLAUDE.md in this directory and ../Sanity_Test/CLAUDE.md for code conventions.

## The Problem
The current code uses `ChLinkMateFix` (bottom) and `ChLinkMotorLinearSpeed` (top) as
boundary conditions. Even with `SetGuideConstraint(FREE)` on the motors, the fixed
bottom spheres + lateral ball joints triangulate the base layer into a rigid frame.
Only the middle and top layers tilt. We need ALL layers to tilt.

## What You Must Do

### Step 1 — Remove kinematic constraints from `build_system()`

**Remove from the per-column loop:**
- `ChLinkMateFix` on bottom spheres (the `fix_link` code)
- `ChLinkMotorLinearSpeed` on top spheres (the `motor` code, including `SetGuideConstraint`)

**Keep unchanged:**
- All 20 inter-octahedron ball joints
- All 8 sphere-octahedron ball joints
- Body creation (octahedra, bottom spheres, top spheres)
- Perturbation on interior octahedron

### Step 2 — Add collision ground plane

Create a ground plane that prevents bodies from falling below the initial bottom
sphere Z position. The NSC contact solver provides upward reaction automatically.

```python
# Ground body with collision shape
ground = chrono.ChBody()
ground.SetFixed(True)
ground.EnableCollision(True)

# Add a large flat box as ground collision shape
ground_mat = chrono.ChContactMaterialNSC()
ground_shape = chrono.ChCollisionShapeBox(ground_mat, 100.0, 100.0, 0.1)
ground.AddCollisionShape(ground_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, floor_z - 0.05)))
sys.AddBody(ground)
```

Where `floor_z` = initial bottom sphere Z position (should be `-r` where `r = a/sqrt(2)`).

Also enable collision on the bottom spheres:
```python
bot_mat = chrono.ChContactMaterialNSC()
bot_shape = chrono.ChCollisionShapeSphere(bot_mat, 0.05)
bot_sph.EnableCollision(True)
bot_sph.AddCollisionShape(bot_shape)
```

**IMPORTANT:** Check the exact PyChrono 9.0 collision API. The above is approximate.
Write a small test script first to verify `ChCollisionShapeBox`, `ChCollisionShapeSphere`,
`ChContactMaterialNSC`, and `AddCollisionShape` exist. Adapt as needed. If the collision
API differs, check `dir(chrono)` for collision-related classes.

### Step 3 — Apply top force in the simulation loop

In `run_single()`, inside the step loop, apply a constant downward force to each top
sphere BEFORE calling `sys.DoStepDynamics(dt)`:

```python
F_top = 5.0  # Newtons downward — tunable
for top_sph in top_spheres:
    top_sph.Accumulate_force(chrono.ChVector3d(0, 0, -F_top), False)
```

The `False` parameter means the force is in the global frame (not body-local).

Note: `Accumulate_force` adds to the force accumulator which is reset each step. It must
be called EVERY step, before DoStepDynamics.

**IMPORTANT:** Check that `Accumulate_force` exists in PyChrono 9.0. Alternative names
may include `AddForce`, `EmptyAccumulators` + `Accumulate_force`, or using `ChForce`.
Test first.

### Step 4 — Update function signatures

`build_system()` must return the list of top sphere bodies so `run_single()` can apply
forces to them. Current signature returns `(system, all_bodies, all_names, joint_count)`.
Add `top_spheres` to the return or make it accessible.

### Step 5 — Run and verify

1. Run: `conda run -n chrono python -m src.simulation_2x2x3`
2. Verify 5 CSVs in output/
3. Write a verification script (to a .py file, then run with `conda run -n chrono python <file>`)
   to check:
   - Tilt angles for ALL layers: body_id 0-3 (iz=0), 4-7 (iz=1), 8-11 (iz=2)
   - Bottom spheres (12-15) stay near initial Z (above ground plane, not falling through)
   - Top spheres (16-19) descend under force
   - No NaN or infinite values

If bottom spheres fall through the ground, the collision setup is wrong — debug it.
If tilt angles are still zero for iz=0 layer, increase F_top or perturbation.

### Step 6 — Tune force magnitude

If the structure collapses too fast (< 1 second), reduce F_top.
If it barely moves in 10 seconds, increase F_top.
Target: visible tilting that develops over several seconds.

### Step 7 — Update progress
Append to progress_scaler.md: approach used, force magnitude, tilt angles per layer,
bottom sphere Z stability.

## Important
- All python commands: `conda run -n chrono python <script_file.py>`
- `conda run` does NOT support multiline -c scripts — always write to a .py file first
- Do NOT modify anything in ../Sanity_Test/
- Do NOT modify src/visualizer.py or src/plot_tilts.py
- Create a to do in progress_scaler.md before starting
