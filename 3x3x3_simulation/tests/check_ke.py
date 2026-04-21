"""Compute kinetic energy from CSV data to determine equilibrium threshold."""
import csv
import math
import os
import sys
import numpy as np
import pychrono as chrono

# Import the simulation module to access build_system and _check_equilibrium
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from src.simulation_3x3x3 import build_system, _check_equilibrium, EDGE_LENGTH

# Run a single quick simulation (seed=1) and print KE at each export step
a = EDGE_LENGTH
dt = 5e-5
duration = 10.0
export_interval = 250

system, bodies, body_names, joint_count, _top_spheres = build_system(1)

n_steps = int(duration / dt)
print(f"Running KE analysis (seed=1), dt={dt}, {n_steps} steps...")
print(f"{'time':>10s}  {'total_KE':>14s}  {'below_1e-4':>10s}  {'below_1e-6':>10s}  {'below_1e-8':>10s}")

for step_idx in range(n_steps):
    system.DoStepDynamics(dt)

    # Manual velocity damping
    if step_idx % 10 == 0:
        for b in bodies:
            v = b.GetPosDt()
            b.SetPosDt(chrono.ChVector3d(v.x*0.999, v.y*0.999, v.z*0.999))
            w = b.GetAngVelLocal()
            b.SetAngVelLocal(chrono.ChVector3d(w.x*0.999, w.y*0.999, w.z*0.999))

    if step_idx % export_interval == 0:
        t = system.GetChTime()
        # Compute total KE
        total_ke = 0.0
        for b in bodies:
            v = b.GetPosDt()
            w = b.GetAngVelLocal()
            m = b.GetMass()
            total_ke += 0.5 * m * (v.x**2 + v.y**2 + v.z**2)
            I = b.GetInertiaXX()
            total_ke += 0.5 * (I.x * w.x**2 + I.y * w.y**2 + I.z * w.z**2)

        b4 = total_ke < 1e-4
        b6 = total_ke < 1e-6
        b8 = total_ke < 1e-8
        print(f"{t:10.4f}  {total_ke:14.10f}  {str(b4):>10s}  {str(b6):>10s}  {str(b8):>10s}")

print("Done.")
