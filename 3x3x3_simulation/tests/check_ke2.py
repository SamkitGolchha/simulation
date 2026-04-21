"""Quick KE check for seed=1 with the current code — print first 200 export steps."""
import os, sys, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pychrono as chrono
from src.simulation_3x3x3 import build_system, _compute_total_ke, STIFFNESS_VARIANTS

dt = 5e-5
export_interval = 250

# Pass bushing params unconditionally; spherical mode ignores them.
_k, _c = STIFFNESS_VARIANTS[0][1], STIFFNESS_VARIANTS[0][2]
system, bodies, body_names, joint_count, _top_spheres, _load_container = build_system(
    1, bushing_k=_k, bushing_c=_c
)

ke_consec = 0
ke_thresh = 0.002
n_steps = int(10.0 / dt)

print(f"{'step':>8s}  {'time':>10s}  {'total_KE':>14s}  {'consec':>6s}")
for step_idx in range(n_steps):
    system.DoStepDynamics(dt)

    if step_idx % 10 == 0:
        for b in bodies:
            v = b.GetPosDt()
            b.SetPosDt(chrono.ChVector3d(v.x*0.999, v.y*0.999, v.z*0.999))
            w = b.GetAngVelLocal()
            b.SetAngVelLocal(chrono.ChVector3d(w.x*0.999, w.y*0.999, w.z*0.999))

    if step_idx % export_interval == 0:
        t = system.GetChTime()
        total_ke = _compute_total_ke(bodies)
        if t > 1.0:
            if total_ke < ke_thresh:
                ke_consec += 1
            else:
                ke_consec = 0
        print(f"{step_idx:8d}  {t:10.4f}  {total_ke:14.10f}  {ke_consec:6d}")
        if ke_consec >= 40:
            print(f"\nEquilibrium at t={t:.4f}s")
            break

    if step_idx > 60000:  # stop after ~3s to save time
        break

print("Done.")
