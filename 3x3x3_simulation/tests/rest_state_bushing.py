"""Rest-state test for joints333 §4.3 item 2.

Builds the bushing system with F_top = 0 and perturb = 0, steps for 1 second,
and reports the max body velocity across all 45 bodies at t = 1.0 s.

Expected per plan §4.3: max |v| < 1e-3 m/s. Bushings hold the lattice together
as a stiff spring network under gravity + ground contact.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.simulation_3x3x3 import (  # noqa: E402
    JOINT_MODE,
    STIFFNESS_VARIANTS,
    build_system,
)


def main() -> int:
    assert JOINT_MODE == "bushing"
    label, k, c = STIFFNESS_VARIANTS[0]
    print(f"Rest-state test: {label} K={k} C={c}, F_top=0, perturb=0")

    system, bodies, _names, _jc, _top, _lc = build_system(
        1, bushing_k=k, bushing_c=c, top_force_n=0.0, perturb_mag=0.0
    )

    # Match committed 2x2x3 bushing dt = 1e-4
    dt = 1e-4
    duration = 1.0
    n_steps = int(duration / dt)

    # Apply the same manual damping the main loop uses so we match the real
    # integration path.
    import pychrono as chrono  # noqa: E402

    for step_idx in range(n_steps):
        system.DoStepDynamics(dt)
        if step_idx % 10 == 0:
            for b in bodies:
                v = b.GetPosDt()
                b.SetPosDt(chrono.ChVector3d(v.x * 0.9999, v.y * 0.9999, v.z * 0.9999))
                w = b.GetAngVelLocal()
                b.SetAngVelLocal(
                    chrono.ChVector3d(w.x * 0.9999, w.y * 0.9999, w.z * 0.9999)
                )
        if step_idx % 1000 == 0:
            max_v = max(
                math.sqrt(b.GetPosDt().x ** 2 + b.GetPosDt().y ** 2 + b.GetPosDt().z ** 2)
                for b in bodies
            )
            print(f"  step={step_idx} t={system.GetChTime():.4f} max_|v|={max_v:.3e}")

    max_v_final = 0.0
    worst_idx = -1
    for i, b in enumerate(bodies):
        v = b.GetPosDt()
        mag = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        if mag > max_v_final:
            max_v_final = mag
            worst_idx = i
    print(f"\n  Final at t=1.0 s: max_|v| = {max_v_final:.3e} m/s (body #{worst_idx})")

    threshold = 1e-3
    if max_v_final < threshold:
        print(f"  PASS: max_|v| < {threshold} m/s")
        return 0
    print(f"  FAIL: max_|v| >= {threshold} m/s")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
