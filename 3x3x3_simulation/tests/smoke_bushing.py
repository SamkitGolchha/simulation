"""Bushing-mode smoke test for joints333 §4.3 validation.

Verifies:
  1. Topology prints: body count = 45, inter-oct joints = 54, total = 72.
  2. ADMM solver accepts the bushing K/C matrices (no
     `System descriptor includes stiffness or damping matrices` error).
  3. A handful of integration steps produce no NaN/Inf.

Not checked in as part of the source tree validation — lives in tests/ so it
is easy to delete or adapt. Run with:

    DYLD_INSERT_LIBRARIES=$CONDA_PREFIX/lib/libomp.dylib \
        conda run -n chrono python tests/smoke_bushing.py
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
    assert JOINT_MODE == "bushing", f"expected JOINT_MODE=bushing, got {JOINT_MODE}"
    label, k, c = STIFFNESS_VARIANTS[0]
    print(f"Smoke test: {label} K={k} C={c}")

    system, bodies, _names, joint_count, _top, _lc = build_system(
        1, bushing_k=k, bushing_c=c
    )
    assert len(bodies) == 45, f"expected 45 bodies, got {len(bodies)}"
    assert joint_count == 72, f"expected 72 joints, got {joint_count}"
    print(f"  OK body count={len(bodies)}, joint count={joint_count}")

    dt = 5e-5
    n_steps = 200  # 0.01 s — enough to exercise ADMM, fast to run
    for step_idx in range(n_steps):
        system.DoStepDynamics(dt)
        if step_idx % 50 == 0:
            max_v = 0.0
            for b in bodies:
                v = b.GetPosDt()
                mag = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
                if math.isnan(mag) or math.isinf(mag):
                    print(f"  FAIL NaN/Inf at step {step_idx}")
                    return 2
                max_v = max(max_v, mag)
            t = system.GetChTime()
            print(f"  step={step_idx} t={t:.6f} max_|v|={max_v:.6e}")

    print("  Smoke test PASSED — ADMM accepted bushing K/C; no NaN/Inf.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
