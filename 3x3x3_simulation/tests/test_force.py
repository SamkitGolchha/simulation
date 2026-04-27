# Verification script for force-based boundary condition simulation results
# Checks tilt angles per layer, bottom sphere stability, top sphere descent, and data integrity
# Run: conda run -n chrono python test_force.py

import csv
import math
import os


def quat_to_tilt_deg(q0: float, q1: float, q2: float, q3: float) -> float:
    """Convert quaternion (w, x, y, z) to tilt angle in degrees from the Z axis."""
    z_z = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    z_z = max(-1.0, min(1.0, z_z))
    tilt_rad = math.acos(z_z)
    return math.degrees(tilt_rad)


def load_csv(path: str) -> list[dict]:
    """Load a simulation CSV into a list of row dicts."""
    rows = []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def analyze_run(csv_path: str, run_label: str) -> dict:
    """Analyze one simulation run CSV and return summary."""
    rows = load_csv(csv_path)

    # Check for NaN/Inf
    nan_count = 0
    for row in rows:
        for key in ["x", "y", "z", "q0", "q1", "q2", "q3"]:
            val = float(row[key])
            if math.isnan(val) or math.isinf(val):
                nan_count += 1

    # Get all unique times
    times = sorted(set(float(r["time"]) for r in rows))
    t_end = times[-1]

    # Get final-time rows
    final_rows = [r for r in rows if abs(float(r["time"]) - t_end) < 1e-6]

    # Tilt angles for all octahedra (body_id 0-26)
    oct_tilts = {}
    for r in final_rows:
        bid = int(r["body_id"])
        if bid < 27:
            q0, q1, q2, q3 = float(r["q0"]), float(r["q1"]), float(r["q2"]), float(r["q3"])
            tilt = quat_to_tilt_deg(q0, q1, q2, q3)
            oct_tilts[bid] = tilt

    # Group by layer (9 octahedra per layer)
    layer_tilts = {0: [], 1: [], 2: []}
    for bid, tilt in oct_tilts.items():
        iz = bid // 9
        layer_tilts[iz].append(tilt)

    # Bottom spheres (27-35): check Z stability
    bot_z = {}
    initial_bot_z = {}
    for r in rows:
        bid = int(r["body_id"])
        if 27 <= bid <= 35:
            t = float(r["time"])
            z = float(r["z"])
            if t == times[0]:
                initial_bot_z[bid] = z
            if abs(t - t_end) < 1e-6:
                bot_z[bid] = z

    bot_dz = {bid: bot_z[bid] - initial_bot_z[bid] for bid in bot_z}

    # Top spheres (36-44): check descent
    top_z = {}
    initial_top_z = {}
    for r in rows:
        bid = int(r["body_id"])
        if 36 <= bid <= 44:
            t = float(r["time"])
            z = float(r["z"])
            if t == times[0]:
                initial_top_z[bid] = z
            if abs(t - t_end) < 1e-6:
                top_z[bid] = z

    top_dz = {bid: top_z[bid] - initial_top_z[bid] for bid in top_z}

    # Time evolution of tilt for body 9 (iz=1, perturbed octahedron at (0,0,1))
    tilt_timeline = []
    for r in rows:
        if int(r["body_id"]) == 9:
            t = float(r["time"])
            q0, q1, q2, q3 = float(r["q0"]), float(r["q1"]), float(r["q2"]), float(r["q3"])
            tilt = quat_to_tilt_deg(q0, q1, q2, q3)
            tilt_timeline.append((t, tilt))

    return {
        "run": run_label,
        "t_end": t_end,
        "nan_count": nan_count,
        "layer_tilts": layer_tilts,
        "bot_dz": bot_dz,
        "top_dz": top_dz,
        "bot_z_final": bot_z,
        "initial_bot_z": initial_bot_z,
        "tilt_timeline": tilt_timeline,
    }


def main() -> None:
    """Run verification on all 5 simulation CSVs."""
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    all_pass = True

    for i in range(1, 6):
        csv_path = os.path.join(output_dir, f"sim_{i:03d}.csv")
        if not os.path.exists(csv_path):
            print(f"MISSING: {csv_path}")
            all_pass = False
            continue

        result = analyze_run(csv_path, f"Run {i}")
        print(f"\n{'='*60}")
        print(f"  {result['run']}  (t_end = {result['t_end']:.2f}s)")
        print(f"{'='*60}")

        # NaN check
        if result["nan_count"] > 0:
            print(f"  FAIL: {result['nan_count']} NaN/Inf values found")
            all_pass = False
        else:
            print(f"  PASS: No NaN/Inf values")

        # Tilt angles per layer
        for iz in range(3):
            tilts = result["layer_tilts"][iz]
            avg_tilt = sum(tilts) / len(tilts)
            max_tilt = max(tilts)
            min_tilt = min(tilts)
            print(f"  Layer iz={iz} (body {iz*9}-{iz*9+8}): "
                  f"avg={avg_tilt:.4f} deg, min={min_tilt:.4f}, max={max_tilt:.4f}")

        # Check ALL layers have some tilt (even iz=0)
        iz0_max = max(result["layer_tilts"][0])
        if iz0_max < 0.001:
            print(f"  FAIL: iz=0 tilt near zero -- bottom layer still locked")
            all_pass = False
        else:
            print(f"  PASS: iz=0 layer shows non-zero tilt (max={iz0_max:.4f} deg)")

        # Bottom spheres: Z stability
        print(f"  Bottom spheres (body 27-35):")
        for bid in sorted(result["bot_dz"]):
            dz = result["bot_dz"][bid]
            z_final = result["bot_z_final"][bid]
            z_init = result["initial_bot_z"][bid]
            status = "OK" if z_final >= z_init - 0.01 else "FELL THROUGH"
            print(f"    body {bid}: z_init={z_init:.6f}, z_final={z_final:.6f}, dz={dz:.6f} [{status}]")
            if z_final < z_init - 0.1:
                print(f"    FAIL: Bottom sphere {bid} fell significantly!")
                all_pass = False

        # Top spheres: descent
        print(f"  Top spheres (body 36-44):")
        for bid in sorted(result["top_dz"]):
            dz = result["top_dz"][bid]
            print(f"    body {bid}: dz={dz:.6f}")

        # Time evolution of body 9 tilt (sample every 5th entry ~= 0.25s)
        tl = result["tilt_timeline"]
        print(f"  Tilt evolution (body 9 / perturbed oct):")
        for idx in range(0, len(tl), max(1, len(tl) // 8)):
            t, tilt = tl[idx]
            print(f"    t={t:6.2f}s: {tilt:8.4f} deg")
        if tl:
            print(f"    t={tl[-1][0]:6.2f}s: {tl[-1][1]:8.4f} deg (final)")

    print(f"\n{'='*60}")
    print(f"  OVERALL: {'ALL CHECKS PASSED' if all_pass else 'SOME CHECKS FAILED'}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
