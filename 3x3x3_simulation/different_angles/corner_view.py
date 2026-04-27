# Corner View — renders all simulation CSVs from an elevated corner camera.
# Camera is at 1.2x structure height, looking from the (+X,+Y) diagonal,
# showing top and both side planes simultaneously.
#
# Usage: conda run -n chrono python different_angles/corner_view.py
# Input:  output/sim_*.csv
# Output: output/corner_view/collapse_*.mp4

import os
import sys
import time

# Ensure project root is on the path so src.* imports work
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from src.visualizer_corner import visualize_corner_all

input_dir = os.path.join(project_root, "output")
output_dir = os.path.join(project_root, "output", "corner_view")

print("=" * 60)
print("Corner View Renderer")
print(f"  Input:  {input_dir}")
print(f"  Output: {output_dir}")
print("=" * 60)

t0 = time.time()
visualize_corner_all(input_dir, output_dir)
elapsed = time.time() - t0

print(f"\nCorner view rendering complete in {elapsed:.1f}s.")
