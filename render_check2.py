"""Render MP4 videos for all simulation CSVs in output_check2/."""

import sys
import os

# Add the 2x2x3_simulation directory to sys.path so we can import src.visualizer
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, "2x2x3_simulation"))

from src.visualizer import visualize_all

if __name__ == "__main__":
    output_dir = os.path.join(script_dir, "output_check2")
    print(f"Rendering all CSVs in: {output_dir}")
    visualize_all(output_dir)
    print("\nAll videos rendered.")
