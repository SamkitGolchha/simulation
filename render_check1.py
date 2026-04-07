"""Render MP4 videos for all simulation CSVs in output_check1/."""

import sys
import os

# Add the 2x2x3_simulation directory to sys.path so we can import src.visualizer
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "2x2x3_simulation"))

from src.visualizer import visualize_all

if __name__ == "__main__":
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output_check1")
    print(f"Rendering all CSVs in: {output_dir}")
    visualize_all(output_dir)
    print("\nAll videos rendered.")
