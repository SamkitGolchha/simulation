# Octahedral Tilt Simulation — Progress

## Project Summary

A PyChrono rigid-body simulation of 3 regular octahedra stacked vertically, connected by spherical (ball) joints at shared vertices. A bottom marker sphere is fixed to the ground; a top marker sphere is driven downward at constant velocity. The simulation runs 5 times with different random perturbations, producing MP4 videos and tilt angle plots.

## Environment Setup

### Prerequisites
- **Miniconda/Anaconda** installed
- **Conda environment** named `chrono` with PyChrono

### Create the environment (if not already set up)
```bash
conda create -n chrono python=3.12
conda activate chrono
conda install -c conda-forge pychrono
pip install numpy scipy matplotlib pyvista imageio-ffmpeg
```

### Verify installation
```bash
conda run -n chrono python -c "import pychrono; import numpy; import scipy; import pyvista; import matplotlib; print('All OK')"
```

## How to Run

### Full pipeline (simulation + videos + plot)
```bash
conda run -n chrono python run.py
```

### Individual stages
```bash
# Simulation only (generates 5 CSVs in output/)
conda run -n chrono python run.py --sim

# Render MP4 videos only (requires CSVs to exist)
conda run -n chrono python run.py --viz

# Generate tilt angle plot only (requires CSVs to exist)
conda run -n chrono python run.py --plot
```

### Running modules directly
```bash
# Simulation
conda run -n chrono python -m src.simulation

# Single video from one CSV
conda run -n chrono python -m src.visualizer output/sim_001.csv

# All 5 videos from a directory
conda run -n chrono python -m src.visualizer output/

# Tilt angle plot
conda run -n chrono python -m src.plot_tilts output/
```

## Project Structure

```
octahedral-tilt/
├── CLAUDE.md            # Project spec and constraints
├── run.py               # Orchestrator script (--sim, --viz, --plot, --all)
├── progress.md          # This file
├── src/
│   ├── __init__.py
│   ├── simulation.py    # PyChrono simulation: geometry, joints, motor, CSV export
│   ├── visualizer.py    # PyVista 3D rendering -> ffmpeg MP4 encoding
│   └── plot_tilts.py    # Matplotlib tilt angle plots from CSV data
└── output/
    ├── sim_001.csv .. sim_005.csv      # Simulation data (5 runs)
    ├── collapse_001.mp4 .. collapse_005.mp4  # 3D rendered videos (front view)
    └── tilt_angles.png                 # Tilt angle vs time plot
```

## Agents Used

| Agent | Responsibility | Files Produced |
|-------|---------------|----------------|
| **physics-engine** | Built the simulation: octahedron geometry, body creation, joint assembly, motor setup, CSV export, 5-run loop | `src/simulation.py` |
| **gui-viz** | Built the visualizer (PyVista frames + ffmpeg MP4) and the tilt angle plotter (matplotlib) | `src/visualizer.py`, `src/plot_tilts.py` |
| **Main agent** | Scaffolding, dependency installation, orchestrator script, integration testing | `run.py`, `src/__init__.py`, `progress.md` |

## Tasks Completed

1. **Scaffolding** — Created `src/`, `output/` directories and `src/__init__.py`
2. **Dependency verification** — Confirmed pychrono in `chrono` conda env; installed scipy, pyvista, matplotlib, imageio-ffmpeg
3. **Simulation (physics-engine agent)** — Built `src/simulation.py` with:
   - 3 regular octahedra (edge length 1.0) stacked along Z
   - Bottom sphere fixed to ground via `ChLinkMateFix`
   - Top sphere driven downward at 0.05 units/s via `ChLinkMotorLinearSpeed`
   - `ChLinkLockSpherical` ball joints at all shared vertices
   - 5 runs with random angular perturbations on middle octahedron
   - CSV export: time, body_id, body_name, x, y, z, q0, q1, q2, q3
4. **Visualization (gui-viz agent)** — Built `src/visualizer.py` with:
   - PyVista offscreen rendering of octahedra and spheres
   - Quaternion-based rotation of geometry per frame
   - ffmpeg stitching via imageio_ffmpeg
   - Front-view camera (azimuth 0, elevation 0)
   - Supports single CSV or batch directory mode
5. **Tilt angle plotting (gui-viz agent)** — Built `src/plot_tilts.py` with:
   - Tilt angle computation: `arccos(R.apply([0,0,1])[2])` using scipy Rotation
   - 3 vertically stacked subplots (one per octahedron)
   - 5 overlaid lines per subplot (one per run)
   - 200 DPI output
6. **Integration testing** — Ran full pipeline, verified all outputs are non-empty
7. **Multi-video support** — Extended visualizer to produce `collapse_001.mp4` through `collapse_005.mp4`
8. **Camera angle change** — Switched from isometric to front view
9. **Orchestrator script** — Created `run.py` with `--sim`, `--viz`, `--plot`, `--all` flags

## Output Summary

| File | Size | Description |
|------|------|-------------|
| `sim_001.csv` – `sim_005.csv` | ~99 KB each | 1000 rows (200 timesteps x 5 bodies) |
| `collapse_001.mp4` – `collapse_005.mp4` | ~180-226 KB each | 200 frames at 30fps, front view |
| `tilt_angles.png` | 333 KB | 3 subplots, 5 runs overlaid |

## Simulation Parameters

| Parameter | Value |
|-----------|-------|
| Timestep | 1e-4 s |
| Duration | 2.0 s |
| Export interval | every 100 steps (0.01s) |
| Edge length | 1.0 |
| Descent speed | 0.05 units/s |
| Perturbation magnitude | ~0.01 rad/s |
| Number of runs | 5 |
