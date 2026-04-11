# gui-viz Agent Progress

## To-Do
- [x] Verify CSV files exist in output/
- [x] Generate MP4 videos via `run.py --viz`
- [x] Generate tilt angle plot via `run.py --plot`
- [x] Verify all output files exist

## Log

### 2026-04-03 — Starting visualization pipeline
- Confirmed 5 CSV files present: sim_001.csv through sim_005.csv
- Source files src/visualizer.py and src/plot_tilts.py confirmed present
- Beginning MP4 rendering...

### 2026-04-03 — MP4 rendering complete
- Command: `conda run -n chrono python run.py --viz`
- Total rendering time: ~4441 seconds (~74 minutes)
- 800 frames per CSV (20 bodies x 800 timesteps)
- Generated 5 MP4 files:
  - collapse_001.mp4 (1.7 MB)
  - collapse_002.mp4 (1.6 MB)
  - collapse_003.mp4 (1.4 MB)
  - collapse_004.mp4 (982 KB)
  - collapse_005.mp4 (26 KB -- run 5 had minimal motion)

### 2026-04-03 — Tilt angle plot complete
- Command: `conda run -n chrono python run.py --plot`
- Plotting time: 1.1 seconds
- Generated: output/tilt_angles.png (308 KB)
- Plot covers all 5 runs for column (0,0) with 3 subplots

### 2026-04-03 — Verification passed
- All 5 MP4 files confirmed present in output/
- tilt_angles.png confirmed present in output/
- No source files were modified

### 2026-04-04 — Re-render after physics/scaler fixes

**Context:** Physics and scaler agents fixed instability issues (reduced F_top, dt,
export interval, perturbation; added damping; removed collision shapes; tuned solver).
New CSVs generated with smooth tilting behavior.

**Step 0 — Cleared old outputs**
- Removed 5 old collapse_*.mp4 files, frames/ directory, and tilt_angles.png
- Kept only sim_*.csv files (dated 2026-04-04 17:10-17:16)

**Step 1 — Verified CSVs**
- 5 CSV files present: sim_001.csv through sim_005.csv (~1.6 MB each)
- Each contains 800 timesteps, 20 bodies (12 octahedra + 8 spheres)

**Step 2 — Generated MP4 videos** (`conda run -n chrono python run.py --viz`)
- Total rendering time: 2016.8s (~33.6 minutes)
- 800 frames per CSV, all rendered without error
- Generated 5 MP4 files:
  - collapse_001.mp4 (3.6 MB)
  - collapse_002.mp4 (2.3 MB)
  - collapse_003.mp4 (2.9 MB)
  - collapse_004.mp4 (2.4 MB)
  - collapse_005.mp4 (3.6 MB)
- All files substantially larger than previous run (was 0.03-1.8 MB), confirming
  smooth sustained motion across all runs (no more erratic/stalled behavior)

**Step 3 — Generated tilt angle plot** (`conda run -n chrono python run.py --plot`)
- Plotting time: 1.2 seconds
- Generated: output/tilt_angles.png (469 KB)
- Plot covers all 5 runs for column (0,0) with 3 subplots

**Step 4 — Verification passed**
- All 5 MP4 files confirmed present in output/
- tilt_angles.png confirmed present in output/
- No source files were modified (visualizer.py and plot_tilts.py unchanged)

### 2026-04-05 — Re-render after physics collision + stop-condition update

**Context:** Physics agent re-enabled collision shapes on top spheres and added
energy-equilibrium stop condition (`_check_columns_collapsed`). Scaler agent tuned
parameters (F_top 0.5N, dt 5e-5, export_interval 250, perturbation 0.005 rad/s).
New CSVs generated on 2026-04-05.

**Step 0 — Cleared old outputs**
- Removed 5 old collapse_*.mp4 files (dated 2026-04-04), frames/ directory, and tilt_angles.png
- Kept only sim_*.csv files (dated 2026-04-05, 1.3-1.6 MB each)

**Step 1 — Verified CSVs**
- 5 CSV files present: sim_001.csv through sim_005.csv
- All dated 2026-04-05 16:01-16:13

**Step 2 — Generated MP4 videos** (`conda run -n chrono python run.py --viz`)
- Total rendering time: 1488.2s (~24.8 minutes)
- 800 frames per CSV, all rendered without error
- Generated 5 MP4 files:
  - collapse_001.mp4 (1.0 MB)
  - collapse_002.mp4 (1.5 MB)
  - collapse_003.mp4 (968 KB)
  - collapse_004.mp4 (1.4 MB)
  - collapse_005.mp4 (315 KB)
- File sizes smaller than previous run (was 2.3-3.6 MB), consistent with collision
  shapes and stop condition limiting over-buckling

**Step 3 — Generated tilt angle plot** (`conda run -n chrono python run.py --plot`)
- Plotting time: 1.1 seconds
- Generated: output/tilt_angles.png (273 KB)
- Plot covers all 5 runs for column (0,0) with 3 subplots

**Step 4 — Verification passed**
- All 5 MP4 files confirmed present in output/
- tilt_angles.png confirmed present in output/
- No source files were modified (visualizer.py and plot_tilts.py unchanged)

### 2026-04-08 — Ghost spheres engine re-render + corner tower-view

**Context:** Simulation engine switched to ghost spheres (bottom sphere collision
removed, octahedra rest on ground via convex hulls). All 5 runs re-executed with
new engine. Added a second camera angle for visualization.

**Step 1 — Re-ran simulation with ghost spheres engine**
- 5 new CSVs generated in output/ (sim_001.csv through sim_005.csv)

**Step 2 — Re-rendered standard side-view MP4s**
- 5 MP4 files in output/ (collapse_001.mp4 through collapse_005.mp4)
- Side view (XZ plane), same camera as before

**Step 3 — New corner tower-view visualizer**
- Created `src/visualizer_corner.py` — reuses parse_csv, build_octahedron_mesh,
  build_sphere_mesh, stitch_frames_to_mp4 from `src/visualizer.py`
- Camera: elevated at 1.2x structure height (~5.09 units), positioned on the
  (+X, +Y) diagonal, looking at structure center — shows top and both side planes
- Runner script: `different_angles/corner_view.py`
- Output: 5 MP4s in `view_change/` (collapse_001.mp4 through collapse_005.mp4)
  - collapse_001.mp4 (1.8 MB)
  - collapse_002.mp4 (1.5 MB)
  - collapse_003.mp4 (1.6 MB)
  - collapse_004.mp4 (1.5 MB)
  - collapse_005.mp4 (1.7 MB)

**Step 4 — File reorganization**
- Moved progress files: `progress_*.md` → `progress/`
- Moved test/check scripts: `check_*.py`, `test_*.py` → `tests/`
- Created `different_angles/` folder for alternate view runner scripts

### 2026-04-11 — Non-agent updates: output restructure

**Context:** Manual (non-agent) reorganization of output directories and file structure.

**Changes:**
- Front view MP4s moved from `output/` root → `output/front_view/`
- Corner view MP4s moved from `view_change/` → `output/corner_view/`
- `src/visualizer.py` updated: `visualize_all()` now renders to `output/front_view/`
- `src/visualizer_corner.py` and `different_angles/corner_view.py` updated to output to `output/corner_view/`
- `run.py` final summary updated to show new paths
- Deleted stale `output_check1/` and `output_check2/` dirs from scaling-test experiments
- Updated all agent files (gui-viz, physics, scaler) to reference `progress/` paths
- Updated physics agent: bottom spheres are now ghost bodies (no collision)
