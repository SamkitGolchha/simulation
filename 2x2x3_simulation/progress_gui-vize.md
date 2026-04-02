# gui-viz Agent Progress

## TODO
- [x] Build `src/visualizer.py` -- 20-body visualizer scaled from Sanity_Test (12 octahedra + 8 spheres)
- [x] Build `src/plot_tilts.py` -- tilt angle plotter for column (0,0) with body_id 0, 4, 8

## Completed

### src/visualizer.py
- Parses CSV dynamically: detects octahedra (name starts with "oct") vs spheres (name contains "sphere") from `body_name` column. No hardcoded body counts.
- 12-color palette grouped by Z-layer: blues (iz=0), greens (iz=1), purples (iz=2).
- 8 spheres rendered as small red spheres (radius 0.05).
- Window size 1920x1088 (divisible by 16 for ffmpeg).
- Camera auto-fits via `pl.reset_camera()`.
- `parse_csv()` returns `SimData`, sorted timesteps, and `BodyKindMap`.
- Full pipeline: parse, render frames, stitch MP4 via imageio_ffmpeg, clean up.
- Supports single CSV or batch mode (`visualize_all`).
- Syntax-checked OK; all imports verified in chrono conda env.

### src/plot_tilts.py
- Plots tilt angles for column (0,0) only: body_id 0 (iz=0), 4 (iz=1), 8 (iz=2).
- 3 vertically-stacked subplots with titles: "Octahedron (0,0,0) -- bottom", "(0,0,1) -- middle", "(0,0,2) -- top".
- 5 overlaid lines per subplot (one per run), using tab10 colormap.
- Tilt angle: `arccos(clip(rot.apply([0,0,1])[2], -1, 1))` in degrees, scipy scalar-last quaternion convention.
- Output: tilt_angles.png at 200 DPI.
- Syntax-checked OK; all imports verified in chrono conda env.
