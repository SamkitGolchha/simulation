# Cuboid Joint Topology — derivation (open strut frame)

The cuboid lattice is an **open strut frame** — the cube analogue of the
octahedron apex-to-apex lattice — not a solid face-to-face packing. It has
exactly **72 couplings**, identical to the octahedron lattice.

## Why not face-to-face

The original design placed 27 cubes at `SPACING = EDGE_LENGTH` (touching
face-to-face) and coupled **every** coincident corner pair (`_find_shared_vertices`,
an 8×8 corner scan): 392 inter-cube + 72 sphere-BC = 464 bushings. With 4+
bushings welding each shared face, adjacent cubes cannot rotate relative to one
another → the assembly is a **rigid block** that cannot buckle under any load
(every run sat at ~0° tilt). Buckling requires an open frame whose bodies are
joined at single points (rotational freedom).

## Open-frame geometry

- `EDGE_LENGTH = 1.0` (cube edge `a`).
- `SPACING = EDGE_LENGTH × √2 ≈ 1.414` along each axis — the same open spacing
  as the octahedron lattice. Facing cube faces are `SPACING − a ≈ 0.414·a`
  apart (a gap); cubes do **not** touch at rest.
- Each cube is spun **45° about the depth (Y) axis** so its front-view (xz)
  profile is a diamond (the `/\/\` lattice). The cube's inertia is isotropic, so
  this changes nothing dynamically — it only re-presents the square section as a
  diamond and makes the tips meet tip-to-tip at `SPACING = a√2`. The ground plane
  sits at the diamond bottom tip `−a/√2` so the bottom row rests on its tips
  (like the octahedron on its apex). Cube-cube and ground collision stay enabled,
  so cubes stop on each other / the floor once the frame folds.

## Inter-cube couplings — `_axis_neighbor_couplings`

One bushing per **axis-neighbour** cube pair, anchored on the line joining the
two cube centres, at the midpoint of the gap. (An axis-aligned cube has no
vertex on the inter-cube axis — its corners point along body-diagonals — so the
single joint sits on the centre-line, the genuine analogue of the octahedron's
on-axis apex-to-apex joint.) One bushing per pair leaves the rotational freedom
the frame needs to tilt and buckle.

Count on a 3×3×3 grid (adjacent pairs per axis = 2 × 3 × 3 = 18):

| Direction | Pairs |
|---|---|
| X-neighbours | 18 |
| Y-neighbours | 18 |
| Z-neighbours | 18 |
| **Inter-cube total** | **54** |

## Boundary couplings

- **9 bottom ghost spheres** — one per iz=0 cube at its bottom-face centre,
  coupled to the cube by **1** bushing.
- **9 top loading spheres** — one per iz=2 cube at its top-face centre, coupled
  by **1** bushing; carries the constant downward `ChForce`.

Sphere-BC total = 9 + 9 = **18**.

## Total

**54 inter-cube + 18 sphere-BC = 72 couplings** — identical to the octahedron
lattice (54 + 18 = 72).

## DOF

45 bodies (27 cubes + 9 bottom + 9 top spheres) × 6 = **270 DOF**, fully
retained — every coupling is a `ChLoadBodyBodyBushingSpherical` force element
(zero rows in Φ_q), so the frame can never be overconstrained.

## Verification

`tests/smoke_bushing.py` asserts: 45 bodies, 54 inter-cube couplings, 18
sphere-BC, 72 total; cubes (0,0,0) & (1,0,0) linked by exactly one coupling with
a 0.414 gap; ADMM accepts K/C; no NaN/Inf. A 0.255 s solo run at F_top = 20 N
took the max cube tilt from 0.00° to 18.5° (vs the old block stuck at 0.00°),
confirming the frame buckles.
