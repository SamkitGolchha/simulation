# 3×3×3 Octahedral Lattice — Joint Construction, Overconstraint, and the Path Forward

## 1. The lattice we are joining

The 3×3×3 build is a regular grid of **27 rigid octahedra**, edge length `a = 1`,
center-to-center spacing `s = a√2`. Each octahedron carries 6 vertices (the
±x, ±y, ±z poles in its body frame). Adjacent octahedra in the grid are
positioned so that **one vertex of body A coincides exactly with one vertex of
body B** in world coordinates. That coincidence is what the joint enforces.

In addition to the 27 octahedra, the build adds boundary-condition bodies:
- **9 bottom ghost spheres** — one per column, attached at the −z vertex of the
  iz=0 octahedron (no collision, joint connectivity only).
- **9 top loading spheres** — one per column, attached at the +z vertex of the
  iz=2 octahedron, each driven by a constant downward `ChForce`.

So the population is **45 free rigid bodies** in total.

## 2. How the joints are constructed

Joint construction (see `simulation_2x2x3.py:_find_shared_vertices` and
`build_system`) is a two-pass procedure:

1. **Shared-vertex detection** — `_find_shared_vertices` does an O(N²·36) scan
   over every body pair, computing the 6×6 vertex-vertex distance matrix and
   recording every coincidence (within `tol = 1e-9`). For the 3×3×3 grid this
   returns one shared vertex per grid edge, i.e. one entry per
   axis-aligned neighbor pair.
2. **Joint instantiation** — for each shared-vertex triple `(idxA, idxB, vpos)`,
   `_add_ball_joint` creates a 3-DOF translational coupling at `vpos`. Whether
   it is realized as a `ChLinkLockSpherical` (rigid) or as a `ChLinkTSDA` triad
   (compliant springs along three orthogonal local axes) is a swap behind the
   same function signature; see §6.

Each ball joint removes exactly **3 translational DOFs** between the two bodies
it couples while leaving all 3 relative rotations free — bodies can pivot
freely about the shared vertex.

Edge counts in a 3×3×3 grid:

```
x-edges = (NX−1)·NY·NZ = 2·3·3 = 18
y-edges = NX·(NY−1)·NZ = 3·2·3 = 18
z-edges = NX·NY·(NZ−1) = 3·3·2 = 18
inter-octahedron joints = 18 + 18 + 18 = 54
```

Plus the boundary joints:
```
bottom-sphere ↔ oct  joints = 9
top-sphere    ↔ oct  joints = 9
total ball joints           = 54 + 9 + 9 = 72
```

## 3. The DOF accounting — and where it breaks

The standard rigid-body Grübler–Kutzbach count is
`F = 6·B − Σ cᵢ`, where `B` is the body count and `cᵢ` is the number of
scalar constraints per joint. For ball joints `cᵢ = 3`.

### 3.1 Global view (everything, including BC spheres)

```
F_global = 6·45 − 3·72
         = 270 − 216
         = 54   DOFs   ✅ positive
```

Looks healthy: 54 DOFs across the whole assembly. Most of those DOFs live in
the rotational freedoms of the loading and ghost spheres (which have no
rotational constraint at all) and in the rotational pivoting of corner
octahedra. So the *overall* assembly is not formally overconstrained.

### 3.2 Octahedron-only subsystem (ignore the spheres)

The structurally interesting subsystem is the lattice of octahedra by itself,
because the ball joints between them are what carry the load.

```
N_oct      = 27
J_inter    = 54
F_oct-only = 6·27 − 3·54
           = 162 − 162
           = 0     ⚠ exactly at the boundary
```

Zero DOFs is the critical case. It means the octahedra-only subsystem has
*just barely* enough constraints to be rigid, with **no slack and no margin**.
Any constraint that the Grübler count overlooks — and there are several —
pushes the subsystem into the overconstrained regime, where `F < 0`.

### 3.3 Why the naive count is optimistic — redundant constraints

Grübler counts assume every constraint is independent. Lattice geometry
violates that. Two structural redundancies show up at 3×3×3:

**(a) Closed loops.** Walk around any unit cell of the lattice
(e.g. a 4-edge loop in a single layer): four ball joints close back to the
starting body. The "closing" constraint duplicates information already implied
by the other three plus the rigid-body kinematics of each octahedron. Each
independent loop subtracts up to 3 effective constraints from the Grübler
count (or equivalently, contributes redundant rows to the constraint Jacobian).
The number of independent loops is `L = J − B + 1` for a connected graph,
which here is `54 − 27 + 1 = 28` loops in the octahedron subgraph.

**(b) Multi-vertex sharing within face-adjacent neighbors.** When the lattice
is built from regular octahedra at the spacing `a√2`, *most* neighbor pairs
share exactly one vertex, but the closure conditions on a 3×3×3 array force
the shared-vertex graph to encode the same rigidity claim multiple ways.

The practical consequence: the **constraint Jacobian `Φ_q` is rank-deficient**.
The true number of independent constraints is `r = rank(Φ_q) < 3J`, and the
true DOF count is

```
F_true = 6B − rank(Φ_q)
```

For the octahedron-only 3×3×3 subsystem, `F_true ≥ 0` only as long as the
rank deficiency exactly matches the apparent deficit. In practice the
deficiency is **larger** than the deficit (because so many cycles in the lattice
duplicate the same rigidity statement), so even though the Grübler count gives
0, the subsystem is structurally over-constrained: there exist constraint rows
that are linear combinations of others.

### 3.4 What overconstraint does numerically

In an NSC (non-smooth contact) solver like PyChrono's, the redundant rows of
`Φ_q` produce a singular Schur complement when the solver tries to compute
Lagrange multipliers:

```
M·v̇ + Φ_qᵀ·λ = f
Φ_q·v̇        = γ
```

If `Φ_q` is rank-deficient, the matrix `Φ_q M⁻¹ Φ_qᵀ` is singular and the
multipliers `λ` are not unique. Iterative projected-Gauss-Seidel solvers (which
NSC uses) tolerate this for *small* redundancy by spreading the impulse across
the redundant rows, but as the deficiency grows:

- Convergence slows (iteration count balloons).
- Multipliers oscillate between equivalent solutions, leaking energy.
- Conditioning of the system matrix degrades; small numerical errors get
  amplified into spurious internal forces.
- Eventually the solver returns garbage or diverges.

This is exactly the failure mode that the 5×5×5 scaling work is hitting and
why the 3×3×3 case matters as a test bed.

## 4. The scaling story — why 3×3×3 is the critical hinge

| Grid    | N_oct | J_inter | F_oct-only |
|---------|------:|--------:|-----------:|
| 2×2×3   | 12    | 20      | **+12**    |
| 3×3×3   | 27    | 54      | **0**      |
| 5×5×5   | 125   | 300     | **−150**   |

The constraint count grows roughly as `3·d·N` (where `d` is the lattice
dimension) while DOFs grow only as `6·N`. The crossover happens **between
2×2×3 and 3×3×3**:

- At 2×2×3 there are 12 spare DOFs — the lattice has slack and the solver
  shrugs off any redundancy.
- At 3×3×3 the budget is exactly 0 — any rank deficiency at all means
  overconstraint.
- At 5×5×5 the deficit is −150 — the system is *deeply* overconstrained, the
  Jacobian is profoundly rank-deficient, and the rigid-joint formulation
  essentially cannot be solved.

That is why 3×3×3 is the right size to study the problem: it is the smallest
grid where overconstraint actually bites, but it is small enough that you can
run hundreds of solver iterations and inspect the constraint Jacobian by hand.

## 5. Why solving it is critical

Three reasons:

1. **It is the only thing blocking 5×5×5 (and beyond).** Every downstream
   scientific result we want — buckling thresholds, collapse modes, statistical
   averages over seeds — lives at the 5×5×5 scale or larger. The rigid-joint
   formulation cannot reach there. Without a fix, the project tops out at
   2×2×3.
2. **It is a structural problem, not a tuning problem.** No amount of
   `SetMaxIterations`, smaller `dt`, or solver substitution will rescue a
   rank-deficient Jacobian. The fix has to change the *constraint topology*
   itself, not the numerics.
3. **The fix generalizes.** The same lattice topology will recur in any
   periodic 3D mechanism we want to study (foams, metamaterials, scaffolds).
   A formulation that solves 3×3×3 cleanly is directly reusable.

## 6. The strategy — make DOF grow faster than constraints

The Grübler inequality `F = 6B − ΣC > 0` can be rescued in only two ways:

**(A) Add DOFs.** Increase `B` or `6B` faster than constraints grow. Concretely:
- **Compliant joints (TSDA springs).** Replace each `ChLinkLockSpherical`
  (3 hard scalar constraints, contributes to `Φ_q`) with three orthogonal
  `ChLinkTSDA` springs at the shared vertex. Springs are **forces, not
  constraints** — they live on the right-hand side of the equations of motion
  and contribute *zero* rows to `Φ_q`. The Jacobian's rank deficiency problem
  literally vanishes because there is no Jacobian for these couplings to be
  deficient in. The cost is physical fidelity (joints can stretch under load)
  and a smaller `dt` for stability of stiff springs. This is the path
  currently being prototyped on the `tsda-solver` branch (see `tsda_plan.md`).
- **Per-vertex auxiliary bodies.** Insert a small "hub" body at each shared
  vertex and connect it to its neighbors with point-on-line or point-on-plane
  constraints that each remove fewer than 3 DOFs. This raises `B` and lowers
  the per-coupling `c` simultaneously.

**(B) Remove constraints.** Lower `ΣC` while keeping the geometry intact:
- **Edge or face joints instead of vertex joints.** A single joint that locks
  a *shared edge* couples two bodies along an edge with `c = 5` rather than
  pairs of vertices contributing `c = 3 + 3 = 6`. Choosing the joint primitive
  to match the geometry of the contact reduces over-counting.
- **Detect and drop redundant constraints.** Build the full Jacobian, compute
  its row-echelon form (or run an SVD), and explicitly remove rows whose
  removal does not change the rank. This is rigorous but expensive and brittle
  — it only works if you can refresh the redundancy set when the geometry
  changes.
- **Use a regularized solver.** Replace pure NSC with a **stabilized index-1
  formulation** (Baumgarte stabilization, GGL, or SMC penalty contacts) that
  tolerates rank-deficient `Φ_q` by projecting the multiplier solution onto
  the consistent subspace.

The cleanest approach — and the one that genuinely makes DOF grow faster than
constraints — is **(A): switch to compliant couplings**. Every spring added is
zero rows in `Φ_q` and zero new constraints. So the constraint count stops
growing entirely. Meanwhile DOFs still grow as `6B`. The inequality
`F = 6B − ΣC > 0` becomes trivially true at every grid size, including 5×5×5
and beyond.

## 7. What this means concretely for the 3×3×3 testbed

The 3×3×3 scaffold (per `3x3x3_plan.md`) ports the rigid-joint baseline so a
joint-experiment branch can A/B against it. The validation protocol is:

1. **Topology sanity** — `build_system` prints `54` inter-oct joints and
   `72` total joints. (Verified by §3 above.)
2. **Rest state** — under gravity only, max velocity stays below `1e-3 m/s`.
3. **Baseline parity** — center-octahedron tilt at `t = 2 s` agrees with the
   rigid-joint baseline within 5° on at least one seed.
4. **Buckling emerges** — at least 3 of 5 seeds reach > 30° tilt by `t = 5 s`
   under the default 0.5 N/column load.
5. **Solver health** — no convergence failures, wall-clock within 3× baseline.

The new joint formulation passes the test if it produces the same qualitative
collapse dynamics as the rigid baseline *and* keeps the constraint Jacobian
out of the rank-deficient regime. The TSDA branch is the leading candidate
because, as argued in §6, it solves the rank deficiency by construction
rather than by tuning.

## 8. Summary

- The 3×3×3 octahedral lattice has **54 inter-octahedron ball joints** at
  shared vertices (plus 18 boundary joints), constructed by an O(N²·36) vertex
  coincidence scan and instantiated through a single `_add_ball_joint` seam.
- The Grübler DOF count for the octahedron-only subsystem is **exactly zero**.
  Lattice cycles (28 independent loops) introduce rank deficiency in the
  constraint Jacobian, pushing the *true* DOF count negative — the system is
  structurally overconstrained even when the naive count says it is just rigid.
- Numerically this manifests as a singular Schur complement, ill-conditioned
  multipliers, and slow / divergent NSC iterations. At 5×5×5 the deficit is
  −150 and the rigid-joint formulation simply cannot be solved.
- The only structural fix is to change the constraint topology so that DOFs
  grow faster than constraints. The cleanest realization of that idea is to
  replace rigid `ChLinkLockSpherical` joints with compliant `ChLinkTSDA`
  springs, which contribute zero rows to `Φ_q` and therefore eliminate the
  rank-deficiency problem at every grid size.
- 3×3×3 is the right testbed: the smallest grid where overconstraint actually
  bites, small enough to inspect the Jacobian directly, large enough that any
  formulation that survives here is a credible candidate for 5×5×5.
