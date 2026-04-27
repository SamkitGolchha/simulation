## 2x2x3 bushing refactor — port source snapshot

`simulation_2x2x3_bushing_snapshot.py` is a frozen copy of
`../../../2x2x3_simulation/src/simulation_2x2x3.py` as it stood in the
working tree when the `joints-3x3x3-bushings` branch was cut
(2026-04-21). It contains the bushing refactor (ChLoadBodyBodyBushingSpherical,
STIFFNESS_VARIANTS, CHANGE2_VARIANTS, ChSolverADMM, ChLoadContainer) that
the plan refers to as "committed 2026-04-21" — that commit never
happened; the refactor was validated locally and not pushed.

Use this snapshot as the canonical reference for the port, not the live
2x2x3 working tree (which can drift if stashed or checked out). The hard
constraint "do not touch `2x2x3_simulation/`" stands — this is a read-only
reference copy living entirely under `3x3x3_simulation/`.
