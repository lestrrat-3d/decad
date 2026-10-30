# decad file layout: booleans

Boolean files.
The rules for rows live in `agent-docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `boolean.go` | Public `Union`/`Cut`/`Intersect`, `BooleanError` mapping, contained-Cut source volume, centroid, and moment capture, and scale-source marking; hands route order to `internal/booleandispatch/` with root-only hooks. See evaluator §9 and operation-history-design. |
| `boolean_fold.go` | Two-at-a-time fold of a many-body `Union` that no certified route takes: a trial chain on copies, then the real one. See union-fold design. |
| `boolean_many.go` | Validates and commits a multi-body Union, dispatching the certified pattern, one-scene prism and straight-source Loft routes and transient prism pair chains. See API §8 and prism-boolean §4.6. |
| `boolean_regroup.go` | Chain regroup for a refused two-body Boolean: reads `Document.producers`, maps invariant-break errors. See boolean-chain-regroup-design. |
| `surface_trim.go` | Public `Trim`, `Extend` and `Split` entry checks and commits; `internal/surfacetrim/` builds them. See surface-intersection §2–§3. |
| `point_plane_scale.go` | ScaledAt: resolves the similarity source by payload kind and commits the image; facetedrecord.ScaleImage admits and scales it. See loft-point design and evaluator §8. |
| `prism_scale.go` | Admits prism and full-Revolve certified similarity sources and transfers their face roles. See evaluator §8. |
| `stack_brep_scale.go` | Admits exact line-only stacks and stack-backed BReps for certified similarity. See evaluator §8. |
| `point_cap_extremes.go` | Resolves Split cap faces and selects the ends using facetedrecord's centroid-order proof. See loft-point design. |
| `boolean_mesh.go` | Runs `internal/booleaneval`'s mesh boolean over two bodies and states the audited mesh's occupied volume as a public measurement, with the exactness the bound implies. See evaluator §9. |
| `internal/boolupstream/` | Attributes an analytic Boolean result's planar faces to the one operand face exact plane identity proves each lies inside, and stores the roles in the payload's `Upstream`. See prism-boolean §11. |
| `internal/capplane/` | Reads a `CapRecord` plane, a sweep axis and plane identity as exact rationals for Loft cap levels and Boolean attribution. See prism-boolean §11, loft §18. |
