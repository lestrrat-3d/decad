# Missing features

Index of what decad cannot build, read, import or export today. Read this first when asked what is missing,
what blocks a "complete" CAD engine, or which gap to work next.

## How to use this page

- This page is an index, not authority. Each row names its owning design doc; the owner wins over this page.
- Before reporting a row as still missing, confirm it in code: grep the refusal named in its "Today" cell.
  A row whose refusal no longer exists is stale → fix this page in the same change.
- Closing a gap or adding a refusal → update the matching row in the same PR. Delete rows that close.
- NEVER list a gap here that the code does not refuse or lack. Planned-but-unshipped work in a design doc
  counts as missing until the code builds it.
- Rows state current behaviour only. NEVER record when a gap opened or closed.

## Gaps that block ordinary parts

Ranked by how many ordinary machined or printed parts each gap stops end to end.

| Gap | Section below |
|---|---|
| Fillet, chamfer and shell take a boolean result only where it reads as a prism, or (fillet, chamfer) at straight edges along an axis | Modify operations |
| No draft: nonzero extrude taper refuses, and no face-draft op exists | Feature operations |
| No helical path → no threads, springs or coils | Feature operations |
| No import of any file format | Data exchange |
| Booleans outside the exact prism classes fall to a faceted mesh result, and refuse touching contact | Booleans |

## Feature operations

| Gap | Today | Owner |
|---|---|---|
| Draft angle on extrude | `WithTaper` nonzero → `ErrUnsupported` (`extrude.go`) | `docs/evaluator-design.md` §5 |
| Draft of existing faces | No entry point exists | none |
| Sweep twist | `WithSweepTwist` nonzero → `ErrUnsupported` (`sweep.go`) | `docs/sweep-design.md` |
| Closed sweep path | `ErrUnsupported`, "closed sweep paths are not implemented" (`sweep.go`) | `docs/sweep-design.md` |
| Helical or free-form sweep path | `Path` holds only `LineTo` and `ArcThrough` segments (`path.go`) | `docs/sweep-design.md` §2–§3 |
| Composite path in `SweepChain` | `ErrUnsupported` (R34); one straight span only | `docs/surface-design.md` §1.2 |
| Loft over more than two sections, guide rails, centerline | No entry point; `Loft` takes exactly two profiles | `docs/loft-design.md` §1 "Deferred reach" |
| Loft of a same-kind free-form pair whose curves convert to different Bézier span counts | `ErrUnsupported` (S17) | `docs/loft-design.md` §12 PR 5 |
| Loft of a profile past 11585 reconstruction chords (the 5-fit-point helical gear outline builds to 69 teeth, refuses at 70) | `ErrUnsupported`: R7 sketch reconstruction work budget | `docs/loft-gear-bounds-design.md` §7, `docs/spline-design.md` R7 |
| Loft with differing loop or segment counts | `ErrUnsupported` (S1/S2) | `docs/loft-design.md` Table S |
| Loft of mixed-kind or reversed pairs | Refused permanently | `docs/loft-design.md` §1 "Permanently out of scope" |
| `LoftChain` with curved segments or non-parallel planes | `ErrUnsupported` (R36, R35) | `docs/surface-design.md` §1.2 |
| Hole, thread, rib, web, emboss, coil features | No entry point exists | none |

## Modify operations

| Gap | Today | Owner |
|---|---|---|
| Fillet/chamfer of a sweep, loft, faceted boolean result or cap blend | `ErrUnsupported`: "fillets a straight prism or a revolve only", SX9, SX10 (`fillet.go`, `chamfer.go`) | `docs/modify-reach-design.md` Table RX |
| Fillet/chamfer of a brep or stacked boolean result's curved edge, edges sharing a vertex, an edge ending on a curved face or an earlier blend, or an edge on or along a straight wall that is oblique, split or has a displaced level, where no prism reading takes the selection | `ErrUnsupported` (brep-modify SB4–SB9) (`brep_modify_edge.go`) | `docs/brep-modify-design.md` Table SB, §12 |
| Shell of a body that is neither a prism nor a revolve; a brep that reads as a prism builds | `ErrUnsupported`: "shells a straight prism only", brep-modify SB3/SB10 (`shell.go`, `brep_modify.go`) | `docs/brep-modify-design.md` SB10 |
| Shell removing a prism side face, or a revolve side run that leaves two wall pieces or meets the kept faces at other than a right angle against a straight removed walk | `ErrUnsupported` (S2 for a prism, SX8 for a revolve) (`shell.go`, `shell_revolve.go`) | `docs/modify-reach-design.md` §9.2's open question, §9.3 |
| Shell of a revolve keeping an angular cap, with a holed meridian or one meeting the axis twice, or whose outward wall reaches the axis | `ErrUnsupported` (SX8) (`shell_revolve.go`) | `docs/modify-reach-design.md` §9.3 |
| Fillet/chamfer of a revolve cap edge or an edge on the axis | `ErrUnsupported` (SX5) (`revolve_blend.go`) | `docs/modify-reach-design.md` §7 |
| Fillet of a cap edge (vertex blend) | `ErrUnsupported`, "the vertex-blend problem, not yet supported" (`fillet.go`) | `docs/modify-design.md` §6 |
| Chamfer of a partial cap loop, or cap and lateral edges together | `ErrUnsupported` (SX4) | `docs/modify-reach-design.md` Table SX |
| Asymmetric chamfer of a brep or stacked boolean result | `ErrUnsupported` (SX16) (`chamfer.go`) | `docs/modify-reach-design.md` §6 |
| Tangent chain that branches or whose G1 continuity the oracle cannot decide | `ErrUnsupported` (SX2) (`tangent_chain.go`) | `docs/modify-reach-design.md` §5 |
| Closed shell (`WithNoOpenings`) of any receiver but a full revolve or a hole-free straight prism | `ErrUnsupported` (SX8/SX9/SX16) (`shell.go`) | `docs/modify-reach-design.md` §9, §14 |
| Modify of a prism whose section carries a displacement bound | `ErrUnsupported` via `requireExactSection` | `docs/modify-design.md` |
| Shell or junction fillet/chamfer of a revolve whose meridian carries a displacement bound — a revolve shell with a slanted cut, such as a cone's | `ErrUnsupported` via `requireExactRevolveSection` | `docs/surface-intersection-design.md` §7.2 |
| Variable-radius fillet, face-to-face fillet | No entry point exists | none |

## Booleans

| Gap | Today | Owner |
|---|---|---|
| Exact result outside the admitted prism classes | Pair takes the mesh boolean → `Faceted` faces with a volume bound | `docs/general-boolean-design.md` §3 |
| Touching, coplanar or face-on-face contact on the mesh path | `BooleanError` code `BooleanUnsupportedContact` | `docs/general-boolean-design.md` §2, `docs/interference-design.md` |
| Curved surfaces tangent with facets that never meet | `ErrUnsupported` | `doc.go` support map |
| Held mesh operand coarser than the pair tolerance | `ErrUnsupported` | `doc.go` support map |
| Sheet body as a boolean operand | Refused permanently; use `Document.Split` | `docs/surface-design.md` Table X |

## Free-form curves

| Gap | Today | Owner |
|---|---|---|
| Tier B (conic, ellipse) or Tier C (unequal-weight NURBS) segment in any build | `ErrUnsupported` (R10) until spline P9 | `docs/spline-design.md` §8 Table C |
| Tier A chain with no single curvature sign | `ErrUnsupported` (R19) | `docs/spline-design.md` §6.5 |
| Modify ops over free-form walls | Refused per R3–R5, except the analytic-corner slice | `docs/spline-design.md` §4.1 |

## Sheets and surfaces

| Gap | Today | Owner |
|---|---|---|
| `Thicken` of sweep, loft, stitched, `Body.Patch` or `Unstitch` sheets | `ErrUnsupported` (R24) | `docs/surface-design.md` §16.8 |
| `Stitch` closing a revolve sheet whose wall tags are not exactly their records: a tilted or round-anchored axis, a near-parallel or near-perpendicular side, a snapped centre | `ErrUnsupported` (R8, `stitchFluxTagsDenoted`) | `docs/surface-design.md` §6.4 |
| `Extend` of a partially revolved ribbon | `ErrUnsupported` (RS14) | `docs/surface-intersection-design.md` §2.2 |
| `Trim`/`Extend`/`Split` over a pair sharing no generator | Refused | `docs/surface-intersection-design.md` §4 |
| Tolerant stitch, Ruled, Boundary Fill | Refused permanently | `docs/surface-design.md` §1.3 |
| Reverse Normal | Named, staged for no increment | `docs/surface-design.md` §1.4 |

## Data exchange

| Gap | Today | Owner |
|---|---|---|
| Import of STEP, IGES, STL, OBJ or 3MF | No import entry point exists; mesh import is a v1 non-goal | `docs/api-design.md` §13 |
| Analytic STEP for cones, spheres, tori, free-form walls | Whole body falls back to faceted STEP; only planes and cylinders write analytic (`export/step_analytic.go`) | `docs/step-export-design.md` |
| STEP of a body with a cavity: a `WithNoOpenings` closed shell, a full revolve of a cavity meridian or a holed meridian | `ErrUnsupported`: "only a single non-void shell is supported" (`export/step.go`) | `docs/step-export-design.md` |

## Queries

| Gap | Today | Owner |
|---|---|---|
| Point containment (is a point inside a body) | No entry point exists | none |
| Distance or closest point from a point to a body or face | No entry point exists; body-pair gaps come only from `Verify` `WithClearances` | `docs/clearance-design.md` |
| Planar cross-section of a body | No entry point exists | none |
| Surveys (undercut, wall, concave radius) of bodies other than prisms, revolves, cups, cap blends | `Verify` reports `Suspect` | `docs/verification-design.md` |
| Wall survey of a sphere's revolve, solid or hollow (its meridian arcs meet the axis at both ends) | `Verify` reports `Suspect` (`DiagUndecidedWall`) | `docs/verification-design.md` |
| Wall, undercut and concave-radius surveys, clearance and the tolerance gate's revolve arm on a revolve whose meridian carries a displacement bound | `Verify` reports `Suspect`; the gate diameter is withheld | `docs/surface-intersection-design.md` §7.2 |
| Tight volume, area and centroid bounds of a revolve whose meridian holds an `ArcSeg` recorded over a narrowed range (a ball `Split` across its axis, a sketch-cut sphere cap) | The reading encloses the true value, but its bound can exceed the value: `circularbounds.AreaInterval` proves no trimmed `ArcSeg`, so its float envelope bounds the region integral | none |

## Model structure — v1 non-goals

`docs/api-design.md` §13 owns this list. Lifting any of them changes that section first.

| Gap | Today |
|---|---|
| Assemblies (`Component`/`Occurrence`) | Bodies are placed explicitly with `Placed`/`PlacedCopy` |
| Feature tree, timeline, rollback, re-evaluation after a sketch edit | Bodies are immutable; a sketch edit rebuilds nothing |
| Sheet metal | No entry point exists |
| GUI or view state | No entry point exists |
| Fusion code generation | No entry point exists |

## Motion and dynamics

| Gap | Today | Owner |
|---|---|---|
| Joints, deformation, fracture in `dynamics` | Not in the program | `docs/multibody-dynamics-design.md` §1 |
| Contact manifold for two non-convex bodies touching on more than one face | Relation only, `ContactNonConvex` | `docs/multibody-dynamics-design.md` §1 |
| Sleeping or deactivation | Not in the program | `docs/multibody-dynamics-design.md` §1 |
