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
| Fillet, chamfer and shell take a boolean result only where it reads as a prism, (shell) as a prism cut by through tools with its caps or a run of straight walls removed, or (fillet, chamfer) at straight edges along an axis, or (chamfer) at complete loops of planar faces | Modify operations |
| No import of any file format | Data exchange |
| Booleans outside the exact prism classes fall to a faceted mesh result, and refuse touching contact | Booleans |

## Feature operations

| Gap | Today | Owner |
|---|---|---|
| Draft angle on a free-form wall, a circular corner that is not G1, or a surface result | `ErrUnsupported`, draft SD3/SD4/SD12 (`draft_build.go`) | `docs/draft-design.md` §14 |
| Boolean, mass and interference readings of a draft body whose circular wall joins a neighbour G1 but not exactly tangent, or whose section holds a trimmed segment | `ErrUnsupported`, "no proof of the volume" (`capblend_admit.go`); `Verify` reads the pair `Suspect`; the mesh exports | `docs/draft-design.md` §9.1 |
| `Body.Draft` of a wall subset that moves one of two walls meeting at a circular corner, about a `NeutralFrame` or a non-cap face, or of a receiver that is not a straight prism | `ErrUnsupported`, draft SD4/SD20/SD23 (`draft.go`, `draft_build.go`) | `docs/draft-design.md` §10.2, §14 |
| Sweep twist beyond one centred convex polygon on an origin XY sketch and one positive-Z line, or beyond one radian | `ErrUnsupported` (`sweep_twist.go`) | `docs/sweep-design.md` §17 |
| Composite sweep mesh over other free-form profile kinds, meridian poles or changed station order | `ErrUnsupported` (`tessellate_sweep_composite.go`); line, circular and `FitSplineSeg` profiles mesh and enter booleans | `docs/sweep-design.md` Table D |
| Closed sweep path | `ErrUnsupported`, "closed sweep paths are not implemented" (`sweep.go`) | `docs/sweep-design.md` |
| Free-form sweep path | `Path` holds only `LineTo` and `ArcThrough` segments (`path.go`) | `docs/sweep-design.md` §2–§3 |
| Coil of a profile with free-form, elliptical or trimmed segments | `ErrUnsupported` (CS7) (`internal/coil/profile.go`) | `docs/helix-design.md` CS7 |
| Composite path in `SweepChain` | `ErrUnsupported` (R34); one straight span only | `docs/surface-design.md` §1.2 |
| General loft over more than two sections, guide rails, centerline | `LoftSections` admits three homothetic line loops on equally spaced XY planes with origin in the first region's kernel; other shapes refuse | `docs/loft-sections-design.md`, `docs/loft-design.md` §1 |
| Point-section loft of a holed profile | `LoftFromPoint` refuses holes because the common apex would have a non-manifold vertex link | `docs/loft-point-design.md` |
| Detached profile validation past 11585 reconstruction chords; authenticated Loft still stops above its 46340-chord cap | `ErrUnsupported`: R7 sketch reconstruction work budget | `docs/loft-gear-bounds-design.md` §7, `docs/spline-design.md` R7 |
| Loft with differing loop or segment counts | `ErrUnsupported` (S1/S2) | `docs/loft-design.md` Table S |
| Loft of mixed-kind or reversed pairs | Refused permanently | `docs/loft-design.md` §1 "Permanently out of scope" |
| `LoftChain` with curved segments or non-parallel planes | `ErrUnsupported` (R36, R35) | `docs/surface-design.md` §1.2 |
| Hole, rib, web, emboss features | No entry point exists | none |

## Modify operations

| Gap | Today | Owner |
|---|---|---|
| Fillet/chamfer of a sweep, faceted boolean result, cap blend or draft body; loft chamfer outside the matching axial, zero-or-one-bore, both-outer-cap route | `ErrUnsupported`: modify S3, SX9 or SX10 (`fillet.go`, `chamfer.go`, `loft_chamfer.go`) | `docs/modify-reach-design.md` Table RX, `docs/loft-design.md` §18 |
| Fillet of a loft with different sections, non-axial translation, holes, or a selected free-form corner | `ErrUnsupported` (`loft_fillet.go`); matching axial lofts accept analytic corners | `docs/loft-design.md` §17 |
| Fillet/chamfer of a brep or stacked result's single edges sharing a vertex with no planar loop after straight-wall restatement, an edge ending on a curved face outside the exact-radius corner-sphere route, or an oblique, split or level-displaced straight wall | `ErrUnsupported` (brep-modify SB5–SB9, modify-general SL1); a convex prism's single cap edge can use the bounded faceted cutter when it or its end wall is oblique | `docs/vertex-blend-design.md` §2, `docs/brep-modify-design.md` Table SB |
| Fillet or chamfer of a loop whose neighbouring face is curved, oblique, split or lies on both sides of the loop's face; a complete-loop fillet whose line–arc or arc–arc offset miter folds or cannot be bounded | `ErrUnsupported` (modify-general SL2, loop-fillet SF1) (`brep_modify_loop.go`, `brep_loop_fillet.go`) | `docs/modify-general-design.md` §4.4, `docs/loop-fillet-design.md` §6 |
| Mesh boolean over a brep body whose complete-loop chamfer band holds an apex cone at a reflex corner (a hole or pocket mouth) | `ErrUnsupported` (the cap-loop chamfer's admission, `capBlendOccupiedVolumeAdmission`) (`boolean.go`, `brep_loop_band.go`) | `docs/modify-general-design.md` §5 Table DG, DG4 |
| Clearance of a brep body carrying complete-loop chamfer or fillet bands | pair reads `Suspect` unless its boxes decide it; no model (`clearance_geom.go`) | `docs/modify-general-design.md` §5 Table DG, DG6 |
| Shell outside the exact rectangular blind-pocket, stacked-boss, and through-cut cases; outward or closed brep Shell; unadmitted openings and offsets | `ErrUnsupported` (SG1–SG7) | `docs/modify-general-design.md` §3.1a–§3.4, §6 |
| Shell side opening on a prism with an oblique removed end face recorded as several segments, a rim cut behind a removed arc whose recorded end lies off the circle its start defines, or a kept arc meeting a removed arc at an end corner whose cut is a float solve, under a kept cap; a revolve side run that leaves two wall pieces; a side opening meeting a kept face smoothly at an end, or whose rim runs past the removed walk's far end | `ErrUnsupported` (SO5 for a prism; SX8 for a revolve; SO1, SO2) (`shell_opening.go`, `shell_opening_brep.go`, `shell_revolve.go`, `internal/offset2d/opening.go`) | `docs/shell-opening-design.md` §5, §12, `docs/modify-reach-design.md` §9.3 |
| Shell of a revolve keeping an angular cap, with a holed meridian or one meeting the axis twice, or whose outward wall reaches the axis | `ErrUnsupported` (SX8) (`shell_revolve.go`) | `docs/modify-reach-design.md` §9.3 |
| Fillet/chamfer of a revolve cap edge or an edge on the axis | `ErrUnsupported` (SX5) (`revolve_blend.go`) | `docs/modify-reach-design.md` §7 |
| Fillet or chamfer of a prism cap edge outside the convex straight-section cutter admission whose end faces route E cannot restate as planes | `ErrUnsupported` (brep-modify SB7); admitted oblique cap edges build as bounded faceted results | `docs/vertex-blend-design.md` §2 |
| Chamfer of two or more edges on part of a cap loop, or cap and lateral edges together | `ErrUnsupported` (SX4) | `docs/modify-reach-design.md` Table SX |
| Cap-loop chamfer at a corner where a circular wall meets a neighbour tangentially but runs back against it (a tangent cusp), or turns past the G1 tolerance too slightly to enclose | `ErrUnsupported` (SX14) (`capblend.go`) | `docs/modify-reach-design.md` Table SX |
| Asymmetric brep or stacked chamfer selection outside routes E/L | `ErrUnsupported` (SX16); E takes straight edges, L takes full planar loops | `docs/modify-reach-design.md` §6, `docs/modify-general-design.md` §4 |
| Tangent chain that branches or whose G1 continuity the oracle cannot decide | `ErrUnsupported` (SX2) (`tangent_chain.go`) | `docs/modify-reach-design.md` §5 |
| Closed shell (`WithNoOpenings`) of any receiver but a full revolve or a hole-free straight prism | `ErrUnsupported` (SX8/SX9; modify-general SG2 for a brep or stacked receiver) (`shell.go`) | `docs/modify-reach-design.md` §9, §14 |
| Modify of a prism whose section carries a displacement bound | `ErrUnsupported` via `requireExactSection` | `docs/modify-design.md` |
| Shell or junction fillet/chamfer of a revolve whose meridian carries a displacement bound — a revolve shell with a slanted cut, such as a cone's | `ErrUnsupported` via `requireExactRevolveSection` | `docs/surface-intersection-design.md` §7.2 |
| Variable-radius fillet, face-to-face fillet | No entry point exists | none |

## Booleans

| Gap | Today | Owner |
|---|---|---|
| Exact result outside the admitted prism and point-loft/cone classes | Pair takes the mesh boolean → `Faceted` faces with a volume bound | `docs/general-boolean-design.md` §3, `docs/loft-point-design.md` "Coaxial finite cone trims" |
| Blind cut with displaced interface levels, touching or overlapping opposed holes, or a crossing hole | Mesh path; coplanar contact can refuse. Exact interface cuts with separated opposed holes build after whole-loop proofs | `docs/stacked-prism-design.md` §7 stage 3 |
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
| `Thicken` of composite sweep, loft, stitched, `Body.Patch` or `Unstitch` sheets | `ErrUnsupported` (R24); one-span straight and arc sweep sheets use their analytic reductions | `docs/surface-design.md` §16.8 |
| `Body.Patch` over a rim with a bounded vertex (a cut junction, a trimmed line end, a circle seam whose centre plus radius rounds) that no straight prism build stamped with one level token, such as a `Patch` sheet's rim | `ErrUnsupported` (R6) | `docs/surface-design.md` §5.2 |
| Tessellation of `Body.Patch` with a curved face or edge outside a single-circle prism sheet filled in one call, or with a nonzero face-normal bound | `ErrUnsupported`: no shared chording for that face or edge | `docs/surface-design.md` §10; `docs/tessellation-design.md` §2 |
| `Stitch` closing a revolve sheet whose wall tags are not exactly their records: a tilted or round-anchored axis, a near-parallel or near-perpendicular side, a snapped centre | `ErrUnsupported` (R8, `stitchFluxTagsDenoted`) | `docs/surface-design.md` §6.4 |
| `Extend` of a partially revolved ribbon | `ErrUnsupported` (RS14) | `docs/surface-intersection-design.md` §2.2 |
| `Trim`/`Extend`/`Split` over a pair sharing no generator | Refused | `docs/surface-intersection-design.md` §4 |
| Tolerant stitch, Ruled, Boundary Fill | Refused permanently | `docs/surface-design.md` §1.3 |
| Reverse Normal | Named, staged for no increment | `docs/surface-design.md` §1.4 |

## Data exchange

| Gap | Today | Owner |
|---|---|---|
| Import of STEP, IGES, STL, OBJ or 3MF | No import entry point exists; mesh import is a v1 non-goal | `docs/api-design.md` §13 |
| Analytic STEP for cones, spheres, horn tori, free-form walls | Whole body falls back to faceted STEP; planes, cylinders and tori with circle, arc, line and ellipse edges write analytic (`export/step_analytic.go`, `export/step_analytic_torus.go`) | `docs/step-export-design.md` |
| STEP of a body with a cavity: a `WithNoOpenings` closed shell, a full revolve of a cavity meridian or a holed meridian | `ErrUnsupported`: "only a single non-void shell is supported" (`export/step.go`) | `docs/step-export-design.md` |

## Queries

| Gap | Today | Owner |
|---|---|---|
| Point containment near an approximate boundary or on a body without a verified occupied-volume mesh | `Body.LocatePoint` returns `PointUndecided` or the mesh's `ErrUnsupported`; certified interior, exterior and exact-boundary points build | `docs/point-containment-design.md` |
| Closest point from a point to a body or face | `Body.DistanceToPoint` measures a solid and `Face.DistanceToPoint` measures a trimmed face; neither returns a nearest point | `docs/point-containment-design.md` §§4–5, `docs/clearance-design.md` |
| Planar cross-section of a body | No entry point exists | none |
| Surveys (undercut, wall, concave radius) of bodies other than prisms, revolves, cups, cap blends and one-sided draft bodies; the wall and concave-radius surveys of a draft body | `Verify` reports `Suspect`; a draft body's with `DiagUnsupportedSurveyPayload` | `docs/verification-design.md`, `docs/draft-design.md` DD8 |
| Wall survey of a sphere's revolve, solid or hollow (its meridian arcs meet the axis at both ends) | `Verify` reports `Suspect` (`DiagUndecidedWall`) | `docs/verification-design.md` |
| Wall, undercut and concave-radius surveys and clearance on a revolve whose meridian carries a displacement bound | `Verify` reports `Suspect` | `docs/surface-intersection-design.md` §7.2 |

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

## Also consider

Features common CAD tools ship that decad lacks and no design doc plans or rejects. Writing a design doc for one
moves its row to the matching section above, with that doc as owner.

| Gap | Today |
|---|---|
| Direct edits: move, offset, delete or replace a face of a solid, press-pull | No entry point exists; `Body.Offset` refuses every solid (`offset.go`, `docs/surface-design.md` R37) |
| Scaling a body, uniform or per axis | No entry point exists; `Placed`/`PlacedCopy` take a rigid motion only (`document.go`) |
| Saving a `Document` to a file or loading one back | No entry point exists; a model is rebuilt by rerunning the Go program that built it |
| Text profiles from a font, for engraved or embossed lettering | No entry point exists |
| 2D drawings: projected views, silhouette and hidden lines, DXF output | No entry point exists |
| PMI or GD&T, as data or in STEP AP242 | No entry point exists; `export.STEP` writes AP214 geometry only |
| IGES, glTF or Parasolid export | `export` writes STL, OBJ, 3MF and STEP only (`docs/api-design.md` §11) |
