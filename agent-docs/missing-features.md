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
| Fillet, chamfer and shell take a boolean result only where it reads as a prism, (shell) as a prism cut by through tools with its caps or a run of straight walls removed, or (fillet, chamfer) at straight edges along an axis, or (chamfer) at complete loops, one or two adjacent rising inner edges, two adjacent outer edges, or every straight outer edge of an admitted axis-aligned box, bored or bare | Modify operations |
| No import of any file format | Data exchange |
| Booleans outside the admitted analytic classes fall to a faceted mesh result, and may refuse touching contact | Booleans |

## Feature operations

| Gap | Today | Owner |
|---|---|---|
| `ToFace` on a tilted or uncertain face, or one without a cap record or exact planar Revolve wall or angular-cap record; use `Distance` | `ErrUnsupported` (`internal/extent/to_face.go`, `ToFaceStop`) | `agent-docs/evaluator-design.md` §5 |
| Draft angle on a free-form wall, a circular corner that is not G1, or a surface result | `ErrUnsupported`, draft SD3/SD4 (`internal/draftrecord`) and SD12 (`draft.go`) | `agent-docs/draft-design.md` §14 |
| Boolean, mass and interference readings of a draft body whose circular wall joins a neighbour G1 but not exactly tangent, or whose section holds a trimmed segment | `ErrUnsupported`, "no proof of the volume" (`internal/capband/admission.go`); `Verify` reads the pair `Suspect`; the mesh exports | `agent-docs/draft-design.md` §9.1 |
| `Draft` of a wall subset that moves one of two walls meeting at a circular corner, about a `NeutralFrame` or a non-cap face, or of a receiver that is not a straight prism | `ErrUnsupported`, draft SD4/SD20/SD23 (`draft.go`, `internal/draftrecord`) | `agent-docs/draft-design.md` §10.2, §14 |
| Sweep twist outside centred convex whole-line or admitted symmetric Line/Arc profiles and paths; straight twist beyond one turn or the station work ceiling; line-arc twist beyond one radian | `ErrUnsupported` (`internal/twistsweep/build_composite.go`) | `agent-docs/sweep-design.md` §17 |
| Composite sweep mesh over other free-form profile kinds, meridian poles or changed station order | `ErrUnsupported` (`internal/compositemesh/plan.go`); line, circular and `FitSplineSeg` profiles mesh and enter booleans | `agent-docs/sweep-design.md` Table D |
| Closed sweep path outside the exact two-arc full circle or exact-turn planar circular-section loops | `ErrUnsupported` (`internal/sweepbuild/build.go`); the admitted solids use certified full-Revolve or cyclic-span proofs | `agent-docs/sweep-design.md` §§3.3–3.5 |
| Free-form sweep path | `Path` holds only `LineTo` and `ArcThrough` segments (`path.go`) | `agent-docs/sweep-design.md` §2–§3 |
| Coil of a profile with free-form, elliptical or trimmed segments | `ErrUnsupported` (CS7) (`internal/coil/profile.go`) | `agent-docs/helix-design.md` CS7 |
| `Sweep` over a chain along other composite paths; sheet mesh for a single arc with a curved or multi-segment source, or an axis endpoint; placement of its composite sheet | `ErrUnsupported`; one straight off-axis Sketch line along one spatial arc builds and meshes, as does the exact cardinal line/arc composite (R34) | `agent-docs/sweep-design.md` §§15.8–15.9 |
| `Body.Section` outside certified prism, BRep, direct Loft, independent rectangular or three-section homothetic `LoftSections`, sphere or cone classes | `ErrUnsupported`; other `LoftSections` and two-cap circle crossings still refuse | `agent-docs/section-design.md` |
| General varying noncircular curved-section lofts, guide rails and centerlines | `LoftSections` admits three-section homothetic solids, three to thirty-two independently varying rectangular solids, selected curved classes, and four rounded-rectangle sections with cubic width; `LoftFaces` admits two to thirty-two ordered caps from one authenticated point-loft Split source; other shapes refuse | `agent-docs/loft-sections-design.md`, `agent-docs/loft-face-sections-design.md`, `agent-docs/loft-design.md` §1 |
| Point-section loft of a holed profile | `LoftFromPoint` refuses holes because the common apex would have a non-manifold vertex link | `agent-docs/loft-point-design.md` |
| Detached profile validation past 11585 reconstruction chords; authenticated Loft still stops above its 46340-chord cap | `ErrUnsupported`: R7 sketch reconstruction work budget | `agent-docs/loft-gear-bounds-design.md` §7, `agent-docs/spline-design.md` R7 |
| Loft with differing loop or segment counts | `ErrUnsupported` (S1/S2) | `agent-docs/loft-design.md` Table S |
| Loft of mixed-kind or reversed pairs | Refused permanently | `agent-docs/loft-design.md` §1 "Permanently out of scope" |
| Analytic cylinder topology for a Loft bore whose held center rounds or whose frame map has a nonzero orthonormality defect | The Loft keeps faceted bore faces because the cylinder builder publishes no wall-normal departure bound | `agent-docs/loft-design.md` §18 |
| `LoftChain` with curved segments or non-parallel planes | `ErrUnsupported` (R36, R35) | `agent-docs/surface-design.md` §1.2 |
| `Extrude` over a chain with `WithTaper` (any angle) or `WithSurfaceResult()` | `ErrUnsupported` after the seam gates and before the extent; a chain extrude always returns a sheet and has no draft build | `agent-docs/surface-design.md` §13.2 |
| `Emboss` on a non-planar face, or a relief with draft or a rim fillet | `Emboss` takes closed profiles on an analytic planar face (`emboss.go`) | `agent-docs/emboss-design.md` §5 |
| `Rib` or `Web` along a chain, arc or polyline, or one that extends itself to the body | `Rib` accepts one straight `*sketch.Line` and the caller sets the extent and overrun (`rib.go`) | `agent-docs/rib-design.md` §4 |

## Modify operations

| Gap | Today | Owner |
|---|---|---|
| Other Sweep Fillet/Chamfer targets and asymmetric Sweep Chamfer | `ErrUnsupported` (S1/S3); straight solid lateral edges use the exact prism reduction, and one straight cap edge takes Route E for either blend | `agent-docs/modify-design.md` R9 |
| Fillet/chamfer of most faceted boolean results, cap blend, or a draft body outside the positive-taper full-circle far-rim fillet and symmetric chamfer; loft chamfer outside the matching axial, zero-or-one-bore, both-outer-cap route | `ErrUnsupported`: modify S3, SX9 or SX10 (`fillet.go`, `chamfer.go`, `internal/brepmodify/loft_chamfer.go`); a complete oblique cylindrical boss root and authenticated copied-prism root join admit bounded fillets, one isolated rectangular-prism Union outer edge admits Fillet and equal-setback Chamfer, one perpendicular cylindrical bore mouth admits a bounded chamfer, and a closed circular rim between a retained plane and a coaxial retained cylinder, a straight edge between two retained planes that ends on retained planes, or a straight edge between a retained plane and a parallel retained cylinder that ends on planes normal to its axis, admits Fillet and equal-setback Chamfer | `agent-docs/modify-design.md` Table R, `agent-docs/modify-reach-design.md` §10.1, `agent-docs/draft-design.md` §10.4–§10.5, `agent-docs/loft-design.md` §18, `agent-docs/oblique-boss-fillet-design.md`, `agent-docs/prism-pattern-union-design.md`, `agent-docs/cross-cylinder-bore-chamfer-design.md`, `agent-docs/retained-edge-blend-design.md` |
| Fillet of a loft with different sections, non-axial translation, holes, or a selected free-form corner | `ErrUnsupported` (`internal/loftfillet/audit.go`); matching axial lofts accept analytic corners | `agent-docs/loft-design.md` §17 |
| Fillet/chamfer of a brep or stacked result's single edges sharing a vertex with no planar loop after straight-wall restatement, an edge ending on a curved face outside the exact-radius corner-sphere route, or an oblique, split or level-displaced straight wall | `ErrUnsupported` (brep-modify SB5–SB9, modify-general SL1); a convex prism's single cap edge can use the bounded faceted cutter when it or its end wall is oblique | `agent-docs/vertex-blend-design.md` §2, `agent-docs/brep-modify-design.md` Table SB |
| Fillet or chamfer of independent straight brep edges whose endpoint-face corners disagree with the interior material wedge | `ErrUnsupported` (brep-modify SB8); one Fillet edge on a planar-face loop or one Chamfer edge on a rising inner loop takes a partial band, including a square boss root | `agent-docs/brep-modify-design.md` EB8, `agent-docs/loop-fillet-design.md` §3, `agent-docs/modify-general-design.md` §4.3a |
| Fillet or chamfer of a loop whose neighbouring face is curved, oblique, split or lies on both sides of the loop's face; a complete-loop fillet whose line–arc or arc–arc offset miter folds or cannot be bounded | `ErrUnsupported` (modify-general SL2, loop-fillet SF1) (`internal/brepmodify/loop_route.go`, `internal/breprecord/fillet.go`) | `agent-docs/modify-general-design.md` §4.4, `agent-docs/loop-fillet-design.md` §6 |
| Mesh boolean over a brep body whose complete-loop chamfer band holds an apex cone at a reflex corner (a hole or pocket mouth) | `ErrUnsupported` (the cap-loop chamfer's admission, `capblend.OccupiedVolumeAdmission`) (`boolean.go`, `internal/brepbandmesh/tessellate.go`) | `agent-docs/modify-general-design.md` §5 Table DG, DG4 |
| Clearance of a brep body carrying complete-loop chamfer or fillet bands | pair reads `Suspect` unless its boxes decide it; no model (`internal/clearance/body_carriers.go`) | `agent-docs/modify-general-design.md` §5 Table DG, DG6 |
| Shell of a draft body outside the inward one-cap tapered circular cup | `ErrUnsupported` (modify S3) | `agent-docs/draft-design.md` §10.6 |
| Shell of a Sweep with a curved or multiple-span path, or nonzero twist outside the smooth symmetric rounded-profile inward top-cap case | `ErrUnsupported` (modify S3); one zero-twist straight profile span admits cap openings, side openings, and a hole-free closed cavity | `agent-docs/modify-design.md` R9–R10, `agent-docs/shell-twisted-sweep-design.md` |
| Shell outside the exact rectangular blind-pocket, rectangular or circular stacked-boss, coaxial two-level counterbore, circular through-wall, and other through-cut cases; outward or closed brep Shell; unadmitted openings and offsets | `ErrUnsupported` (SG1–SG7) | `agent-docs/modify-general-design.md` §3.1a–§3.4, §6 |
| Shell side opening on a prism with an oblique removed end face recorded as several segments, a rim cut behind a removed arc whose recorded end lies off the circle its start defines, or a kept arc meeting a removed arc at an end corner whose cut is a float solve, under a kept cap; a revolve side run that leaves two wall pieces; a side opening meeting a kept face smoothly at an end, or whose rim runs past the removed walk's far end | `ErrUnsupported` (SO5 for a prism; SX8 for a revolve; SO1, SO2) (`internal/brepmodify/shell.go`, `internal/revolveshell/shell.go`, `internal/offset2d/opening.go`) | `agent-docs/shell-opening-design.md` §5, §12, `agent-docs/modify-reach-design.md` §9.3 |
| Shell of a revolve keeping an angular cap, with a holed meridian or one meeting the axis twice, or whose outward wall reaches the axis | `ErrUnsupported` (SX8) (`internal/revolveshell/shell.go`) | `agent-docs/modify-reach-design.md` §9.3 |
| Fillet/chamfer of a revolve cap edge outside one bounded 90° rectangular radial-rim blend, or an edge on the axis | `ErrUnsupported` (SX5) (`internal/brepmodify/revolve_blend.go`) | `agent-docs/modify-reach-design.md` §7 |
| Fillet or chamfer of a cap edge outside the convex prism cutter or exact P8 front top edge | `ErrUnsupported` (SB7); admitted cases build as faceted results | `agent-docs/modify-general-design.md` §4.3c |
| Chamfer of a prism's partial cap loop or a brep/stacked cap chain outside one or two adjacent straight rising inner edges or two adjacent straight descending outer edges, or cap and lateral edges together outside the all-edge case of an admitted axis-aligned box, bored or bare | `ErrUnsupported` (SX4, SL1) | `agent-docs/modify-reach-design.md` Table SX, `agent-docs/modify-general-design.md` §4.3a–§4.3b |
| Placement of the dedicated all-edge chamfer result under an oblique transform, one moving a bore off y, or one rounding a required corner | `ErrUnsupported` (`breprecord.AllEdgeChamferPayload.Placed`) | `agent-docs/modify-general-design.md` §4.3b |
| Cap-loop chamfer at a corner where a circular wall meets a neighbour tangentially but runs back against it (a tangent cusp), or turns past the G1 tolerance too slightly to enclose | `ErrUnsupported` (SX14) (`internal/capblend/gate.go`) | `agent-docs/modify-reach-design.md` Table SX |
| Asymmetric brep or stacked chamfer selection outside routes E/L | `ErrUnsupported` (SX16); E takes straight edges, L takes full planar loops | `agent-docs/modify-reach-design.md` §6, `agent-docs/modify-general-design.md` §4 |
| Tangent chain that branches or whose G1 continuity the oracle cannot decide | `ErrUnsupported` (SX2) (`brep_modify_partial.go`, `internal/tangentchain/geometry.go`) | `agent-docs/modify-reach-design.md` §5 |
| Closed shell (`WithNoOpenings`) outside a full revolve, a hole-free straight prism or a two-slab nested rectangular boss | `ErrUnsupported` (SX8/SX9; modify-general SG2) (`shell.go`) | `agent-docs/modify-reach-design.md` §9, §14; `agent-docs/modify-general-design.md` §3.1a |
| Reverse-order nested rectangular boss whose frame cannot flip exactly | `ErrUnsupported` (`internal/prismshell/boss_stack.go`) | `agent-docs/modify-general-design.md` §3.1a |
| Modify of a prism whose section carries a displacement bound | `ErrUnsupported` via `prismrecord.Payload.RequireExactSection` | `agent-docs/modify-design.md` |
| Shell or junction fillet/chamfer of a revolve whose meridian carries a displacement bound — a revolve shell with a slanted cut, such as a cone's | `ErrUnsupported` via `revolveplan.RequireExactSection` | `agent-docs/surface-intersection-design.md` §7.2 |
| Variable-radius fillet, face-to-face fillet | No entry point exists | none |

## Booleans

| Gap | Today | Owner |
|---|---|---|
| Multi-body Union outside certified prism and straight-source axial Loft pair chains, equal-interval one-scene prisms, and specialized copied-source joins; the patterned spiral-tooth join refuses any copy made without `WithCopyRotation` about the X axis through the origin | `ErrUnsupported` (`boolean_many.go`); each chained pair may also refuse on its own geometry | `agent-docs/api-design.md` §8, `agent-docs/prism-boolean-design.md` §4.6, `agent-docs/general-boolean-design.md` §12, `agent-docs/revolve-loft-sector-design.md`, `agent-docs/prism-pattern-union-design.md` |
| Analytic readings of a faceted copy made with `WithCopyRotation`: face normals, `ToFace` stops, `MirrorFace`, point-loft cone trims, loft mirror cylinder joins, the other copied-source proofs and motion verification of the copy | `ErrUnsupported`: every face reads `Faceted` (`Face.NormalAt` refuses; `ToFace` and `MirrorFace` need a plane), and the copy is not a source for the structural trims that read the float placement. `VerifyMotion` and `VerifyLinkage` also refuse to re-place the copy or any `Copy`, `Transform` or `ScaledAt` descendant of it. `WithCopyPlacement` of a source with no preceding exact placement keeps them | `agent-docs/api-design.md` §8, `agent-docs/evaluator-design.md` §8, `agent-docs/mirror-pattern-design.md` §4.4 |
| Exact result outside the admitted analytic classes | Mesh boolean gives `Faceted` faces with a volume bound | `agent-docs/general-boolean-design.md` §3, `agent-docs/loft-point-design.md` "Coaxial finite cone trims" |
| Blind cuts with displaced levels, opposed-hole contact, or other crossing holes | Mesh path, possibly coplanar refusal. A7 builds one exact rectangular crossing | `agent-docs/stacked-prism-design.md` §7; `agent-docs/general-boolean-design.md` §3 A7 |
| Touching, coplanar or face-on-face contact on the mesh path | `BooleanError` code `BooleanUnsupportedContact` | `agent-docs/general-boolean-design.md` §2, `agent-docs/interference-design.md` |
| Varying-section or curved-wall axial Loft joined to a noncircular prism | The straight-source reduction declines; the mesh path may refuse coplanar contact | `agent-docs/general-boolean-design.md` §12, `agent-docs/loft-cylinder-boolean-design.md` |
| Curved surfaces tangent with facets that never meet | `ErrUnsupported` | `doc.go` support map |
| Held mesh operand coarser than the pair tolerance after a Boolean chain | `ErrUnsupported` except for fixed held primitives including twisted Sweeps and certified `LoftFaces` results, certified Revolve/sector joins with a centered spanning cylinder Cut, and authenticated mirrored-Loft pairs joined to a centered cylinder | `agent-docs/faceted-vertex-bounds-design.md` §5, `agent-docs/loft-face-sections-design.md`, `agent-docs/revolve-loft-sector-design.md`, `agent-docs/loft-mirror-boolean-design.md`, `agent-docs/sweep-design.md` §17.4 |
| Sheet body as a boolean operand | Refused permanently; use `Split` | `agent-docs/surface-design.md` Table X |

## Free-form curves

| Gap | Today | Owner |
|---|---|---|
| Tier B (conic, ellipse) or Tier C (unequal-weight NURBS) segment in any build | `ErrUnsupported` (R10) until spline P9 | `agent-docs/spline-design.md` §8 Table C |
| Tier A chain with no single curvature sign | `ErrUnsupported` (R19) | `agent-docs/spline-design.md` §6.5 |
| Modify ops over free-form walls | Refused per R3–R5, except the analytic-corner slice | `agent-docs/spline-design.md` §4.1 |

## Sheets and surfaces

| Gap | Today | Owner |
|---|---|---|
| `Thicken` of other composite or chain sweeps, loft, stitched, `PatchEdges` or `Unstitch` sheets | `ErrUnsupported` (R24); cardinal composite and exact-axis one-arc chain sweeps build, as do one-span profile sweeps | `agent-docs/surface-design.md` §16.8 |
| `PatchEdges` over a rim with a bounded vertex (a cut junction, a trimmed line end, a circle seam whose centre plus radius rounds) that no straight prism build stamped with one level token, such as a `Patch` sheet's rim | `ErrUnsupported` (R6) | `agent-docs/surface-design.md` §5.2 |
| Tessellation of `PatchEdges` with a curved face or edge outside a single-circle prism sheet filled in one call, or with a nonzero face-normal bound | `ErrUnsupported`: no shared chording for that face or edge | `agent-docs/surface-design.md` §10; `agent-docs/tessellation-design.md` §2 |
| `Stitch` closing a revolve sheet whose wall tags are not exactly their records: a tilted or round-anchored axis, a near-parallel or near-perpendicular side, a snapped centre | `ErrUnsupported` (R8, `stitchrecord`'s `fluxTagsDenoted`) | `agent-docs/surface-design.md` §6.4 |
| `Extend` of a partially revolved ribbon | `ErrUnsupported` (RS14) | `agent-docs/surface-intersection-design.md` §2.2 |
| `Trim`/`Extend`/`Split` over a pair sharing no generator, except a point loft split by a finite world-XY line chain `Extrude` sheet | Refused outside the bounded point-loft split class | `agent-docs/surface-intersection-design.md` §4, `agent-docs/loft-point-design.md` "Planar split of a point loft" |
| `Face.Centroid` outside an authenticated point-loft far cap or planar Split cap | `ErrUnsupported`; only complete caps carry a bounded area-centroid certificate | `agent-docs/loft-point-design.md` "Planar split of a point loft" |
| `Body.ExtremeFacesByCentroid` outside a complete point-loft planar Split slab, its rigid copy, or its positive uniform image | `ErrUnsupported`; this source class has a cone bound for every side-face centroid | `agent-docs/loft-point-design.md` "Planar split of a point loft" |
| `ScaledAt` of other bodies, including partial and sheet Revolves and non-Boolean faceted results | `ErrUnsupported`; no similarity certificate | `agent-docs/loft-point-design.md` "Anchored uniform scale of a Split slab"; `agent-docs/evaluator-design.md` "Positive uniform scale of a prism, stack, full Revolve, or faceted Boolean" |
| Tolerant stitch, Ruled, Boundary Fill | Refused permanently | `agent-docs/surface-design.md` §1.3 |
| Reverse Normal | Named, staged for no increment | `agent-docs/surface-design.md` §1.4 |

## Data exchange

| Gap | Today | Owner |
|---|---|---|
| Import of STEP, IGES, STL, OBJ or 3MF | No import entry point exists; mesh import is a v1 non-goal | `agent-docs/api-design.md` §13 |
| Analytic STEP for cones, spheres, horn tori, free-form walls | Whole body falls back to faceted STEP; planes, cylinders and tori with circle, arc, line and ellipse edges write analytic (`export/step_analytic.go`, `export/step_analytic_torus.go`) | `agent-docs/step-export-design.md` |
| STEP of a body with a cavity: a `WithNoOpenings` closed shell, a full revolve of a cavity meridian or a holed meridian | `ErrUnsupported`: "only a single non-void shell is supported" (`export/step.go`) | `agent-docs/step-export-design.md` |

## Queries

| Gap | Today | Owner |
|---|---|---|
| Point containment near an approximate boundary or on a body without a verified occupied-volume mesh | `Body.LocatePoint` returns `PointUndecided` or the mesh's `ErrUnsupported`; certified interior, exterior and exact-boundary points build | `agent-docs/point-containment-design.md` |
| Closest point from a point to a body or face | `Body.DistanceToPoint` measures a solid and `Face.DistanceToPoint` measures a trimmed face; neither returns a nearest point | `agent-docs/point-containment-design.md` §§4–5, `agent-docs/clearance-design.md` |
| Planar cross-section outside certified prism, BRep, direct Loft, independent rectangular or three-section homothetic `LoftSections`, sphere or cone classes | `Body.Section` returns `ErrUnsupported` | `agent-docs/section-design.md` |
| Surveys (undercut, wall, concave radius) of bodies other than prisms, revolves, cups, cap blends and one-sided draft bodies; the wall and concave-radius surveys of a draft body | `Verify` reports `Suspect`; a draft body's with `DiagUnsupportedSurveyPayload` | `agent-docs/verification-design.md`, `agent-docs/draft-design.md` DD8 |
| Wall survey of a sphere's revolve, solid or hollow (its meridian arcs meet the axis at both ends) | `Verify` reports `Suspect` (`DiagUndecidedWall`) | `agent-docs/verification-design.md` |
| Wall, undercut and concave-radius surveys and clearance on a revolve whose meridian carries a displacement bound | `Verify` reports `Suspect` | `agent-docs/surface-intersection-design.md` §7.2 |

## Model structure — v1 non-goals

`agent-docs/api-design.md` §13 owns this list. Lifting any of them changes that section first.

| Gap | Today |
|---|---|
| Assemblies (`Component`/`Occurrence`) | Bodies are placed explicitly with `Transform`/`Copy` |
| Feature tree, timeline, rollback, re-evaluation after a sketch edit | Bodies are immutable; a sketch edit rebuilds nothing |
| Sheet metal | No entry point exists |
| GUI or view state | No entry point exists |
| Fusion code generation | No entry point exists |

## Motion and dynamics

| Gap | Today | Owner |
|---|---|---|
| Joints, deformation, fracture in `dynamics` | Not in the program | `agent-docs/multibody-dynamics-design.md` §1 |
| Contact manifold for two non-convex bodies touching on more than one face | Relation only, `ContactNonConvex` | `agent-docs/multibody-dynamics-design.md` §1 |
| Sleeping or deactivation | Not in the program | `agent-docs/multibody-dynamics-design.md` §1 |

## Also consider

Features common CAD tools ship that decad lacks and no design doc plans or rejects. Writing a design doc for one
moves its row to the matching section above, with that doc as owner.

| Gap | Today |
|---|---|
| Direct edits: move, offset, delete or replace a face of a solid, press-pull | No entry point exists; `Offset` refuses every solid (`offset.go`, `agent-docs/surface-design.md` R37) |
| Per-axis (non-uniform) scaling of a body | No entry point exists; `ScaledAt` scales uniformly (`point_plane_scale.go`) and `Transform`/`Copy` take a rigid motion only (`document.go`) |
| Saving a `Document` to a file or loading one back | No entry point exists; a model is rebuilt by rerunning the Go program that built it |
| Text profiles from a font, for engraved or embossed lettering | No entry point exists |
| 2D drawings: projected views, silhouette and hidden lines, DXF output | No entry point exists |
| PMI or GD&T, as data or in STEP AP242 | No entry point exists; `export.STEP` writes AP214 geometry only |
| IGES, glTF or Parasolid export | `export` writes STL, OBJ, 3MF and STEP only (`agent-docs/api-design.md` §11) |
