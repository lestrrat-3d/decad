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
| Fillet, chamfer and shell take a boolean result only where it reads as a prism, (shell) as a prism cut by through tools with its caps or a run of straight walls removed, or (fillet, chamfer) at straight edges along an axis, or (chamfer) at complete loops, one rising inner edge, two adjacent outer edges, or every straight outer edge of the admitted cross-drilled box | Modify operations |
| No import of any file format | Data exchange |
| Booleans outside the admitted analytic classes fall to a faceted mesh result, and may refuse touching contact | Booleans |

## Feature operations

| Gap | Today | Owner |
|---|---|---|
| `ToFace` onto a face whose normal is not exactly parallel to the sweep, whose denoted plane is not an exact level set of the sweep, that carries a normal bound or denoted record, or that carries no cap record (a rotated stop body whose float normal is tilted, a revolve cap, a band patch, a faceted boolean face, a placed sheet copy, a sweep or coil cap, a chain patch); use a `Distance` extent | `ErrUnsupported` (`stops.go`, `resolveToFace`) | `docs/evaluator-design.md` §5 |
| Draft angle on a free-form wall, a circular corner that is not G1, or a surface result | `ErrUnsupported`, draft SD3/SD4/SD12 (`draft_build.go`) | `docs/draft-design.md` §14 |
| Boolean, mass and interference readings of a draft body whose circular wall joins a neighbour G1 but not exactly tangent, or whose section holds a trimmed segment | `ErrUnsupported`, "no proof of the volume" (`capblend_admit.go`); `Verify` reads the pair `Suspect`; the mesh exports | `docs/draft-design.md` §9.1 |
| `Draft` of a wall subset that moves one of two walls meeting at a circular corner, about a `NeutralFrame` or a non-cap face, or of a receiver that is not a straight prism | `ErrUnsupported`, draft SD4/SD20/SD23 (`draft.go`, `draft_build.go`) | `docs/draft-design.md` §10.2, §14 |
| Sweep twist outside centred convex whole-line or admitted symmetric Line/Arc profiles and paths; straight twist beyond one turn or the station work ceiling; line-arc twist beyond one radian | `ErrUnsupported` | `docs/sweep-design.md` §17 |
| Composite sweep mesh over other free-form profile kinds, meridian poles or changed station order | `ErrUnsupported` (`tessellate_sweep_composite.go`); line, circular and `FitSplineSeg` profiles mesh and enter booleans | `docs/sweep-design.md` Table D |
| Closed sweep path outside the exact two-arc full circle or exact-turn planar circular-section loops | `ErrUnsupported` (`sweep.go`); the admitted solids use certified full-Revolve or cyclic-span proofs | `docs/sweep-design.md` §§3.3–3.5 |
| Free-form sweep path | `Path` holds only `LineTo` and `ArcThrough` segments (`path.go`) | `docs/sweep-design.md` §2–§3 |
| Coil of a profile with free-form, elliptical or trimmed segments | `ErrUnsupported` (CS7) (`internal/coil/profile.go`) | `docs/helix-design.md` CS7 |
| `Sweep` over a chain along other composite paths; sheet mesh for a single arc with a curved or multi-segment source, or an axis endpoint; placement of its composite sheet | `ErrUnsupported`; one straight off-axis Sketch line along one spatial arc builds and meshes, as does the exact cardinal line/arc composite (R34) | `docs/sweep-design.md` §§15.8–15.9 |
| `Body.Section` outside its certified prism, stack-backed BRep, exact world-XY matching hole-free or circular-hole Loft, cardinal and oblique source-sphere, and exact cardinal right-cone cuts | `ErrUnsupported`; two-cap circle crossings, non-exact cone radii and other body classes still need a section proof | `docs/section-design.md` |
| General varying noncircular curved-section lofts, guide rails and centerlines | `LoftSections` admits three-section homothetic solids, three to thirty-two independently varying rectangular solids, selected curved classes, and four rounded-rectangle sections with cubic width; `LoftFaces` admits two to thirty-two ordered caps from one authenticated point-loft Split source; other shapes refuse | `docs/loft-sections-design.md`, `docs/loft-face-sections-design.md`, `docs/loft-design.md` §1 |
| Point-section loft of a holed profile | `LoftFromPoint` refuses holes because the common apex would have a non-manifold vertex link | `docs/loft-point-design.md` |
| Detached profile validation past 11585 reconstruction chords; authenticated Loft still stops above its 46340-chord cap | `ErrUnsupported`: R7 sketch reconstruction work budget | `docs/loft-gear-bounds-design.md` §7, `docs/spline-design.md` R7 |
| Loft with differing loop or segment counts | `ErrUnsupported` (S1/S2) | `docs/loft-design.md` Table S |
| Loft of mixed-kind or reversed pairs | Refused permanently | `docs/loft-design.md` §1 "Permanently out of scope" |
| `LoftChain` with curved segments or non-parallel planes | `ErrUnsupported` (R36, R35) | `docs/surface-design.md` §1.2 |
| `Extrude` over a chain with `WithTaper` (any angle) or `WithSurfaceResult()` | `ErrUnsupported` after the seam gates and before the extent; a chain extrude always returns a sheet and has no draft build | `docs/surface-design.md` §13.2 |
| Hole, rib, web, emboss features | No entry point exists | none |

## Modify operations

| Gap | Today | Owner |
|---|---|---|
| Other Sweep Fillet/Chamfer targets and asymmetric Sweep Chamfer | `ErrUnsupported` (S1/S3); straight solid lateral edges use the exact prism reduction, and one straight cap edge takes Route E for either blend | `docs/modify-design.md` R9 |
| Fillet/chamfer of most faceted boolean results, cap blend, or a draft body outside the positive-taper full-circle far-rim fillet and symmetric chamfer; loft chamfer outside the matching axial, zero-or-one-bore, both-outer-cap route | `ErrUnsupported`: modify S3, SX9 or SX10 (`fillet.go`, `chamfer.go`, `loft_chamfer.go`); a complete oblique cylindrical boss root and authenticated copied-prism root join admit bounded fillets, one isolated rectangular-prism Union outer edge admits Fillet and equal-setback Chamfer, and one perpendicular cylindrical bore mouth admits a bounded chamfer | `docs/modify-design.md` Table R, `docs/modify-reach-design.md` §10.1, `docs/draft-design.md` §10.4–§10.5, `docs/loft-design.md` §18, `docs/oblique-boss-fillet-design.md`, `docs/prism-pattern-union-design.md`, `docs/cross-cylinder-bore-chamfer-design.md` |
| Fillet of a loft with different sections, non-axial translation, holes, or a selected free-form corner | `ErrUnsupported` (`loft_fillet.go`); matching axial lofts accept analytic corners | `docs/loft-design.md` §17 |
| Fillet/chamfer of a brep or stacked result's single edges sharing a vertex with no planar loop after straight-wall restatement, an edge ending on a curved face outside the exact-radius corner-sphere route, or an oblique, split or level-displaced straight wall | `ErrUnsupported` (brep-modify SB5–SB9, modify-general SL1); a convex prism's single cap edge can use the bounded faceted cutter when it or its end wall is oblique | `docs/vertex-blend-design.md` §2, `docs/brep-modify-design.md` Table SB |
| Fillet or chamfer of independent straight brep edges whose endpoint-face corners disagree with the interior material wedge | `ErrUnsupported` (brep-modify SB8); one Fillet edge on a planar-face loop or one Chamfer edge on a rising inner loop takes a partial band, including a square boss root | `docs/brep-modify-design.md` EB8, `docs/loop-fillet-design.md` §3, `docs/modify-general-design.md` §4.3a |
| Fillet or chamfer of a loop whose neighbouring face is curved, oblique, split or lies on both sides of the loop's face; a complete-loop fillet whose line–arc or arc–arc offset miter folds or cannot be bounded | `ErrUnsupported` (modify-general SL2, loop-fillet SF1) (`brep_modify_loop.go`, `brep_loop_fillet.go`) | `docs/modify-general-design.md` §4.4, `docs/loop-fillet-design.md` §6 |
| Mesh boolean over a brep body whose complete-loop chamfer band holds an apex cone at a reflex corner (a hole or pocket mouth) | `ErrUnsupported` (the cap-loop chamfer's admission, `capBlendOccupiedVolumeAdmission`) (`boolean.go`, `brep_loop_band.go`) | `docs/modify-general-design.md` §5 Table DG, DG4 |
| Clearance of a brep body carrying complete-loop chamfer or fillet bands | pair reads `Suspect` unless its boxes decide it; no model (`clearance_geom.go`) | `docs/modify-general-design.md` §5 Table DG, DG6 |
| Shell of a draft body outside the inward one-cap tapered circular cup | `ErrUnsupported` (modify S3) | `docs/draft-design.md` §10.6 |
| Shell of a Sweep with a curved or multiple-span path, or nonzero twist outside the smooth symmetric rounded-profile inward top-cap case | `ErrUnsupported` (modify S3); one zero-twist straight profile span admits cap openings, side openings, and a hole-free closed cavity | `docs/modify-design.md` R9–R10, `docs/shell-twisted-sweep-design.md` |
| Shell outside the exact rectangular blind-pocket, rectangular or circular stacked-boss, coaxial two-level counterbore, circular through-wall, and other through-cut cases; outward or closed brep Shell; unadmitted openings and offsets | `ErrUnsupported` (SG1–SG7) | `docs/modify-general-design.md` §3.1a–§3.4, §6 |
| Shell side opening on a prism with an oblique removed end face recorded as several segments, a rim cut behind a removed arc whose recorded end lies off the circle its start defines, or a kept arc meeting a removed arc at an end corner whose cut is a float solve, under a kept cap; a revolve side run that leaves two wall pieces; a side opening meeting a kept face smoothly at an end, or whose rim runs past the removed walk's far end | `ErrUnsupported` (SO5 for a prism; SX8 for a revolve; SO1, SO2) (`shell_opening.go`, `shell_opening_brep.go`, `shell_revolve.go`, `internal/offset2d/opening.go`) | `docs/shell-opening-design.md` §5, §12, `docs/modify-reach-design.md` §9.3 |
| Shell of a revolve keeping an angular cap, with a holed meridian or one meeting the axis twice, or whose outward wall reaches the axis | `ErrUnsupported` (SX8) (`shell_revolve.go`) | `docs/modify-reach-design.md` §9.3 |
| Fillet/chamfer of a revolve cap edge outside one bounded 90° rectangular radial-rim blend, or an edge on the axis | `ErrUnsupported` (SX5) (`revolve_blend.go`, `revolve_cap_edge_blend.go`) | `docs/modify-reach-design.md` §7 |
| Fillet or chamfer of a cap edge outside the convex prism cutter or exact P8 front top edge | `ErrUnsupported` (SB7); admitted cases build as faceted results | `docs/modify-general-design.md` §4.3c |
| Chamfer of a prism's partial cap loop or a brep/stacked cap chain other than one straight rising inner edge or two adjacent straight descending outer edges, or cap and lateral edges together outside the admitted cross-drilled box's all-edge case | `ErrUnsupported` (SX4, SL1) | `docs/modify-reach-design.md` Table SX, `docs/modify-general-design.md` §4.3a–§4.3b |
| Placement of the dedicated all-edge chamfer result under an oblique transform, one moving its bore off y, or one rounding a required corner | `ErrUnsupported` (`allEdgeChamferPayload.placed`) | `docs/modify-general-design.md` §4.3b |
| Cap-loop chamfer at a corner where a circular wall meets a neighbour tangentially but runs back against it (a tangent cusp), or turns past the G1 tolerance too slightly to enclose | `ErrUnsupported` (SX14) (`capblend.go`) | `docs/modify-reach-design.md` Table SX |
| Asymmetric brep or stacked chamfer selection outside routes E/L | `ErrUnsupported` (SX16); E takes straight edges, L takes full planar loops | `docs/modify-reach-design.md` §6, `docs/modify-general-design.md` §4 |
| Tangent chain that branches or whose G1 continuity the oracle cannot decide | `ErrUnsupported` (SX2) (`tangent_chain.go`) | `docs/modify-reach-design.md` §5 |
| Closed shell (`WithNoOpenings`) outside a full revolve, a hole-free straight prism or a two-slab nested rectangular boss | `ErrUnsupported` (SX8/SX9; modify-general SG2) (`shell.go`) | `docs/modify-reach-design.md` §9, §14; `docs/modify-general-design.md` §3.1a |
| Reverse-order nested rectangular boss whose frame cannot flip exactly | `ErrUnsupported` (`shell_stacked_boss.go`) | `docs/modify-general-design.md` §3.1a |
| Modify of a prism whose section carries a displacement bound | `ErrUnsupported` via `prismrecord.Payload.RequireExactSection` | `docs/modify-design.md` |
| Shell or junction fillet/chamfer of a revolve whose meridian carries a displacement bound — a revolve shell with a slanted cut, such as a cone's | `ErrUnsupported` via `requireExactRevolveSection` | `docs/surface-intersection-design.md` §7.2 |
| Variable-radius fillet, face-to-face fillet | No entry point exists | none |

## Booleans

| Gap | Today | Owner |
|---|---|---|
| Multi-body Union outside certified prism pair chains, equal-interval one-scene prisms, and specialized copied-source joins; the patterned spiral-tooth join refuses any copy made without `WithCopyRotation` about the X axis through the origin | `ErrUnsupported` (`boolean_many.go`); each chained prism pair may also refuse on its own geometry | `docs/api-design.md` §8, `docs/prism-boolean-design.md` §4.6, `docs/revolve-loft-sector-design.md`, `docs/prism-pattern-union-design.md` |
| Analytic readings of a faceted copy made with `WithCopyRotation`: face normals, `ToFace` stops, `MirrorFace`, point-loft cone trims, loft mirror cylinder joins, the other copied-source proofs and motion verification of the copy | `ErrUnsupported`: every face reads `Faceted` (`Face.NormalAt` refuses; `ToFace` and `MirrorFace` need a plane), and the copy is not a source for the structural trims that read the float placement. `VerifyMotion` and `VerifyLinkage` also refuse to re-place the copy or any `Copy`, `Transform` or `ScaledAt` descendant of it. `WithCopyPlacement` of a source with no preceding exact placement keeps them | `docs/api-design.md` §8, `docs/evaluator-design.md` §8, `docs/mirror-pattern-design.md` §4.4 |
| Patterned spiral Union of a cone-tangent first tooth (a `LoftFacesToConeTangent` tooth trimmed by its cone) | `ErrUnsupported` (`spiral_combine_sector.go`); the first Union of the tooth with its blank is certified | `docs/loft-face-sections-design.md`, `docs/faceted-vertex-bounds-design.md` §3.7 |
| Exact result outside the admitted analytic classes | Mesh boolean gives `Faceted` faces with a volume bound | `docs/general-boolean-design.md` §3, `docs/loft-point-design.md` "Coaxial finite cone trims" |
| Blind cuts with displaced levels, opposed-hole contact, or other crossing holes | Mesh path, possibly coplanar refusal. A7 builds one exact rectangular crossing | `docs/stacked-prism-design.md` §7; `docs/general-boolean-design.md` §3 A7 |
| Touching, coplanar or face-on-face contact on the mesh path | `BooleanError` code `BooleanUnsupportedContact` | `docs/general-boolean-design.md` §2, `docs/interference-design.md` |
| Curved surfaces tangent with facets that never meet | `ErrUnsupported` | `doc.go` support map |
| Held mesh operand coarser than the pair tolerance after a Boolean chain | `ErrUnsupported` except for fixed held primitives including twisted Sweeps and certified `LoftFaces` results, certified Revolve/sector joins with a centered spanning cylinder Cut, and authenticated mirrored-Loft pairs joined to a centered cylinder | `docs/faceted-vertex-bounds-design.md` §5, `docs/loft-face-sections-design.md`, `docs/revolve-loft-sector-design.md`, `docs/loft-mirror-boolean-design.md`, `docs/sweep-design.md` §17.4 |
| Sheet body as a boolean operand | Refused permanently; use `Split` | `docs/surface-design.md` Table X |

## Free-form curves

| Gap | Today | Owner |
|---|---|---|
| Tier B (conic, ellipse) or Tier C (unequal-weight NURBS) segment in any build | `ErrUnsupported` (R10) until spline P9 | `docs/spline-design.md` §8 Table C |
| Tier A chain with no single curvature sign | `ErrUnsupported` (R19) | `docs/spline-design.md` §6.5 |
| Modify ops over free-form walls | Refused per R3–R5, except the analytic-corner slice | `docs/spline-design.md` §4.1 |

## Sheets and surfaces

| Gap | Today | Owner |
|---|---|---|
| `Thicken` of other composite or chain sweeps, loft, stitched, `PatchEdges` or `Unstitch` sheets | `ErrUnsupported` (R24); cardinal composite and exact-axis one-arc chain sweeps build, as do one-span profile sweeps | `docs/surface-design.md` §16.8 |
| `PatchEdges` over a rim with a bounded vertex (a cut junction, a trimmed line end, a circle seam whose centre plus radius rounds) that no straight prism build stamped with one level token, such as a `Patch` sheet's rim | `ErrUnsupported` (R6) | `docs/surface-design.md` §5.2 |
| Tessellation of `PatchEdges` with a curved face or edge outside a single-circle prism sheet filled in one call, or with a nonzero face-normal bound | `ErrUnsupported`: no shared chording for that face or edge | `docs/surface-design.md` §10; `docs/tessellation-design.md` §2 |
| `Stitch` closing a revolve sheet whose wall tags are not exactly their records: a tilted or round-anchored axis, a near-parallel or near-perpendicular side, a snapped centre | `ErrUnsupported` (R8, `stitchFluxTagsDenoted`) | `docs/surface-design.md` §6.4 |
| `Extend` of a partially revolved ribbon | `ErrUnsupported` (RS14) | `docs/surface-intersection-design.md` §2.2 |
| `Trim`/`Extend`/`Split` over a pair sharing no generator, except a point loft split by a finite world-XY line chain `Extrude` sheet | Refused outside the bounded point-loft split class | `docs/surface-intersection-design.md` §4, `docs/loft-point-design.md` "Planar split of a point loft" |
| `Face.Centroid` outside an authenticated point-loft far cap or planar Split cap | `ErrUnsupported`; only complete caps carry a bounded area-centroid certificate | `docs/loft-point-design.md` "Planar split of a point loft" |
| `Body.ExtremeFacesByCentroid` outside a complete point-loft planar Split slab, its rigid copy, or its positive uniform image | `ErrUnsupported`; this source class has a cone bound for every side-face centroid | `docs/loft-point-design.md` "Planar split of a point loft" |
| `ScaledAt` of other bodies, including partial and sheet Revolves and non-Boolean faceted results | `ErrUnsupported`; no similarity certificate | `docs/loft-point-design.md` "Anchored uniform scale of a Split slab"; `docs/evaluator-design.md` "Positive uniform scale of a prism, stack, full Revolve, or faceted Boolean" |
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
| Planar cross-section outside the certified prism, stack-backed BRep, exact world-XY matching hole-free or circular-hole Loft, cardinal and oblique sphere, and exact cardinal right-cone classes | `Body.Section` returns `ErrUnsupported` | `docs/section-design.md` |
| Surveys (undercut, wall, concave radius) of bodies other than prisms, revolves, cups, cap blends and one-sided draft bodies; the wall and concave-radius surveys of a draft body | `Verify` reports `Suspect`; a draft body's with `DiagUnsupportedSurveyPayload` | `docs/verification-design.md`, `docs/draft-design.md` DD8 |
| Wall survey of a sphere's revolve, solid or hollow (its meridian arcs meet the axis at both ends) | `Verify` reports `Suspect` (`DiagUndecidedWall`) | `docs/verification-design.md` |
| Wall, undercut and concave-radius surveys and clearance on a revolve whose meridian carries a displacement bound | `Verify` reports `Suspect` | `docs/surface-intersection-design.md` §7.2 |

## Model structure — v1 non-goals

`docs/api-design.md` §13 owns this list. Lifting any of them changes that section first.

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
| Joints, deformation, fracture in `dynamics` | Not in the program | `docs/multibody-dynamics-design.md` §1 |
| Contact manifold for two non-convex bodies touching on more than one face | Relation only, `ContactNonConvex` | `docs/multibody-dynamics-design.md` §1 |
| Sleeping or deactivation | Not in the program | `docs/multibody-dynamics-design.md` §1 |

## Also consider

Features common CAD tools ship that decad lacks and no design doc plans or rejects. Writing a design doc for one
moves its row to the matching section above, with that doc as owner.

| Gap | Today |
|---|---|
| Direct edits: move, offset, delete or replace a face of a solid, press-pull | No entry point exists; `Offset` refuses every solid (`offset.go`, `docs/surface-design.md` R37) |
| Scaling a body, uniform or per axis | No entry point exists; `Transform`/`Copy` take a rigid motion only (`document.go`) |
| Saving a `Document` to a file or loading one back | No entry point exists; a model is rebuilt by rerunning the Go program that built it |
| Text profiles from a font, for engraved or embossed lettering | No entry point exists |
| 2D drawings: projected views, silhouette and hidden lines, DXF output | No entry point exists |
| PMI or GD&T, as data or in STEP AP242 | No entry point exists; `export.STEP` writes AP214 geometry only |
| IGES, glTF or Parasolid export | `export` writes STL, OBJ, 3MF and STEP only (`docs/api-design.md` §11) |
