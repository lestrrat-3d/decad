# Loft Gear Bounds Design

Companion to `agent-docs/loft-design.md` ("loft §N"), which owns `Loft`'s pairing, construction, refusals and
consumers. This document owns four things loft does not state: the chorded volume residual as a per-cell wall
leg plus a skirt leg (§2), the centroid bound in shift form (§3), the chord target read from the section's own
feature size with bisection on the matched departure (§5), and the crossing audit as a box-overlap enumeration
over wall pairs plus one exact proof per cap (§6). It also fixes the shape of every resource ceiling a chorded loft meets (§7).
Loft stays the owner of each term it names (§5.2, §6, §8, §8.1, Table S); §12 lists the loft sections that
read from here.

Router (navigation, not authority):

| Your question | Section |
|---|---|
| why the earlier bounds read every gear `Suspect` | §1 |
| the volume residual: one wall leg per cell and a skirt leg, and the proof | §2 |
| the centroid bound as a shift of the held centroid | §3 |
| `Area`'s per-cell cap tube | §4 |
| the chord target, its constant, and bisection on the matched departure | §5 |
| the audit: box-overlap enumeration, wall-wall pairs, the per-cap family proof | §6 |
| the ceilings R7, S15, S8 and the reconstruction limit, and the sketch hand-off | §7 |
| required tests | §8 |
| where each piece of code lives | §9 |
| do not do this | §10 |
| open questions, each settled | §11 |
| which loft sections read from this document | §12 |

The reference case is a helical spur gear: module 2, 20° pressure angle, 10 mm face, 20° helix twist, 5 fit points
per Tier A flank, as one tooth or the full outline. Sizes and bounds below are the formulas and constants the code
and the tests in §8 show.

## 1. Measured problem

The earlier bounds, with the chord target `fraction × coordinate envelope` (loft §5.1's envelope rule), read
every gear case `Suspect` at the default `1e-3` tolerance. Three facts drove the design:

- The cap and seam legs dominated the volume bound and scale as `matchedDelta × radius × perimeter`, so
  they grow with the gear's radius rather than with its teeth.
- The centroid bound multiplied the volume allowance by the body's distance from the mass anchor
  (`coordUpper`), which for one tooth is the gear radius rather than the tooth's size.
- The crossing audit tested every pair of assembled triangles. The pairs that survive the box test are
  dominated by cap-vs-wall and cap-vs-cap pairs whose exact classification is never informative.

## 2. The volume residual: a per-cell wall leg, a skirt leg and the vertex sweep

**`Volume`'s chorded residual is `sweptLeg + wallLeg + skirtLeg`.** `Volume.Value` holds the ruled body (loft
§8.1: held triangles plus the exact twist correction), so the residual bounds `|V_true − V_ruled|`.

```text
sweptLeg = SweptVolumeAllow(delta, PerturbedAreaUpper(verts, tris, delta))      // exactly 0 at delta == 0
wallLeg  = Σ_charged cells ProductUpper(cellMatched_k, cellWallUpper_k)
skirtLeg = ProductUpper(ProductUpper(matchedDelta, delta), seamPerimeterUpper)   // exactly 0 at delta == 0
```

- A CHARGED cell is every cell except a FACETED one: a `LineSeg` pair's cell whose chord-to-curve departure is
  zero (`p.Faceted[j] && p.MatchedDelta[j] <= 0`). `ComputeLoftChordedAllow` walks the same cells for its other
  chorded terms. A circular or free-form cell is charged even at zero departure, since it stands for the
  bilinear patch the twist correction moves `Volume` onto.
- `cellMatched_k` is loft §5.2's `matchedDelta` row read at cell `k` (`ChordCellDeltaUpper(cell's chord-to-curve
  half, delta)`). `cellWallUpper_k` is `CellChordCurveAreaUpper` at that cell. `ComputeLoftChordedAllow` forms
  both per cell.
- `seamPerimeterUpper` sums, over EVERY wall cell, charged or not, and both of its sides, the larger of that
  side's `arcLenUpper_k` and its held chord's exact length (`CellSpanUpper`).
- `PerturbedAreaUpper` (`internal/proofbound`) runs over every held triangle, walls and caps alike. It reads
  each triangle's area as `RatSqrtUp` of the exact rational `|u × v|²/4` and its edge lengths through
  `RatSqrtUp` of their exact squared lengths, so no float cross product enters it.

**Derivation.** Write `w_S` for the winding function of a closed oriented surface `S`; the signed tetrahedron
sum of a closed triangulated surface is `∫ w_S`. If a family of closed surfaces is the image of a map
`H(t, x)` whose moving part is a patch `M`, the winding number about a point `p` changes only when the moving
part passes through `p`. So `|∫ w_end − ∫ w_start| ≤ ∫ N(p) dp`, where `N(p)` counts the `(t, x)` with
`H(t, x) = p`, and the area formula gives `∫ N = ∫ |det DH| ≤ ∫∫ |∂H/∂t| · |area element|`. Call `Π` a cap's
exact placed plane. Three steps, in this order, carry `S_0` (bilinear patches through the held corners on
charged cells, held triangle pairs on every other cell, held cap triangles) to the true body:

1. **Project each held cap onto `Π`.** `H(λ, x) = x − λ·((x − o)·n)·n` on every held cap triangle. The strip
   between the held seam and its projection (the SKIRT) is the trace of the cap's boundary, so the surface
   stays closed. Every held vertex lies within `delta` of its denoted station, which lies on `Π`, so every
   cap point lies within `delta` of `Π` and moves at speed at most `delta`; the map `I − λ·n·nᵀ` does not
   increase area. The step costs at most `delta · Σ_cap triangles held area`.
2. **Move every wall cell to the true wall at once**, `Φ_k(t, s, r) = (1 − t)·held_k(s, r) + t·true_k(s, r)`,
   carrying the skirt and the cap filling with it. Every cell runs over the same `t` and moves its corners by
   the same vertex interpolation, so cells that share a rung stay joined along it; moving one family of
   cells first would tear the surface there. The filling is the planar region of `Π` bounded by the
   projection of the moving seam. After step 1 the projected held triangles are that region (two 2-chains in
   a plane with the same boundary agree almost everywhere), and it stays inside `Π`, so its map has rank two
   and contributes nothing to `∫ |det|`.
   - A charged cell moves at speed at most `cellMatched_k` over a surface of area at most
     `CellChordCurveAreaUpper` at every `t` (`internal/proofbound/bounds.go`, its `eA·eB` argument): `wallLeg`.
   - A FACETED cell's held triangle pair IS the boundary loft §5 gives it, so `true_k` is the triangle pair
     through the denoted corners. Its points move by convex combinations of corner displacements, at most
     `delta`, and each triangle's area stays below its held area plus `PerturbedTriangleAreaAllow`: at most
     `delta · Σ (held area + allowance)` over those triangles.
   - The skirt: the held seam lies within `delta` of `Π` and the true seam lies in `Π`, and signed distance to
     a plane is affine, so the moving seam lies within `(1 − t)·delta ≤ delta` of `Π` and the skirt's width is
     at most `delta`. Its length is at most the moving seam's speed summed over EVERY seam cell, charged or
     not. That speed is a convex combination of the held chord's and the true curve's, so it is at most the
     larger of the held chord's exact length and `arcLenUpper_k`; a held chord joining two displaced stations
     can be up to `2·delta` longer than the true segment. The sum is `seamPerimeterUpper`. Every skirt point is
     a convex combination of a seam point and its orthogonal projection, which is 1-Lipschitz, so it moves at
     most `matchedDelta`: `skirtLeg`.
3. At `t = 1` the walls are the true walls, the seam is the true seam in `Π`, the skirt has zero width and the
   filling is the true cap region: the true body.

**`sweptLeg` pays for step 1 and for the faceted cells of step 2.** Their two costs together are at most
`delta · Σ (held area + PerturbedTriangleAreaAllow)` over disjoint triangle sets. `PerturbedAreaUpper` sums
exactly that over every held triangle, so `SweptVolumeAllow(delta, PerturbedAreaUpper)` dominates both. The
wall leg reads only charged cells, and the skirt leg bounds the strip's motion, not the cap's. So `sweptLeg`
is REQUIRED whenever `delta > 0`, chorded or not. At `delta == 0` every held vertex is its denoted station
(loft §5.2's `delta` row), the held caps lie in `Π`, the faceted cells never move, and `sweptLeg`, step 1 and
the skirt are exactly zero. No step assumes the held cap polygon is planar, the ruled wall embedded, or the
seam simple at an intermediate `t`.

**There is no cap leg or seam leg.** Loft §8.1 states them as the two residues of a by-parts split of the
wall's flux integral over an OPEN patch. The chain above moves a CLOSED surface at every step and has no such
residue: the cap's in-plane change is the filling's rank-two map, and the seam's departure from the cap plane
is the skirt.

**Per cell, never the maximum.** The build-wide `matchedDelta × wallAreaUpper` charges every cell at the worst
cell's departure. On the gear a LineSeg cell's departure is zero, an arc cell's is its own sagitta, and a
free-form cell's is its own matched bound, so the per-cell sum is below the maximum form.

Summing the skirt over every seam cell, the root `LineSeg`s included, leaves the leg at the coordinates' rounding
scale, since `delta` is.

The tessellation's `volSymDiff` (`agent-docs/tessellation-design.md` §2's `loftPayload` row) composes
`SweptVolumeAllow + wallLeg + twistVolumeUpper + skirtLeg`. The mesh holds the uncorrected triangles, so the
twist leg stays as a measure there.

## 3. The centroid bound in shift form

**`Centroid`'s allowance is one length, `epsV · R_c / clearance`, not a per-coordinate quotient about the
anchor.**

```text
epsV      = absSumUpper(SweptVolumeAllow(delta, PerturbedAreaUpper), wallLeg, skirtLeg)
clearance = roundDown(|vol| − volumeAllow)               // exact rational vol; S12 refuses at clearance <= 0
R_c       = absSumUpper(max_v sqrtUp(|v − c_f|²), radius3D(rounding), max(matchedDelta, delta))
shift     = divUpper(productUpper(epsV, R_c), clearance)
Bound     = absSumUpper(radius3D(rounding), shift)
```

- `c_f` is the published float centroid, `rounding` the largest per-coordinate rounding of the exact rational
  centroid into it, and `v` runs over every vertex the triangle set references (an exact dyadic squared
  distance and its outward square root, the route `distUpper` takes over rationals).
- The radius term is the larger of `matchedDelta` and `delta`, never their sum.
- `epsV` is §2's chain: it bounds `∫ |w_1 − w_0|` over the three steps §2 writes down, each with its charge, so
  the vertex sweep is REQUIRED in `epsV` whenever `delta > 0`, as it is in `Volume`.

**The clearance is computed over exact rationals.** `vol` is the exact corrected volume `Volume` rounds, and
`volumeAllow` is `Volume`'s own proven allowance (§2's `wallLeg + skirtLeg` beside the vertex sweep) without that
rounding; the difference rounds down once. Subtracting from the rounded `V_value` instead carries `V_value`'s
half-ulp into the result, and where `epsV` is most of `V` that half-ulp is many ulps of the small difference,
so no single outward step covers it. The clearance reads `Volume`'s proven enclosure of `V'`, so S12 stays
exactly the test `Volume` states.

**Derivation.** Let `w_0`, `w_1` be the winding functions of §2's `S_0` and `S_1`, `V' = ∫ w_1` the true volume
and `c` the exact centroid of the ruled body, so that `∫ (p − c) w_0 dp = 0`. Then
`c' − c = (1/V') ∫ (p − c)(w_1 − w_0) dp`, and with `|w_1 − w_0| ≤ N(p)` from §2,
`|c' − c| ≤ (1/V') · sup_{p ∈ image Φ} |p − c| · ∫ N`. Every point of the wall sweep lies within `cellMatched_k`
of a bilinear patch, every bilinear patch lies in the convex hull of its four held corners, every skirt point
within `delta` of a seam point, and every point of the vertex-displacement sweep (`SweptVolumeAllow`'s own
homotopy) within `delta` of a held triangle. So `sup |p − c| ≤ max_v |v − c| + max(matchedDelta, delta)`, and
`|c − c_f| ≤ radius3D(rounding)` moves the reference from the exact centroid to the published one.
`V' ≥ clearance` because `Volume`'s proven enclosure contains `V'`.

**`R_c` is the tooth's size, not the gear's.** An anchor-based form reads `SweptMomentAllow` and the
chorded-boundary moment residual at `coordUpper`, the body's inf-norm extent from the mass ANCHOR, then
multiplies the largest coordinate by `√3`. The anchor is `p0`'s plane origin, which for one tooth is the gear
centre. `R_c` is measured from the body's own centroid; §8's one-tooth centroid row asserts it.

The loft path does not use the anchor-based moment helpers. `proofbound.SweptMomentAllow` stays for the modify
designs that own it.

## 4. `Area`: the per-cell cap tube

```text
capAreaExcess = Σ_caps Σ_cells ( productUpper(2·cellMatched_k, arcLenUpper_k) + π⁺·cellMatched_k² )
```

The sum runs over the same cells `ComputeLoftChordedAllow` walks; `π⁺` is `nextafter(π, +Inf)`.
`sectionDisplacementArea`'s argument (`internal/stackedrecord/build.go`) is read per chord with that chord's own departure instead of the
build-wide maximum: a point of the symmetric difference between the held cap polygon and the denoted region
lies within the matched departure of SOME chord point, and a chord's neighbourhood is a rectangle of that
half-width plus two half-disks. The wall's ruled and station-shift legs are already per cell.

With §2 and §3 in place `Area` is the binding reading on the gear cases. `CellBilinearArea`'s integration
enclosure (`divisions = 4`) is the largest part of its bound; it does not shrink with the chord target and is left
alone (§11).

## 5. The chord target and bisection on the matched departure

**The target is a fraction of the section's own feature size `A / P`.**

```text
chordTarget = ChordFraction · min( |area(p0)| / perimeterUpper(p0), |area(p1)| / perimeterUpper(p1) )
ChordFraction = 2.5e-4
```

- `area(p)` is the record's own area reading, which `momentinput.Profile.FalsifyLoftRecordedArea` computes
  through `EvaluatorArea`.
- `perimeterUpper(p)` is the sum over the record's segments of `PerCellArcUpper(seg, walk, 1)`: the exact
  `circularLengthInterval` bracket for an arc, the walk's own `LengthUpper` otherwise.
- A target decides the vertex set. For a record of `LineSeg` and polynomial Tier A segments with recorded
  control points, both inputs are exact-rational computations rounded once, so the same record gives the same
  target on every platform; `TestLoftChordTargetReadsFeatureSize` checks one such record's target against
  `fraction · A/P`.
- A circular segment's area contribution is a float trig expression (`internal/momentregion`'s `AddCircular`),
  and a `FitSplineSeg`'s spans come from sketch's float interpolation solve, so a record holding either can
  read a target whose last bits differ between platforms.

**Why `A / P`.** Loft §5.1's envelope rule reads the coordinate envelope, which grows with the gear radius and
with the section's distance from the sketch origin. §2's volume residual is
`Σ cellMatched_k · cellWallUpper_k ≈ matched · P · h` against `V ≈ A · h`. With every
`cellMatched_k ≤ chordTarget` the Volume ratio is at most `ChordFraction · σ`, where `σ` is the wall's slant
factor (1.06 on the 20° helix), whatever the section's size or shape, and the one-tooth case gets the tooth's
target instead of the gear's. A per-segment target was considered and declined: the residual is a
perimeter-weighted sum of departures, so no allocation across segments lowers it for a given station count by
more than a constant.

**Where it is computed.** `Loft` stores the two record-area readings
`momentinput.Profile.FalsifyLoftRecordedArea` computed on the payload (`loftPayload.RecordArea`), and `Placed`
carries them over unchanged. `StationCapGate` computes the target once per evaluation from them and the walks
`ValidateLoftRecords` resolves, decides S15 against it, and returns it to the station generators. The target
is not stored: its perimeter half reads the walks, which only `loftrecord.Build` resolves, and recomputing it
from the stored areas is one pass over the segments, where recomputing the area integral is far more work.

**Bisect on `max(sagitta, matched)`.** `SagittaStationWalk.WalkCell` measures each side's
`SpanMatchedDeltaUpper` beside its sagitta and bisects while either exceeds the target; the accepted cell's
matched value is handed to the reader instead of recomputed. `SagittaUpper` stays the sagitta (loft §5.2's
`sectionDelta` and `Bounds.Bound` read it). This makes "every `cellMatched_k ≤ target`" a property of the walk
rather than of the input: without it a free-form cell's matched departure can exceed the target and is
unbounded in general (`TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses`).

**The constant.** `ChordFraction = 2.5e-4`. The Volume ratio is at most `ChordFraction · σ` (above), and the
Area bound is the binding reading on the gear cases, so a coarser fraction leaves `Area` less margin below the
default tolerance. §8's tooth and gear rows assert `Sound` and a ratio below `1e-3` at this constant.

`TestLoftArcWedgeVerifiesSound` pins loft §14's arc wedge (`m = 75`) under this constant.

## 6. The audit: sweep enumeration, wall-wall pairs, one proof per cap

Loft §6's audit keeps its verdict rule, its per-pair classification, its two reject-only tiers and its three
certificates. This section states how pairs are ENUMERATED, which pairs are tested, and how S8 counts.

**Enumeration emits every pair whose boxes overlap, through the cheapest of three box scans.** The scans are
the sweep over the sorted boxes, a uniform grid and a balanced box tree (`loft_audit_grid.go`,
`loft_audit_tree.go`; `loft_audit_group_tree.go` serves audits with proven projected patches). `newPairScan`
counts each scan's work before any runs and takes the lowest; a tie keeps the earlier scan. All three report
exactly the box-overlap pairs, each once, so the candidate set, its test order and every verdict are the same
whichever runs. The sweep sorts the triangle indices by their box's lower bound on the axis of largest total
extent (ties by index), walks that order, and emits every pair whose boxes overlap on all three axes. Boxes
are closed intervals: boxes that touch on a face, edge or corner overlap, as
`BoxesOverlap`'s `<=` decides. A pair of disjoint boxes shares no point (loft §6's box argument), so the
enumerated set contains every touching pair, and a pair sharing a vertex has overlapping boxes, so every pair
Table C expects to touch is enumerated. Candidates are sorted lexicographically before testing, so the first
refused pair is the same `(i, j)` the zero-shortcut reference loop reports. Every scan is float comparisons
only, and the sort is deterministic.

**Only wall-wall candidates are tested pairwise.** Each is decided exactly (box tier already passed;
`LoftPlaneSeparated`; certificates; exact classification). The two caps are decided by one proof each, over
exact signs alone:

Let `C` be one cap's triangles, `L` its polygon loops as vertex-index cycles (the cap's `vIdx` or `wIdx`), `O`
the other cap's vertices together with `LoftAuditStructure.Interior` (the rings of a banded loft's interior
sections; nil for a two-ring loft), and `Π` the exact plane of `C`'s first triangle with normal `n`.

- **(s)** the index structure is the one assembly builds: loops of at least three vertices, no index repeated
  across `L` and `O`, no vertex of `C` in `O` and no vertex of the other cap in `L`, every wall triangle with
  a vertex in `O` and its others in `L` forming one vertex or one loop edge, and every loop edge an edge of
  some wall triangle;
- **(a)** every vertex of `L`, and every vertex of every triangle of `C`, has exact sign 0 against `Π`;
- **(b)** every vertex of `O`, and every vertex of every triangle of the other cap, has a nonzero sign against
  `Π`, all the same;
- **(c)** every triangle of `C` has `n · ((B − A) × (C − A)) > 0`;
- **(d)** the directed-edge multiset of `C` nets to exactly the edges of `L`, each loop traversed in one
  consistent direction, every other edge netting to zero;
- **(e)** in the projection `projAxes(n)`, exactly one loop's oriented signed area (its exact shoelace sign
  times its (d) direction) has the triangles' orientation sign and every other loop's has the opposite sign.

Together with the wall-wall audit, which proves every chord of `L` meets every other wall edge only as Table C
expects (so the loops of `L` are simple and pairwise disjoint), these give:

- (c)+(d) make `Σ_T 1_T` equal the winding number `w_L` of the oriented boundary almost everywhere (a 2-chain
  and the region chain with the same boundary differ by a 2-cycle, which is zero in the plane).
- (e) with simple disjoint loops makes `w_L ∈ {0, 1}`, so the triangles of `C` are interior disjoint and cover
  the polygon exactly.
- (s) ties every loop edge to a wall triangle the wall-wall audit tests.
- A cap triangle therefore meets a polygon edge only in shared vertices or as that edge (a triangulation edge
  through a reflex vertex would put a neighbouring triangle outside the polygon, and a polygon vertex inside a
  polygon edge contradicts simplicity).
- A wall triangle `U` meets `Π` in exactly its vertices on `Π` ((b) puts its other vertices strictly on one
  side), which are its cap-side vertex or edge, a polygon vertex or edge by index. So `T ∩ U = T ∩ (U ∩ Π)` is
  exactly the contact their shared indices expect, for every cap triangle `T` and wall triangle `U`, and two
  triangles of `C` meet exactly in their shared edge or vertex.
- (b) also proves the two caps disjoint.

Any condition failing falls back to pairwise testing of that cap's pairs through the same candidate list;
nothing is admitted on a failed proof. A pair is left to the proofs when either of its triangles is in a proven
cap. When a wall-wall pair fails, the audit tests the decided pairs that precede it lexicographically before
refusing, so the refused pair is the reference path's.

**S8 counts candidates.** The ceiling `MaxFacetPairTestsPerCall` is `8_000_000` and is compared against the
number of enumerated candidates the pairwise pass will test (wall-wall, plus any cap family that fell back),
counted in a first box-scan pass before the second pass tests them. The counting pass steps the budget once per
scan unit and refuses once that count passes the same ceiling. The sweep, grid and tree scans bound work before
refusal by `O(F log² F + ceiling)`. `F` itself is refused past `8·8192 − 8` (`MaxLoftAuditTriangles`) before
any exact lift (§7). The budget steps once per tested pair.

**Entry points.** `LoftCrossingAuditStructured(budget, verts, tris, walls, capStartCount, loops0, loops1)` is
the loft's (`loftrecord.Build` calls it with the assembly's `Walls`, `CapStartCount`, `VIdx`, `WIdx`).
`LoftCrossingAuditBanded` is the same audit over a `LoftAuditStructure` that carries `Interior` rings
(`internal/loftmirror/union.go`). The generic `LoftCrossingAudit(budget, verts, tris)` serves every other caller
(for example `internal/sweepmitre/build.go`) with the box enumeration and pairwise testing of every candidate.
`LoftAuditShortcuts` has the fields `BroadPhase`, `Certificates`, `Sweep`, `CapProof` and `ExactInput`
(`ExactInput` disables the float-only filters). The zero value is the reference path (every pair of every
triangle through the exact classification) that every test compares verdicts against; the generic entry sets
`BroadPhase`, `Certificates` and `Sweep`, and the structured entry sets those three and `CapProof`.

The sweep alone does not remove the cost: every wall triangle has a vertex on each cap plane, every ear-clipped cap
sliver spans the section, and `LoftPlaneSeparated`'s float filter cannot certify a determinant that is exactly
zero, so those pairs reach the exact lift (`XptOf` under `OrientSign`) inside that tier. The cap proofs remove
the cap pairs from the pairwise pass.

## 7. Resource ceilings

Every ceiling has a hard constant and a shape that scales with the record. `P` and `C` are loft §5.1's
paired-segment and chorded-pair counts.

| Ceiling | Rule | Hard ceiling | Why this shape |
|---|---|---|---|
| S15 station cap `loftmesh.StationCap` | `StationCap(P) = min(max(512, 64·P), 8192)` (`StationCapFloor`, `StationsPerPair`, `StationCapCeiling`) | 8192 stations, `F ≤ 8·8192 − 8` | the floor and slope scale with `P`; S15 reserves one cell per pair and charges each curve's actual extra cells against the shared remainder, so a bore circle may use more cells than a short flank; the hard ceiling bounds the triangle count `NewLoftAuditData` lifts exactly |
| S8 pair ceiling | the smallest sweep, grid or tree scan count ≤ `8_000_000`, and enumerated candidates ≤ `8_000_000` (§6); `F ≤ 8·8192 − 8` before any exact lift | `8_000_000` pairs | the scan count is at least the candidate count, so the counting pass refuses after at most `8_000_000` scan units and the work before a refusal is `O(F log² F + ceiling)`; the triangle ceiling bounds the exact lifts, which the station cap does not bound for a build with no chorded pair (`StationCapGate` returns early there) |
| R7 `FreeformWorkLimit` for Loft's area, walks and stations (default `1 << 20` per record per operation) | Public `Loft` raises each record's counter before area integration to `max(1 << 20, 8192 · StationCap(P))` (`StationWorkLimit`); re-evaluations raise fresh counters before walks. `P` comes from the first authenticated record. | `1 << 20 + 8192 · 8192 = 2^26 + 2^20` | The full embedded 60-tooth area's first integral exceeds the default ceiling before the walk gate. The same counter then pays for walks and stations; saturated single charges still refuse. |
| reconstruction (default `1 << 28`, 11585 chords) | trusted Loft: `1 << 32`, 46340 chords (`LoftReconstructionWorkLimit`, `LoftReconstructionChordCeiling`); others unchanged | `1 << 32` | the charge is `2·chords²` (`LoftReconstructionChordCeiling`'s comment), so `1 << 32` admits 46340 chords where `1 << 28` admits 11585 |

The certified fit-spline fillet and centered-bore Cut each resolve the section for their own contact or
containment proof, then integrate its area and rebuild the loft on the same counter. They may use twice the
ordinary station-scaled R7 allowance (`loftrecord.Payload.RewriteWorkLimit`), at most `1 << 27`, after the original public Loft authenticated both
profiles. Ordinary Loft, other Cut paths, and detached records keep their existing ceilings.

The reconstruction ceiling is decad's model of sketch's quadratic arranger. A public Loft may raise only its
private per-profile counters after both Sketch profiles pass the seam. Its certified fillet, bore and
placement rewrites carry that provenance to their fresh counters. Standalone record validation and public
moment methods keep the `1 << 28` ceiling. The larger cap still charges the global chord count before any
actual reconstruction and refuses above its hard limit. decad needs
two things from sketch to lower this cost: a sub-quadratic arrangement, and a chord count that scales with a
curve's size rather than `16 · controls` with a floor of 64. The gear is the reference case. sketch owns both.

Each ceiling refuses an adversarial record in bounded time. One free-form pair among 256 paired segments,
handed a chord target no station allowance can meet, refuses S15 under the shared cap; the shared allocation
has no cost measurement. The reconstruction charge refuses a record past its ceiling before sketch arranges
anything.

## 8. Required tests

Each bound test deletes its leg or gate and records the red run in the test
(`decad-fixtures-must-be-shown-to-fail`). No bound literal is pinned; margins are asserted as ratios against the
tolerance. Each test names its package. A row whose test is "none" states an assertion no current test makes.

| Test | Asserts | Shown to fail by |
|---|---|---|
| `TestLoftVolumeResidualIsPerCellWallLegPlusSkirt` (`internal/loftmesh`) | on four wall cells with departures 0.1, 0.01, 0.001 and 0 (the last one faceted), at `delta` 0 and 0.01: `WallLeg` is the per-cell sum and below the build-wide maximum form; `SkirtLeg` runs along every seam cell, the faceted one included, and is exactly 0 at `delta == 0` and positive above it; `Volume.Bound` equals the rounding, the vertex sweep, the per-cell sum and the skirt composed in order | dropping the skirt from `VolumeAllow`; charging the build-wide maximum instead of the per-cell sum |
| none | the exact ring volume lies inside `[Value − Bound, Value + Bound]` at station counts from 4 to 64 per quarter arc, twisted and untwisted; `TestLoftArcWedgeVerifiesSound` checks one untwisted quarter arc at the shipped target only | deleting the wall leg |
| none | a placed thin slab with one tiny chorded arc: the exact volume lies inside `Volume`'s bound and outside the bound without `sweptLeg`; `TestLoftVolumeResidualIsPerCellWallLegPlusSkirt` checks only that `Volume` adds the sweep at `delta > 0` | deleting `SweptVolumeAllow` from the chorded `Volume` composition |
| none | on a section of two circular arcs (no `LineSeg`, so every wall is ruled; a ring cannot show the shift red, its chorded centroid equals the true one by symmetry), untwisted, twisted and placed at three bulge offsets: the exact centroid lies inside `Bound` of the published one; `R_c` reaches every densely sampled true boundary point; the bound is below the anchor form's on every row | deleting `R_c`'s `max(matchedDelta, delta)` term (6 of 7 rows); deleting the shift (every row) |
| none | one tooth at z = 8, 20, 40: `R_c` within the tooth's own diameter; the Centroid ratio below the anchor form's and below `2.5e-4 · 4` on every row | the anchor-based form, read over §2's volume legs, exceeds `2.5e-4 · 4` at z40 |
| `TestCentroidClearanceIsExact` (`internal/loftmesh`) | `CentroidClearance` is the largest float at or below the exact `|vol| − volumeAllow` where that allowance sits inside `V_value`'s half-ulp | the rounded-volume form overstates it 1.7x |
| `TestLoftAreaCapTubeIsPerCell` (`internal/loftmesh`) | `CapAreaExcess` is the per-cell tube; it matches the maximum form to rounding when every cell's departure is equal and is below it when the departures are 0.1, 0.01, 0.001 and 0.05 | charging the maximum form |
| `TestLoftChordTargetReadsFeatureSize` (`internal/loftmesh`) | an exact `LineSeg`/`NURBSSeg` record (the region under a parabola, area 16/3) lofted to a copy of itself, and the same record translated 40 mm: the record's own area is 16/3, the perimeter bound encloses the true boundary length and lies within 1e-3 of it, the target is `fraction · A/P` to two ulps and does not change under the translation; a zero area refuses | reading the envelope rule |
| `TestLoftFreeformWalkBisectsOnMatched` (`internal/loftmesh`) | on the zigzag span of `TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses` (`internal/freeform`) paired with itself, the walk bisects although the sagitta is 0, and every accepted cell's matched value is at or below the target | bisecting on the sagitta alone |
| `TestLoftArcWedgeVerifiesSound` (`apitest`) | the §14 wedge reads `Sound`; `Volume`, `Area` and `Centroid` enclose the closed-form wedge; `Volume` and `Area` clear the default tolerance by more than 2x | — |
| `TestLoftSweepEnumeratesEveryTouchingPair` (`internal/loftmesh`) | on a holed square prism, a ring frustum, a crossing pair and a pair touching at one corner: the candidate list is exactly the box-overlap set, in strict lexicographic order, and holds every pair sharing a vertex; the audit's refused pair is the zero-shortcut reference's | dropping the three-axis box test from the sweep |
| `TestLoftCapProofAgreesWithReference` (`internal/loftmesh`) | on every fixture of `TestLoftCapProofRefusesOverlappingTriangulation`, the structured audit's verdict and refusal text equal the zero-shortcut reference's | deleting condition (d); deleting (e) |
| `TestLoftCapProofRefusesOverlappingTriangulation` (`internal/loftmesh`) | on a holed square prism, the first cap fails (e) when the hole is covered by its own two triangles as well as the outer square's (a positively oriented hole), (d) with a second copy of one triangle, (c) with one triangle flipped and (a) with a loop vertex lifted off the plane; the second cap's proof still holds, and the failed cap's pairs join the pairwise pass | — |
| none | one tooth at z = 8, 20 and 40: `Sound`, every ratio below `1e-3`, station count below `StationCap(P)`, and `Volume`, `Centroid` and `Area` each enclosing a dense-sample reference (ruled patches over 512 samples per span or arc, the loft's own triangle pair over each `LineSeg`); `TestLoftHelicalToothClearsDefaultTolerance` (`apitest`) checks only a four-edge arc-and-line tooth: `Sound`, and `Area` within `1e-3` | deleting the wall leg from `Volume` |
| none | full gears z = 8, 20 and 40: the tooth row's assertions | restoring any one of the shipped ceilings; 32 stations per paired segment; deleting the wall leg from `Volume` |
| `TestLoftStationCapScalesWithPairs` (`internal/loftmesh`) | `StationCap(P)` at `P` = 3, 6, 8, 9, 48, 120, 127, 128, 240, 10_000, and the shared extra-cell budget S15 reads from it | — |
| none | on every gear case, the counter has `StationWorkLimit(0,P)` before the area falsifier, keeps it through the walk gate, and walks plus stations charge below 8192 per station | — |
| `TestLoftStationWalkRefusesAtTheRaisedWorkLimit` (`internal/loftmesh`) | on the parabola record of `TestLoftChordTargetReadsFeatureSize`, a station walk left half its cost below the raised ceiling runs past `1 << 20`, refuses R7 at the raised ceiling and names it | removing the raise |
| `TestFreeformWorkRaisedLimitBinds` (`internal/freeform`) | `RaiseLimit` never lowers the ceiling, charges run to the raised ceiling and refuse one unit past it, and a saturated charge refuses under any ceiling | removing `Step`'s saturation test |
| `TestLoftCertifiedRewriteWorkLimitKeepsDefault` (root package) | an authenticated payload's rewrite limit is twice `StationWorkLimit(0, P)` and at most `1 << 27`; an untrusted payload keeps the ordinary limit; a counter at the ordinary limit refuses the next unit while the raised one admits it | — |
| `TestLoftAuditCandidateCeilingRefuses` (`internal/loftmesh`) | the structured audit of the holed square prism refuses S8 at a ceiling one scan unit below its scan count, before any pair test, and admits at the count | removing the scan-count refusal |

Root-package and `apitest/` tests land in their shard manifests (`agent-docs/testing.md`, "Shard manifests").

## 9. PR split

| Area | Files and functions |
|---|---|
| volume residual and area tube | `internal/loftmesh/loft_chord_allow.go`: `ComputeLoftChordedAllow` accumulates `WallLeg`, `CapAreaExcess` per cell and `SkirtLeg` over every seam cell; `internal/loftmesh/mass_accumulator.go` `Volume`/`Area`, and `Centroid`'s `epsV` reads the same chain; `internal/loftmesh/mass_proof.go` `MeshProofOf` (`VolSymDiff`); `internal/proofbound/bounds.go` `PerturbedAreaUpper` (exact) |
| centroid shift form | `internal/loftmesh/mass_accumulator.go` `Centroid`, `CentroidMeasureAllow`, `CentroidRadius`, `CentroidClearance`, `VolumeAllow` |
| audit | `internal/loftmesh/loft_audit_sweep.go` (`LoftCrossingAuditStructured`, `sweepCandidates`), `loft_cap_proof.go` `CapFamilyProof`, `loft_audit.go` `LoftAuditShortcuts.Sweep`/`.CapProof`; `internal/loftrecord/build.go` calls the structured entry; `internal/sweepmitre/build.go` uses the generic entry; `internal/proofbound/budget.go` `MaxFacetPairTestsPerCall` and the comment on what S8 counts |
| chord target and matched bisection | `internal/loftmesh/station_gate.go`: `ChordTarget(area0, perim0, area1, perim1)`, `FeatureSize`, `ChordFraction`, `StationCapGate`; `internal/loftrecord/payload.go` `RecordArea`; `internal/freeform/spline_stations.go` `WalkCell` measures and forwards the matched value, `StationCellReader.AcceptCell` takes it |
| ceilings | `internal/loftmesh/record_stations.go` `StationCap(P)`, `StationExtraBudget` and `StationWorkLimit`; `internal/freeform/work_budget.go` `FreeformWork.Limit`, `RaiseLimit`, `Step` reads it, `ReconstructionWorkLimit = 1 << 28`, `ReconstructionChordCeiling = 11585`; `internal/loftmesh/pairing_gates.go` `ValidateLoftRecords` raises the limit before it resolves a walk |

## 10. Do not do this

- Do not keep a cap or seam leg "for safety" in `Volume`: §2's argument covers the closed surface, and a
  redundant leg made the gear `Suspect`.
- Do not drop `SweptVolumeAllow(delta, PerturbedAreaUpper)` from `Volume`, `Centroid`'s `epsV` or `volSymDiff`
  whenever `delta > 0`, chorded or not. It is the only charge for §2's step 1 and for the faceted cells'
  motion in step 2.
- Do not compute `PerturbedAreaUpper` from a float cross product or a float edge length: each triangle's area
  and edge lengths go through the exact rational cross product and `RatSqrtUp` (§2).
- Do not sum `seamPerimeterUpper` over charged cells alone: the skirt runs along every seam cell, and a
  faceted cell's seam moves by `delta` too.
- Do not read `sectionDelta` where `cellMatched_k` is owed, and do not substitute the build-wide
  `matchedDelta` for the per-cell value in `wallLeg` or the cap tube.
- Do not bisect on the sagitta alone and rely on the sagitta-to-matched ratio; the ratio is unbounded on a general
  span.
- Do not recompute the area integral inside `loftrecord.Build` for the target: carry the area
  `Profile.FalsifyLoftRecordedArea` computed on the payload.
- Do not derive the target from the sketch's claimed area; the record's own exact integral is the
  deterministic input.
- Do not make the target a caller option or a function of a published measurement (loft §5.1, §14).
- Do not admit a cap on a failed family condition, and do not skip the wall-wall pass when the cap proofs
  pass: the proofs depend on it for loop simplicity.
- Do not run the sweep without the lexicographic sort of candidates; the refused pair must be the one the
  reference path reports.
- Do not replace `ReconstructionWorkLimit` with a time limit or a context; the public record methods have none.
- Do not raise `FreeformWorkLimit` globally; the raise is the loft operation's, sized by its own station cap.
- Do not change `CellBilinearArea`'s partition without a separate change (§11).
- Do not pin any bound literal; assert ratios and enclosure.

## 11. Open questions, settled

- **Per-cell or per-segment target?** One target per build (§5): a per-segment allocation changes the residual
  by at most a constant.
- **Bisect on matched?** Yes (§5). It costs more stations than the sagitta alone.
- **`ChordFraction`.** `2.5e-4` (§5).
- **Should `Area`'s `CellBilinearArea` enclosure be deepened now that `Area` binds?** No. It is a fixed `4 × 4`
  partition independent of the target. Deepening it is a separate change with its own cost, the next lever if
  a section reads `Suspect` on `Area` alone.
- **Is the reconstruction ceiling decad's or sketch's?** Both (§7): decad raises its model to `1 << 28`;
  sketch owns the arranger's cost and receives the hand-off.
- **Where does the loft raise its records' work ceiling?** Before `ValidateLoftRecords` resolves a walk, with
  `P` read from the first record's segment counts (§7). The walks' arc-length brackets alone pass `1 << 20` on
  every full gear outline, so a raise after the cap gate comes too late.
- **How many stations per paired segment?** 64 (§7). S15 charges actual cells against the shared cap; each build
  still stops at 8192 total stations.
- **Keep the `(sectionDelta, areaUpper)` two-leg warning in loft §8?** Yes. It warns against dropping the TWIST
  leg from a bound over held triangles, which §2 keeps for the tessellation's `volSymDiff`.

## 12. Loft sections each PR edits

Loft owns the terms; these loft sections read from this document.

| Topic here | `agent-docs/loft-design.md` sections |
|---|---|
| §2 volume residual, §4 area tube | §5.2 (`wallLeg`, `skirtLeg`, `capAreaAllow` rows), §8 `Volume` paragraph, §8.1 (wall, twist, skirt rows and the telescoping paragraph), Table S S14, §15's tessellation bullet |
| §3 centroid shift | §8 `Centroid` paragraph, Table S S12 (`epsV`, `R_c`) |
| §6 audit | §6 (enumeration, the cap proof, the work budget), Table S S8 |
| §5 chord target | §5.1 "The chord target and its constant" and the free-form arm's bisection bullet, §14's calibration paragraphs |
| §7 ceilings | §5.1 station-cap and work-budget paragraphs, Table S S15, §12 increment table, `agent-docs/spline-design.md` R7 row, `agent-docs/missing-features.md` gear row, `doc.go` |
