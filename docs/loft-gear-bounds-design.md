# Loft Gear Bounds Design

Companion to `docs/loft-design.md` ("loft §N"), which stays the owner of `Loft`'s
pairing, construction, refusals and consumers. This document owns four things
loft does not yet state: the chorded volume residual as a single per-cell wall
leg plus a skirt leg (§2), the centroid bound in shift form (§3), the chord
target read from the section's own feature size with bisection on the matched
departure (§5), and the crossing audit as a sweep over wall pairs plus one
exact proof per cap (§6). It also fixes the shape of every resource ceiling a
chorded loft meets (§7). Each increment in §9 lands with the loft §5.2, §6, §8
and §8.1 edits §12 lists, so loft stays the single owner of each term once the
increment ships; until then the rows here are the contract and loft's are the
shipped state.

Router (navigation, not authority):

| Your question | Section |
|---|---|
| what the gear probe measures today, and why every reading is `Suspect` | §1 |
| the volume residual: one wall leg per cell and a skirt leg, and the proof | §2 |
| the centroid bound as a shift of the held centroid | §3 |
| `Area`'s per-cell cap tube | §4 |
| the chord target, its constant, and bisection on the matched departure | §5 |
| the audit: sweep enumeration, wall-wall pairs, the per-cap family proof | §6 |
| the ceilings R7, S15, S8 and the reconstruction limit, and the sketch hand-off | §7 |
| required tests | §8 |
| the PR split | §9 |
| do not do this | §10 |
| open questions, each settled | §11 |
| which loft sections each PR edits | §12 |

Every number below comes from the probe `.tmp/zz_probe_gear_measure_internal_test.go.txt`
(repo root): helical spur gears, module 2, 20° pressure angle, 10 mm face, 20° helix
twist, 5 fit points per Tier A flank, one tooth (`toothZ`) or the full outline (`zZ`),
built with every ceiling lifted so the bounds themselves can be read. The prototype
that produced the "new" columns is `.tmp/loft-gear-bounds-proto.diff` with its logs in
`.tmp/loft-gear-bounds-logs/`.

## 1. Measured problem

With the shipped bounds and the shipped chord target (`loftChordFraction ×
coordinate envelope`) every gear case reads `Suspect` at the default `1e-3`
tolerance, and the volume bound's three largest legs scale with the gear's radius
rather than with its teeth:

| Case | Stations | Binding reading | Ratio | Volume legs: wall / cap / seam | Build |
|---|---|---|---|---|---|
| tooth8 | 64 | Centroid | 3.1e-3 | 0.083 / 0.060 / 0.073 | 0.15 s |
| tooth20 | 44 | Centroid | 2.3e-2 | 0.23 / 0.30 / 0.35 | 0.09 s |
| tooth40 | 33 | Centroid | 1.3e-1 | 0.45 / 1.1 / 1.2 | 0.07 s |
| z8 | 512 | Volume | 1.7e-3 | 1.15 / 0.83 / 1.01 | 2.1 s (audit 1.1 s) |
| z20 | 780 | Volume | 2.3e-3 | 7.5 / 9.6 / 11.3 | 3.4 s (audit 1.5 s) |
| z40 | 1200 | Volume | 4.5e-3 | 36 / 90 / 98 | 6.4 s (audit 3.1 s) |

Three facts drive the design. The cap and seam legs are 60–85 % of the volume bound
and scale as `matchedDelta × radius × perimeter`. The centroid bound multiplies the
volume allowance by the body's distance from the mass anchor (`coordUpper`), which
for one tooth is the gear radius rather than the tooth's size. The crossing audit
tests every pair of the assembled triangles, and the pairs that survive the box test
are dominated by cap-vs-wall and cap-vs-cap pairs whose exact classification is
never informative: at z40, 2.15 M of the 30.7 M pairs overlap in boxes, and 2.05 M of
those involve a cap triangle.

## 2. The volume residual: a per-cell wall leg, a skirt leg and the vertex sweep

**`Volume`'s chorded residual is `sweptLeg + wallLeg + skirtLeg`.** `Volume.Value`
already holds the ruled body (loft §8.1: held triangles plus the exact twist
correction), so the residual bounds `|V_true − V_ruled|`.

```text
sweptLeg = sweptVolumeAllow(delta, perturbedAreaUpper(verts, tris, delta))      // exactly 0 at delta == 0
wallLeg  = Σ_charged cells productUpper(cellMatched_k, cellWallUpper_k)
skirtLeg = productUpper(productUpper(matchedDelta, delta), seamPerimeterUpper)   // exactly 0 at delta == 0
```

A CHARGED cell is every cell except a FACETED one, a `LineSeg` pair's cell whose
chord-to-curve departure is zero (`p.Faceted[j] && p.MatchedDelta[j] <= 0`): the
cells `ComputeLoftChordedAllow` walks for its other chorded terms. A circular or
free-form cell is charged even at zero departure, since it stands for the
bilinear patch the twist correction moves `Volume` onto. `cellMatched_k` is loft §5.2's `matchedDelta` row read at cell `k`
(`chordCellDeltaUpper(cell's chord-to-curve half, delta)`), and `cellWallUpper_k`
is `cellChordCurveAreaUpper` at that cell — the same two numbers
`computeLoftChordedAllow` already forms per cell and today sums into
`wallAreaUpper` before multiplying by the build-wide maximum.
`seamPerimeterUpper` sums, over EVERY wall cell, charged or not, and both of its
sides, the larger of that side's `arcLenUpper_k` and its held chord's exact
length (`CellSpanUpper`). `perturbedAreaUpper` is `proofbound.PerturbedAreaUpper` over
every held triangle, walls and caps alike; it reads each triangle's area as
`RatSqrtUp` of the exact rational `|u × v|²/4` and its edge lengths through
`RatSqrtUp` of their exact squared lengths, so no float cross product enters it.

**Derivation.** Write `w_S` for the winding function of a closed oriented surface
`S`; the signed tetrahedron sum of a closed triangulated surface is `∫ w_S`. If a
family of closed surfaces is the image of a map `H(t, x)` whose moving part is a
patch `M`, the winding number about a point `p` changes only when the moving part
passes through `p`, so `|∫ w_end − ∫ w_start| ≤ ∫ N(p) dp`, where `N(p)` counts the
`(t, x)` with `H(t, x) = p`, and the area formula gives
`∫ N = ∫ |det DH| ≤ ∫∫ |∂H/∂t| · |area element|`. Call `Π` a cap's exact placed
plane. Three steps, in this order, carry `S_0` — bilinear patches through the
held corners on charged cells, held triangle pairs on every other cell, held cap
triangles — to the true body:

1. **Project each held cap onto `Π`.** `H(λ, x) = x − λ·((x − o)·n)·n` on every held
   cap triangle, and the strip between the held seam and its projection (the
   SKIRT) is the trace of the cap's boundary, so the surface stays closed. Every
   held vertex lies within `delta` of its denoted station, which lies on `Π`, so
   every cap point lies within `delta` of `Π` and moves at speed at most `delta`;
   the map `I − λ·n·nᵀ` does not increase area. The step costs at most
   `delta · Σ_cap triangles held area`.
2. **Move every wall cell to the true wall at once**, `Φ_k(t, s, r) = (1 −
   t)·held_k(s, r) + t·true_k(s, r)`, carrying the skirt and the cap filling with
   it. Every cell runs over the same `t` and moves its corners by the same
   vertex interpolation, so cells that share a rung stay joined along it;
   moving one family of cells first would tear the surface there. The filling is
   the planar region of `Π` bounded by the projection of the moving seam; after
   step 1 the projected held triangles are that region (two 2-chains in a plane
   with the same boundary agree almost everywhere), and it stays inside `Π`, so
   its map has rank two and contributes nothing to `∫ |det|`.
   - A charged cell moves at speed at most `cellMatched_k` over a surface of area
     at most `cellChordCurveAreaUpper` at every `t` (`internal/proofbound/bounds.go`,
     its `eA·eB` argument): `wallLeg`.
   - A FACETED cell is a cell whose held triangle pair IS the boundary loft §5
     gives it, so `true_k` is the triangle pair through the
     denoted corners. Its points move by convex combinations of corner
     displacements, at most `delta`, and each triangle's area stays below its held
     area plus `perturbedTriangleAreaAllow`: at most `delta · Σ (held area +
     allowance)` over those triangles.
   - The skirt: the held seam lies within `delta` of `Π` and the true seam lies in
     `Π`, and signed distance to a plane is affine, so the moving seam lies within
     `(1 − t)·delta ≤ delta` of `Π` and the skirt's width is at most `delta`. Its
     length is at most the moving seam's speed summed over EVERY seam cell,
     charged or not. That speed is a convex combination of the held chord's
     and the true curve's, so it is at most the larger of the held chord's exact
     length and `arcLenUpper_k`; a held chord joining two displaced stations
     can be up to `2·delta` longer than the true segment. The sum is
     `seamPerimeterUpper`. Every skirt point is a convex
     combination of a seam point and its orthogonal projection, which is
     1-Lipschitz, so it moves at most `matchedDelta`: `skirtLeg`.
3. At `t = 1` the walls are the true walls, the seam is the true seam in `Π`, the
   skirt has zero width and the filling is the true cap region: the true body.

**Steps 1 and the faceted cells of step 2 are what `sweptLeg` pays for.** Their
two costs together are at most `delta · Σ (held area + perturbedTriangleAreaAllow)`
over disjoint triangle sets, and `perturbedAreaUpper` sums exactly that over every
held triangle, so `sweptVolumeAllow(delta, perturbedAreaUpper)` dominates both.
Neither the wall leg nor the skirt leg reaches them: the wall leg reads only
charged cells, and the skirt leg bounds the strip's motion, not the cap's. So
`sweptLeg` is REQUIRED whenever `delta > 0`, chorded or not. At `delta == 0` every
held vertex is its denoted station (loft §5.2's `delta` row), the held caps lie in
`Π`, the faceted cells never move, and all three of `sweptLeg`, step 1 and the
skirt are exactly zero. No step assumes the held cap polygon is planar, the ruled
wall embedded, or the seam simple at an intermediate `t`.

**The cap leg and the seam leg are not spent.** Loft §8.1 states them as the two
residues of a by-parts split of the wall's flux integral over an OPEN patch; the
chain above moves a CLOSED surface at every step and so has no such residue. The
cap's in-plane change is the filling's rank-two map, and the seam's departure from
the cap plane is the skirt. `ChordedBoundaryVolumeAllow`,
`ChordedBoundaryVolumeResidualAllow`, `ChordedBoundarySeamAllow`,
`CapAreaVolumeAllow`, the cap `planeOffsetUpper` row and the `posUpper` row leave
with this increment; the loft was their only production caller.

**Per cell, never the maximum.** The build-wide `matchedDelta × wallAreaUpper`
charges every cell at the worst cell's departure. On the gear a LineSeg cell's
departure is zero, an arc cell's is its own sagitta, and a free-form cell's is its
own matched bound, so the per-cell sum reads 0.47–0.62 of the maximum form.

Measured on the probe (shipped target, ceilings lifted):

| Case | wall / cap / seam (today) | per-cell wallLeg | skirtLeg | Volume bound today → new |
|---|---|---|---|---|
| tooth8 | 0.083 / 0.060 / 0.073 | 0.051 | 4.6e-17 | 0.215 → 0.051 |
| z8 | 1.15 / 0.83 / 1.01 | 0.69 | 1.0e-15 | 3.0 → 0.69 |
| z20 | 7.5 / 9.6 / 11.3 | 4.3 | 2.3e-14 | 28.4 → 4.3 |
| z40 | 36.4 / 90.5 / 98.4 | 16.9 | 1.7e-13 | 225 → 16.9 |

The skirt column is the prototype's, which summed only charged cells' sides;
summing every seam cell adds the root `LineSeg`s and leaves the leg at the
coordinates' rounding scale, since `delta` is.

The tessellation's `volSymDiff` (`docs/tessellation-design.md` §2's `loftPayload`
row) composes `sweptVolumeAllow + wallLeg + twistVolumeUpper + skirtLeg`: the mesh
holds the uncorrected triangles, so the twist leg stays as a measure there, and the
cap and seam legs leave for the same reason they leave `Volume`.

## 3. The centroid bound in shift form

**`Centroid`'s allowance is one length, `epsV · R_c / clearance`, not a per-coordinate
quotient about the anchor.**

```text
epsV      = absSumUpper(sweptVolumeAllow(delta, perturbedAreaUpper), wallLeg, skirtLeg)
clearance = roundDown(|vol| − volumeAllow)               // exact rational vol; S12 refuses at clearance <= 0
R_c       = absSumUpper(max_v sqrtUp(|v − c_f|²), radius3D(rounding), max(matchedDelta, delta))
shift     = divUpper(productUpper(epsV, R_c), clearance)
Bound     = absSumUpper(radius3D(rounding), shift)
```

`c_f` is the published float centroid, `rounding` the largest per-coordinate
rounding of the exact rational centroid into it, and `v` runs over every
vertex the triangle set references (an exact dyadic squared distance and its
outward square root, the route `distUpper` takes over rationals). The radius
term is the larger of `matchedDelta` and `delta`, never their sum.

**The clearance is computed over exact rationals.** `vol` is the exact
corrected volume `Volume` rounds and `volumeAllow` is `Volume`'s own proven
allowance without that rounding; the difference rounds down once. Subtracting
from the rounded `V_value` instead carries `V_value`'s half-ulp into the
result, and where `epsV` is most of `V` that half-ulp is many ulps of the
small difference, so no single outward step covers it. `volumeAllow` is
the residual `Volume` composes, §2's `wallLeg + skirtLeg` beside the vertex
sweep: the clearance reads `Volume`'s proven enclosure of `V'`, so S12 stays exactly
the test `Volume` states.

**The measure `epsV` is §2's chain.** `epsV` bounds `∫ |w_1 − w_0|` over the
three steps §2 writes down, each with its charge, so the vertex sweep is
REQUIRED in `epsV` whenever `delta > 0`, as it is in `Volume`.

**Derivation.** Let `w_0`, `w_1` be the winding functions of §2's `S_0` and `S_1`,
`V' = ∫ w_1` the true volume and `c` the exact centroid of the ruled body, so that
`∫ (p − c) w_0 dp = 0`. Then `c' − c = (1/V') ∫ (p − c)(w_1 − w_0) dp`, and with
`|w_1 − w_0| ≤ N(p)` from §2, `|c' − c| ≤ (1/V') · sup_{p ∈ image Φ} |p − c| · ∫ N`.
Every point of the wall sweep lies within `cellMatched_k` of a bilinear patch, every
bilinear patch lies in the convex hull of its four held corners, every skirt point
within `delta` of a seam point, and every point of the vertex-displacement sweep
(`sweptVolumeAllow`'s own homotopy) within `delta` of a held triangle; so
`sup |p − c| ≤ max_v |v − c| + max(matchedDelta, delta)`, and `|c − c_f| ≤
radius3D(rounding)` moves the reference from the exact centroid to the published
one. `V' ≥ clearance` because `Volume`'s proven enclosure contains `V'`.

The old form reads `sweptMomentAllow` and `chordedBoundaryMomentResidualAllow` at
`coordUpper`, the body's inf-norm extent from the mass ANCHOR, then multiplies the
largest coordinate by `√3`; the anchor is `p0`'s plane origin, which for one tooth is
the gear centre. `R_c` is measured from the body's own centroid, so it is the tooth's
size, not the gear's. Measured (prototype target, new `epsV`):

| Case | old-form bound | shift-form bound | `R_c` |
|---|---|---|---|
| tooth8 | 7.1e-3 | 6.1e-4 | 6.1 |
| tooth40 | 9.1e-2 | 7.8e-4 | 6.6 |
| z8 | 6.9e-3 | 1.3e-3 | 11.2 |
| z40 | 3.3e-2 | 5.8e-3 | 42.3 |

`PlacedCentroidAllow`, `sweptMomentAllow` and `chordedBoundaryMomentResidualAllow`
leave the loft path; the two `internal/proofbound` helpers stay for the modify
designs that own them.

## 4. `Area`: the per-cell cap tube

`capAreaExcess` becomes the per-cell tube

```text
capAreaExcess = Σ_caps Σ_cells ( productUpper(2·cellMatched_k, arcLenUpper_k) + π⁺·cellMatched_k² )
```

over the same cells `computeLoftChordedAllow` walks, `π⁺` being
`nextafter(π, +Inf)`. `sectionDisplacementArea`'s
argument is unchanged — a point of the symmetric difference between the held cap
polygon and the denoted region lies within the matched departure of SOME chord
point, and a chord's neighbourhood is a rectangle of that half-width plus two
half-disks — read per chord with that chord's own departure instead of the
build-wide maximum. Measured: 0.0166 → 0.0079 (tooth8), 0.23 → 0.14 (z8), 5.0 → 2.6
(z40, prototype target). The wall's ruled and station-shift legs are already per cell.

With §2 and §3 in place `Area` becomes the binding reading on every probe case
(§5's table). At z8 its bound is 0.58 of `cellBilinearArea`'s integration
enclosure (`divisions = 4`) beside 0.09 of ruled leg and 0.08 of cap tube; that
enclosure does not shrink with the chord target, and this design leaves it alone
(§11).

## 5. The chord target and bisection on the matched departure

**The target is a fraction of the section's own feature size `A / P`.**

```text
chordTarget = loftChordFraction · min( |area(p0)| / perimeterUpper(p0), |area(p1)| / perimeterUpper(p1) )
loftChordFraction = 2.5e-4
```

`area(p)` is the record's own exact region integral — the `ig.area`
`falsifyRecordedArea` already computes through `evaluatorIntegrals` — and
`perimeterUpper(p)` is the sum over the record's segments of `PerCellArcUpper(seg,
walk, 1)`: the exact `circularLengthInterval` bracket for an arc, the walk's own
`LengthUpper` otherwise. A target decides the vertex set. For a record of `LineSeg`
and polynomial Tier A segments with recorded control points, both inputs are
exact-rational computations rounded once, so the same record gives the same target
on every platform; `TestLoftChordTargetReadsFeatureSize` pins one such record's
target bits, and CI reads them on amd64 and arm64. A circular segment's area
contribution is a float trig expression (`internal/momentregion`'s `AddCircular`),
and a `FitSplineSeg`'s spans come from sketch's float interpolation solve, so a
record holding either can read a target whose last bits differ between platforms.

**Why `A / P`.** Loft §5.1's rule reads the coordinate envelope, which grows with
the gear radius and with the section's distance from the sketch origin, while the
volume residual of §2 is `Σ cellMatched_k · cellWallUpper_k ≈ matched · P · h` against
`V ≈ A · h`. With every `cellMatched_k ≤ chordTarget` the Volume ratio is at most
`loftChordFraction · σ`, where `σ` is the wall's slant factor (1.06 on the 20°
helix), whatever the section's size or shape, and the one-tooth case gets the
tooth's target instead of the gear's. A per-segment target was considered and
declined: the residual is a perimeter-weighted sum of departures, so no allocation
across segments lowers it for a given station count by more than a constant.

**Where it is computed.** `Document.Loft` stores the two integrals
`falsifyRecordedArea` already ran on the payload (`loftPayload.recordArea`), and
`placed` carries them over unchanged. `loftStationCapGate` computes the target once
per evaluation from them and the walks `validateLoftRecords` resolves, decides S15
against it, and returns it to the station generators. The target itself is not
stored: its perimeter half reads the walks, which only `evalLoft` resolves, and
recomputing it from the stored areas is one pass over the segments. Recomputing the
area integral costs 1.1 s per record at z40, which the prototype paid and the
implementation does not.

**Bisect on `max(sagitta, matched)`.** `SagittaStationWalk.WalkCell` measures each
side's `SpanMatchedDeltaUpper` beside its sagitta and bisects while either exceeds
the target; the accepted cell's matched value is handed to the reader instead of
recomputed. `SagittaUpper` stays the sagitta (loft §5.2's `sectionDelta` and
`Bounds.Bound` read it). This is what makes "every `cellMatched_k ≤ target`" a
property of the walk rather than of the probe: without it a free-form cell's
matched departure sits 1.5× above the target on the gear flank and is unbounded in
general (`TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses`). Cost on the
probe at z40: 1960 stations against 1480, Volume ratio 1.4e-4 against 2.1e-4.

**The constant.** `2.5e-4` is the finest value whose z40 build the audit of §6
leaves near 5 s (§7's timing table); the coarser `5e-4` was measured and still reads
`Sound` on every case but leaves `Area` a 1.5× margin at z8. Measured with §2–§4 and
this rule, every case `Sound`:

| Case | target | Stations | Binding | Ratio | Volume ratio | Centroid bound | Build (old audit) | new audit |
|---|---|---|---|---|---|---|---|---|
| tooth8 | 2.1e-4 | 132 | Area | 4.0e-4 | 9.9e-5 | 6.1e-4 | 0.24 s | 12 ms |
| tooth20 | 2.3e-4 | 124 | Area | 1.1e-4 | 9.9e-5 | 6.3e-4 | 0.22 s | 6 ms |
| tooth40 | 2.4e-4 | 125 | Volume | 1.2e-4 | 1.2e-4 | 7.8e-4 | 0.21 s | 4 ms |
| z8 | 4.2e-4 | 952 | Area | 5.2e-4 | 1.2e-4 | 1.3e-3 | 3.2 s (1.8 s) | 0.15 s |
| z20 | 1.1e-3 | 1280 | Area | 2.6e-4 | 1.4e-4 | 3.1e-3 | 5.8 s (3.1 s) | 0.16 s |
| z40 | 2.4e-3 | 1960 | Area | 2.8e-4 | 1.4e-4 | 5.8e-3 | 15.7 s (9.0 s + 1.7 s area recompute) | 0.35 s |

At `5e-4`: z8 640 stations, Area 6.5e-4, Volume 2.5e-4; z40 1360 stations, Area
5.4e-4, Volume 2.6e-4, build 8.0 s (old audit 4.5 s).

Loft §14's calibration paragraph (the arc wedge at `m = 65`) is retired by this rule;
§8 re-pins the wedge under the new constant.

## 6. The audit: sweep enumeration, wall-wall pairs, one proof per cap

Loft §6's audit keeps its verdict rule, its per-pair classification, its two
reject-only tiers and its three certificates. Three things change: how pairs are
ENUMERATED, which pairs are tested, and how S8 counts.

**Enumeration is sweep-and-prune over the triangles' boxes.** Sort the triangle
indices by their box's lower bound on the axis of largest total extent (ties by
index), sweep, and emit every pair whose boxes overlap on all three axes. Boxes are
closed intervals: boxes that touch on a face, edge or corner overlap, as
`BoxesOverlap`'s `<=` already decides. A pair of
disjoint boxes shares no point (loft §6's existing box argument), so the enumerated
set contains every touching pair; a pair sharing a vertex has overlapping boxes, so
every pair Table C expects to touch is enumerated. Candidates are sorted
lexicographically before testing, so the first refused pair is the same `(i, j)` the
all-pairs loop reports today. Both sweep passes are float comparisons only; the
sort is deterministic.

**Only wall-wall candidates are tested pairwise.** Each is decided exactly as today
(box tier already passed; `LoftPlaneSeparated`; certificates; exact classification).
The two caps are decided by one proof each, over exact signs alone:

Let `C` be one cap's triangles, `L` its polygon loops as vertex-index cycles (the
cap's `vIdx` or `wIdx`), `O` the other cap's vertices, and `Π` the exact plane of
`C`'s first triangle with normal `n`.

- **(s)** the index structure is the one assembly builds: loops of at least
  three vertices, no index repeated across `L` and `O`, no vertex of `C` in `O`
  and no vertex of the other cap in `L`, every wall triangle with a vertex in `O`
  and its others in `L` forming one vertex or one loop edge, and every loop edge
  an edge of some wall triangle;
- **(a)** every vertex of `L`, and every vertex of every triangle of `C`, has
  exact sign 0 against `Π`;
- **(b)** every vertex of `O`, and every vertex of every triangle of the other
  cap, has a nonzero sign against `Π`, all the same;
- **(c)** every triangle of `C` has `n · ((B − A) × (C − A)) > 0`;
- **(d)** the directed-edge multiset of `C` nets to exactly the edges of `L`, each
  loop traversed in one consistent direction, every other edge netting to zero;
- **(e)** in the projection `projAxes(n)`, exactly one loop's oriented signed area
  (its exact shoelace sign times its (d) direction) has the triangles' orientation
  sign and every other loop's has the opposite sign.

Together with the wall-wall audit, which proves every chord of `L` meets every
other wall edge only as Table C expects — so the loops of `L` are simple and
pairwise disjoint — these give: (c)+(d) make `Σ_T 1_T` equal the winding number
`w_L` of the oriented boundary almost everywhere (a 2-chain and the region chain
with the same boundary differ by a 2-cycle, which is zero in the plane); (e) with
simple disjoint loops makes `w_L ∈ {0, 1}`; so the triangles of `C` are interior
disjoint and cover the polygon exactly. (s) is what ties every loop edge to a wall
triangle the wall-wall audit tests. A cap triangle therefore meets a polygon
edge only in shared vertices or as that edge (a triangulation edge through a
reflex vertex would put a neighbouring triangle outside the polygon, and a polygon
vertex inside a polygon edge contradicts simplicity). A wall triangle `U` meets `Π`
in exactly its vertices on `Π` — (b) puts its other vertices strictly on one side —
which are its cap-side vertex or edge, a polygon vertex or edge by index; so
`T ∩ U = T ∩ (U ∩ Π)` is exactly the contact their shared indices expect, for every
cap triangle `T` and wall triangle `U`, and two triangles of `C` meet exactly in
their shared edge or vertex. (b) also proves the two caps disjoint. Any condition
failing falls back to pairwise testing of that cap's pairs through the same
candidate list; nothing is admitted on a failed proof. A pair is left to the proofs
when either of its triangles is in a proven cap. When a wall-wall pair fails, the
audit tests the decided pairs that precede it lexicographically before refusing, so
the refused pair is the reference path's.

**S8 counts candidates.** The ceiling `maxFacetPairTestsPerCall` stays `8_000_000`
and is compared against the number of enumerated candidates the pairwise pass will
test (wall-wall, plus any cap family that fell back), counted in a first sweep pass
before the second pass tests them; the `F·(F − 1)/2` preflight goes. The counting
pass steps the budget once per pair it compares on the sweep axis and refuses once
that scan count passes the same ceiling, so a tall shape whose boxes all overlap on
the sweep axis refuses after `O(F log F + ceiling)` work. `F` itself is refused past
`8·8192 − 8` before any exact lift (§7). The budget still steps once per tested pair.

**Entry points.** `LoftCrossingAuditStructured(budget, verts, tris, walls,
capStartCount, loops0, loops1)` is the loft's; the generic
`LoftCrossingAudit(budget, verts, tris)` keeps every other caller
(`sweep_mitre_build.go`) on the sweep enumeration with pairwise testing of every
candidate. `LoftAuditShortcuts` gains `Sweep` and `CapProof` fields; the zero value
stays the reference path — every pair of every triangle through the exact
classification — that every test compares verdicts against.

Measured on the probe (prototype target):

| Case | F | all pairs | box candidates | wall-wall (no shared vertex) | wall-cap | cap-cap | old audit | pairwise over all candidates | wall-wall pairs + cap proofs |
|---|---|---|---|---|---|---|---|---|---|
| tooth8 | 524 | 137 K | 62.7 K | 16.7 K (16.2 K) | 29.2 K | 16.8 K | 50 ms | 45 ms | 9.5 + 1.1 ms |
| z8 | 3804 | 7.23 M | 701 K | 139 K (135 K) | 321 K | 242 K | 1.81 s | 1.59 s | 71 + 8 ms |
| z20 | 5116 | 13.1 M | 886 K | 88 K (83 K) | 377 K | 421 K | 3.11 s | 2.71 s | 48 + 9 ms |
| z40 | 7836 | 30.7 M | 2.15 M | 96 K (88 K) | 884 K | 1.17 M | 9.04 s | 9.03 s | 56 + 15 ms |

The sweep itself costs 1.6 ms (tooth8) to 276 ms (z40). A sweep alone buys
nothing: every wall triangle has a vertex on each cap plane, every ear-clipped cap
sliver spans the section, and `LoftPlaneSeparated`'s float filter cannot certify a
determinant that is exactly zero, so 41 % of the old audit's CPU at z40 is the
exact lift (`XptOf` under `OrientSign`) inside that tier. The cap proofs are what
remove the cost: 25× to 130× on the gear.

## 7. Resource ceilings

Every ceiling keeps a hard constant and gains a shape that scales with the record.
`P` and `C` are loft §5.1's paired-segment and chorded-pair counts.

| Ceiling | Today | Becomes | Hard ceiling | Why this shape |
|---|---|---|---|---|
| S15 station cap `loftStationCap` | 500 | `stationCap(P) = min(max(512, 32·P), 8192)` | 8192 stations, `F ≤ 8·8192 − 8` | the probe needs 125–132 stations per tooth (`P = 6`) and 952–1960 for the gears (`P = 48–240`); 32 per paired segment covers every case with ≥ 3.9× room; the hard ceiling keeps `NewLoftAuditData`'s exact lifts under ~70 MB |
| S8 pair ceiling | `F·(F−1)/2 ≤ 8_000_000` | pairs the sweep scans on its axis ≤ `8_000_000`, and enumerated candidates ≤ `8_000_000` (§6); `F ≤ 8·8192 − 8` before any exact lift | unchanged | the scan count is at least the candidate count, so the counting pass refuses after at most `8_000_000` comparisons and the work before a refusal is `O(F log F + ceiling)`; the triangle ceiling bounds the exact lifts, which the station cap does not bound for a build with no chorded pair (`loftStationCapGate` returns early there) |
| R7 `FreeformWorkLimit` for the loft's station walk | `1 << 20` per record per operation | `FreeformWork.Limit`, raised in `evalLoft` once `loftStationCapGate` has passed (where `P` is known) to `max(1 << 20, Spent + 8192 · stationCap(P))` | `1 << 20 + 8192 · 8192 = 2^26 + 2^20` | measured 5000–8300 work units per station per record on the probe (`.tmp/loft-gear-bounds-logs/`); the same counter stays the record's one counter for the operation |
| reconstruction `ReconstructionWorkLimit` | `1 << 26` (chord ceiling 5792) | `1 << 28` (chord ceiling 11585) | `1 << 28` | the z40 record charges `2 · 6640²` = 88 M; sketch's arranger took 0.7 s for both records' admission at that size; `1 << 28` is ~3 s of the same work |

The reconstruction ceiling is decad's model of sketch's quadratic arranger, so
raising it is decad's decision; making the model obsolete is sketch's. The hand-off
`../sketch/.tmp/decad-handoff-arrangement-chords.md` asks sketch for a sub-quadratic
arrangement and for a chord count that scales with a curve's size rather than
`16 · controls` with a floor of 64, with the gear as the reference case.

Build time after §2–§6, estimated from the probe's measured components (build
minus the old audit and the prototype's area recompute, plus the new audit): tooth
cases 0.2 s; z8 1.4 s; z20 2.7 s; z40 4.5 s. The z40 profile puts the remainder in
`validateLoftRecords`' free-form arc-length brackets (1.3 s), `recordProfile` plus
`falsifyRecordedArea` (1.7 s) and station generation (1.1 s). Under `-race` the root
package runs 5–8× slower, so the race shards carry the three tooth cases and the z8
outline; z20 and z40 run under `testing.Short()` skips (§8).

## 8. Required tests

Each bound test deletes its leg or gate and records the red run in the test
(`decad-fixtures-must-be-shown-to-fail`). No bound literal is pinned; margins are
asserted as ratios against the tolerance.

| Test | Asserts | Shown to fail by |
|---|---|---|
| `TestLoftVolumeResidualIsPerCellWallLegPlusSkirt` | on an oval whose two arcs have radii 3 and 5, `Volume.Bound` equals the rounding, the vertex sweep, the per-cell sum and the skirt composed in order; the skirt is exactly 0 at `delta == 0` (one chord per arc) and positive on the placed copy | dropping the skirt on the placed copy; charging the build-wide maximum instead of the per-cell sum (bound rises) |
| `TestLoftVolumeBoundEnclosesRefinedRing` | the exact ring volume lies inside `[Value − Bound, Value + Bound]` at station counts from 4 to 64 per quarter arc, twisted and untwisted | deleting the wall leg |
| `TestLoftPlacedVolumeNeedsTheVertexSweep` | a placed thin slab with one tiny chorded arc: the exact volume lies inside `Volume`'s bound and outside the bound without `sweptLeg` | deleting `sweptVolumeAllow` from the chorded `Volume` composition |
| `TestLoftCentroidShiftFormEnclosesTwoArcLobe` | on a section of two circular arcs (no `LineSeg`, so every wall is ruled; a ring cannot show the shift red, its chorded centroid equals the true one by symmetry), untwisted, twisted and placed at three bulge offsets: the exact centroid lies inside `Bound` of the published one; `R_c` reaches every densely sampled true boundary point; the bound is below the old form's on every row | deleting `R_c`'s `max(matchedDelta, delta)` term (6 of 7 rows); deleting the shift (every row) |
| `TestLoftCentroidToothBoundReadsToothSize` | one tooth at z = 8, 20, 40: `R_c` within the tooth's own diameter; the Centroid ratio below the old form's and below `2.5e-4 · 4` on every row (5.0e-5, 5.0e-5, 6.1e-5 measured) | the anchor-based form, read over §2's volume legs, exceeds `2.5e-4 · 4` at z40 (1.7e-3; 3.1e-4 and 7.3e-4 at z8 and z20) |
| `TestCentroidClearanceIsExact` | `CentroidClearance` is the largest float at or below the exact `|vol| − volumeAllow` where that allowance sits inside `V_value`'s half-ulp | the rounded-volume form overstates it 1.7x |
| `TestLoftAreaCapTubeIsPerCell` | `CapAreaExcess` is the per-cell tube; it matches the maximum form to rounding when every cell's departure is equal (a uniform full circle) and is below it on the two-radius oval | charging the maximum form |
| `TestLoftChordTargetReadsFeatureSize` | the target of a tooth, a gear and a copy of each translated 10 outer radii: `fraction · A/P` to one ulp, translation-invariant; an exact `LineSeg`/`NURBSSeg` record's target bits pinned, read on amd64 and arm64 by CI | reading the envelope rule |
| `TestLoftFreeformWalkBisectsOnMatched` | on the zigzag span of `TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses` paired with itself, every accepted cell's matched value is at or below the target | bisecting on the sagitta alone |
| `TestLoftArcWedgeVerifiesSound` (re-pinned) | the §14 wedge's Volume and Centroid margins under the new constant, as ratios | — |
| `TestLoftSweepEnumeratesEveryTouchingPair` | on the gear and on the existing crossing fixtures, the sweep's candidate set contains every pair the all-pairs reference finds touching, and the refused pair is identical | enumerating from a shifted axis without the full box test |
| `TestLoftCapProofAgreesWithReference` | every §6 fixture: the structured audit's verdict equals the zero-shortcut reference's | deleting condition (d); deleting (e) |
| `TestLoftCapProofRefusesOverlappingTriangulation` | a hand-built cap whose triangles double-cover a region (a positively oriented hole) fails (e); a cap with one triangle flipped fails (c); a cap with a vertex lifted off the plane fails (a) and the pair falls back to pairwise testing | — |
| `TestLoftGearOutlineVerifiesSound` | full gears z = 8 (race shards), 20 and 40 (`testing.Short` skip): `Sound`, every ratio below `1e-3`, station count below `stationCap(P)`, work below the raised limit | restoring any one of the shipped ceilings |
| `TestLoftStationCapScalesWithPairs` | `stationCap(P)` at `P` = 3, 6, 48, 240, 10_000 | — |
| `TestLoftStationWalkWorkPerStation` | the probe's pairing work divided by its stations is below 8192 on every gear case | — |
| `TestLoftAuditCandidateCeilingRefuses` | a build whose candidate count exceeds a lowered test ceiling refuses S8 before any exact test | — |

Every test lands in `.github/test-shards.txt`.

## 9. PR split

Five PRs; 1–3 are independent of one another and of 4; 5 follows 1–4. Each PR edits
the loft sections §12 lists for it.

| PR | Files and functions | Proves itself by |
|---|---|---|
| **1 — volume residual and area tube** | `internal/loftmesh/loft_chord_allow.go`: `ComputeLoftChordedAllow` accumulates `WallLeg`, `CapAreaExcess` per cell and `SkirtLeg` over every seam cell, and drops `CapVolumeUpper`, `SeamAllow`, `h1Upper`, `posUpper`; `loft_moments.go` (`computeLoftChordedAllow` loses the cap-offset scan); `internal/loftmesh/mass_accumulator.go` `Volume`/`Area`, and `Centroid`'s `epsV` reads the same chain; `loft_build.go` `loftMeshProofOf` (`volSymDiff`); `internal/proofbound/bounds.go` makes `PerturbedAreaUpper` exact and deletes `ChordedBoundarySeamAllow`, `ChordedBoundaryVolumeAllow`, `ChordedBoundaryVolumeResidualAllow`, `CapAreaVolumeAllow` and their tests | the tests of §8 that name Volume or Area, and `TestLoftPlacedVolumeNeedsTheVertexSweep` |
| **2 — centroid shift form** | `internal/loftmesh/mass_accumulator.go` `Centroid`, `CentroidMeasureAllow`, `CentroidRadius`, `CentroidClearance`, `VolumeAllow`; `loft_chord_allow.go` accumulates `WallLeg` and `SkirtLeg` (fields only; `Volume` keeps reading the shipped residual until PR 1); delete `PlacedCentroidAllow` and the loft's `ChordedBoundaryMomentResidualAllow` call (the `internal/proofbound` helpers stay) | the three centroid tests |
| **3 — audit** | `internal/loftmesh/loft_audit.go`: `sweepCandidates`, `LoftCrossingAuditStructured`, `capFamilyProof`, `LoftAuditShortcuts.Sweep`/`.CapProof`; `loft_build.go` calls the structured entry with `a.walls`, `a.capStartCount`, `a.vIdx`, `a.wIdx`; `sweep_mitre_build.go` unchanged; `internal/proofbound/budget.go` comment on what S8 counts | the four audit tests |
| **4 — chord target and matched bisection** | `loft_stations.go`: `loftChordTarget(area0, perim0, area1, perim1)`, `loftFeatureSize`, `loftChordFraction = 2.5e-4`; `loft.go` passes `falsifyRecordedArea`'s integrals on `loftPayload.recordArea`; `loftStationCapGate` computes the target from them and `loft_build.go` chords at it; `internal/freeform/spline_stations.go` `WalkCell` measures and forwards the matched value, `StationCellReader.AcceptCell` takes it; `loft_chord_calibration_internal_test.go` re-pinned | the target, walk and wedge tests |
| **5 — ceilings and the gear fixture** | `internal/loftmesh/record_stations.go` `StationCap(P)` and `StationShare`; `loft_stations.go` `loftStationCapGate`; `internal/freeform/work_budget.go` `FreeformWork.Limit`, `Step` reads it, `ReconstructionWorkLimit = 1 << 28`, `ReconstructionChordCeiling = 11585`; `loft.go` raises the limit after the cap gate; `doc.go` support map, `docs/missing-features.md` gear row, loft §12 increment table | the gear, cap, work and S8 tests; `go test . ./apitest/ -run '^TestCI'` |

Each PR runs the probe (kept as `loft_gear_internal_test.go` behind a `-run` filter
and `testing.Short`) and records the ratios it reached in its description.

## 10. Do not do this

- Do not keep the cap or seam leg "for safety" in `Volume`: §2's argument is written
  down, and a redundant leg is what made the gear `Suspect`.
- Do not drop `sweptVolumeAllow(delta, perturbedAreaUpper)` from `Volume`,
  `Centroid`'s `epsV` or `volSymDiff` whenever `delta > 0`, chorded or not: it is the
  only charge for §2's step 1 (projecting the held caps onto their planes) and for
  the faceted cells' motion in step 2. It is required, not kept for safety.
- Do not compute `perturbedAreaUpper` from a float cross product or a float edge
  length: §2 depends on it, so each triangle's area and edge lengths go through the
  exact rational cross product and `RatSqrtUp`.
- Do not sum `seamPerimeterUpper` over charged cells alone: the skirt runs along
  every seam cell, and a faceted cell's seam moves by `delta` too.
- Do not read `sectionDelta` where `cellMatched_k` is owed, and do not substitute the
  build-wide `matchedDelta` for the per-cell value in `wallLeg` or the cap tube.
- Do not bisect on the sagitta alone and rely on the measured 1.5× ratio; the ratio
  is unbounded on a general span.
- Do not recompute the area integral inside `evalLoft` for the target: carry the
  integrals `falsifyRecordedArea` computed on the payload.
- Do not derive the target from the sketch's claimed area; the record's own exact
  integral is the deterministic input.
- Do not make the target a caller option or a function of a published measurement
  (loft §5.1, §14).
- Do not admit a cap on a failed family condition, and do not skip the wall-wall
  pass when the cap proofs pass: the proofs depend on it for loop simplicity.
- Do not run the sweep without the lexicographic sort of candidates; the refused
  pair must be the one the reference path reports.
- Do not replace `ReconstructionWorkLimit` with a time limit or a context; the
  public record methods have none.
- Do not raise `FreeformWorkLimit` globally; the raise is the loft operation's, sized
  by its own station cap.
- Do not change `cellBilinearArea`'s partition in these PRs (§11).
- Do not pin any bound literal; assert ratios and enclosure.

## 11. Open questions, settled

- **Per-cell or per-segment target?** One target per build (§5). Recommended and
  taken: a per-segment allocation changes the residual by at most a constant.
- **Bisect on matched?** Yes (§5). The cost is 32 % more stations at z40.
- **`loftChordFraction`.** `2.5e-4` (§5). `5e-4` reads `Sound` on every probe case
  but leaves `Area` a 1.5× margin at z8.
- **Should `Area`'s `cellBilinearArea` enclosure be deepened now that `Area` binds?**
  No. It is a fixed `4 × 4` partition independent of the target; deepening it is a
  separate change with its own cost, recorded here as the next lever if a section
  reads `Suspect` on `Area` alone.
- **Is the reconstruction ceiling decad's or sketch's?** Both (§7): decad raises its
  model to `1 << 28`; sketch owns the arranger's cost and receives the hand-off.
- **Keep the `(sectionDelta, areaUpper)` two-leg warning in loft §8?** Yes, reworded:
  the warning was against dropping the TWIST leg from a bound over held triangles,
  which §2 keeps for the tessellation's `volSymDiff`.

## 12. Loft sections each PR edits

| PR | `docs/loft-design.md` edits |
|---|---|
| 1 | §5.2: `capVolumeUpper`, `seamAllow`, `posUpper`, `seamPerimeterUpper`, cap `planeOffsetUpper` rows replaced by `wallLeg` and `skirtLeg` rows; `capAreaAllow` row reads the per-cell tube; §8 `Volume` paragraph; §8.1 table (three rows: wall, twist, skirt) and its telescoping paragraph; Table S S14's cap-offset arm; §15's tessellation bullet |
| 2 | §8 `Centroid` paragraph; Table S S12 wording (`epsV`, `R_c`) |
| 3 | §6: enumeration paragraph, the cap proof paragraph, "the work budget" paragraph (S8 counts candidates); Table S S8; §12 PR 5 row drops the broad-phase item |
| 4 | §5.1 "The chord target and its constant" and the free-form arm's bisection bullet; §14's calibration paragraphs |
| 5 | §5.1 station-cap paragraphs (`stationCap(P)`, work-budget paragraph); Table S S15; §12 increment table; `docs/spline-design.md` R7 row; `docs/missing-features.md` gear row; `doc.go` |
