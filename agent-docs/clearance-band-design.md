# Clearance Band Design

Navigation: input class → §1; why a cell needs a band → §2; shared rules →
§3; pin-in-bore readings → §4; flat-feature readings → §5; what stays
undecided → §6; soundness → §7; tests → §8; increments → §9.

The pin-in-bore readings of §4 are implemented. The flat-feature readings of
§5 are the planned second increment (§9); the code does not read them yet.

This document owns the band readings of the clearance kernel's face and edge
cells: lower bounds read without the degeneracy oracle's answer, for carriers
that an exact construction makes coaxial, parallel or perpendicular and a
float placement moves a few roundings off. `agent-docs/clearance-design.md`
owns the kernel, its proof obligation (§1), candidate enumeration (§3), face
pair table (§4), intervals (§5) and touching pairs (§6). Its windowed nested
cell (§4) is the first band reading and the model for every reading here.
Nothing here changes those contracts: a band reading only adds a proven
interval where a cell would otherwise contribute the coarse enclosure's zero
lower bound or mark the pair unsure.

## 1. Input class

The readings accept two trimmed analytic features of two proven solids, each
from a carrier model `pairkernel` builds (`clearance-design.md` §2), whose
cell needs a degeneracy answer and gets none:

- two circular edges whose spine pair `spine.CircleCircleCrits` cannot
  answer (§4.1);
- a plane face crossed by a cylinder face whose axis the oracle does not
  prove parallel to the plane's normal (§4.2);
- a circular edge whose axis the oracle cannot prove parallel or not
  parallel to a plane face's normal (§4.3);
- a straight edge, a plane face or a cylinder face the oracle cannot prove
  parallel or perpendicular to a plane face, a straight edge or a cylinder
  axis (§5, planned).

The class is defined by these geometric relations alone. A reading never
reads a body's name, a feature's construction history or the placement that
produced it. It reads the carriers' floats, their trims and the kernel's
tolerance `tol` (1e-9 of the pair's coordinate scale).

## 2. Why a cell needs a band

The oracle (`internal/clearance/oracle.go`) answers `DegYes` only when a
degeneracy holds exactly over the carriers' dyadic values, `DegNo` only when
a float residual exceeds the kernel's noise (`tol` for an offset, `ClrAngTol`
for an angle), and `DegUnknown` in between. A rigid placement through
`Transform` lifts every recorded point through a float rotation. Two points on
one exact axis, such as a bore's rim centre and a pin's rim centre at
different heights, land a few ulps off a common float line. Two directions
built by different float compositions, such as one rotation and two half
rotations, differ in their last bits. The degeneracy is then neither provable
nor disprovable: the answer is `DegUnknown`.

A cell that branches on the answer has a closed form for `DegYes` and for
`DegNo` and nothing for `DegUnknown`. Without a band reading it falls to
`CellSink.Coarse`, whose lower bound is the distance between the two features'
boxes, or it marks the sink unsure. Two features whose boxes meet then
contribute a zero lower bound, `Pair` reads `lo ≤ tol` and the pair is
undecided. The pruning of `clearance-design.md` §5 skips a cell whose box
distance exceeds the best upper bound, so whether the failure shows depends on how far apart the features'
boxes are, not on the placement angle as such. A pin and its bore rotated
5° about Y keep their rim boxes 1.45 mm apart and the rim cell is pruned; at
20° the boxes meet and the pair reads undecided. The same pin with its rims
5 mm from the bore's rims reads a measured gap at 20°.

## 3. Shared rules

Every band reading obeys these rules. They are the windowed nested cell's
rules (`clearance-design.md` §4), stated once.

- **The bound holds over the whole carrier.** A reading bounds the distance
  from every point of one feature's untrimmed carrier (a full circle, an
  infinite line, a whole plane) to the other's. A trimmed feature is a
  subset, so the bound holds for every trim.
- **Every float term carries its charge.** The reading's floats take fewer
  than 128 additions, multiplications, divisions and correctly rounded square
  roots at the magnitude of an envelope of every coordinate and radius it
  reads. `clearance.EnvelopeCharge` over that envelope covers their rounding.
  A lower bound subtracts the charge and rounds down (`freeform.DownRound`);
  an upper bound adds it and rounds up (`proofbound.AbsSumUpper`).
- **The tilt is bounded, never assumed.** For two float unit directions `a`
  and `b`, `s = min(1, |a × b| + AnalyticRoundBound(1))` bounds the sine of
  the angle between the directions they stand for, as `windowedCircleFE`
  reads it. `√(1 − s²)` is read rounded down.
- **A reading certifies only when it clears the tolerance.** A band whose
  lower bound is at or below `tol` certifies nothing, and the cell answers as
  it does without the reading: the coarse enclosure or unsure. A band never
  turns a contact, a crossing or an equal-radius fit into a positive gap.
- **A reading never replaces a decided answer.** It runs only on the branch
  that has no closed form. An exactly coaxial pair still reads its ring
  family, and a provenly offset pair still reads its exact offsets.
- **A band tightens the coarse enclosure.** A distance reading contributes
  through `CellSink.CoarseWith`: the larger of its lower bound and the box
  distance, the smaller of its upper bound and the coarse witnesses'. A
  crossing exclusion contributes nothing, because the lower tiers hold the
  minimum (§5's argument).
- **An upper bound is an admitted sample.** A reading samples the feature at
  16 uniform azimuths plus an arc's window ends and midpoint, maps each sample
  to its partner carrier, and keeps the nearest pair that both trims admit
  with margin `tol`. That distance plus the charge is the upper bound. No
  admitted sample leaves the upper bound at `+Inf`, which `CoarseWith` replaces
  with the coarse witnesses' bound.
- **A reading is never `Exact`.** Its interval carries a positive charge.

## 4. Pin-in-bore readings

A pin of radius `r` through a bore of radius `R > r` in a placed plate reaches
three cells besides the two walls (which the windowed nested cell already
reads): the rims against each other, the plate's faces against the pin's wall,
and the rims against the other body's faces. §4.1–§4.3 read each one.

### 4.1 Two circular edges: the circle pair band

`curvecells.circleCircleEE` reads two circular edges `A` (centre `c_A`, unit
axis `n_A`, radius `ρ_A`) and `B` (`c_B`, `n_B`, `ρ_B`). When
`spine.CircleCircleCrits` answers `ok = false`, whether the coaxiality is
undecided, the kink guard refuses, or the bracket is not certified, the cell
reads the circle pair band (`windowedCircleEE`, `internal/clearance/curvecells`)
before it falls to the coarse enclosure.

Every point of `B` is `q = c_B + ρ_B·v` for a unit `v ⊥ n_B`. Let
`h₀ = (c_B − c_A)·n_A`, `δ = |(c_B − c_A) − h₀·n_A|` (the distance from `c_B`
to `A`'s axis line) and `s` the tilt bound of `n_A`, `n_B`. Then:

- `|v·n_A| ≤ s`, since `v ⊥ n_B`, so the height of `q` along `A`'s axis is
  `h(q) = h₀ + ρ_B·(v·n_A)` and `|h(q)| ≥ H = |h₀| − ρ_B·s`;
- the distance of `q` from `A`'s axis is `ρ(q) = |P(c_B − c_A) + ρ_B·P v|`,
  `P` the projection off `n_A`, with `|P(c_B − c_A)| = δ` and
  `|P v| ∈ [√(1 − s²), 1]`, so `ρ(q) ∈ [ρ_B·√(1 − s²) − δ, ρ_B + δ]`;
- the distance from `q` to the full circle `A` is `√(h(q)² + (ρ(q) − ρ_A)²)`.

Hence every point pair of the two circles lies at least
`lo_AB = √(max(0, H)² + max(0, R_AB)²)` apart, where
`R_AB = max(ρ_A − ρ_B − δ, ρ_B·√(1 − s²) − δ − ρ_A)`. The reading takes the
larger of `lo_AB` and `lo_BA` (roles swapped), each term charged per §3.

The upper bound maps each sample `q` of `B` to `p = c_A + ρ_A·u`, `u` the unit
projection of `q − c_A` off `n_A`, and keeps the nearest pair both arcs admit
(`clearance.CircleAngleAdmit`).

The radial part is `windowedCircleFE`'s line-spine band with `A`'s axis as
the spine. The two readings share one helper, `circleAxisBand`, which returns
`[ρ·√(1 − s²) − δ, ρ + δ]` for a circle about a line.

A pin's rim 2 mm below its bore's rim, radii 3 and 3.2, reads
`lo ≈ √(2² + 0.2²)`. Two flush rims read `lo ≈ 0.2`. Equal radii at equal
height read `lo ≤ tol` and keep the coarse enclosure.

### 4.2 A plane crossed by a tilted cylinder: the crossing annulus

`facepair.planeCrossesRevolved` excludes the crossing of a plane face `f`
(unit normal `n`, offset `c`) with a cylinder face `g` (anchor `a₀`, unit axis
`a`, radius `R`). It reads the crossing circle only when the oracle proves
`a ∥ n`. When it does not, and `|n·a| ≥ ½`, the cell reads the crossing
annulus (`tiltedCrossingExcluded`, `internal/clearance/facepair`) before it
marks the sink unsure.

The axis meets the plane at `P₀ = a₀ + a·(c − n·a₀)/(n·a)`. A point `x = P₀ + w`
of the plane, `w ⊥ n`, lies `√(|w|² − (w·a)²)` from the axis, and
`|w·a| ≤ |w|·s` with `s` the tilt bound of `a` and `n`. So every point of the
carrier crossing satisfies `R ≤ |w| ≤ R/√(1 − s²)`: the crossing lies in the
annulus about `P₀` between those radii. When the annulus misses `f`'s trim,
the two faces never cross.

The miss test is `clearance.AnnulusRegionMiss`, the annulus form of
`CircleRegionHits`' exclusion. It probes the annulus's middle circle, of
radius `(R + R')/2` with `R' = R/√(1 − s²)` rounded up. Every element of the
region must lie farther than `(R' − R)/2 + tol + charge` from that circle
(`clearance.ElemElemDistLB`), and one point of the middle circle must classify
outside the region with that margin. The annulus is connected and meets no
region boundary, so the whole annulus then lies outside the region. The
charge is `EnvelopeCharge` over `a₀`, `f.O`, `R` and `|c − n·a₀|`, divided by
`|n·a|` rounded down.

A miss contributes nothing: the minimum between the plane face and the
cylinder face lies on the hole's rim or on the cylinder's rims, which §4.1,
§4.3 and the windowed edge cell read. Any other outcome keeps the unsure
answer: a sample of the annulus is not a point of the crossing, so the reading
never proves a crossing. The annulus holds for every tilt up to 60°, so the
reading also runs where the oracle proves the axis not parallel and the cell
has no other exclusion.

### 4.3 A circular edge over a tilted plane: the rim height band

`curvecells.circlePlaneFE` reads a circular edge (centre `c`, unit axis `m`,
radius `ρ`) against a plane face `f`. When the oracle cannot decide `m ∥ n`,
the cell reads the rim height band (`tiltedRimOverPlane`,
`internal/clearance/curvecells`) before it marks the sink unsure.

Every point of the circle is `c + ρ·u` with `u ⊥ m`, so `|u·n| ≤ s` and its
height over `f`'s plane lies in `[b − ρ·s, b + ρ·s]`, `b = (c − f.O)·n` read
through `clearance.Height`. A rim whose box lies farther than `tol` from the
face's box is excluded before the reading, as it was. When `|b| − ρ·s − charge > tol`, the whole circle
lies strictly on one side of the plane: no crossing, and every point pair is
at least that far apart. The upper bound reads each admitted sample's exact
height (`clearance.Height`) where its foot lies in `f`'s trim. A band that
does not clear keeps the unsure answer.

## 5. Flat-feature readings (planned)

These readings are the second increment and are not implemented. Until they
land, each cell below keeps its present answer: the coarse enclosure or
unsure.

A placed block beside a placed block, or a pin lying along a block's face,
reaches the same undecided band through parallel planes, edges and axes. Each
reading below either excludes a crossing or contributes a distance band.

**Why an exclusion may contribute nothing.** When a feature provably lies
strictly on one side of a plane, the distance from its points to the plane
face is affine along every straight element and has no interior critical
unless the feature is exactly parallel. If it is not parallel, the minimum
lies at an end of the admitted part: a vertex (the vertex tiers) or where a
foot leaves the trim (the edge tiers). If it is exactly parallel, the plateau
takes the same value at those ends. Either way §3's enumeration of
`clearance-design.md` still holds the minimum, so the cell owes only the
crossing exclusion, as its `DegNo` branch already does.

- **A segment over a plane** (`curvecells.linePlaneFE`, `DegUnknown`). Both
  ends' exact heights (`clearance.Height`, widened by `DirCharge`) above `tol`
  on one side exclude a crossing. Otherwise the cell keeps the unsure answer.
- **A plane face over a plane** (`facepair.planePlane`, normals not exactly
  parallel). The heights of `g`'s trimmed region over `f`'s plane are affine
  over the region. Their range is the range over every line element's ends
  and every arc element's centre height plus its radius times
  `clearance.TrigRange` of the in-plane amplitude over its window, widened by
  `EnvelopeCharge`. A range strictly above `tol` or below `−tol`, for `g` over
  `f` or `f` over `g`, excludes a crossing before the crossing-line path runs.
- **A cylinder lying along a plane** (`facepair.planeCrossesRevolved`,
  `|n·a| < ½`). The axis's exact heights at the two ends of the axial window
  bound every axis point's height, since the height is affine along the axis.
  Both ends more than `R + tol` on one side exclude a crossing.
- **Two near-parallel segments** (`curvecells.lineLineEE`, `DegUnknown`).
  Every point of segment `B`, of length `L`, lies at least
  `max(δ₀, δ₁) − L·s` from `A`'s line, where `δᵢ` are the exact distances of
  `B`'s ends from that line (`clearance.PointLineDist`). The larger of the two
  role orders, charged, is the band's lower bound, through `CoarseWith`.
- **A segment along a cylinder** (`curvecells.lineOffsetFE`, line spine,
  `DegUnknown`). The distance from the segment to the axis lies in
  `[max(δ₀, δ₁) − L·s, max(δ₀, δ₁)]`: the lower end as above, the upper end
  because distance to a line is convex along a segment. A band clearing `R`
  by more than `tol` on either side bounds the pair through `CoarseWith`.
- **A circle by a segment** (`curvecells.lineCircleEE`, `DegUnknown`). The
  circle lies at least `ρ·√(1 − s²) − δ` from the segment's line, `δ` the
  distance from its centre (`circleAxisBand`). A positive band bounds the pair
  through `CoarseWith`.

## 6. What stays undecided

- **Contact of displaced bodies.** `clearance-design.md` §6 certifies a
  touching pair only when both bodies' `bodyGeom.delta` (or, for a ruling, the
  carrier part) is exactly zero. A placed body's displacement is positive, so
  a placed face-on-face or pin-on-face contact stays undecided. Band readings
  measure positive gaps only.
- **A pin that fills its bore.** Equal radii read a band at or below `tol`.
- **Near-parallel plane faces.** A pin placed by two half rotations has end
  caps a few roundings off parallel to its plate's faces. `planePlane` finds
  no closed form for the pair and marks it unsure whenever the faces' boxes
  meet, so the pin at 45° and 137° about Y reads `Suspect`. The plane-over-plane
  reading of §5 decides it.
- **Faceted payloads.** A mesh Boolean result, a mitred sweep, a coil and a
  mesh body have no carrier model here. The exact planar arm of
  `agent-docs/interference-design.md` §3.2 partitions such a pair; this design
  does not change it. That arm spends its time on the per-site local
  separation proof (`internal/pair/planar/planar_local.go`) when two finely
  tessellated faces touch over a large area.
- **Cone, torus and spindle cells.** They keep the coarse enclosure of
  `clearance-design.md` §4.
- **A crossing proof.** No band reading proves an overlap. A jammed pin
  (`r > R`) reaches the overlap through the crossing cells, as
  `clearance-design.md` §4 states.

## 7. Soundness

A wrong positive gap is worse than an undecided pair: `Verify` would report a
clearance the geometry does not have. The readings keep that risk closed by
construction and by test.

- Each lower bound is a closed-form inequality over the whole carrier (§4,
  §5), never a sample. A sample only places an upper bound, and an upper
  bound is any admitted point pair.
- Each float term is charged (§3). The kernel's own carrier displacement is
  charged once more after aggregation (`pairkernel.DeltaWiden`).
- A reading that does not clear `tol` changes nothing (§3), so the readings
  cannot mint a contact or hide a crossing that a cell would otherwise mark
  unsure.
- §8's differential tests compare every certified row against a brute-force
  distance over boundary points of the same bodies, the one-sided check of
  `apitest/clearance_property_test.go`: the proven lower bound never exceeds
  the sampled minimum.

The readings can regress three things, each covered by §8: an existing exact
row (a reading must not run on a decided branch), an existing unsure answer
that guards a contact (a reading must not clear on a contact), and the row's
interval width (`CoarseWith` keeps the tighter of the two bounds).

## 8. Tests

Every test asserts computed geometry: an interval against a known gap, a lower
bound against an independent minimum, or a crossing verdict.

**Unit tests in the owning packages.** Each builds `CFace` and `CEdge` values
directly, as `curvecells/cells_test.go` and `facepair/cells_test.go` do.

- `curvecells`: the circle pair band. It covers flush rims `1e-14` off a
  common axis (gap `0.5`), rims `2` apart axially (gap `√4.25`), arcs with
  partial windows, a rim of radius `5.5` tilted `0.05` rad with its centre
  `1` above a rim of radius `5` on the same axis, and equal radii at equal
  height. The tilted pair's lower bound is at or below a 4096-sample minimum
  plus `1e-12`; the band attains `0.8769`, and with `ρ_B·s` dropped from `H`
  it reads `1.115` and fails. The equal-radius pair keeps the coarse
  enclosure: its lower bound is at or below `tol` and its interval equals the
  coarse one.
- `curvecells`: the rim height band. It covers a rim `1e-12` rad off the
  normal of a plane turned 45°, at height `2` (interval within `2·tol` of `2`;
  the turned plane's box meets the rim's, so the box exclusion does not prune
  the cell), a rim crossing the plane and a rim one tilt-radius from it
  (unsure kept).
- `facepair`: the crossing annulus. A plane with a hole of radius `3.2`
  crossed by a cylinder of radius `3`, tilted `1e-12` rad and `0.01` rad, is
  excluded. The same tilt `0.6` rad (ellipse semi-major `3.6`) stays unsure;
  red when `R'` reads `R`. A hole of radius `3` stays unsure.
- `clearance`: `AnnulusRegionMiss` against a disk, a hole and a rectangle,
  with an annulus inside, outside and straddling each boundary.
- `curvecells` and `facepair`: the §5 readings, each with a near-parallel
  case that clears, a crossing that stays unsure, and a decided branch whose
  output is unchanged (second increment).

**API tests** (`apitest/clearance_band_test.go`, exported API only):

- `TestClearancePlacedPinInBore`: a 20×20×10 plate with a bore of diameter
  6.4 and a pin of diameter 6 through it, once with the bore made by `Cut`
  and once by extruding a holed profile. Each is placed by `Transform` at
  0°, 5°, 20°, 45°, 90° and 137° about Y and 30° about X and about
  `(1, 1, 0)`. Each reads `Sound`, one `Clearance` row and a gap within `1e-9`
  of `0.2`. Seen red without §4.1: the 20°, 45°, 137° and off-Y rows read
  `Suspect` with `DiagUndecidedClearance`.
- `TestClearancePlacedPinTwoPlacements`: the same pair with the plate placed
  by one rotation and the pin by two half rotations. The placements at 0°, 5°,
  20°, 90° and off Y read the same row. The placements at 45° and 137° about Y
  read `Suspect` with `DiagUndecidedClearance` and no row, because §6's
  near-parallel plane faces meet there; the second increment turns them into
  rows.
- `TestClearancePlacedPinInBoreIsFast`: the 20° pair verifies in under five
  seconds.
- `TestClearancePlacedPinDifferential`: three of those placements. Every row's
  lower bound is at or below the minimum distance between the two bodies'
  tessellation vertices at a 0.01 mm chord. Vertices lie on the boundary; a
  facet's interior lies inside a concave bore and would understate the
  minimum, so the oracle samples vertices only. The upper bound plus the
  covering radius is at or above it. The race build runs one placement
  (`raceDetector`).
- `TestClearancePlacedPinThatFillsItsBore`: equal radii, placed at 20°, reads
  `Suspect` with no row.
- `TestClearancePlacedFlatPairs` (second increment, with §5): two blocks with facing
  faces `0.5` apart and a pin `0.5` from a block's face, placed at 30° about
  `(1, 1, 0)` and at 137° about Y, read a gap within `1e-9` of `0.5`.

**Shards and cost.** New `apitest/` names go in the last numbered shard of
`.github/test-shards-apitest.txt`; run `go test . ./apitest/ -run '^TestCI'`.
Internal package tests run in their packages' own jobs and need no manifest
entry. A pin-in-bore `Verify` takes milliseconds; each reading adds at most 38
samples to one cell. No benchmark is added.

## 9. Increments

| PR | lands | still `Suspect` after it |
|---|---|---|
| 1 (implemented) | §4: the circle pair band, the crossing annulus with `AnnulusRegionMiss`, the rim height band, the shared `circleAxisBand`, and §8's unit tests and pin-in-bore API tests | §5's flat features in the undecided band; §6 |
| 2 (planned) | §5: the six flat-feature readings and their tests | §6 less the near-parallel plane faces |
