# Draft design

How decad builds a drafted body: a tapered extrude (`Extrude` with a nonzero
`WithTaper`) and a face draft of an existing prism (`Body.Draft`). Both
produce one payload class, the `draftPayload`, whose walls lean at a stated
angle to the sweep axis so that a molded part releases along that axis.

Companion to `docs/api-design.md` ("core §N"), `docs/evaluator-design.md`
§5 (the prism this body generalises), `docs/modify-design.md` §5 and §8 (the
exact section offset and its audit, which this design reuses) and
`docs/modify-reach-design.md` §8.3–§8.4 (the cap-loop chamfer band, whose
patch, flux and bound machinery this body is built from). Nothing here changes
those contracts.

Four tables are normative and each states its facts once:

| Table | States | Section |
|---|---|---|
| **RD** — the inputs | which profile, extent, receiver and selection each entry point admits | §4 |
| **SD** — the refusals | every refusal, modify §1's existence test, its sentinel, and the gate order | §5 |
| **BD** — the result | the payload, its faces, edges and roles | §6 |
| **DD** — downstream | one row per consumer and what it does with a draft body | §9 |

Question router (navigation, not authority):

| Question | Section |
|---|---|
| What is a draft, and which offset family defines it | §1, §2 |
| Why the far section is decad's own geometry, not a `sketch` answer | §3 |
| What `WithTaper` admits | §4 Table RD |
| What `Body.Draft` admits, and its signature | §4, §10 |
| Why a call refused, and in which order the gates run | §5 Table SD |
| What faces, edges and roles come out | §6 Table BD |
| How the body is built | §7 |
| How volume, area, centroid and bounds are computed and bounded | §8 |
| What tessellation, export, `Verify`, booleans and modify ops do | §9 Table DD, §9.1 |
| Tests an implementer must write | §11 |
| Rejected approaches | §12 |
| Decided questions | §13 |
| PR split and what each PR leaves refused | §14 |

## 1. Problem

A molded part needs every wall that runs along the pull direction to lean
away from the mold by a few degrees. A straight prism's walls run parallel to
the sweep, so a caller proving a molded part needs a tapered extrude and a
draft of the faces of a body that already exists, and `Verify`'s
pull-direction survey needs drafted walls to read. §14 states which of these
build today and `docs/missing-features.md` ("Feature operations") lists the
rest.

The evaluator already builds exactly one drafted surface family: the cap-loop
chamfer band of modify-reach §8.3, which joins a cap loop at one level to its
exact offset at another level with `Plane` and `Cone` patches, and proves
every reading off it. A tapered extrude is that band with the receiver's
straight part removed, so this design reuses the band's construction and
proofs and adds the far section's offset rule, the entry points and the
refusals.

## 2. The model: the sharp offset family

**A draft body is the sweep of a section family.** Write `P` for the recorded
section in its plane, `α` for the signed taper, `h` for the sweep height and
`e` for the unit sweep direction from the sketch plane toward the far end.
The section at axial distance `z ∈ [0, h]` from the sketch plane is

```text
P(z) = P offset by t(z) = z · tan α,   inward for t > 0, outward for t < 0
```

and the body is `{ p + z·e : z ∈ [0, h], p ∈ P(z) }`. The far section is
`Q = P(h)`, offset by `d = h · tan α`.

**The sign.** A positive `α` narrows the body with distance from the sketch
plane: every wall's carrier moves `t(z)` into the material. A negative `α`
widens it. `WithTaper` keeps its existing meaning (core §12: a signed
displacement, outside `ErrNegativeMagnitude`), and `Body.Draft` (§10) reads
its angle the same way about its neutral plane.

**The offset is sharp.** `P(z)` is the section whose every wall carrier is
moved by `t(z)` and whose every corner is the intersection of its two moved
carriers:

| Feature of `P` | In `P(z)` |
|---|---|
| a line | the parallel line, `t(z)` into the material |
| a circular walk, material inside (a counter-clockwise round) | the concentric circle of radius `R − t(z)` |
| a circular walk, material outside (a hole wall, a concave round) | the concentric circle of radius `R + t(z)` |
| a corner between two lines, convex or reflex | the **miter**: the intersection of the two moved lines |
| a G1 join (modify §8's row: the two walks leave and arrive along one direction, decided on the held unit tangents with the same dead zone) | the corner moved along the shared left normal, `v + t(z)·n̂` |
| a whole circle | the concentric circle; no corner |

A reflex corner between two lines is a miter here, where modify §8's shell
offset inserts an arc. The two families answer two different questions: a
shell's wall must be `t` thick everywhere, so its reflex corner is the arc at
distance `t` from the corner point; a draft tilts each wall about its trace
and re-intersects the neighbours, so a reflex corner stays sharp, as the
molded part's does. §12 records the rejected alternative.

**The far section is a `ProfileRecord` in the recorded vocabulary.** Every
piece of the table is a line, an arc or a circle, so `Q` records as
`LineSeg`/`ArcSeg`/`CircleSeg` walks and every consumer that reads a recorded
section reads it. That closure is what the admitted profile class (Table RD)
protects: a corner where a circular walk meets its neighbour other than
tangentially has a miter locus that is a conic, not a line, and a free-form
walk's offset is not a free-form kind. Both refuse.

**Every corner locus is affine in `t`.** A miter between two moved lines moves
along a fixed ray at a fixed rate; a G1 foot moves along the shared normal; a
circle's radius moves linearly. So every wall between `P` and `Q` is ruled by
straight lines between corresponding points, and each wall is an exact
analytic surface:

| Wall of `P` | Surface | Normal |
|---|---|---|
| a line | `Plane` through the near segment and its far segment, leaning `α` from the sweep axis | `n̂·cos α + e·sin α`, with `n̂` the section's outward unit normal in the plane |
| a circular walk or whole circle | `Cone` about the axis through the centre along `e`, radius `R` at the sketch plane and `R ∓ d` at the far end, half angle `|α|` | the cone's own |

A circular wall's two directrices sweep **one** angular window, because both
of its corners are G1 feet (which lie on the wall's own radial through the
corner) or it has no corner. The ruled surface between two concentric arcs
over one window about one axis is the cone of revolution itself, so the
`Cone` tag names the surface built, with only the placement's rounding
between them (modify-reach §8.3's skew term is identically zero here).

**The lateral edges are `Line3`.** Each corner of `P` and its image in `Q` are
joined by the straight ruling the walls share. A miter ruling between two
`Plane` walls is their exact intersection line; a G1 ruling between a `Plane`
and a `Cone` is the line along which they are tangent.

## 3. What `sketch` is asked

Nothing beyond the recorded profile. `Q` is decad's own synthesized geometry
under modify §5's rule: the offset loop exists in no sketch the caller drew,
so there is no upstream answer to consume, and decad proves its validity with
the exact closed-form audit of `internal/sectionaudit/` (modify §5's four
tests) rather than with a residual.

The reason that rule applies here, stated once per CLAUDE.md's hard rule: at
decad's pin, `sketch.CreateOffset` offsets lines only (non-line entities are
skipped), places the copies through the floating Levenberg–Marquardt solve,
certifies nothing (`Enclose` refuses `Offset` constraints with
`ErrUncertifiedConstraint`) and checks neither collapse nor self-intersection.
No answer decad could consume exists upstream, and a certified one would still
leave the audit decad's, since the loop is decad's construction. No hand-off
file is written.

## 4. Table RD — the inputs

| RD | Entry point | Admits | Result |
|---|---|---|---|
| **RD1** | `Extrude` with nonzero `WithTaper` | a profile of `LineSeg`, `ArcSeg` and `CircleSeg` walks in which every corner at a circular walk is a G1 join; a `Distance` extent, `Along` or `Against`; `|α| < 90°`; no `WithSurfaceResult` | a `draftPayload` solid (Table BD) |
| **RD2** | `Body.Draft` | a live `prismPayload` receiver (an extrude, a filleted or chamfered body, a tube, or any of these `Placed`) whose section carries no displacement; a `NeutralFace` naming one cap of the receiver; a selection resolving to exactly the receiver's complete wall set; `0 < |α| < 90°` | a `draftPayload` solid over the receiver's section, the receiver retired |
| **RD3** | `Body.Draft`, PR 5 | as RD2 with a selection naming a **subset** of the walls, where every corner between a selected and an unselected wall is a line–line corner | a `draftPayload` whose far section moves the selected walls only (§10.2) |

Everything else is Table SD.

## 5. Table SD — the refusals

Each row states what was asked, modify §1's test (does the body exist under
any evaluator?) and the sentinel that follows from it.

| SD | The call asked for | Exists? | Sentinel |
|---|---|---|---|
| **SD1** | a taper that is not an angle, or not representable in radians | — | `ErrUnitKind` / `ErrNotFinite`, as for a straight prism |
| **SD2** | `|α| ≥ 90°`, or an angle whose certified cosine enclosure (`RadSinCosInterval`) reaches zero | no — a wall at a right angle to the axis sweeps nothing | `ErrDegenerate` |
| **SD3** | a free-form walk in the section | yes | `ErrUnsupported` — the offset of a Bézier span is not a recorded kind; §12 names the chorded alternative and rejects it |
| **SD4** | a corner where a circular walk meets its neighbour that the held-tangent rule classifies as a miter (not G1) | yes | `ErrUnsupported` — the junction is a conic; the ruled stand-in of modify-reach §8.3 is not scheduled here (§14) |
| **SD5** | a circular walk whose far radius collapses: `offset2d.OffsetRadius` reports `ok == false` (`R ≤ d` for a round with the material inside under a positive taper; a hole wall under a negative one) | yes — the cone's apex lies inside the sweep and the body past it has another topology | `ErrUnsupported` (modify S11a) |
| **SD6** | a far outer loop whose signed area has changed sign (modify S8), whose every walk the offset consumes (`offset2d.ErrLoopConsumed`) under a positive taper, or a far hole the audit decides lies outside the far outer loop (modify S9's decided row) | no — the taper consumed the region | `ErrDegenerate` |
| **SD7** | a far walk `offset2d.WalkConsumed` reports consumed — a line whose two miters have crossed — where some walk of its loop survives, or a hole every walk of which is consumed | yes | `ErrUnsupported` (modify S11a) |
| **SD8** | far loops that cross or make boundary contact at modify §5's scale-anchored floor (S7/S11b) — two walls closing on each other, a widened hole reaching the outer loop | yes | `ErrUnsupported` |
| **SD9** | far loops whose nesting the containment classifier cannot decide (modify S9) | undecidable here | `ErrUnsupported` |
| **SD10** | a far section that passes every audit at the held `d` and fails at the top of `d`'s span (§8.1; modify-reach §8.3.1's second audit) | undecidable here | `ErrUnsupported` |
| **SD11** | a nonzero taper with an extent other than `Distance` — `Symmetric`, `TwoSided`, `ThroughAll`, `ToFace`, or a side form | yes | `ErrUnsupported` (staged, §14) |
| **SD12** | a nonzero taper with `WithSurfaceResult` | yes | `ErrUnsupported` (staged, §14) |
| **SD13** | a `d` that rounds to zero, a far vertex bit-identical to its near vertex, or a far radius bit-identical to its near radius (`capband.BandRadius`) | yes — float64 cannot name the far section at this scale | `ErrUnsupported` (modify-reach SX13) |
| **SD14** | a far corner whose displacement enclosure cannot be built (`offset2d.ErrUnbounded`) | yes | `ErrUnsupported` (SX14) |
| **SD15** | a corner whose two moved carriers have no intersection — a cusp (`|cross| ≤ tol`, `dot ≤ 0`) between two lines | yes | `ErrUnsupported` (`offset2d.ErrTopology`, modify S11's row) |
| **SD16** | a wall patch whose outward orientation `fixPatchOrientation` cannot certify | yes | `ErrUnsupported` (SX15) |
| **SD17** | `Draft` of a retired receiver, or of a sheet | — | `ErrRetiredBody`; a sheet as `Fillet`'s `refuseSheetOperand` refuses it |
| **SD18** | `Draft` with a zero angle | it exists and is the receiver (modify S13) | `ErrDegenerate` |
| **SD19** | a neutral selector resolving to zero or several faces; a curved neutral face; a `Faceted` neutral face | — | `ErrCardinality` (Expected "exactly 1"); `ErrDegenerate`; `ErrUnsupported` — `MirrorFace`'s own three rules |
| **SD20** | a `NeutralFrame`, or a `NeutralFace` that is not a cap of the receiver | yes | `ErrUnsupported` (staged, §14) |
| **SD21** | a selection that is not exactly the receiver's complete wall set (PR 5 narrows this to RD3's rule); an empty selection | yes | `ErrUnsupported` (staged); `ErrNoMatch`/`ErrCardinality` as core §9 |
| **SD22** | a selected face parallel to the neutral plane — the other cap — or the neutral face itself | no — a face with no trace on the neutral plane has no line to tilt about | `ErrDegenerate` |
| **SD23** | `Draft` of a receiver that is not a `prismPayload`, or whose section carries a displacement (`requireExactSection`), or that is itself a draft body | yes | `ErrUnsupported` (modify S3; a second draft is SX10's composition rule) |

The sharp offset of a convex section past its collapse is the section
point-reflected through a centre, which keeps its walk sense, so S8 never sees
a square tapered past its apex; stage 4 reads that outer loop, every walk
consumed, as SD6. Two concentric far circles never cross: past the closure of
F7's ring the far hole lies outside the far outer loop, which is SD6 through
the audit's decided nesting row, not SD8. No recorded profile holds a cusp
between two lines, and a cusp at a circular walk is SD4 first, so Extrude never
reaches SD15; `internal/offset2d/sharp_test.go` pins it.

**Gate order.** Each stage needs the one before it, and where two gates read
one constructed section the existence question is asked first (modify §4).

| Stage | Gates, in order |
|---|---|
| 1 — the pre-gates | `Extrude`: the seam gates of core §7, then SD1, SD2 (the seam gates run first, as they do for SD1 on a straight prism). `Draft`: SD1, SD2, SD17, SD18, SD19's cardinality |
| 2 — the receiver and its inputs | `Draft`: SD23, SD20's `NeutralFrame` refusal, SD19's kind rules, SD20's cap test, SD22, SD21. `Extrude`: the extent's own validation (unchanged), SD11, SD12 |
| 3 — the section's kinds | SD3; then per corner SD4 and SD15 |
| 4 — the far section is built | the span of `d` (§8.1); SD5, SD7 and SD6's consumed outer loop as the walks are offset, then SD13 |
| 5 — the audit | SD6, then SD8, then SD9, over the far section at `d` |
| 6 — the span re-audit | SD10 |
| 7 — the patches | SD14, SD16 |

Every gate runs before a face is made, and a refused call leaves the
document unchanged (evaluator §8).

## 6. Table BD — the result

| BD | Entry | Payload | Lumps | Faces and roles | Edges |
|---|---|---|---|---|---|
| **BD1** | `Extrude` + taper | `draftPayload`: the near record `P`, the far record `Q` with its displacement, the frame, `[z0, z1]` with the end displacements, `α` with the span of `d`, the placement | 1 | `capStart` over the section at `z0`, `capEnd` over the section at `z1` (the prism's convention: `Along` puts `P` at `z0`, `Against` puts `P` at `z1`); one wall per walk of `P` with role `side(i, j)`: `Plane` for a line, `Cone` for a circular walk, a seamless `Cone` for a whole circle | cap rims `Line3`/`Arc3`/`Circle3` from each record; lateral edges `Line3`, convex by the walk's turn exactly as a prism's (evaluator §3) |
| **BD2** | `Draft` (RD2) | the same payload over the receiver's record, under the receiver's placement | 1 | as BD1, every role minted under the result's own producer (modify §11: a result's roles are its own) | as BD1 |
| **BD3** | `Draft` (RD3) | the same payload; `Q` moves the selected walls only | 1 | as BD2; an unselected wall keeps its vertical `Plane`/`Cylinder` | as BD1 |

A full-circle loop builds one `Cone` with two `Circle3` rims and no seam edge,
as a cylinder does today. Adjacent `Plane` walls whose two recorded segments
lie on one exact line canonicalize into one face by evaluator §3's rule, which
reads the record and not the surface.

## 7. Construction

`draft_build.go` builds the body from the payload. The steps, in order:

1. **Resolve the walks.** `boundarywalk.WalkOf`, `RequireAnalyticWalk` (SD3)
   and `CoalesceWalksBudget` over every loop of `P`, as `capblend_geom.go`'s
   `oneLoopCornerLoop` does.
2. **Compute `d` and its span** (§8.1).
3. **Classify every corner** with `offset2d.CornerJoin`'s held-tangent rule.
   A corner at a circular walk that is not G1 is SD4.
4. **Offset every loop sharply.** `offset2d.BuildSharpLoop(budget, walks, s,
   d, tol)` is `BuildLoop` with one change: a corner the rule would send to
   the reflex-arc row takes the miter row instead. Its corner table is §2's,
   and it returns the joins beside the record for SD13's corner test. A
   missing miter root is SD15; a collapsed radius is SD5; a consumed walk is
   SD7, and a loop whose every walk is consumed is `offset2d.ErrLoopConsumed`
   (SD6 for the outer loop under a positive taper). The result is `Q` as
   `[]CurveSegment` per loop, assembled into a `ProfileRecord` with the same
   loop order as `P`.
5. **Audit `Q`** with `auditOffsetSectionBudget` (modify §5): SD6, SD8, SD9.
   Re-run steps 4–5 at the top of `d`'s span (`auditCapBlendSetbackSpan`'s
   rule): SD10.
6. **Enclose the far corners.** `capband.ContourDisplacement`
   (`capcontour.Displacement` over the sharp joins, across the span of `d`)
   gives each loop's far
   contour displacement, and the payload's `farDelta` is the largest; a
   corner it cannot enclose is SD14. `offset2d.SectionDeltaFromWalks` is not
   used: it re-derives the joins by the shell's rule, which closes a reflex
   corner with an arc the sharp offset never builds.
7. **Build the caps** with the prism's cap builder over `P` at its level and
   over `Q` at the far level. The far cap's vertices carry `farDelta` beside
   their frame-lift rounding (evaluator §5's `liftedVertex`), as the cap-blend
   cap does.
8. **Build the walls.** Per walk, one `capband.Patch`: a line gives a Plane
   patch with `SideA/B` the near segment and `CapA/B` the far one; a circular
   walk gives a Cone patch with `SideRadius = R`, `CapRadius = R ∓ d` and
   `CapTh0/CapTh1 = Th0/Th1` (one window). The surface is
   `capblend_geom.go`'s `planeFromThree` or `coneSurface(pl, cu, cv, R,
   R ∓ d, zNear, zFar)`; the rims are `arcEdge` / `wholeCircleEdge` / line
   edges; the rulings are `Line3` between corresponding corners; the
   orientation is certified by `fixPatchOrientation` (SD16). Each patch
   carries `ContourAllow = farDelta`, `LevelDelta` = the far end's axial
   displacement, and its `HeldAllow` as modify-reach §8.4's table states.
9. **Stamp roles and convexity**: `side(i, j)` per walk, `capStart`/`capEnd`,
   lateral edges convex by the walk's turn.
10. **Measure** (§8), then commit.

Steps 7–9 run through the cap-loop band itself. `draftPayload.band()` is a
`capBlendPayload` view with every loop chamfered on the far cap, `dc = d`
across the cap and `ds = h` down the sweep, so the band's side level is the
sketch plane and no straight slab remains. Its `draft` flag makes
`buildCapBand` (`capblend_geom.go`) take the sharp joins
(`offset2d.SharpJoinsBudget`), stamp `side(i, j)` roles, give rulings and far
rims the prism's convexity, and flip the axial half of the orientation
reference for a negative taper. The near rim is `draftNearRim`'s: the bottom
rim `buildLoopSidesAs` builds for a prism's wall, at the sketch plane.

`Body.Draft` reaches step 1 with the receiver's record and placement after its
own gates (§10), and builds the identical body `Extrude` would build from the
same section and sweep.

### 7.1 Vertex bounds

A near vertex is the prism's: its recorded coordinates, its junction bound
(`boundarywalk.JunctionVertex`) and its own frame-lift rounding. A far vertex
adds `farDelta` and the far level's axial displacement. Every reading that
takes a far coordinate charges both terms, and no reading reads the far level
as exact.

## 8. Measurements

The draft body is one closed sub-solid: the near disk, the far disk and the
wall band between them. Every reading reuses the cap-loop band's closed forms
over the patches of step 8, with the band's `dc = d`, `ds = h`, and no
receiver slab.

| Reading | Closed form | Owner of the code |
|---|---|---|
| Volume | the divergence theorem in plane-local coordinates: the near disk's area at its level, the far disk's area at its level, and `Σ capband.RawFlux(patch)`, divided by 3 (`capBandVolume`'s sum). A Plane patch's flux is the exact rational tetrahedron identity; a Cone patch's is the closed-form polynomial-plus-trig flux over rationals with certified `sin`/`cos` enclosures | `capblend_moments.go`, `internal/capband/` |
| Area | the two disks' region areas plus `Σ capband.AreaOf(patch)`: a Plane patch's two-triangle sum, a Cone patch's frustum sector `(αc/2)(R0 + R1)·L` with `L = √(ΔR² + H²)` | `internal/capband/area.go` |
| Centroid | the first moment by the same divergence theorem (`capBandMoment`), divided by the volume and lifted to world through the frame and placement; the geometric safety-net bound (the centroid lies within `Bounds`) as a ceiling | `capblend_centroid.go`, `internal/capband/moment.go` |
| `Bounds` | per direction the extreme over both records' analytic extremes at their levels, each charged with its own displacement and the lift rounding; a placed body charges the placement's rounding as evaluator §5 states | `prism_extent.go`'s readers over both records |

**Bounds compose as modify-reach §8.4 states, with the same four helpers.**
The far cap's area composes `sectionDisplacementArea(farDelta, walks,
perimeterUpper)`; the volume composes `SweptVolumeAllow(farDelta, areaUpper)`
once after the flux sum; each patch's area composes `BandPatchAreaAllow` and
`BandLevelAreaAllow`; the first moment composes `SweptMomentAllow` once per
body. `capband.ClosureOf` charges the sliver at every corner whose held ends
are not provably one point, exactly as the band does. The chord-versus-locus
term (`ChordLocusVolumeAllow`) is read and is zero, because both windows of
every Cone patch coincide; an implementer keeps the call and asserts the zero
rather than omitting the term.

**The wall normal.** `Face.NormalAt` on a draft wall carries the same
denoted-wall term a chamfer band's patch does (modify-reach §8.3): the turn
between the built wall and the wall the taper denotes, since the far rim sits
within the contour displacement of the denoted one. A Plane wall's tag passes
through its exact near edge and one far corner, and moving that corner by `e`
turns the normal by at most `2|e|/h`; a Cone wall's half angle moves by at most
`(e_r + |Δr|·e_z/h)/h`. `capband.DenotedNormalAllow` charges
`2·(δ + δ_z·max(1, |Δr|/h))/h` with `h` at the bottom of its span, and
`setPatchReadings` (`capblend_geom.go`) adds it to every draft wall and chamfer
patch alike. `δ_z` is the far level's displacement beside the near level's
rounding, and no near-level displacement enters. SD11 admits only a `Distance`
extent, and `resolveLinearExtent` puts that extent's near end on the sketch
plane at exactly zero with a zero displacement; only the far end carries the
distance's conversion rounding. The near edge is therefore the recorded
section itself. The far displacement stays in `δ_z` even though it would cancel
for a chamfer, whose two levels both follow one held cap level: here
`h = z1 − z0` is read from the held far level, so that displacement moves the
height the taper is measured over. An extent that SD11 admits later and that
moves the near level must add the near displacement to `δ_z`.

**Exactness.** A draft measurement is `Exact` only where every term of it is
exactly representable. The tangent of the taper is a certified enclosure of
positive width (§8.1), so `d` is never exact and every measurement of a draft
body carries a nonzero bound; an axis-aligned drafted box at the identity
placement reads its volume `Approximate` with a bound of the order of `d`'s
own enclosure width times the box's lateral area. That is the same status a
`Cone` patch has today ("never `Exact`: the enclosure always has width",
modify-reach §8.4) and nothing here weakens core §5.3: a zero bound is a
claim, and the claim is false for every drafted body.

**The closed forms the tests check against** (§11) are the same quantities
read the other way: with `A(z)` the area of `P(z)`, a quadratic in `z` because
every corner locus is affine and every circular radius linear, `Volume =
∫₀ʰ A(z) dz` and the axial first moment is `∫₀ʰ z·A(z) dz`. For a drafted
rectangle `a × b` that is `h·(ab − (a + b)·d + 4d²/3)`; for a drafted disk the
frustum `πh(R² + Rr + r²)/3` with `r = R − d`. The production path integrates
the patches, never these polynomials; §12 records why.

### 8.1 The offset amount and its span

`d` is read from three held numbers, and every one of them rounds:

| Term | Source | Charged how |
|---|---|---|
| `α` in radians | `extent.MagnitudeInBounded` of the stated angle (degrees convert with rounding) | the conversion's own displacement, as a chamfer setback's `dcDelta` is |
| `tan α` | `RadSinCosInterval` over the radian enclosure: `tan = sin / cos` in rational interval arithmetic, `cos` bounded away from zero (SD2 otherwise) | the enclosure's width |
| `h` | `|z1 − z0|` with the ends' own axial displacements (`z0Delta`/`z1Delta`) | the far end's displacement |

`d` is the float product; `dDelta` is the reach of the exact rational product
of the three enclosures from it. `offset2d.OffsetAmount(s, d, dDelta)` then
spans every denoted amount, every corner enclosure (step 6) is taken over that
span, and the audit runs a second time at the span's top (SD10). A millimetre
sweep and a degree angle still carry a positive `dDelta`, from the tangent.

The far level is `zFar = ±h` as the extent resolved it; the draft adds no
level of its own, so `LevelDelta` is the extent's own axial displacement and
nothing else.

## 9. Table DD — downstream

| DD | Consumer | What it does with a `draftPayload` | PR |
|---|---|---|---|
| **DD1** | `Body.Tessellate` | chords both records once with the shared curve samples (tessellation §3), rules each wall between its two rims with the cap-band ring builder (`tessellate_capblend.go` over `internal/tessellation`'s rings), triangulates both caps; manifold by construction; the volume proof is the band admission of `docs/tessellation-reach-design.md` §7 with no slab (§9.1). A band the admission refuses meshes for export with no volume proof | 2 |
| **DD2** | `Union`/`Cut`/`Intersect` | the mesh path (evaluator §9) over DD1's mesh, admitted by `requireVolumeProvingPayload` through the same band admission; the analytic prism reduction's entry gate (`prism_boolean.go`) does not admit the class and takes the mesh path | 2 |
| **DD3** | STL / OBJ / 3MF | the mesh of DD1 | 2 |
| **DD4** | STEP | the analytic writer when every face is a `Plane` — a drafted polygon writes `ADVANCED_FACE`s over tilted planes with `Line3` loops, which `supportsAnalyticSTEP` admits today; any `Cone` sends the whole body to the faceted writer, as `docs/missing-features.md`'s STEP row states for cones | 2 |
| **DD5** | `Verify` validity | built by construction after the §5 audit, as a prism is; `Built` and `Solid` read true | 1 |
| **DD6** | `Verify` tolerance reference | `bodyGateDiameter` gains an arm (`draftGateDiameter`): the lower-bound diameter over wall stations on the near section at the near level and on the recorded far section at the far level, each charged its gap plus the level displacement and `farDelta`, falling back to the held vertices with their bounds (verification §3). Without it every draft body would read `DiagToleranceReferenceUnavailable`, since its bounds are never zero | 1 |
| **DD7** | `Verify` undercut survey (`WithPullDirection`) | decided (`draft_survey.go`): each wall patch's normal component along the pull is read through `Face.NormalAt` and enclosed by `capPatchNormalRange` (DX7's reading; `capPatchUndercuts` in `capblend_survey.go`, which the cap-blend survey runs over its own patches), each cap by `CapNormalDecision`, and `DecidePull` lists, clears or leaves undecided per modify-reach §8.3's rule. A wall drafted at `α > 0` against a pull along `e` reads `sin α > 0` and clears. A pull in a wall's own tangent plane leaves that wall undecided: its reading carries the taper's enclosure, so its sign is not proven | 4 |
| **DD8** | `Verify` wall and concave-radius surveys | `Suspect` with `DiagUnsupportedSurveyPayload` (staged). The reduction a later PR takes: the thinnest wall between two drafted skins is the far section's 2D reading scaled by `cos α`, and the smallest concave radius of a hole wall is its radius at the narrower end; neither is scheduled here | — |
| **DD9** | `Verify` clearance (`WithClearances`) | `Suspect` (`pairUndecided`): `newBodyGeomBudget` has no carrier model for the class. A later PR adds the Plane/Cone carriers; not scheduled | — |
| **DD10** | `Verify` interference | box-disjoint proofs from the body's `Bounds`; the read-only mesh intersection over DD1's mesh | 2 |
| **DD11** | `MassProperties` and `dynamics` | the verified-mesh path (`verifiedMeshMassProperties`) over DD1's mesh; `ErrUnsupported` where the band admission refuses | 2 |
| **DD12** | `Placed`, `Duplicate`, `PlacedCopy`, `Mirrored`, `MirroredCopy` | the payload implements `transform()` and `placed()`: a placement re-runs §7 under the composed motion, as a prism's does. `WithJoin` refuses (it rewrites a prism's or stacked prism's section) | 1 |
| **DD13** | `Patterned` | `placedInstance` and a boolean `Union` (the mesh path); the frame-keeping arm does not take the class | 2 |
| **DD14** | `Fillet`, `Chamfer`, `Shell` | modify S3, `ErrUnsupported`; each refusal message names the draft body among the classes it does not take. A shell of a draft body — the molded cup — is the first reach a later design should take: its cavity is a draft body of the same angle over the section offset by `t / cos α`, floored `t` above the kept cap. Not scheduled here | — |
| **DD15** | `Draft` of a draft body | SD23 | — |
| **DD16** | `Thicken`, `Offset`, `Patch`, `Stitch`, `Trim`/`Extend`/`Split` | not reached: a draft body is a solid (SD12 refuses the sheet form) | — |
| **DD17** | Motion and linkage bounds | `motion_bound.go`'s record radius arm: the larger of the two records' coordinate bounds, the far one widened by `farDelta`, since every point lies on a ruling or generator between them. Each record is read segment by segment: a line by its ends, an arc by its extent (`capband.SegmentCoordinateUpper`), a circle by its centre plus its radius | 2 |
| **DD18** | Selectors | `Planar()`, `FaceCreatedBy`, `CapStart`/`CapEnd` and the new `Walls(b)` (§10) select as on a prism. `Facing(v)` and `NormalTo(v)` match a drafted `Plane` wall only for its own tilted normal, since both require parallelism; a caller naming a drafted wall by direction passes that normal, or selects by role | 1, 3 |

### 9.1 The mesh

`tessellate_draft.go` hands `draftPayload.band()` to the cap-loop chamfer
tessellator with the payload's own wall geometry and far contour displacement
(`patches`, `bandDelta`, filled by the build), so the mesh charges the numbers
every reading of the body charged. Three things differ from a chamfer's mesh:

- no trimmed side wall is emitted, and the near cap's rim is written once and
  read by both the near cap and the walls (`draftNearRing`);
- a wall patch carries `side(i, j)` and the band has no apex patch, since the
  corners are read by the sharp rule (`capBlendPayload.offsetJoins`), which the
  admission (`capBlendOccupiedVolumeAdmission`) reads too: a reflex line-line
  corner is a miter, affine in the amount, and is admitted;
- the slice-wise chord term integrates over the whole sweep, `|z1 − z0|` with
  both ends' displacements (`draftBandHeightUpper`), not over a side setback.

A wall face's bound charges the far level's displacement and the band's, as a
chamfer patch's does, and no near-level term: the near end of a `Distance`
extent is the sketch plane, exact (§8). An extent SD11 admits later that moves
the near level must add it there, as it must to `δ_z`.

The admission refuses what it refuses for a chamfer: a circular wall whose
join is G1 by the held-tangent rule but not exactly tangent over the
rationals, a trimmed segment, or an arc whose recorded end is off its circle.
That mesh serves export, and every boolean, interference and mass reading
refuses it, naming the loop, the corner's plane-local point and the two
recorded segments that meet there (or the one segment).

The refusal of a near-tangent join is required, not a missing tolerance. Such
a corner turns by a sliver over the rationals, so the sharp offset family's
foot there runs along a conic, not along the ruling the mesh holds, and the
slice-wise proof's premise (every band section is a chord polygon of the true
offset section) fails at that corner. A bound on the tangents' cross product
would admit on a residual, which `CLAUDE.md` forbids, and a Hausdorff bound
times area is not an occupied-volume proof (tessellation §11). Admitting it
needs a proven bound on the area between the conic foot locus and the ruling
in every slice, integrated over the sweep: the per-cell integral of
`docs/tessellation-reach-design.md` §9's open question, a separate increment.

## 10. `Body.Draft`

```go
// NeutralPlane is the plane a draft tilts faces about. The set is sealed.
type NeutralPlane interface{ neutralPlane() }

// NeutralFace names a planar face of Body, selected and never pointed at
// (core §9), under MirrorFace's rules: exactly one face, planar, analytic.
type NeutralFace struct {
    Body *Body
    Face FaceSelector
}

// NeutralFrame names the plane through Frame's origin spanned by U and V.
// Staged: ErrUnsupported until the increment that lifts SD20.
type NeutralFrame struct {
    Frame r3.Frame
}

func (b *Body) Draft(ctx context.Context, sel FaceSelector, neutral NeutralPlane, angle units.Value, opts ...DraftOption) (*Body, error)

// Walls matches every face a sweep made: the side(i, j) roles of b's own
// producer. It is the sibling of CapStart and CapEnd.
func Walls(b *Body) FacePredicate
```

`Draft` is a modify operation on core §8's terms: it takes a context, resolves
`sel` against the live receiver, returns a new body and retires the receiver,
and leaves the document unchanged on any error, including `ctx.Err()`.
`angle` is a signed `units.Value` of `Kind` Angle (core §12 lists it with
`ToFace.Offset` and `WithTaper` as the signed displacements outside
`ErrNegativeMagnitude`); a positive angle narrows the body with distance from
the neutral plane. `DraftOption` is a sealed option tier with no option in
this design; it exists so a later increment can add one without changing the
signature.

### 10.1 Resolution (RD2)

1. The neutral face resolves through `SelectFaces(neutral.Body)` under the
   implicit exactly-one rule (SD19). It must be `capStart` or `capEnd` of the
   receiver (SD20).
2. `sel` resolves against the receiver. The resolved set must be exactly the
   set of faces carrying a `side(i, j)` role (SD21), and must not contain a
   cap (SD22).
3. The sweep direction `e` runs from the neutral cap toward the other cap.
   With the neutral cap at `z0`, `e` is the frame normal and the far end is
   `z1`; with the neutral cap at `z1`, `e` is its negation and the far end is
   `z0`. The section `P` is the receiver's record, which lies in the frame's
   plane at level 0 and is the section of every level of a straight prism, so
   the neutral cap's section is `P` whichever cap is named.
4. `h` is the receiver's height with its end displacements; `d = h · tan α`
   per §8.1.
5. §7 builds the body over `P`, `Q` and `[z0, z1]`; the far record sits at
   the far end. The result's roles are minted under the new producer.

A `NeutralFace` whose `Body` is not the receiver passes SD19's cardinality and
kind rules on that body's face and is then SD20, so a curved neutral face is
`ErrDegenerate` wherever it lies. `Walls(b)` is a face predicate over the
`side(i, j)` roles of `b`'s own producer (`selectorquery.WallsKind`), and the
complete wall set SD21 tests is the receiver's faces carrying such a role,
fillet and chamfer rounds included.

The body `Draft` builds equals the body `Extrude` would build from the same
sketch with `WithTaper(angle)` and a `Distance` extent toward the far end,
bit for bit when the receiver is unplaced. §11 asserts it.

### 10.2 The subset case (RD3, PR 5)

A selection naming some walls moves only their carriers. The far section is
`offset2d.BuildSharpLoop` with a per-walk amount, `d` for a selected walk and
`0` for an unselected one, and §2's corner table read with those amounts: a
corner between two lines miters whatever the two amounts are; a G1 join
between a line and a circular walk, or between two circular walks, is a G1
foot only when both amounts are equal, and a mixed pair there is SD4 (the
moved carrier and the unmoved one are no longer tangent, and their junction
is a conic). A whole circle is selected alone or not at all. An unselected
wall builds as the prism's `Plane` or `Cylinder`, and the lateral edge between
a tilted wall and a vertical one is their exact intersection line. Every
reading of §8 is unchanged: a patch with a zero amount is a Plane or Cylinder
patch whose far directrix is its near one translated.

## 11. Required tests

Every fixture asserts computed geometry against a closed form and asserts
that the closed form lies within the published bound. Closed forms are
written in the test from §8's polynomials, with `d = h · tan α` evaluated in
the test, never read back from the body. No bound literal is pinned
(amd64 and arm64 round FMA differently); a bound is asserted to cover the
closed form and to be below a stated relative ceiling. Each bound leg is
shown to fail first: the implementer deletes the leg, runs the fixture, records
the red run's failing assertion in the test's comment, and restores the leg.

**Extrude fixtures** (`apitest/extrude_taper_test.go`,
`draft_build_internal_test.go`):

| Fixture | Section, sweep, taper | Asserts |
|---|---|---|
| F1 box | square `a = 20`, `h = 10`, `α = 5°`, `Along` | `Volume = h(a² − 2ad + 4d²/3)`; axial centroid `h(a²/2 − 4ad/3 + d²)/(a² − 2ad + 4d²/3)` above the sketch plane; far-cap vertices at `(±(a/2 − d), ±(a/2 − d), h)`; each wall `Planar()` with `NormalAt` equal to `(n̂·cos α, sin α)` within its bound; `Area = a² + (a − 2d)² + 4·(a − d)·√(h² + d²)`; 6 faces, 12 edges, 8 vertices; every measurement `Approximate` with a nonzero bound |
| F2 frustum | circle `R = 10`, `h = 10`, `α = 10°` | `Volume = πh(R² + Rr + r²)/3`, `r = R − d`; one `Cone` wall with `HalfAngle = α` and no seam edge; lateral area `π(R + r)√(h² + d²)`; axial centroid `h(R² + 2Rr + 3r²) / (4(R² + Rr + r²))` |
| F3 slot | two lines `ℓ = 30` apart joined by semicircles `R = 5` (G1 joins), `h = 8`, `α = 3°` | `Volume = h[2ℓ(R − d/2) + π(R² − Rd + d²/3)]`; two `Plane` and two `Cone` walls; the four G1 rulings are `Line3` |
| F4 L-section | `[0,20]²` minus `[10,20]²`, `h = 10`, `α = 5°` | `Volume = h(300 − 40d + 4d²/3)` (the sharp reflex miter: the notch square keeps its side); the reflex far vertex at `(10 − d, 10 − d, h)` |
| F5 flare | F1 with `α = −5°` | `Volume = h(a² + 2ad + 4d²/3)`; far vertices at `±(a/2 + d)` |
| F6 against | F1 with `Against` | the far cap at `z = −h` and `capStart`; volume as F1 |
| F7 ring | annulus `R = 10`, `r = 4`, `h = 10`, `α = 5°` | `Volume = πh[R² − r² − d(R + r)]`: the hole widens; the hole wall is a `Cone` opening toward the far end, its rims concave |
| F8 far origin | F1 and F3 on a sketch plane whose origin is `(10⁶ + 0.1, 0.1, 0)` and under a rotation placement; F3 drawn at `v = 10⁶` in the plane | every vertex's published bound covers the exact rational lift (the `apitest/vertex_frame_origin_test.go` pattern); F1's volume and area and F3's volume bounds cover their closed forms; the slot at `v = 10⁶` covers its volume, its centroid and each straight wall's denoted normal |
| F9 placed equivalence | F1 built, then `Placed` under a translation and a rotation | volume, area and centroid equal the unplaced readings within bounds; `Bounds` covers the placed box |
| F10 tangent span | F1 with `a = 2` | each far corner `a/2 − d`, a Sterbenz subtraction exact for the held `d`, lies within its bound of the exact corner the stated taper denotes |

**Refusal fixtures**, one per SD row that `Extrude` can reach, each asserting
`errors.Is` on the sentinel and that `Document.Bodies()` is unchanged: SD2
(`90°`), SD3 (a spline wall), SD4 (a semicircular bite meeting its lines at
right angles), SD5 (F2 with `α` such that `d ≥ R`), SD6 (F1 with `d ≥ a/2`,
and F7 with `d` closing the ring), SD7 (a thin rectangle whose short walls'
miters cross), SD8 (a plate whose off-centre hole widens across the outer
wall), SD11 (`Symmetric`, `ToFace`, `ThroughAll`), SD12, SD13 (an angle of
`1e-14°` on a 1 mm sweep). SD15 is pinned in `internal/offset2d/sharp_test.go`
(§5). Beside them, the draft body's downstream refusals: `Fillet`, `Chamfer`
and `Shell` refuse it by name (DD14), and `MirroredCopy` builds it (DD12).

**`Draft` fixtures** (`apitest/draft_test.go`, `draft_internal_test.go`):

| Fixture | Asserts |
|---|---|
| D1 equivalence | `Draft(Walls(b), NeutralFace{b, Faces(FaceCreatedBy(CapStart(b)))}, 5°)` of F1's untapered box equals F1's body: same volume, area, centroid, vertex coordinates and face count, bit for bit |
| D2 other cap | the neutral face `CapEnd`: the far cap is `capStart`, at `z0`; volume as F1 |
| D3 negative | `−5°` widens: volume as F5 |
| D4 retire | the receiver is retired; the result is live; the document holds one body |
| D5 refusals | SD17, SD18, SD19 (zero faces, two faces, a cylindrical neutral face), SD20 (`NeutralFrame`; a wall as the neutral face), SD21 (one wall of four), SD22 (the other cap selected), SD23 (a revolve; a drafted body drafted again) |

**PR 2 fixtures**: F1–F7 tessellated at three tolerances, each mesh closed,
embedded, with `Mesh.Bound` covering the sagitta and the volume proof's
symmetric difference; `Union` of F1 with a straight prism through it; STEP of
F1 analytic (`analytic decad solid`) and of F2 faceted; `MassProperties` of
F1 against the closed form.

**PR 4 fixtures** (`apitest/draft_verify_test.go`): F1 under
`WithPullDirection(+e)` reads every wall clear and `Coverage` complete; under
`WithPullDirection(−e)` every wall is listed with `DiagUndercut`; F5 the
reverse; F3's `Cone` walls decided, not undecided; F7's hole wall clear along
`+e`, and a wider ring swept `Against` clear along its own `e`; F9's placed F1 and D1's `Draft` result read as F1 does along their own
sweep; a pull perpendicular to one wall's published normal leaves that wall
undecided.

**Fail-first legs**, each named in the test that owns it:

| Leg | Deleted | Goes red on |
|---|---|---|
| `farDelta` in the volume (`SweptVolumeAllow`) | the composition after the flux sum | F1, F6 and F8's placed F1 volumes |
| `dDelta` in the corner enclosures | the band's `dcDelta`, so the span is the point `d` | F10: every far corner reads `Exact` with a zero bound about `1.1e-16` from its exact point |
| the tangent enclosure | `RadTanSpan` replaced by the point `math.Tan` | `TestRadTanSpanEnclosesSeriesReference` against a 75-digit `big.Float` series reference |
| the closure sliver | `ClosureOf`'s result omitted | no fixture: F3's axis-aligned slot holds no sliver, and a slanted slot's slivers sit four orders below the far contour's volume term |
| the far cap's `sectionDisplacementArea` | omitted | F6's far cap area |
| the wall normal's denoted term (`capband.DenotedNormalAllow`) | omitted | F8's slot at `v = 10⁶`: a straight wall publishes a `2.2e-16` bound `3.3e-12` from its denoted normal |
| DD6's diameter arm | omitted | every F fixture under `Verify` (`DiagToleranceReferenceUnavailable`) |
| the mesh proof's chord term (`draftBandHeightUpper`) | zeroed | F2, F3 and F7: each mesh volume sits tens of mm³ outside a `volSymDiff` near `1e-11` |
| the mesh proof's vertex motion (`SweptVolumeAllow`) | omitted | F1, F4, F5 and F6: a zero `volSymDiff` against the far corners' rounding |
| DD7's dispatch arm (`draftUndercuts`) | removed from `runSurveys` | every PR 4 fixture: the survey reads the draft body as an unsupported payload |
| the wall reading's allowance (`capPatchNormalRange`'s third result) | zeroed in `capPatchUndercuts` | the tangent pull: the wall reads the sign of its own rounding and no `DiagUndecidedUndercut` is reported |

## 12. Do not do this

- **Build the taper as a `Loft` between `P` and `Q`.** A loft walls a circular
  pair with chorded flat triangles and a section displacement, mints
  `side(i, j, k)` triangle roles, refuses every modify op and reads
  `Suspect` on every survey. A drafted box must be a first-class body with
  `Plane` walls and `side(i, j)` roles that DX7 can read.
- **Offset reflex corners with the shell's arc.** That is a wall-thickness
  feature (modify §8); a draft keeps a reflex corner sharp (§2). F4 fails
  under the arc rule.
- **Integrate `A(z)` as a polynomial for the production volume.** The
  polynomial's coefficients are the far section's own readings, which carry
  the same displacement; the patch flux already exists, is proven, and is
  what the tessellation proof is reconciled against. The polynomial is the
  test's cross-check (§8).
- **Admit a near-tangent line–arc corner as G1 on a small cross product and
  then publish the wall as exact.** The dead-zone rule decides which closed
  form is built; the held foot's hull and the closure sliver charge what it
  cost. Deleting either is the first fail-first leg.
- **Hold a non-G1 circular corner as a ruled `Cone` with a bounded skew.**
  That is the cap-loop chamfer's stand-in (modify-reach §8.3), with its
  departure terms, its DX8 refusal and its conic ruling bound. It is a
  separate increment (§14) and not a shortcut in this one.
- **Build `Symmetric` or `TwoSided` by offsetting both ends from the sketch
  plane and gluing two bands at level 0.** The seam at level 0 is an edge
  between two walls of different lean and a `stackedPrismPayload` holds no
  tapered slab; it needs its own record (§14).
- **Ask `sketch.CreateOffset` for `Q`.** Lines only, a float solve, no
  certificate, and no collapse or crossing check (§3).
- **Let `Draft` take a `nil` selector meaning "every wall".** A nil selector
  with a default is the implicit-target guess core §4 rejects; `Walls(b)`
  spells the intent.
- **Decide `Draft`'s sweep direction from a pull vector argument.** The
  neutral plane and the sign of the angle determine the body; a second
  direction argument can only agree or conflict.
- **Read `tan α` from `math.Tan` and publish a zero bound on an axis-aligned
  box.** The enclosure has width; the box is `Approximate` (§8).

## 13. Decided questions

- **One payload for both entry points**: `draftPayload`, over the receiver's
  or the sketch's record. `Draft` is `Extrude` with a taper over an existing
  prism's section (§10.1), so the two share §7 entirely.
- **Order**: the tapered extrude ships first (it has no selection or neutral
  plane to resolve), tessellation second, `Draft` third (§14).
- **Sign**: positive narrows away from the sketch plane or neutral plane
  (§2). `Verify`'s pull survey then reads a positive draft as clear along
  the sweep.
- **Sharp corners**: the miter at every line–line corner, convex or reflex
  (§2, §12).
- **Circular corners**: G1 joins only in this design; the ruled stand-in for
  a mitered circular corner is deferred (SD4, §14).
- **Free-form walks**: refused (SD3); the chorded alternative is a loft and
  is rejected (§12).
- **Exactness**: every draft measurement is `Approximate` (§8).
- **Extents**: `Distance` only; the two-sided families need a two-slab draft
  record and are deferred (SD11, §14).
- **Neutral plane**: a cap of the receiver, through a `NeutralFace`; a
  `NeutralFrame` and an interior neutral level are deferred (SD20, §14).
- **Selection vocabulary**: `Walls(b)` added as a `FacePredicate` beside
  `CapStart`/`CapEnd`; no nil-selector form.
- **Surveys**: undercut decided in PR 4 off the patches' normal model; wall
  and concave radius staged (DD8).
- **Hand-offs**: none (§3).

## 14. PR split

Each PR ships code, tests and the documentation updates for every refusal it
lifts: this document's increment table, `doc.go`'s support map,
`docs/missing-features.md`, the function doc comments, and the layout rows of
every new root file. This document ships with PR 1.

| PR | Lands | Files and functions | Proves | After |
|---|---|---|---|---|
| **1** | `Extrude` + `WithTaper` over Table RD1: `offset2d.BuildSharpLoop`; `draftPayload` with `transform`/`placed`; `draft_build.go` (§7); `draft_moments.go` (§8 over `internal/capband`); `d` and its span (§8.1) with a certified tangent in `internal/proofbound`; `extrude.go` dispatches a nonzero taper to the draft build and refuses SD11/SD12; DD6's gate arm; DD12; the modify refusal messages naming the class; `Tessellate` refuses the class through its default | `internal/offset2d/sharp.go`, `internal/proofbound/interval_trig.go` (tangent), `draft_payload.go`, `draft_build.go`, `draft_moments.go`, `extrude.go`, `capblend*.go` (the band's `draft` view), `verify_gate.go`, `fillet.go`/`chamfer.go`/`shell.go` (messages), `apitest/extrude_taper_test.go`, `draft_build_internal_test.go`, `internal/offset2d/sharp_test.go`, `examples/decad_extrude_taper_example_test.go` | F1–F10 and the SD refusals of §11; the fail-first legs | — |
| **2** | tessellation and the mesh volume proof (DD1), which opens DD2, DD3, DD4, DD10, DD11, DD13, DD17 | `tessellate_draft.go`, `tessellate.go` (dispatch), `tessellate_capblend.go` and `capblend_admit.go` (§9.1's three differences), `draft_payload.go`/`draft_build.go` (`patches`, `bandDelta`), `boolean.go` (the admission arm), `mass_properties.go` (the mesh path needs no arm), `motion_bound.go`, `apitest/draft_mesh_test.go`, `tessellate_draft_internal_test.go` | the PR 2 fixtures | 1 |
| **3** | `Body.Draft` over RD2: `NeutralPlane`, `NeutralFace`, `NeutralFrame` (refusing), `DraftOption`, `Walls(b)`; §10.1's resolution; SD17–SD23; core §8's pointer to this document and core §12's signed-displacement list | `draft.go`, `selector.go` (`Walls`), `internal/selectorquery/predicate.go`, `docs/api-design.md` §8/§12, `apitest/draft_test.go`, `draft_internal_test.go`, `examples/decad_draft_example_test.go` | D1–D5 | 1 |
| **4** | DD7: the undercut survey over draft bodies | `draft_survey.go`, `survey.go` (dispatch), `capblend_survey.go` (`capPatchUndercuts`, the patch loop both surveys run), `apitest/draft_verify_test.go` | the PR 4 fixtures; the `−e` pull lists every wall | 2 |
| **5** | RD3: the subset draft (§10.2): per-walk amounts in `BuildSharpLoop`, the mixed-corner rule, SD21 narrowed | `internal/offset2d/sharp.go`, `draft.go`, `draft_build.go`, tests | one wall of F1's box drafted: `A(z) = a(a − z·tan α)`, so `Volume = h·a(a − d/2)`; the L with one notch wall drafted; a hole drafted alone (F7's cone with vertical outer walls) | 3 |

PRs 2 and 3 run in parallel after PR 1; PR 4 follows 2; PR 5 follows 3.
PRs 1, 2 and 4 are proof specifications (bounds, closure terms, the mesh
proof and the normal model) and are implemented at the higher reasoning
tier; PRs 3 and 5 are file-by-file plans over PR 1's builder.

Increment table — what still refuses after each PR. PR 2's mesh serves every
draft body, `Draft`'s included:

| After | State | Still refused |
|---|---|---|
| 1 | landed | every draft body's tessellation, boolean, export, mass and interference reading (DD1's dependants); `Body.Draft` (no entry point); surveys `Suspect`; SD3, SD4, SD11, SD12 |
| 2 | landed | surveys `Suspect`; SD3, SD4, SD11, SD12, SD20, SD21; booleans, interference and mass of a body the band admission refuses (§9.1) |
| 3 | landed | surveys `Suspect`; SD3, SD4, SD11, SD12, SD20, SD21 |
| 4 | landed | wall and concave-radius surveys `Suspect` (DD8); SD3, SD4, SD11, SD12, SD20, SD21 |
| 5 | planned | DD8; SD3, SD4, SD11, SD12, SD20; clearance carriers (DD9); shell, fillet and chamfer of a draft body (DD14); the mitered circular corner; the two-sided extents; `NeutralFrame` and the interior neutral level; the surface result |

The unscheduled reach, in the order a later design should take it: the
drafted shell (DD14, the molded cup), the two-sided extents and the interior
neutral level (one two-slab draft record serves both), the mitered circular
corner as modify-reach §8.3's ruled stand-in, the wall and concave-radius
surveys (DD8), and the clearance carriers (DD9).
