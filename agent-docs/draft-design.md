# Draft design

How decad builds a drafted body: a tapered extrude (`Extrude` with a nonzero
`WithTaper`) and a face draft of an existing prism (`Draft`). Both
produce a `draftPayload` for one-sided extents or a `twoSidedDraftPayload`
for extents that cross the sketch plane. Their walls lean at a stated angle
to the sweep axis so that a molded part releases along that axis.

Companion to `agent-docs/api-design.md` ("core §N"), `agent-docs/evaluator-design.md`
§5 (the prism this body generalises), `agent-docs/modify-design.md` §5 and §8 (the
exact section offset and its audit, which this design reuses) and
`agent-docs/modify-reach-design.md` §8.3–§8.4 (the cap-loop chamfer band, whose
patch, flux and bound machinery this body is built from). Nothing here changes
those contracts.

Four tables are normative and each states its facts once:

| Table | States | Section |
|---|---|---|
| **RD** — the inputs | which profile, extent, operand and selection each entry point admits | §4 |
| **SD** — the refusals | every refusal, modify §1's existence test, its sentinel, and the gate order | §5 |
| **BD** — the result | the payload, its faces, edges and roles | §6 |
| **DD** — downstream | one row per consumer and what it does with a draft body | §9 |

Question router (navigation, not authority):

| Question | Section |
|---|---|
| What is a draft, and which offset family defines it | §1, §2 |
| Why the far section is decad's own geometry, not a `sketch` answer | §3 |
| What `WithTaper` admits | §4 Table RD |
| What `Draft` admits, and its signature | §4, §10 |
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
build today and `agent-docs/missing-features.md` ("Feature operations") lists the
rest.

The evaluator already builds exactly one drafted surface family: the cap-loop
chamfer band of modify-reach §8.3, which joins a cap loop at one level to its
exact offset at another level with `Plane` and `Cone` patches, and proves
every reading off it. A tapered extrude is that band with the operand's
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

For a two-sided extent, each side starts at the same sketch section `P` and
uses its own travel direction away from the sketch plane. The negative slab
occupies `[z0, 0]`; the positive slab occupies `[0, z1]`. Each far section
uses that side's distance in `d = h · tan α`. Their common section is one
sewn rim, not an internal cap.

**The sign.** A positive `α` narrows the body with distance from the sketch
plane: every wall's carrier moves `t(z)` into the material. A negative `α`
widens it. `WithTaper` keeps its existing meaning (core §12: a signed
displacement, outside `ErrNegativeMagnitude`), and `Draft` (§10) reads
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
| **RD1** | `Extrude` with nonzero `WithTaper` | analytic walks with G1 circular joins; `Distance`, `ToFace`, `ThroughAll`, `Symmetric` or `TwoSided`; `|α| < 90°`; no `WithSurfaceResult` | a one- or two-slab draft solid (Table BD) |
| **RD2** | `Draft` | a live `prismPayload` operand (an extrude, a filleted or chamfered body, a tube, or any of these after `Transform`) whose section carries no displacement; a `NeutralFace` naming one cap of the operand; a selection resolving to exactly the operand's complete wall set; `0 < |α| < 90°` | a `draftPayload` solid over the operand's section, the operand retired |
| **RD3** | `Draft` | as RD2 with a selection naming a **subset** of the walls, where every corner between a selected and an unselected wall is a line–line corner | a `draftPayload` whose far section moves the selected walls only (§10.2) |

Everything else is Table SD.

## 5. Table SD — the refusals

Each row states what was asked, modify §1's test (does the body exist under
any evaluator?) and the sentinel that follows from it.

| SD | The call asked for | Exists? | Sentinel |
|---|---|---|---|
| **SD1** | a taper that is not an angle, or not representable in radians | — | `ErrUnitKind` / `ErrNotFinite`, as for a straight prism |
| **SD2** | `|α| ≥ 90°`, or an angle whose certified cosine enclosure (`RadSinCosInterval`) reaches zero | no — a wall at a right angle to the axis sweeps nothing | `ErrDegenerate` |
| **SD3** | a free-form walk in the section | yes | `ErrUnsupported` — the offset of a Bézier span is not a recorded kind; §12 names the chorded alternative and rejects it |
| **SD4** | a corner where a circular walk meets its neighbour that the held-tangent rule classifies as a miter (not G1), or, in a subset draft, a G1 corner at a circular walk whose two walks move by different amounts (§10.2) | yes | `ErrUnsupported` — the junction is a conic; the ruled stand-in of modify-reach §8.3 is not scheduled here (§14) |
| **SD5** | a circular walk whose far radius collapses: `offset2d.OffsetRadius` reports `ok == false` (`R ≤ d` for a round with the material inside under a positive taper; a hole wall under a negative one) | yes — the cone's apex lies inside the sweep and the body past it has another topology | `ErrUnsupported` (modify S11a) |
| **SD6** | a far outer loop whose signed area has changed sign (modify S8), whose every walk the offset consumes (`offset2d.ErrLoopConsumed`) under a positive taper, or a far hole the audit decides lies outside the far outer loop (modify S9's decided row) | no — the taper consumed the region | `ErrDegenerate` |
| **SD7** | a far walk `offset2d.WalkConsumed` reports consumed — a line whose two miters have crossed — where some walk of its loop survives, or a hole every walk of which is consumed | yes | `ErrUnsupported` (modify S11a) |
| **SD8** | far loops that cross or make boundary contact at modify §5's scale-anchored floor (S7/S11b) — two walls closing on each other, a widened hole reaching the outer loop | yes | `ErrUnsupported` |
| **SD9** | far loops whose nesting the containment classifier cannot decide (modify S9) | undecidable here | `ErrUnsupported` |
| **SD10** | a far section that passes every audit at the held `d` and fails at the top of `d`'s span (§8.1; modify-reach §8.3.1's second audit) | undecidable here | `ErrUnsupported` |
| **SD11** | retired for `Symmetric` and `TwoSided`; each side now has its own draft slab | — | — |
| **SD12** | a nonzero taper with `WithSurfaceResult` | yes | `ErrUnsupported` (staged, §14) |
| **SD13** | a `d` that rounds to zero, a far vertex bit-identical to its near vertex, or a far radius bit-identical to its near radius (`capband.BandRadius`) | yes — float64 cannot name the far section at this scale | `ErrUnsupported` (modify-reach SX13) |
| **SD14** | a far corner whose displacement enclosure cannot be built (`offset2d.ErrUnbounded`) | yes | `ErrUnsupported` (SX14) |
| **SD15** | a corner whose two moved carriers have no intersection — a cusp (`|cross| ≤ tol`, `dot ≤ 0`) between two lines | yes | `ErrUnsupported` (`offset2d.ErrTopology`, modify S11's row) |
| **SD16** | a wall patch whose outward orientation `capblend.FixPatchOrientation` cannot certify | yes | `ErrUnsupported` (SX15) |
| **SD17** | `Draft` of a retired operand, or of a sheet | — | `ErrRetiredBody`; a sheet as `Fillet`'s `refuseSheetOperand` refuses it |
| **SD18** | `Draft` with a zero angle | it exists and is the operand (modify S13) | `ErrDegenerate` |
| **SD19** | a neutral selector resolving to zero or several faces; a curved neutral face; a `Faceted` neutral face | — | `ErrCardinality` (Expected "exactly 1"); `ErrDegenerate`; `ErrUnsupported` — `MirrorFace`'s own three rules |
| **SD20** | a `NeutralFrame`, or a `NeutralFace` that is not a cap of the operand | yes | `ErrUnsupported` (staged, §14) |
| **SD21** | a selected face that is no wall of the operand, or a selection that splits the recorded segments of one coalesced walk; an empty selection | yes | `ErrUnsupported`; `ErrNoMatch`/`ErrCardinality` as core §9 |
| **SD22** | a selected face parallel to the neutral plane — the other cap — or the neutral face itself | no — a face with no trace on the neutral plane has no line to tilt about | `ErrDegenerate` |
| **SD23** | `Draft` of an operand that is not a `prismPayload`, or whose section carries a displacement (`prismrecord.Payload.RequireExactSection`), or that is itself a draft body | yes | `ErrUnsupported` (modify S3; a second draft is SX10's composition rule) |

The sharp offset of a convex section past its collapse is the section
point-reflected through a centre, which keeps its walk sense, so S8 never sees
a square tapered past its apex; stage 4 reads that outer loop, every walk
consumed, as SD6. Two concentric far circles never cross: past the closure of
F7's ring the far hole lies outside the far outer loop, which is SD6 through
the audit's decided nesting row, not SD8. No recorded profile holds a cusp
between two lines, and a cusp at a circular walk is SD4 first, so Extrude never
reaches SD15; `internal/offset2d/sharp_test.go` pins it.

**A chain section never reaches this build.** `Extrude` over an open
`*sketch.Chain` returns `ErrUnsupported` for `WithTaper` at any angle, zero
included, after the chain's seam gates and before extent resolution
(`agent-docs/surface-design.md` §13.2), so none of Table SD applies to a chain.
`Extrude` threads its `ctx` through the extent resolution and both slab builds
of a profile with a nonzero taper; a canceled `ctx` returns `ctx.Err()` before
commit, unwrapped by the stage names the two-sided build adds to other errors.

**Gate order.** Each stage needs the one before it, and where two gates read
one constructed section the existence question is asked first (modify §4).

| Stage | Gates, in order |
|---|---|
| 1 — the pre-gates | `Extrude`: the seam gates of core §7, then SD1, SD2 (the seam gates run first, as they do for SD1 on a straight prism). `Draft`: SD1, SD2, SD17, SD18, SD19's cardinality |
| 2 — the operand and its inputs | `Draft`: SD23, SD20's `NeutralFrame` refusal, SD19's kind rules, SD20's cap test, SD22, SD21. `Extrude`: the extent's own validation (unchanged), SD12 |
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
| **BD2** | `Draft` (RD2) | the same payload over the operand's record, under the operand's placement | 1 | as BD1, every role minted under the result's own producer (modify §11: a result's roles are its own) | as BD1 |
| **BD3** | `Draft` (RD3) | the same payload; `Q` moves the selected walls only | 1 | as BD2; an unselected wall keeps its vertical `Plane`/`Cylinder` | as BD1 |
| **BD4** | two-sided `Extrude` + taper | `twoSidedDraftPayload` with a negative and positive `draftPayload` | 1 | outer `capStart` and `capEnd`; walls `side(0,i,j)` below and `side(1,i,j)` above the sketch plane | one shared middle rim; the two slabs have no published middle cap |

A full-circle loop builds one `Cone` with two `Circle3` rims and no seam edge,
as a cylinder does today. Adjacent `Plane` walls whose two recorded segments
lie on one exact line canonicalize into one face by evaluator §3's rule, which
reads the record and not the surface.

## 7. Construction

`draftrecord.Build` (`internal/draftrecord/build.go`) builds the body from the
payload over topology nodes. The steps, in order:

1. **Resolve the walks.** `boundarywalk.WalkOf`, `RequireAnalyticWalk` (SD3)
   and `CoalesceWalksBudget` over every loop of `P`, as
   `capblend.OneLoopCornerLoop` does.
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
5. **Audit `Q`** with `offset2d.AuditOffsetSectionBudget` (modify §5): SD6, SD8, SD9.
   Re-run steps 4–5 at the top of `d`'s span (`capblend.AuditSetbackSpan`'s
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
   `capblend.PlaneFromThree` or `coneSurface(pl, cu, cv, R,
   R ∓ d, zNear, zFar)`; the rims are `arcEdge` / `wholeCircleEdge` / line
   edges; the rulings are `Line3` between corresponding corners; the
   orientation is certified by `capblend.FixPatchOrientation` (SD16). Each patch
   carries `ContourAllow = farDelta`, `LevelDelta` = the far end's axial
   displacement, and its `HeldAllow` as modify-reach §8.4's table states.
9. **Stamp roles and convexity**: `side(i, j)` per walk, `capStart`/`capEnd`,
   lateral edges convex by the walk's turn.
10. **Measure** (§8), then commit.

Steps 7–9 run through the cap-loop band itself. `draftPayload.Band()` is a
`capBlendPayload` view with every loop chamfered on the far cap, `dc = d`
across the cap and `ds = h` down the sweep, so the band's side level is the
sketch plane and no straight slab remains. Its `draft` flag makes
`capblend.BuildBand` (`internal/capblend/band.go`) take the sharp joins
(`offset2d.SharpJoinsBudget`), stamp `side(i, j)` roles, give rulings and far
rims the prism's convexity, and flip the axial half of the orientation
reference for a negative taper. The near rim is `draftrecord`'s `nearRim`: the bottom
rim `prismrecord.BuildLoopSidesAs` builds for a prism's wall, at the sketch plane.

For a crossing extent, `draftrecord.BuildTwoSided` builds the negative and positive
slabs independently with steps 1–9. It checks that their held sketch rims
match vertex for vertex, then removes both middle caps and reuses the same
rim edges and vertices. The negative slab's published face loops are reversed
so the two wall faces use each sewn edge in opposite directions. The
composite sweep boundary and adjacent-span audits check the resulting shell.
The two slab volumes and first moments add; the body area sums its published
faces, and its bounds enclose both slabs.

`Draft` reaches step 1 with the operand's record and placement after its
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
operand slab.

| Reading | Closed form | Owner of the code |
|---|---|---|
| Volume | the divergence theorem in plane-local coordinates: the near disk's area at its level, the far disk's area at its level, and `Σ capband.RawFlux(patch)`, divided by 3 (`capband.BandVolume`). A Plane patch's flux is the exact rational tetrahedron identity; a Cone patch's is the closed-form polynomial-plus-trig flux over rationals with certified `sin`/`cos` enclosures | `internal/capband/band_mass.go` |
| Area | the two disks' region areas plus `Σ capband.AreaOf(patch)`: a Plane patch's two-triangle sum, a Cone patch's frustum sector `(αc/2)(R0 + R1)·L` with `L = √(ΔR² + H²)` | `internal/capband/area.go` |
| Centroid | the first moment by the same divergence theorem (`capband.BandMoment`), divided by the volume and lifted to world through the frame and placement; the geometric safety-net bound (the centroid lies within `Bounds`) as a ceiling | `internal/capband/band_mass.go`, `internal/capband/moment.go` |
| `Bounds` | per direction the extreme over both records' analytic extremes at their levels, each charged with its own displacement and the lift rounding; a placed body charges the placement's rounding as evaluator §5 states | `internal/prismrecord/extent.go`'s readers over both records |

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
`setPatchReadings` (`internal/capblend/band.go`) adds it to every draft wall and chamfer
patch alike. `δ_z` is the far level's displacement beside the near level's
rounding, and no near-level displacement enters. Every admitted one-sided
extent puts its near end on the sketch plane at exactly zero with a zero
displacement; the far end carries the distance's conversion rounding or the
stop level's bound. The near edge is therefore the recorded section itself.
The far displacement stays in `δ_z` even though it would cancel
for a chamfer, whose two levels both follow one held cap level: here
`h = z1 − z0` is read from the held far level, so that displacement moves the
height the taper is measured over. Each two-sided slab also puts its near
level at the exact sketch plane; its far level carries its own displacement.

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

| DD | Consumer | What it does with a draft body | PR |
|---|---|---|---|
| **DD1** | `Body.Tessellate` | chords both records once with the shared curve samples (tessellation §3), rules each wall between its two rims with the cap-band strip emitter (`capblend.EmitBand` in `internal/capblend/mesh_band.go`, over `internal/tessellation`'s rings), triangulates both caps; manifold by construction; the volume proof is the band admission of `agent-docs/tessellation-reach-design.md` §7 with no slab (§9.1). A band the admission refuses meshes for export with no volume proof | 2 |
| **DD2** | `Union`/`Cut`/`Intersect` | the mesh path (evaluator §9) over DD1's mesh, admitted by `booloperand.RequireVolumeProof` through the same band admission; the analytic prism reduction's entry gate (`internal/prismboolean`) does not admit the class and takes the mesh path | 2 |
| **DD3** | STL / OBJ / 3MF | the mesh of DD1 | 2 |
| **DD4** | STEP | the analytic writer when every face is a `Plane` — a drafted polygon writes `ADVANCED_FACE`s over tilted planes with `Line3` loops, which `supportsAnalyticSTEP` admits today; any `Cone` sends the whole body to the faceted writer, as `agent-docs/missing-features.md`'s STEP row states for cones | 2 |
| **DD5** | `Verify` validity | built by construction after the §5 audit, as a prism is; `Built` and `Solid` read true | 1 |
| **DD6** | `Verify` tolerance reference | `pairkernel.BodyGateDiameter` gains an arm (`draftGateDiameter`): the lower-bound diameter over wall stations on the near section at the near level and on the recorded far section at the far level, each charged its gap plus the level displacement and `farDelta`, falling back to the held vertices with their bounds (verification §3). Without it every draft body would read `DiagToleranceReferenceUnavailable`, since its bounds are never zero | 1 |
| **DD7** | `Verify` undercut survey (`WithPullDirection`) | `internal/capblendsurvey` reads built face normals and decides each wall and cap as described below. | 4 |
| **DD8** | `Verify` wall and concave-radius surveys | `Suspect` with `DiagUnsupportedSurveyPayload` (staged). The reduction a later PR takes: the thinnest wall between two drafted skins is the far section's 2D reading scaled by `cos α`, and the smallest concave radius of a hole wall is its radius at the narrower end; neither is scheduled here | — |
| **DD9** | `Verify` clearance (`WithClearances`) | `Suspect` (`reportvocab.PairUndecided`): `newBodyGeomBudget` has no carrier model for the class. A later PR adds the Plane/Cone carriers; not scheduled | — |
| **DD10** | `Verify` interference | box-disjoint proofs from the body's `Bounds`; the read-only mesh intersection over DD1's mesh | 2 |
| **DD11** | `MassProperties` and `dynamics` | the verified-mesh path (`verifiedMeshMassProperties`) over DD1's mesh; `ErrUnsupported` where the band admission refuses | 2 |
| **DD12** | `Transform`, `Copy`, `Mirrored`, `MirroredCopy` | the payload implements `Transform()` and `Placed()`: a placement re-runs §7 under the composed motion, as a prism's does. `WithJoin` refuses (it rewrites a prism's or stacked prism's section) | 1 |
| **DD13** | `Patterned` | `placedInstance` and a boolean `Union` (the mesh path); the frame-keeping arm does not take the class | 2 |
| **DD14** | `Fillet`, `Chamfer`, `Shell` | A positive-taper, one-sided, full circular solid admits one complete far-cap outer rim fillet (§10.4), symmetric chamfer (§10.5), or inward shell with that cap removed (§10.6). Other selections and draft shapes remain modify S3, `ErrUnsupported` | — |
| **DD15** | `Draft` of a draft body | SD23 | — |
| **DD16** | `Thicken`, `Offset`, `Patch`, `Stitch`, `Trim`/`Extend`/`Split` | not reached: a draft body is a solid (SD12 refuses the sheet form) | — |
| **DD17** | Motion and linkage bounds | `motion_bound.go` reads near and widened far section bounds. `internal/motionbound/record_radius.go` reads each segment's carrier and walk. | 2 |
| **DD18** | Selectors | `Planar()`, `FaceCreatedBy`, `CapStart`/`CapEnd` and the new `Walls(b)` (§10) select as on a prism. `Facing(v)` and `NormalTo(v)` match a drafted `Plane` wall only for its own tilted normal, since both require parallelism; a caller naming a drafted wall by direction passes that normal, or selects by role | 1, 3 |

DD7's `capblendsurvey.DraftUndercuts` reads each built wall's `*topo.Face`
through `topobuild.NormalAt`. `capblendsurvey.PatchNormalRange` encloses its
normal component along the pull, and `capblendsurvey.PatchUndercuts` shares
that reading with cap-blend bodies (DX7). `survey2d.CapNormalDecision` reads
each cap; `survey2d.DecidePull` lists, clears or leaves a face undecided under
modify-reach §8.3's rule. A wall drafted at `α > 0` against a pull along `e`
reads `sin α > 0` and clears. A pull in a wall's tangent plane leaves it
undecided because the reading includes the taper's enclosure.

The two-sided payload composes DD1–DD6, DD10–DD13 and DD17 from its two
slabs. Its undercut survey is still `Suspect`: DD7's current face-role lookup
handles one slab only. Wall and concave-radius surveys remain staged for both
payloads (DD8).

### 9.1 The mesh

`draftrecord.Tessellate` hands `draftPayload.Band()` to the cap-loop chamfer
tessellator with the payload's own wall geometry and far contour displacement
(`patches`, `bandDelta`, filled by the build), so the mesh charges the numbers
every reading of the body charged. Three things differ from a chamfer's mesh:

- no trimmed side wall is emitted, and the near cap's rim is written once and
  read by both the near cap and the walls (`draftNearRing`);
- a wall patch carries `side(i, j)` and the band has no apex patch, since the
  corners are read by the sharp rule (`capBlendPayload.offsetJoins`), which the
  admission (`capblend.OccupiedVolumeAdmission`) reads too: a reflex line-line
  corner is a miter, affine in the amount, and is admitted;
- the slice-wise chord term integrates over the whole sweep, `|z1 − z0|` with
  both ends' displacements (`draftBandHeightUpper`), not over a side setback.

A wall face's bound charges the far level's displacement and the band's, as a
chamfer patch's does, and no near-level term: the near end of every admitted
one-sided extent is the sketch plane, exact (§8). Each slab of a two-sided
extent has the same exact near level and its own bounded far level.

`draftrecord.TessellateTwoSided` meshes both slabs through that same
tessellator, drops the two middle caps, and identifies vertices with equal
held coordinates. The closed-mesh audit refuses a mismatch in the two middle
rings. The outer facets retain their source faces and displacement bounds;
area slack and occupied-volume difference bounds add over the two slabs.

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
`agent-docs/tessellation-reach-design.md` §9's open question, a separate increment.

## 10. `Draft`

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

func Draft(ctx context.Context, b *Body, sel FaceSelector, neutral NeutralPlane, angle units.Value, opts ...DraftOption) (*Body, error)

// Walls matches every face a sweep made: the side(i, j) roles of b's own
// producer. It is the sibling of CapStart and CapEnd.
func Walls(b *Body) FacePredicate
```

`Draft` is a modify operation on core §8's terms: it takes a context, resolves
`sel` against the live operand, returns a new body and retires the operand,
and leaves the document unchanged on any error, including `ctx.Err()`.
`angle` is a signed `units.Value` of `Kind` Angle (core §12 lists it with
`ToFace.Offset` and `WithTaper` as the signed displacements outside
`ErrNegativeMagnitude`); a positive angle narrows the body with distance from
the neutral plane. `DraftOption` is a sealed option tier with no option in
this design; it exists so a later increment can add one without changing the
signature. It includes `option.Interface` in its method set for that later
option.

### 10.1 Resolution (RD2)

1. The neutral face resolves through `SelectFaces(neutral.Body)` under the
   implicit exactly-one rule (SD19). It must be `capStart` or `capEnd` of the
   operand (SD20).
2. `sel` resolves against the operand. The resolved set must not contain a
   cap (SD22), and every face of it must carry a `side(i, j)` role (SD21). The
   complete set of such faces is RD2; any other set is RD3 (§10.2).
3. The sweep direction `e` runs from the neutral cap toward the other cap.
   With the neutral cap at `z0`, `e` is the frame normal and the far end is
   `z1`; with the neutral cap at `z1`, `e` is its negation and the far end is
   `z0`. The section `P` is the operand's record, which lies in the frame's
   plane at level 0 and is the section of every level of a straight prism, so
   the neutral cap's section is `P` whichever cap is named.
4. `h` is the operand's height with its end displacements; `d = h · tan α`
   per §8.1.
5. §7 builds the body over `P`, `Q` and `[z0, z1]`; the far record sits at
   the far end. The result's roles are minted under the new producer.

A `NeutralFace` whose `Body` is not the operand passes SD19's cardinality and
kind rules on that body's face and is then SD20, so a curved neutral face is
`ErrDegenerate` wherever it lies. `Walls(b)` is a face predicate over the
`side(i, j)` roles of `b`'s own producer (`selectorquery.WallsKind`), and the
complete wall set is the operand's faces carrying such a role, fillet and
chamfer rounds included.

The body `Draft` builds equals the body `Extrude` would build from the same
sketch with `WithTaper(angle)` and a `Distance` extent toward the far end,
bit for bit when the operand is unplaced. §11 asserts it.

### 10.2 The subset case (RD3)

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

The payload records the unselected walls as `draftPayload.Kept`, the
`(loop, segment)` pairs read from their `side(i, j)` roles, nil when every
wall is selected; a placement re-runs §7 over the same set (DD12). Decided
details, each the reading the rule above implies:

- **Kept amounts carry no span.** A kept wall's amount is exactly zero, so its
  carrier enters the corner and contour proofs (`capcontour.AmountsDisplacement`,
  `capband.AmountsContourDisplacement`) as a point interval, while a moved
  walk keeps `d`'s span (§8.1). A kept circular wall's far radius is its own
  (`capband.WallRadius`), not `BandRadius`'s offset.
- **A corner between two kept walls stays put.** Its far point is the recorded
  corner itself, never a re-solved intersection of the two unmoved carriers;
  the contour proof still encloses the carriers' exact intersection and charges
  the gap. SD13 reads only corners and radii a moved walk touches.
- **The profile class is RD1's.** A non-G1 corner at a circular walk is SD4
  even when both of its walks are kept: the rule above admits a mixed corner
  only between lines and narrows nothing else.
- **Unequal amounts inside the G1 dead zone.** Two nearly collinear lines with
  different amounts take the miter, as the rule says; where their moved
  carriers do not meet the corner is SD15.
- **A walk is one wall.** The amounts are per coalesced walk. A selection that
  names some recorded segments of one walk and not others names no single wall
  to tilt and is SD21; a face built from one walk carries every segment's role,
  so a face selection never splits one.
- **Gate order.** The selection is read at stage 2 (SD22, SD21); the mixed
  corners are read with SD4 at stage 3, at a unit amount per moved walk.
- **The mesh (DD1).** The band tessellator reads each walk's own setback
  where it read the one setback: a kept circular wall's cap radius, its
  band patch's radius rounding, and its cap samples' motion against the
  recorded circle with no span. Nothing else in §9.1's proof changes, since a
  mixed line–line miter is still a corner locus affine in the amount.
- **The undercut survey (DD7).** A kept wall's patch is parallel to the
  sweep, so its normal component along a pull along `±e` is zero within its
  bound and the survey leaves it undecided; a moved wall reads as in a full
  draft.

### 10.4 One circular far-rim fillet (DD14)

`Fillet` accepts the one complete far cap rim of a one-sided inward draft
whose source is a single full circular outer loop with no holes or kept walls.
The source sketch frame must have exact signed-axis basis vectors. The draft
payload authenticates the near circle, sweep levels, taper and far contour.
An arbitrary rigid placement of that body is retained by the result.

For near radius `R`, height `h`, inward amount `d`, slope `k = d/h`, far radius
`r = R−d`, and fillet radius `f`, the rounded meridian's arc centre is
`(z,ρ) = (h−f, r+kf−f√(1+k²))`. Its top contact is directly above the
centre by `f`; its side contact is
`(h−f+fk/√(1+k²), r+kf−f√(1+k²)+f/√(1+k²))`.
The five-segment meridian closes through the rotation axis and revolves a
top disk, toroidal fillet, conical side and bottom disk. The full-revolution
evaluator supplies topology, mass readings and occupied-volume proof. The
top and bottom disks also carry the draft-style `capEnd` and `capStart`
roles; the torus carries `fillet(0,2)`.

Bounded arithmetic encloses the stated radius, height, inward amount and
their derived centre and contact coordinates. The result's `sectionDelta`
charges the largest coordinate and frame-origin displacement by a factor of
64. With that charge below `f/16`, the per-coordinate error is below
`f/1024`. An endpoint ray changes by at most `2√2` times that error; the
resulting angle changes by less than four times the error divided by `f`.
Interpolating the unwrapped arc sweep of at most a quarter turn moves every
arc point by less than 16 times the error. Positive interval
clearances keep the arc outside the axis, on the cone, above the near cap and
inside the original near rim. Unproved clearances, other cap selections,
partial circles, holes, reverse taper and non-axis sketch frames remain
outside this bounded route.

### 10.5 One circular far-rim chamfer (DD14)

`Chamfer` accepts the same source and rim as §10.4, with equal setbacks along
the cap plane and conical wall. `WithAsymmetricChamfer` remains outside this
route. The draft record and exact signed-axis sketch frame authenticate the
source. The result keeps its placement and axial cap roles.

For near radius `R`, height `h`, inward amount `d`, slope `k = d/h`, far radius
`r = R−d`, bevel setback `c`, and `n = √(1+k²)`, the cap contact is
`(z,ρ) = (h,r−c)`. The side contact is `(h−c/n,r+kc/n)`: its distance along
the slanted wall from the original rim is `c`. The meridian closes through
the axis with these two contacts, the near rim and two axial segments. Its
full revolution builds the bottom and top disks and the original and bevel
cones. The bevel cone carries `chamfer(0,2)`.

The operation requires positive interval clearance for `r−c`, `h−c/n` and
the side contact below the far level and inside the near rim. Bounded
arithmetic encloses source dimensions and both contacts. The largest
coordinate and frame-origin displacement is charged to the whole meridian's
`sectionDelta`; a line between held endpoints stays within their maximum
displacement. `evalRevolve` then supplies bounded mass and occupied-volume
proof. The removed volume has the independent closed form
`πc²/n · [r + c(k/n−1)/3]`. The far circle survives as a smaller cap rim,
and the new cone meets the source cone at one whole circular junction.

### 10.6 One circular drafted cup (DD14)

`Shell` admits the same positive-taper full-circle source as §10.4 when the
one selected opening is its far cap and the sense is inward. The source must
have one near-start draft slab, no hole or kept wall, and an exact signed-axis
sketch frame. `WithNoOpenings`, outward shelling, another cap, a partial
circle and an additional region remain S3.

Let the source's near radius be `R`, height `h`, inward draft amount `d`,
`k = d/h`, and shell thickness `t`. The outer cone has radius `R−kz`.
The cavity floor is at `z=t`; its conical wall has radius
`R−kz−t√(1+k²)`. Its top opening therefore has radius
`R−d−t√(1+k²)`. The build requires positive interval lower bounds for
`h−t`, that opening radius, and the offset. A six-line meridian records the
axis segment below the floor, floor, cavity cone, top annulus, outer cone and
kept near cap. Its full revolution has one shell and five faces.

The near circle, far offset interval, far-contour displacement and source
frame come from the authenticated `draftPayload`. Bounded arithmetic encloses
the floor and both cone endpoints. The largest endpoint, frame-lift and
far-contour displacement receives a 64-fold whole-section charge; the
gate also requires that charge to be less than one sixteenth of the floor,
cavity and rim clearances. Each denoted meridian segment pairs with a held
line of the same kind by its affine parameter, so its interior displacement
does not exceed its endpoint displacement. `evalRevolve` applies its mass,
face, placement and occupied-volume proofs to the charged meridian. The
result adds `capStart`, `rim(0)`, `shellCap` and `shellSide(0,0)` roles to
its own revolved faces; placement re-mints them.

The independent remaining-volume formula is the difference of two conical
frusta. With `r=R−d`, `c0=R−kt−t√(1+k²)`, and
`c1=r−t√(1+k²)`, it is
`π/3·[h(R²+Rr+r²)−(h−t)(c0²+c0c1+c1²)]`.

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
`internal/draftrecord/build_test.go`):

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
| F9 placed equivalence | F1 built, then `Transform` under a translation and a rotation | volume, area and centroid equal the unplaced readings within bounds; `Bounds` covers the placed box |
| F10 tangent span | F1 with `a = 2` | each far corner `a/2 − d`, a Sterbenz subtraction exact for the held `d`, lies within its bound of the exact corner the stated taper denotes |
| F11 stop extents | F1's section, with a planar stop at 30 mm and a 5 mm thick stop body | `ToFace` ends at +30 or −30 mm; `ThroughAll` ends at +35 or −35 mm; an offset in inches propagates its rounding; the far corner and `Volume = h(a² − 2ad + 4d²/3)` lie within their bounds |

**Refusal fixtures**, one per SD row that `Extrude` can reach, each asserting
`errors.Is` on the sentinel and that `Document.Bodies()` is unchanged: SD2
(`90°`), SD3 (a spline wall), SD4 (a semicircular bite meeting its lines at
right angles), SD5 (F2 with `α` such that `d ≥ R`), SD6 (F1 with `d ≥ a/2`,
and F7 with `d` closing the ring), SD7 (a thin rectangle whose short walls'
miters cross), SD8 (a plate whose off-centre hole widens across the outer
wall), SD12, SD13 (an angle of
`1e-14°` on a 1 mm sweep). SD15 is pinned in `internal/offset2d/sharp_test.go`
(§5). Beside them, a draft body's requests outside §10.4–§10.6 refuse it by
name (DD14), and `MirroredCopy` builds it (DD12).

**Circular shell fixtures** (`apitest/draft_circle_shell_test.go`): a
Sketch-produced radius-8 circle drafted at `5°` over 10 mm has its far cap
removed inward at `t=1`. Its one-shell result has a kept cap, top rim,
cavity cone and floor with their own roles. Its bounded volume covers
§10.6's frustum difference; its 0.1 mm `VerifyAll` mesh proves its boundary
and occupied volume. Placement preserves those roles and the volume.
Thickness that consumes the opening, a near-cap selection, outward sense,
negative taper and a holed section refuse.

**`Draft` fixtures** (`apitest/draft_test.go`):

| Fixture | Asserts |
|---|---|
| D1 equivalence | `Draft(ctx, b, Faces(Walls(b)), NeutralFace{b, Faces(FaceCreatedBy(CapStart(b)))}, 5°)` of F1's untapered box equals F1's body: same volume, area, centroid, vertex coordinates and face count, bit for bit |
| D2 other cap | the neutral face `CapEnd`: the far cap is `capStart`, at `z0`; volume as F1 |
| D3 negative | `−5°` widens: volume as F5 |
| D4 retire | the operand is retired; the result is live; the document holds one body |
| D5 refusals | SD17, SD18, SD19 (zero faces, two faces, a cylindrical neutral face), SD20 (`NeutralFrame`; a wall as the neutral face), SD21 (an empty selection), SD22 (the other cap selected), SD23 (a revolve; a drafted body drafted again) |

**PR 2 fixtures** (`apitest/draft_mesh_test.go`): F1–F7 tessellated at three
tolerances, each mesh closed and consistently wound, its boundary and volume
proofs published, every facet attributed to a face of the body, and
`Mesh.Bound` covering the chording it took; the same F1–F7 records meshed at
three tolerances through `draftrecord.Tessellate`, each held mesh embedded and
its exact volume within `volSymDiff` of the closed form, and each record's
DD17 radius at least every mesh vertex's L1 norm and at most twice the largest,
and F1's mesh holding its eight corners and two triangles on each of its six
faces (`internal/draftrecord/draft_fixtures_test.go`); `Union` of F1 with a straight
prism through it; STEP of F1 analytic (`analytic decad solid`) and of F2
faceted; `MassProperties` of F1 against the closed form.

**PR 5 fixtures** (`apitest/draft_subset_test.go`): F1's box with its `+u`
wall drafted alone, `Volume = h·a(a − d/2)`, its area, centroid, far corners
and the kept walls' horizontal normals, and the same body placed; F4's L with
its notch wall `u = 10` drafted alone, `Volume = h(300 − 5d)`, the far reflex
corner `(10 − d, 10, h)`; F7's hole drafted alone, `Volume = πh(R² − r² − rd −
d²/3)`, the outer wall a `Cylinder` of radius `R`; F10's 2 mm square with one
wall drafted, each moved far corner within its bound; `Verify` on the subset
bodies (DD5, DD6); SD4 for one straight wall of F3's slot; the mixed-amount
corner rows in `internal/offset2d/sharp_test.go`; the box and ring subsets
meshed at three tolerances, closed, with the volume proof published, and each
mesh `Bound` no larger than its tapered extrude's; the one-wall, notch-wall and
hole-alone records meshed through `draftrecord.Tessellate`, each exact volume
within `volSymDiff` of its closed form (`internal/draftrecord/draft_fixtures_test.go`); the one-wall box under DD7, its drafted
wall listed along `−e` and cleared along `+e`, its kept walls undecided.

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
| the mesh proof's chord term (`DraftBandHeightUpper`) | zeroed | F2, F3, F7 and the hole-alone subset: each mesh volume sits outside its `volSymDiff` |
| the mesh proof's vertex motion (`SweptVolumeAllow`) | omitted | F1, F4, F5, F6 and the one-wall and notch-wall subsets: `volSymDiff` misses the far corners' rounding |
| the far section in DD17's record radius | dropped from `MoverRecordRadius` | F5's flare: a far vertex lies past the radius |
| DD7's dispatch arm (`capblendsurvey.DraftUndercuts`) | removed from `runSurveys` | every PR 4 fixture: the survey reads the draft body as an unsupported payload |
| the wall reading's allowance (`capblendsurvey.PatchNormalRange`'s third result) | zeroed in `capblendsurvey.PatchUndercuts` | the tangent pull: the wall reads the sign of its own rounding and no `DiagUndecidedUndercut` is reported |
| a moved walk's span at a mixed corner | `capcontour.AmountsDisplacement` reads every span as its point amount | the subset F10: each moved far corner `1.06e-16` outside its bound |
| the mixed-amount refusal | `offset2d.SharpCornerJoin` reads a mixed circular corner as an equal one | the slot's SD4 fixture refuses as SD13 instead |
| a kept wall's setback in the mesh | the chord pass's cap radius, or the cap motion, reads the draft's `d` | the hole-alone subset: its `volSymDiff` grows past a tenth of the body's volume, so the proof bounds nothing |
| a kept wall's setback in the band's radius rounding | read against the draft's `d` | the hole-alone mesh `Bound` above F7's |

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

- **One payload for both entry points**: `draftPayload`, over the operand's
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
- **Extents**: `Distance`, `ToFace`, `ThroughAll`, `Symmetric` and `TwoSided`.
  A crossing extent uses two slabs with one sewn sketch rim (§7, §14).
- **Neutral plane**: a cap of the operand, through a `NeutralFace`; a
  `NeutralFrame` and an interior neutral level are deferred (SD20, §14).
- **Selection vocabulary**: `Walls(b)` added as a `FacePredicate` beside
  `CapStart`/`CapEnd`; no nil-selector form.
- **Surveys**: undercut decided in PR 4 off the patches' normal model; wall
  and concave radius staged (DD8).
- **Hand-offs**: none (§3).

## 14. PR split

Each PR ships code, tests and the documentation updates for every refusal it
lifts: this document's increment table, `doc.go`'s support map,
`agent-docs/missing-features.md`, the function doc comments, and the layout rows of
every new root file. This document ships with PR 1.

| PR | Lands | Files and functions | Proves | After |
|---|---|---|---|---|
| **1** | `Extrude` + `WithTaper` over Table RD1: `offset2d.BuildSharpLoop`; `draftPayload` with `Transform`/`Placed`; `internal/draftrecord/build.go` (§7); `internal/draftrecord/measure.go` (§8 over `internal/capband`); `d` and its span (§8.1) with a certified tangent in `internal/proofbound`; `extrude.go` dispatches a nonzero taper to the draft build and refuses SD11/SD12; DD6's gate arm; DD12; the modify refusal messages naming the class; `Tessellate` refuses the class through its default | `internal/offset2d/sharp.go`, `internal/proofbound/interval_trig.go` (tangent), `internal/draftrecord/` (`payload.go`, `build.go`, `measure.go`), `draft.go`, `extrude.go`, `capblend*.go` (the band's `draft` view), `internal/clearance/pairkernel/gate.go`, `fillet.go`/`chamfer.go`/`shell.go` (messages), `apitest/extrude_taper_test.go`, `internal/draftrecord/build_test.go`, `internal/offset2d/sharp_test.go`, `examples/decad_extrude_taper_example_test.go` | F1–F10 and the SD refusals of §11; the fail-first legs | — |
| **2** | tessellation and the mesh volume proof (DD1), which opens DD2, DD3, DD4, DD10, DD11, DD13, DD17 | `internal/draftrecord/tessellate.go`, `tessellate.go` (dispatch), `tessellate_capblend.go` and `internal/capblend/admit.go` (§9.1's three differences), `internal/draftrecord/payload.go`/`build.go` (`Patches`, `BandDelta`), `boolean.go` (the admission arm), `mass_properties.go` (the mesh path needs no arm), `motion_bound.go`, `apitest/draft_mesh_test.go`, `internal/draftrecord/tessellate_test.go` | the PR 2 fixtures | 1 |
| **3** | `Draft` over RD2: `NeutralPlane`, `NeutralFace`, `NeutralFrame` (refusing), `DraftOption`, `Walls(b)`; §10.1's resolution; SD17–SD23; core §8's pointer to this document and core §12's signed-displacement list | `draft.go`, `selector.go` (`Walls`), `internal/selectorquery/predicate.go`, `agent-docs/api-design.md` §8/§12, `apitest/draft_test.go`, `examples/decad_draft_example_test.go` | D1–D5 | 1 |
| **4** | DD7: the undercut survey over draft bodies | `draft_survey.go`, `survey.go` (dispatch), `capblend_survey.go` (`capPatchUndercuts`, the patch loop both surveys run), `apitest/draft_verify_test.go` | the PR 4 fixtures; the `−e` pull lists every wall | 2 |
| **5** | RD3: the subset draft (§10.2): per-walk amounts in `BuildSharpLoop`, the mixed-corner rule, SD21 narrowed | `internal/offset2d/sharp.go`, `internal/capcontour/displacement.go`, `internal/capband/`, `draft.go`, `internal/draftrecord/build.go`, `internal/draftrecord/payload.go`, `capblend*.go` (the band view's per-walk amounts), `tessellate_capblend.go` and `internal/tessellation/` (the mesh's per-walk setbacks), `apitest/draft_subset_test.go` | one wall of F1's box drafted: `A(z) = a(a − z·tan α)`, so `Volume = h·a(a − d/2)`; the L with one notch wall drafted; a hole drafted alone (F7's cone with vertical outer walls) | 3 |
| **6** | RD1 with a one-sided `ToFace` or `ThroughAll`: resolve the stop through `resolveLinearExtent`, then build the existing single-slab draft with the stop's axial bound | `draft.go`, `apitest/extrude_taper_test.go` | the far-cap vertex and volume match the selected stop level within their bounds | 5 |
| **7** | RD1 with `Symmetric` or `TwoSided`: build and sew the two slabs, combine bounded readings and meshes | `internal/draftrecord/two_sided.go`, `internal/draftrecord/tessellate.go`, `draft.go`, `internal/compositesweep/body_audit.go` | public volume, manifold, curved mesh and placement cases | 6 |

PRs 2 and 3 run in parallel after PR 1; PR 4 follows 2; PR 5 follows 3.
PRs 1, 2 and 4 are proof specifications (bounds, closure terms, the mesh
proof and the normal model) and are implemented at the higher reasoning
tier; PRs 3 and 5 are file-by-file plans over PR 1's builder.

Increment table — what still refuses after each PR. PR 2's mesh serves every
draft body, `Draft`'s included:

| After | State | Still refused |
|---|---|---|
| 1 | landed | every draft body's tessellation, boolean, export, mass and interference reading (DD1's dependants); `Draft` (no entry point); surveys `Suspect`; SD3, SD4, SD11, SD12 |
| 2 | landed | surveys `Suspect`; SD3, SD4, SD11, SD12, SD20, SD21; booleans, interference and mass of a body the band admission refuses (§9.1) |
| 3 | landed | surveys `Suspect`; SD3, SD4, SD11, SD12, SD20, SD21 |
| 4 | landed | wall and concave-radius surveys `Suspect` (DD8); SD3, SD4, SD11, SD12, SD20 |
| 5 | landed | DD8; SD3, SD4, SD11, SD12, SD20; clearance carriers (DD9); shell, fillet and chamfer of a draft body (DD14); the mitered circular corner; the two-sided extents; `NeutralFrame` and the interior neutral level; the surface result |
| 6 | landed | DD8; SD3, SD4, two-sided SD11, SD12, SD20; clearance carriers (DD9); shell, fillet and chamfer of a draft body (DD14); `NeutralFrame` and the interior neutral level |
| 7 | landed | DD8; SD3, SD4, SD12, SD20; two-sided undercut survey; clearance carriers (DD9); DD14 outside §10.4–§10.6; `NeutralFrame` and the interior neutral level |

The unscheduled reach, in the order a later design should take it: the
interior neutral level, the mitered circular
corner as modify-reach §8.3's ruled stand-in, the wall and concave-radius
surveys (DD8), and the clearance carriers (DD9).
