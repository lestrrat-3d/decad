# Carrier Record Design

`Fillet` and `Chamfer` of a prism whose recorded section is displaced from the
section it denotes, and the producers that make such a prism carry the proof
those ops need. References of the form "prism-boolean §N" are to
`agent-docs/prism-boolean-design.md`; "A6" and "A3" to
`agent-docs/general-boolean-design.md` §3; "modify §N" to
`agent-docs/modify-design.md`; "brep-modify §N" to
`agent-docs/brep-modify-design.md`; "reach §N" to
`agent-docs/modify-reach-design.md`; "mirror-pattern §N" to
`agent-docs/mirror-pattern-design.md`; "tessellation §N" to
`agent-docs/tessellation-design.md`.

Navigation only (each section owns its rules):

| Question | Section |
|---|---|
| What lands in which PR? | §2.1 |
| What does a carrier record claim? | §3.2 |
| How far does a displaced segment move, point for point? | §3.3 (Lemma H) |
| How is a scene counted against the arrangement cap? | §4 |
| Which constructions set a record? | §5 |
| How does the one-scene `Union` take circles, holes and displaced copies? | §5.3 |
| How does a clean-nesting `Cut` keep a record? | §5.4 |
| Why does a crossing exist, and why only one? | §6.1 (Lemma T) |
| How is an arrangement proven stable under displacement? | §6.3, §6.4 |
| How does a lateral `Fillet` or `Chamfer` of a displaced prism build? | §7 |
| What does a displaced blend's result denote? | §7.3 |
| What is the second increment? | §8 |
| How does a cap-loop `Chamfer` of exact loops dispatch on a recorded receiver? | §8.1 |
| Which `BuildChecked` gates read non-selected loops, and how do they transfer? | §8.3 |
| What does `capblend.DisplacedPayload` charge, and who declines it? | §8.4 to §8.8 |

## 1. Problem

Every prism modify op refuses a receiver with a nonzero `SectionDelta`
(`prismrecord.Payload.RequireExactSection`, prism-boolean §8 row 3, brep-modify
Table RB row RB6). Every analytic `Union` that cuts an edge carries
`δ_cut > 0` (prism-boolean §7). The input class this design serves is a disc
(a `CircleSeg`, with holes bounded by lines and arcs) united on its own sketch
plane with all-line teeth (one tooth and its frame-keeping `PatternCopies`
instances), then a concave `Fillet` of every root, then a `Cut` by a bore tool
lying inside the disc, then a `Chamfer` of the hole rims on both caps. The
motivating part has 5 windows (each an inner arc, an outer arc, 2 lines and 4
sketch-fillet arcs), 15 teeth (a tooth and 14 copies), a 30-root fillet and a
D bore; it stays in the consumer repository (`agent-docs/testing.md`, Test
scope). Today:

1. A hole-free disc united with one tooth takes the pair route (prism-boolean
   G6 refuses a holed operand there) and records the disc as
   a trimmed `CircleSeg` and the flanks as narrowed `LineSeg`s. Its
   `SectionDelta` is a few `1e-13` mm of cut-parameter rounding, so `Fillet` of
   the root edges refuses.
2. The disc united with the tooth and its copies misses every analytic route.
   Each copy is displaced (mirror-pattern §6.2 charges its rounding). The
   one-scene route (prism-boolean §4.6) takes only undisplaced, hole-free,
   whole-line prisms, and the pair fold stops once the disc circle is trimmed
   (prism-boolean §4.1). The set falls to the mesh path and its edges are
   `FacetedCurve`s.
3. `prismcells.RegionsWithinWorkCap` counts 256 densified segments per circle
   and per arc. The wheel's union counts about 8000 and the bore `Cut` after the
   fillet about 19000, against `MaxArrangementSegments = 4096`.
4. The windows make the disc a holed operand, and the cap-loop chamfer
   (`capBlendPayload`, reach §8.3) has no displacement term.

The displacement in case 1 is cut ends sliding along exact carriers. In case 2
the copies' own carriers also moved by their motion's rounding. Every op stays
sound against the section each body DENOTES; this design never redefines a
denotation to fit a record.

## 2. Approach

A displaced prism may carry a **carrier record**: per segment and per junction,
a bound on how far the denoted section's carriers and junctions sit from the
recorded ones, with the denoted section proven to have the recorded section's
structure. A lateral `Fillet` or `Chamfer` reads it, builds from the recorded
carriers, holds a margin on every decision the displacement could flip, and
charges what it cannot remove. Constructions that can prove a record set one:
a frame-keeping pattern instance, the pair `Union` of two undisplaced operands,
an extended one-scene `Union`, a clean-nesting `Cut`, and the blends
themselves.

### 2.1 Increments

**PR 1** (§3 to §7, §9, the PR-1 rows of §10): the carrier record and Lemma H;
the per-entity arrangement count (§4); the frame-keeping instance record; the
pair `Union` record; the extended one-scene `Union`; the clean-nesting `Cut`
record; the arrangement stability audit; the lateral carrier route of `Fillet`
and `Chamfer`; and `sectionaudit.Nesting`'s candidate points (§6.5). It builds
the wheel's union, its root fillet and its bore `Cut` with exact analytic
faces. In PR 1 a cap-loop `Chamfer` of a displaced receiver keeps today's
answer: `RequireExactSection`'s refusal, or brep-modify §2.2's faceted view.

**PR 2** (§8 and the PR-2 rows of §10): the cap-loop `Chamfer` of exact loops
of a receiver with a valid record, built as a separate payload type
`capblend.DisplacedPayload` that every reader unaware of it declines. It
chamfers the wheel's window rims on both caps with exact analytic faces. PR 2
also measures a lateral blend's arc ends at their read ends (§7.7, "Arc read
ends" and "Seam snap"), and declines a mesh `Boolean`'s source tag on every
displaced wall of a `DisplacedPayload` (§8.7, rule DW; displaced prisms keep
today's tags). Nothing in PR 1 depends on it.

### 2.2 Alternatives considered

- **Denote the record.** Treat a displaced union's recorded section as its own
  denotation, as a loft join's fillet does (pattern-union, Axial pattern join
  §4). That removes every claim the result makes about the operands the caller
  combined. Rejected.
- **Faceted fallback.** brep-modify §2.2's retained blend already chamfers
  exact-carrier loops of a displaced prism, but its result is a
  `facetedPayload`. Rejected: the part must keep exact analytic faces.
- **Faceted axial-section route** (local commit `a6c77827d8`): rejected by the
  user. Its test shapes are reused in §10.
- **One global carrier displacement.** A single value per payload makes every
  window loop non-exact as soon as one copy is displaced, and §8 needs exact
  loops. Rejected for per-segment values.

## 3. The carrier record

### 3.1 Data

`prismrecord.Payload` gains `Carriers *CarrierRecord`:

```go
type CarrierRecord struct {
	SectionDelta float64             // the payload's SectionDelta when the record was made
	Profile      momentinput.Profile // a deep copy of the section the record describes
	Segment      [][]float64         // ε[l][j]: loop l (0 outer, i+1 hole i), segment j
	Junction     [][]float64         // δ[l][j]: the junction at the end of segment j of loop l
	owner        *topo.Body          // the one body the record was minted for (§3.4)
}
```

Every entry is finite and non-negative, and the shapes match the profile's
loops and segments. Junction `j` of loop `l` joins segment `j` to segment
`j+1` (mod the loop's length). Every producer stores `Profile` as a deep copy
(`sectionrecord.CloneLoopRecord` per loop), so no segment slice is shared with
the payload's own profile.

### 3.2 The claim

**Recorded data.** Segment `j`'s recorded carrier is: for a `LineSeg`, the line
through its `Start` and `End`; for a `CircleSeg`, the circle of its `Center` and
`Radius`; for an `ArcSeg`, the circle of its `Center` and radius
`|Start − Center|`. Its **read ends** are the two points the segment DENOTES at
its ends (evaluator §4): the exact point at its recorded parameter on its
recorded carrier. For a whole `LineSeg` that is its `Start` and `End`; for a
whole `ArcSeg` it is `Start`, and the point at `Start`'s radius on `End`'s
bearing, not the recorded `End`. An `ArcSeg`'s **radial residual**
`ρ_j = | |End − Center| − |Start − Center| |` (outward-rounded square roots) is
the distance between that read end and the recorded `End`. A consumer that
reads the pinned `End` (as `pinArcWalkEnds` does) charges `ρ_j` itself, as
`loftmesh.ArcNaturalEndRadialUpper` does for the loft; a consumer that reads a
float walk charges that walk end's `proofbound.WalkEndBoundAllow`.

**Junction paths.** Where segment `j`'s read end and segment `j+1`'s read start
differ, the section closes the gap with evaluator §4's closing path: through
the two supporting lines' exact crossing where both neighbours are `LineSeg`s
and that corner is provably near, else the straight chord. A junction's
**path** is that closing path with its two ends, or the single shared point.

A valid record proves, for the payload's recorded section `S` and the section
`S*` its construction denotes:

- **CR1 — structure.** `S*` has the same loops, each with the same segments in
  the same order, of the same kinds (line where `S` has a line, circular where
  `S` has an arc or circle) and walk senses, joined by junction paths. Each
  circular arc of `S*` runs between its two ends the short way: each end's
  slide, measured as an angle on `S*`'s circle from the radial image of the
  matching read end (`C* + R*·(read end − C)/R`), is below `α_j` of Lemma H, and
  `2·α_j` is below both `σ_j` and `2π − σ_j`, with `σ_j` the recorded sweep.
- **CR2 — carriers.** A line of `S*` lies on the line through two points
  `Start*`, `End*` with `|Start − Start*| ≤ ε` and `|End − End*| ≤ ε`. A circular
  segment of `S*` lies on a circle `(C*, R*)` with `|C − C*| + |R − R*| ≤ ε`,
  where `(C, R)` is the recorded carrier.
- **CR3 — junctions.** `S`'s and `S*`'s junction paths between segments `j` and
  `j+1` have parametrizations over `[0, 1]` (each continuous and weakly
  monotone in arc length along its path, either possibly constant, so two such
  pairs chain through a shared path) that start at segment `j`'s ends and
  finish at segment `j+1`'s
  starts, with every pair of points at the same parameter within `δ[l][j]`.
  Where `S*`'s two segments meet at one point `J*` (on both of its carriers),
  this says every point of `S`'s path lies within `δ[l][j]` of `J*`. Where both
  paths are polylines (a point, a chord, or end–corner–end), matching their
  breakpoints and interpolating linearly between them gives such a pair, and
  then the largest breakpoint distance bounds `δ` (**path rule**: a chord
  matched to a corner path takes the chord point at the corner's arc-length
  fraction as its breakpoint).
- **CR4 — the bound.** Every entry, and every loop's `h_loop` (Lemma H), is at
  most `SectionDelta`. So every boundary point of `S*` lies within
  `SectionDelta` of the matched point of `S`, the displacement map
  `prismrecord.Build`'s section charges and `proofbound.SectionDisplacementArea`
  read.

With every entry zero the claim says `S*` is `S` as evaluator §4 denotes it,
closing paths included. So a payload with `SectionDelta == 0`, and a loop a
producer copies from such a payload unchanged, has a true zero record whatever
its arcs' radial residuals.

### 3.3 Lemma H — the pointwise segment bound

For segment `j` with start junction entry `δ_s = δ[l][j−1]`, end junction entry
`δ_e = δ[l][j]`, carrier entry `ε` and recorded radius `R`, let
`δ_m = max(δ_s, δ_e)` and define `h[l][j]`:

| Segment | `h` | Condition (else `h = +∞`) |
|---|---|---|
| line | `max(δ_s, δ_e)` | none |
| whole circle | `ε` | none |
| arc (an `ArcSeg`, with `σ_j` its recorded sweep times `|TEnd − TStart|` when narrowed; or a narrowed `CircleSeg` with `σ_j = 2π·|TEnd − TStart|`) | `ε + 2·(ε + δ_m)` | `ε + δ_m ≤ 2·(R − ε)` and `2·α_j < min(σ_j, 2π − σ_j)` with `α_j = 2·(ε + δ_m)/(R − ε)` |

Every value is rounded up. The sweep test reads `σ_j` as a proven enclosure
(`circularbounds`' `Atan2Interval` enclosures of the two bearings, the same
branch correction a whole arc takes) and holds for every value in it. A
consumer that meets `h = +∞` misses or refuses as its own section says.

**Lemma H.** Under CR1–CR3 there is a homeomorphism `φ` from each recorded
segment (between its read ends) onto `S*`'s segment that takes the read start
and read end to `S*`'s two segment ends and moves every point by at most `h`.

Proof. A line: map `(1 − t)·A + t·B` to `(1 − t)·A* + t·B*`, where `A`, `B` are
the read ends and `A*`, `B*` `S*`'s segment ends; the distance is a convex
combination of `|A − A*| ≤ δ_s` and `|B − B*| ≤ δ_e` (CR3: both lie on the
junction paths). A whole circle: the radial map `P ↦ C* + R*·(P − C)/R` moves
`P` by at most `|C − C*| + |R − R*| ≤ ε`. An arc: both read ends lie on the
recorded circle. The radial map takes each read end to a point `J'` of `S*`'s
circle within `ε` of it, so `|J' − B*| ≤ ε + δ_m` for `S*`'s end `B*`. That chord
is at most `2R*` (first condition, with `R* ≥ R − ε`), so the shorter arc of
`S*`'s circle from `J'` to `B*` is at most `(π/2)·(ε + δ_m) ≤ 2·(ε + δ_m)` long,
an angle below `α_j`. CR1 says the end slides that short way. Map the point at
normalized angle `t` of the recorded arc to the point at normalized angle `t`
of `S*`'s arc: the radial part moves it by at most `ε`, and the angular part by
at most `R*` times the larger end slide, so by at most `2·(ε + δ_m)`.

**Loops.** Parametrize each segment of `S` and its image under `φ` by the same
parameter, and each junction path pair by CR3's parametrizations. Joined in loop
order, they parametrize each loop of `S` and of `S*` over one circle, `φ` on
every segment, with matched points within `h_loop[l]`, the largest `h` and `δ`
of the loop.

**Corollary W.** The straight-line homotopy `u ↦ (1 − t)·S(u) + t·S*(u)` between
the two loop parametrizations moves every point by at most `h_loop`. A point
farther than `h_loop` from a loop
has the same winding number about that loop and about its `S*` image; two loops
whose recorded distance exceeds the sum of their `h_loop` stay disjoint through
the homotopy, and each keeps its winding number about the other's points.

### 3.4 Validity, staleness and readers

**The record fails closed.** A record is bound to the one body it was minted
for. `prismrecord.Build` always clears `Carriers`; only
`prismrecord.BuildWithRecord(ctx, env, p, rec)`, which the producers of §5
call, stores a deep copy of `rec` in the new body's payload with `owner` set to
that new body. `prismrecord.CarrierRecordOf(budget, body)` returns the record
and true only when:

- `body.Payload` is a `prismrecord.Payload` whose `Carriers.owner` is `body`;
- the payload's `SectionDelta` is bit-identical to the record's;
- the entries are well formed (finite, non-negative, at most `SectionDelta`,
  shaped like the profile) and every loop's `h_loop` is finite and at most
  `SectionDelta` (CR4);
- the payload's profile equals the record's copy loop for loop and segment for
  segment (`momentinput.ExactProfileEqual`).

A payload with `SectionDelta == 0` answers a record of zeros whatever body holds
it: it denotes its own record. Anything else answers false, which keeps today's
refusals. Every reader (§5.3's operands, §5.4's target, §7's receiver, §8's
receiver) reads through `CarrierRecordOf` on the body it was handed.

So no pass-through can carry a record. A construction that copies a payload into
its result (`prismcells.MatchCutNoOp` returning the target for a tool inside a
displaced hole, a contained `Union` returning an operand, a scale or mirror, or a
site that builds `&topo.Body{Payload: p}` directly) produces a body that is not
the record's owner, so its record claims nothing; no list of such sites needs
auditing. `Placed` re-evaluates through `prismrecord.Build` and so drops the
record too: a `Transform` or `Copy` of a recorded body carries none, and a
later blend of it refuses as today. `ExactCarriers` (prism-boolean §7) is
unchanged, and `Payload.CarrierExact` reads only it; a reader that needs a
record's exact loop asks `CarrierRecordOf(...).ExactLoop(l)`, which reports a
loop whose entries are all zero: `S*` then holds that loop of `S` unchanged,
point for point.

## 4. Arrangement cap

The pinned `sketch` (`go.mod`, `geom/arrange.go` `sampleParams`) densifies a line
to 1 segment, a whole circle to 256, and an arc to `max(2, ⌈256·sweep/(2π)⌉)`,
where `sweep = Arc.Sweep()` is `wrapSweep(atan2(Start − Center), atan2(End −
Center))` over the scene entity's own floats, a value in `(0, 2π]`. The cap
`MaxArrangementSegments = 4096` bounds the densified count of one scene, which
is what bounds sketch's quadratic pass (about 1.4 s and 16 MB, unchanged).

**The screen.** `RegionsWithinWorkCap` counts a line as 1 and a circular segment
as `prismcells.CircularSegmentCount(seg)`:

- a whole `CircleSeg`: 256, sketch's own count for a closed circle, which reads
  no float sweep;
- an `ArcSeg`: `turn = sweep/(2π)` with
  `a0 = math.Atan2(Start.V − Center.V, Start.U − Center.U)`,
  `a1 = math.Atan2(End.V − Center.V, End.U − Center.U)`,
  `d = math.Mod(a1 − a0, 2π)`, `sweep = d` when `d > 0` else `d + 2π`: sketch's
  `wrapSweep` (`geom/geom.go`) on the record's floats, written with
  `math.Mod` as sketch writes it, not with a repeated `+2π` loop. A narrowed
  `ArcSeg` multiplies `turn` by `|TEnd − TStart|`. A narrowed `CircleSeg` takes
  `turn = |TEnd − TStart|`;
- the count is `min(257, max(2, ⌈256·(turn + 2⁻⁴⁰)⌉ + 1))`, and 257 for a
  non-finite or non-positive `turn` or one above 1.

For a whole `ArcSeg` that enters a scene unmapped (operand A, or any operand
under an identity re-expression), the scene builder creates
`CreateArc(Center, Start, End)` from the pinned walk ends (prism-boolean §4.1's
ascending-angle order puts `Start` first for either walk sense), so `sketch`
evaluates the same expressions on the same floats and the screen's count is at
least sketch's. A whole circle counts exactly sketch's 256.

**The authoritative count.** For a narrowed or mapped circular segment the
record's floats are not the floats `sketch` reads. So the scene builders
(`prismcells.BuildScene`, `prismcells.BuildSceneRegions`,
`classbgeom.CreateNaturalSegment`) sum, over the entities they create,
sketch's own densification computed from each entity's floats (the formula
above), and return `prismcells.WithWorkLimit(ErrUnsupported…)` when the sum
exceeds the cap. A caller whose own over-cap answer is a silent miss maps that
error (`errors.As` a `*prismcells.WorkLimitError`) to its miss; every other
caller returns it, as it returns its screen refusal today. The screen stays
first so a scene far over the cap is refused before any entity is built.

The per-entity count admits more scenes for every `RegionsWithinWorkCap`
caller (the stacked-union, section-history, within, pattern-assembly,
revolve-bore, revolve-plan, loft-direct, class-B, stacked-record, pair,
one-scene and group callers): a pair with arcs that used to stop at the cap
now reaches its route. A whole circle keeps its count of 256, so the
rectangular plate with fourteen circular holes and a circular tool still fits. As a dated illustration, the wheel's union counts about 2000
and its bore `Cut` after the fillet about 3700.

A caller that used to stop at the cap now runs its analytic route further, so a
set that used to mesh (a silent cap miss) or refuse with the cap's
`WorkLimitError` can now refuse with that route's own typed error instead. PR 1
tests one such caller: an arc-heavy pair that used to exceed the cap now reaches
the pair route and answers that route's result or refusal.

## 5. Producers (PR 1)

Every producer below sets the record together with `SectionDelta`, sets
`SectionDelta` to at least every entry and every loop's `h_loop` (CR4), stores
a deep copy of the profile (§3.1), and sets no record when any arc of its result
has `h = +∞` (Lemma H); the extended `Union` (§5.3) misses instead, since its
proof needs every `h`. A construction not listed sets no record.

### 5.1 Frame-keeping pattern instance

`patternassembly.FrameKeepingInstance` moves a prism's record by an in-plane
motion and charges the rounding into `SectionDelta` (mirror-pattern §4.3,
§6.2). The instance denotes the source section moved by the EXACT motion: a
rotation by exactly `i/Count` of a turn about the exact centre, or the exact
translation `t_i`. When the source has `SectionDelta == 0`, the instance gets a
record: every segment entry and every junction entry is the motion's charge
`δ_pattern` (mirror-pattern §6.2, the instance's `SectionDelta` before any raise
below), except the junction at an `ArcSeg`'s `End` in record-field order
(§7.7): for a counter-clockwise arc segment `j`'s end junction `j`, for a
clockwise one (`End` = walk start) its start junction `j − 1`. That junction
takes
`δ_End = up(3e + 4e·(R + 2e)/(R_E − 2e))` with `R` the recorded radius and
`R_E` a lower bound on the moved `|End − Center|`. The instance's
`SectionDelta` is raised to the largest entry and `h_loop` when either exceeds
the charge. An all-line source keeps
`prismrecord.UniformCarrierRecord(moved, SectionDelta)` (every line's `h` is its
larger junction entry). A source with a line–line junction whose two read ends
differ (two narrowed lines) gets no record: its closing path may run through the
supports' corner, which a motion's rounding moves by up to `e/sin θ` and whose
corner-or-chord choice it can flip.

Proof. Let `e` be the largest point charge `patternrecord.MoveRegion` takes. The
frame-keeping arm admits no trimmed arc or circle, so every circular segment is
whole. CR1: `MoveRegion` keeps every segment's kind, sense and range, and the
exact motion keeps each arc's sweep. CR2: a line's moved `Start` and `End` are
each within `e` of their exact images; a circle's centre moves by at most `e`
and its radius not at all; an arc's centre and `Start` each move by at most
`e`, so its radius `|Start − Center|` moves by at most `2e` and
`|ΔC| + |ΔR| ≤ 3e`, the arc charge `MoveRegion` takes. CR3: a read end that is
a recorded endpoint, or a narrowed line's point at a recorded parameter, lies
within `e` of its exact image. An arc's end read end is `C + R·u`, with `u` the
unit vector to `End`. Centre and radius move by at most `3e` together, and `u`
by at most `2·2e/(R_E − 2e)` (`End` and `Center` each move by at most `e`),
which gives `δ_End`. With no line–line junction of distinct read ends, every
junction path of the source and of the instance is a chord or a point, whose
ends move by at most these amounts, and the path rule (CR3) gives the entry.

### 5.2 Pair `Union` of two undisplaced operands

The pair route's `Union` (`prismcells.ResolveAndBuildUnion`) sets a record when
both operands have `SectionDelta == 0`, the re-expression is the identity
(prism-boolean §3.4), every consumed segment is whole (both walk charges zero),
and the arrangement withdraws no span (`SceneDelta.HasSharedSpans` false: two
near-identical carriers that `sketch` merges are two walls of `S*`, which breaks
CR1). Every carrier entry is zero. A junction where both result segments end at
their natural bounds (an operand's own recorded point) takes zero; a junction
where the arrangement computed a parameter on either side takes the larger
`prismcells.CutDelta` of its two edges (`prismcells.ExactOperandsRecord`). A
trimmed arc's `h` is twice its larger junction entry, so the result's
`SectionDelta` is raised to `max_l h_loop[l]` when that exceeds prism-boolean
§7's value.

Proof. Every scene entity is built from the operand's own exact record data
(prism-boolean §4.1), and the operands denote their records. `sketch` arranged
those carriers; the arrangement's topology and its certified cut parameters are
`sketch`'s answer, which decad consumes (CLAUDE.md). So `S*` is the arrangement
of the same carriers with the same cells (CR1, CR2 with `ε = 0`). At an own
recorded point the result's junction path is the operand's own, which `S*`
keeps (zero). At a cut, `S*` passes through the true crossing `O`; each read end
is the exact point at its computed parameter, within its `δ_cut` of `O`, and the
closing path between them (a chord, or the supporting lines' corner, which is
`O` itself) stays within the larger `δ_cut` of `O` (CR3). The proof applies no
V1: with no displacement nothing moves, and it rests on `sketch`'s certified cut
parameters (`TExact`, which `sketchrecord.RecordEdge` requires) and topology.

### 5.3 Extended one-scene `Union`

**Dispatch.** `prismboolean.UnionMany` keeps prism-boolean §4.6's class (whole
lines, hole-free, undisplaced: `lineClass`) with its behaviour and refusals.
Every other set is the **extended class** (`unionRegions`), which never refuses:
every failure is a silent miss and the set takes the next `Union` route
(`unionmany.Routes`), as A6's `AmplifiedFallback` does for a pair; cancellation
still returns its error. `StraightLoftUnionMany` converts its lofts and calls
`UnionMany`, so a converted set outside the line class reaches the extended
class too, where it used to miss. `UnionMany` runs first in `unionmany.Routes`,
and the extended class takes no set a later route owns: the free-form routes
(`axialsection`) fail U2, the loft and revolve routes fail U1, and
`prismpattern.UnionMany`'s copied-prism join roots each copy on an arc of the
blank's own circle, which U5b misses (§10 pins that set still taking its
route).

**Admission, before the arrangement (U1–U5a), each a miss when it fails:**

| # | Condition |
|---|---|
| U1 | Every operand is a `prismPayload` with no `SurfaceResult`, on the first operand's frame and placement, with sweep intervals `prismplacement.UnionZIntervalMatches` proves equal. |
| U2 | Every segment of every loop is a `LineSeg`, `CircleSeg` or `ArcSeg` over its whole range (`prismcells.WholeSegmentRange`). |
| U3 | Every operand has `SectionDelta == 0` or a valid carrier record (`prismrecord.CarrierRecordOf` on its body, §3.4). |
| U4 | The set fits the arrangement cap (§4, screen and authoritative count). |
| U5a | No two operands' lines lie on one infinite line (`prismcells.SameLineCarrier`). |

**Arrangement.** Every operand enters `BuildSceneRegions` as one region, under
the identity, with zero walk charge (a nonzero charge is a miss). After
`ProfilesContext` and `SplitRuns`, these are misses:

- **U5b.** The arrangement withdraws a span (`prismcells.CoincidentEdgesRegions`
  reports one): two operands' near-identical carriers merged by `sketch`.
- **V1.** At an arranged vertex where entities of two operands meet, other than
  exactly two entities each passing through it at a parameter the arrangement
  computed on both of its edges (`cutAt`). A vertex where an operand's own
  recorded vertex meets another operand's entity could change its order under
  any displacement, and nothing charges it (prism-boolean §4.4's known limit).
- An unresolved cell (`prismcells.ClassifyRegions`), a crossing with no charge,
  or a stability audit failure (V2, below).

**Vertex reading** (`prismcells.ReadVertices`). Each V1 vertex `v` takes
`Δ_v = up(δ_cut(v) + err_v + L_v)`: `δ_cut(v)` the larger `CutDelta` of its two
edges, `P̂_v` A6's exact junction point (`junctionPoint`) with its error
`err_v`, and `L_v` Lemma T's distance for the two DENOTED source carriers about
`p = P̂_v` with `q = err_v + h` of each source segment (Lemma H). `L_v` is read
as §7.5 reads the centre: start with `ρ = 4·(q_e + q_f)`, read `s` over
`B(P̂_v, ρ + max ε)`, set `s* = s − θ_e − θ_f` (§6.1), and accept when
`s* > ContactEps` and `2L ≤ ρ`, else set `ρ = 4L` and retry, at most four times;
no acceptance is a crossing with no charge (a miss). When both source segments
have `h = 0`, `Δ_v = δ_cut(v)` (§5.2). `Δ_v` bounds how far each read end at `v`
lies from `S*`'s junction there.

**Stability (V2).** When any operand is displaced, §6's audit passes over every
pair of arranged edges from different operands of which at least one is
displaced. A pair from two undisplaced operands needs no audit: both carriers
are the recorded ones, and their crossings and topology are `sketch`'s (§5.2).

**Order (V3).** When any operand is displaced, along every source segment that
carries an arranged vertex with `L_v > 0` (an undisplaced disc circle cut by
displaced flanks included), with `h` that segment's own (possibly zero), each two
consecutive arranged vertices `v`, `w` (a segment's own read end counts as a
vertex with `L = err = 0`) satisfy the exact test
`|P̂_v − P̂_w| > 4·(L_v + 2·err_v + L_w + 2·err_w + 2h)`. Each denoted crossing
`c*_v` lies within `L_v + 2·err_v + h` of `φ(O_v)`, the Lemma H image of the
recorded crossing, so along the denoted segment (arc length at most `π/2` times
the chord) it lies within `(π/2)·(L_v + 2·err_v + h)` of `φ(O_v)`. `φ` keeps
order, and `φ(O_v)`, `φ(O_w)` lie more than `|P̂_v − P̂_w| − err_v − err_w − 2h`
apart, which the test makes larger than the two slides together. So the
denoted vertices keep their order, and every arranged edge maps to the piece
between its own two denoted ends. On a whole circle the vertices are cyclic, so
the test also pairs the last vertex with the first. V3 also covers the edges of
undisplaced operands that meet a displaced one: their cut vertices move by
`L_v`, and near a point where three operands' edges come close, V3 and ASA2's
apart balls together keep each vertex on its own piece.

**Selection and merge.** A cell is selected when it lies in at least one
operand. `prismcells.MergeLoopsWithChains` keeps every edge of the selected
cells counted once. Each merged loop's signed area is read as an enclosure: the
sum of `circularbounds.AreaInterval` over its segments about one anchor, widened
by the area its junction gaps can move (`internal/momentinput/junctions.go`'s
charge) and by `proofbound.SectionDisplacementArea(h_loop, walks, perimeter)`
for the result's own `h_loop`. Exactly one loop's enclosure is positive, and it
is the outer loop; every other loop's enclosure is negative and it is a hole.
An enclosure that holds zero, or more than one positive loop, is a miss.
`prismcells.AuditMergeSection` then runs S7 and S9 on the holed profile (its S8
compares only the outer loop, so hole orientation rests on this check). When any
operand is displaced, the merged profile must also pass CB7's certified contact
test (every non-adjacent pair `G > F`, with `4·max h_loop < F`), so S9's
candidate-point verdict (§6.5) rests on a certified gap; the float S7 inside
`AuditMergeSection` may still refuse.

**Charges.** The result's `SectionDelta` is the larger of
`up(max(max_i SectionDelta_i, max_v Δ_v) + δ_cut)`, prism-boolean §7's formula
with zero walk and re-expression terms, and `max_l h_loop[l]` of the result
record (CR4).

**Result record.** Each result segment takes its source segment's `ε`. A
junction where both result segments end at their natural bounds is a recorded
vertex of one operand and takes that operand's junction `δ`. A junction at a cut
vertex takes `Δ_v`. A whole circle's seam junction, where its two pieces meet,
is charged the circle's `ε`, and `err_v` includes `δ_cut`, so `Δ_v` counts
`δ_cut` twice, which is conservative.

**Claim U.** Under U1–U5b and V1–V3, `S*` is the union of the denoted operands
and satisfies CR1–CR3 with the entries above.

Proof. With no displaced operand the proof is §5.2's. Otherwise let `A` be the
arrangement of the recorded operand boundaries and `A*` that of the denoted
ones. Every operand satisfies CR1–CR3, so Lemma H maps each of its segments onto
the denoted one, moving points by at most `h`.

- *Vertices.* Each own vertex maps to the operand's denoted junction (CR3). At
  each cut vertex `v`, Lemma T (§6.1) gives exactly one crossing `c*` of the two
  denoted carriers in its ball, transversal, within `L_v` of `P̂_v`; §6.3's
  ASA2 keeps both source segments' own vertices out of the ball, so `c*` lies on
  the two denoted segments.
- *Edges.* Each arranged edge `e` (a piece of one source segment between two
  arranged vertices) maps to the piece `e*` of the denoted segment between the
  images of its end vertices, which V3 keeps in order, within `η_e` of §6.3
  pointwise. Lemma S (§6.4)
  shows two images from different operands meet exactly at the images of their
  shared cut vertices, once each. Images from one operand keep that operand's
  structure, since the denoted operand section is a valid section.
- *Rotation.* At an own vertex the cyclic order of edges is the operand's own.
  At a cut vertex the sign of the tangents' cross product is the same on `A` and
  `A*` (`s* > 0`), so the four half-edges keep their cyclic order.
- *Faces.* Each connected component of `A*` is therefore embedded as its
  component of `A`, with the same faces. Components not joined by a vertex stay
  more than `F − 2·max η > 0` apart through Corollary W's homotopy, so each keeps
  its winding number about the others, and the nesting of components is the
  same. Two loops of one operand nest in `A*` as in `A` because the denoted
  operand is a valid section with the recorded loop structure (CR1). Each face's
  membership in each operand is read across that operand's edges from their
  material side (`ClassifyRegions`), which the embedding keeps.

So the same cells are selected and `S*`'s merged loops are the images of `S`'s,
segment for segment (CR1, with each arc's end slide below `L_v` and Lemma H's
condition checked). Each result segment lies on its denoted source carrier
(CR2 with the source `ε`). Each junction is an own vertex, whose junction path
is the operand's own (the operand's `δ`), or a cut vertex, where both read ends
and the closing path between them lie within `δ_cut(v)` of the recorded crossing
and `c*` within `err_v + L_v` of that crossing (CR3 with `Δ_v`; the closing
path is a chord, or for two lines possibly the supports' corner, which is the
recorded crossing itself).
CR4 follows from the formula.

### 5.4 Clean-nesting `Cut`

A clean-nesting `Cut` (prism-boolean §4.2, `prismcells.ResolveAndBuildCut`)
sets a record when the target body has a valid record (`CarrierRecordOf`; the
`Cut` dispatch reads it from the target body and hands it to the payload-level
resolver), the tool has `SectionDelta == 0`, the re-expression is the
identity, the tool's walk charge is zero (`sceneDelta.B == 0`), and §6's
stability audit passes over every pair of a tool segment and a target segment
with a nonzero `h`. All pairs are ASA1
pairs: a clean-nesting match shares no vertex. When any condition fails the
`Cut` builds as today, with no record.

**Mapping.** The matched profile is recorded from the private scene
(`RecordProfileContext`), which may order loops and start segments differently
from the target. The record therefore reads each result segment's entity tag
(`prismcells.ResolveCutWithTags`) for its target `(hole, segment)` or the tool's
outer loop:

- a result segment from a whole target segment records the entity's own data,
  which is the target segment's data, and takes its `ε` verbatim;
- a result segment from a narrowed target segment records a whole `LineSeg`
  through the scene's walked endpoints (prism-boolean §4.1 refuses a narrowed
  circular target first). With `a` each walked end's `WalkEndBoundAllow` and
  `δ` the target junction entry at that end, its `ε` is `max(δ + a)` over its
  two ends;
- a junction between two result segments of one target loop takes the target's
  junction `δ` (in either walk direction, `ownJunction`) plus `τ`, the path
  rule's distance (CR3) between the result's junction path and the target's
  there. Both paths are recorded data, each closed by the evaluator's own rule
  (`internal/momentinput` `lineCorner` and its 1024-chord test) computed
  exactly, so `τ` is an exact breakpoint distance; for two chords it is the
  larger walk allowance `a`. `S*`'s path enters only through the target's own
  CR3;
- every entry of the tool's loop is zero;
- the result's `SectionDelta` is raised to `max_l h_loop[l]` when that exceeds
  prism-boolean §7's value (CR4).

The tool must record its circular segments whole: a D bore drawn as a whole
`ArcSeg` and a line takes this route, while a tool holding a trimmed
`CircleSeg` is refused by `prismcells.ProfileHasTrimmedCircularSource`
(`internal/prismcells/analytic_entry.go`) and the `Cut` takes the mesh path.

Proof. `S*` is the denoted target less the exact tool. ASA1 keeps the tool more
than `F` from every displaced recorded target loop, and by Corollary W each such
loop moves by at most its `h_loop < F/4`, so the denoted target loops neither
meet the tool nor change their winding numbers about it. The result's
loops are the denoted target's loops plus the tool's loop, nested as recorded.
A re-recorded narrowed segment's two ends lie within `a` of the exact read ends,
which lie within `δ` of `S*`'s segment ends (CR2), and each result junction path
lies within `τ` of the target's, which lies within the target's `δ` of `S*`'s
(CR3, composing the two parametrizations).

**Order of operations.** The bore `Cut` follows the root `Fillet`. The union's
outer loop holds trimmed `CircleSeg`s, which prism-boolean §4.1 refuses
(`prismcells.ProfileHasTrimmedCircularSource`), so a bore `Cut` of the union
itself takes the mesh path. The fillet rewrite re-emits every walk of a filleted
loop as a whole `LineSeg` or `ArcSeg` (`offset2d.OriginalSegment`) and clones
every other loop, so the filleted part enters the `Cut` scene with whole
segments.

### 5.5 Blends

§7.7 gives a lateral blend's result record.

## 6. Arrangement stability

### 6.1 Lemma T — one transversal crossing

**Lemma T.** Let `γ1`, `γ2` each be a line or a circle, `p` a point,
`q1 ≥ dist(p, γ1)`, `q2 ≥ dist(p, γ2)`, and `s > 0` a lower bound on `|sin|` of
the angle between any tangent of `γ1` and any tangent of `γ2` at points of the
ball `B(p, ρ)`, with `ρ ≤ R/2` for each circle of radius `R`. Let
`L = (q1 + q2)/s + min(q1, q2)`. If `2L ≤ ρ`, then `γ1 ∩ γ2 ∩ B(p, ρ/2)` is
exactly one point `c`, `|c − p| ≤ L`, and the two curves cross there.

Proof. Say `q1 ≤ q2`, and let `Q1` be the point of `γ1` nearest `p`. Walk along
`γ1` from `Q1` by arc length `t` and let `g(t)` be the signed distance from
`γ1(t)` to `γ2`. While `γ1(t)` and its nearest point on `γ2` lie in `B(p, ρ)`,
`g` is differentiable with `|g′(t)| ≥ s`, and `g′` keeps one sign because the
tangent angle never reaches `0` or `π`. `|g(0)| ≤ q1 + q2`, so `g` has exactly one
zero with `|t| ≤ (q1 + q2)/s`, and every point walked lies within
`q1 + (q1 + q2)/s = L ≤ ρ/2` of `p`; `g` changes sign at `c`. Uniqueness: two
crossings `c ≠ c′` in `B(p, ρ/2)` are joined on each curve by an arc inside
`B(p, ρ)` (a line's segment, or a circle's shorter arc, since `ρ ≤ R/2`), and
by the mean value theorem each of those arcs has a tangent parallel to
`c′ − c`; the two tangents are then parallel, against `s > 0`.

decad computes these crossings, and V3's order, itself (the reason CLAUDE.md
asks the owning doc to state): they are statements about the DENOTED carriers,
whose data are exact images or bounds that `sketch` never sees, so there is no
`sketch` answer to consume. `sketch` still arranges the recorded carriers.

A6's distance bound is `L`; Lemma T adds existence and uniqueness. Every use
below reads `s` with `prismcells.sinLower` over the recorded carriers and
accepts only `s > sectionaudit.ContactEps`. `sinLower` answers 0 for a circle
read over a ball wider than half its radius (`internal/prismcells/crossing.go`),
so every accepted `s` meets `ρ ≤ R/2`.

**Denoted carriers.** For a denoted carrier, the tangent allowance `θ` is
`4ε/ℓ` for a line of recorded length `ℓ` (`|ΔEnd − ΔStart| ≤ 2ε` and
`|a/|a| − b/|b|| ≤ 2|a − b|/|a|`) and zero for a circle when `s` is read over the
ball widened by `ε`: the radial map takes each point of the denoted circle to a
point of the recorded circle within `ε` with the same tangent. So
`s* = s − Σθ` bounds the denoted carriers' separation.

### 6.2 Certified gap lower bound

`sketch` answers no distance query between two arranged edges, so decad computes
the gap itself (the reason CLAUDE.md asks the owning doc to state). It does not
use `sectionaudit.segMinDist` as a lower bound: its crossing tests count only
crossings more than `segCrossEps = 1e-7` (in parameter) from a segment end, so a
true crossing near an end reads as a gap of about `1e-7·ℓ`, far above the
floor.

`sectionaudit.SegGapLower(a, b survey2d.SegmentWalk)` is a proven lower bound on
the distance between two closed walks, each read as exact data: its float ends,
centre and radius as exact rationals, an arc being the points at that radius
whose bearing lies in the walk's range from its start to its end in its sense.
An arc's two endpoints are the points at its radius on its float ends' bearings
(the float ends need not lie on the circle); each is enclosed by scaling the
end's offset from the centre by the outward-rounded ratio `R/|end − C|`, and
its distances are read from that enclosure.

1. **Meeting.** Two lines: exact orientation tests on the closed segments. A
   line and a circle, or two circles: decide whether the carriers meet by exact
   comparison of squared quantities; when they do, enclose each meeting point
   with outward-rounded square roots. The answer is 0 unless every meeting
   point is proven outside one of the two pieces (a line parameter outside
   `[0, 1]`, or a bearing outside the arc by exact orientation tests on every
   corner of the enclosure).
2. **Candidates.** Otherwise the answer is the smallest of these lower bounds:
   each endpoint's distance to the other piece; for a line and an arc, the gap
   along the perpendicular from the centre (its foot is rational) when the foot
   lies inside the segment and its bearing inside the arc; for two arcs, the
   two centre-line points of each circle (bearing along the rational direction
   `C2 − C1`), or the radial gap for concentric arcs whose ranges overlap. A
   point-to-segment distance is exact rational before one downward square root;
   a point-to-arc distance is `| |p − C| − R |` from outward-rounded enclosures
   when the bearing lies in the range, else the nearer endpoint.
3. **Undecided membership includes.** A candidate whose membership is not
   decided exactly is kept: it is the distance to a point of the extended
   curve, which can only lower the minimum.

The candidates are complete: for two disjoint pieces the distance is attained at
two points each of which is an endpoint or an interior point where the joining
segment is normal to its curve, which for lines and circles are the points
listed (`segMinDist` documents the same candidate set).

An arranged edge's walk `w_e` differs from its exact recorded edge by
`c_e`, Lemma H's `h` read with the walk's radius bound
(`SegmentWalk.RadiusBound`; zero for a `CircleSeg`) as `ε` and each end's
`WalkEndBoundAllow` as `δ`. Where an edge ends at an own vertex whose two read
ends differ, the junction path between them is boundary that no edge holds; its
exact length (both legs of a corner path, or the chord) is added to that edge's
`c_e`, so every `G` also covers it. The certified gap is
`G(e, f) = SegGapLower(w_e, w_f) − c_e − c_f`.

A scene can hold about `10⁴` edge pairs. Each pair first compares the exact
distance between the two walks' bounding boxes (each box from the walk's ends
and, for an arc, from the same endpoint enclosures `SegGapLower` uses and its
extreme points in range, widened by `c`): a box gap above
`F` is itself a proven lower bound and skips `SegGapLower`.

### 6.3 The audit

The audit runs over arranged edges (`sketch`'s split edges, read from the
returned cells' boundary edges, each with its scene tag and recorded segment)
and over pairs from different operands of which at least one is displaced.

- **η.** An edge's `η_e` is Lemma H's `h` read for the edge itself, with its two
  end displacements in place of junction entries: an own-vertex end takes the
  operand's junction `δ`, a cut-vertex end takes `Δ_v` (§5.3). A line edge's
  `η_e` is its larger end displacement; a circular edge's is
  `ε + 2·(ε + larger end displacement)`, with `ε` its source segment's (read
  ends lie on the recorded circle, §3.2). A segment that the arrangement leaves
  whole (a clean-nesting `Cut`'s target and tool loops) is its own edge, and its
  `η` is its `h`.
- **Floor.** `F` is `sectionaudit.ContactEps` times the diagonal of the true
  bounding box of every operand segment (`sectionaudit.ContactFloor`). The audit
  first requires `4·max(η_e, h, c_e) < F`, over every edge, its walk allowance
  `c_e` (§6.2) and every source segment. Every `h`, `c`, `err_v` (at most `Δ_v`),
  `η`, and every circular carrier's `ε` (at most its `h`) is then below `F/4`; a
  line's `ε` enters only through `θ`.
- **ASA1 — apart.** Two edges that share no arranged vertex need
  `G(e, f) > F`.
- **ASA2 — crossing.** Two edges that share cut vertices need, at each shared
  vertex `v` (a chord and its arc share two): read `s0` over
  `B(P̂_v, 2F + err_v)` and set the trim length `ρ1 = 8F/s0`. Each walk is
  trimmed from its walked end `W_v` at `v` by arc length `ρ1` (a line shortened
  by `ρ1·(1 + 2⁻⁴⁰)`, an arc by the angle `(ρ1/R)·(1 + 2⁻⁴⁰)`). The trimmed end
  is recomputed in float; its error `κ_t` from the exact point at the trimmed
  parameter (a `WalkEndBoundAllow`-style bound) joins that walk's allowance `c`
  for every `G` read on the trimmed walk. The ball of `v` is `B_v = B(P̂_v, R_v)`
  with `R_v = ρ1·(1 + 2⁻⁴⁰) + max(w_v + c + κ_t)` over the two walks, `w_v` the
  exact distance `|W_v − P̂_v|` rounded up; it holds every exact edge point
  whose walk point the trim removes, over-trim included. Set Lemma T's
  radius `ρ_T = 2·(R_v + F)`, read `s1` over `B(P̂_v, ρ_T + F)`, and accept only
  when `s1 > ContactEps`, `5F/s1 ≤ ρ1`, and `s1 − θ_e − θ_f ≥ s1/2` (§6.1). Both
  source segments' own read ends lie farther than `R_v + F` from `P̂_v` (exact
  distance; a whole circle has none), and two shared vertices' balls are more
  than `F` apart. Two lines need nothing more. Otherwise require
  `G(e∖B, f) > F` and `G(e, f∖B) > F` over the trimmed walks; an empty remainder
  skips its test.
- **ASA3 — shared ends.** Two edges that meet at an own vertex of one operand
  belong to that operand and are not paired.

### 6.4 Lemma S

**Lemma S.** If ASA1–ASA3 hold and each edge's denoted image lies within `η_e`
of it pointwise (Lemma H's map), two images of edges from different operands
meet only inside the balls of their shared cut vertices, and there exactly
once, transversally.

Proof. Let `η = max η < F/4`. An ASA1 pair's images are at least
`F − 2η > 0` apart. For an ASA2 pair, suppose `p*` on `e*` meets `q*` on `f*`;
their recorded preimages `p`, `q` satisfy `|p − q| ≤ 2η < F`. If `p` lay outside
every ball, its walk point (within `c` of it) would lie farther than the
trimmed length `ρ1·(1 + 2⁻⁴⁰)` from every walked end `W_v`, so it would survive
the trim and `G(e∖B, f) > F` would
be contradicted. So `p` lies in some `B_v`; likewise `q`, and since the balls
are more than `F` apart, in the same `B_v`. So `p*` lies within
`R_v + η < ρ_T/2` of `P̂_v`. Lemma T applies
about `P̂_v` with radius `ρ_T` (the ball read for `s1` is `ρ_T` widened by more
than every circle's `ε`), `q = err_v + h ≤ F/2` for each denoted carrier (`O`
lies on the recorded edge, within `err_v` of `P̂_v`, and Lemma H moves it by at
most `h`) and `s* ≥ s1/2`: `L ≤ 2F/s1 + F/2 ≤ 5F/(2·s1) ≤ ρ1/2`, so `2L ≤ ρ_T`.
The denoted carriers therefore meet in `B(P̂_v, ρ_T/2)` exactly once, which
covers every possible meeting `p*`. That crossing lies within `ρ1/2` of `P̂_v`,
while both segments' own read ends lie farther than `R_v + F` (their images
farther than `R_v`), so it lies on both denoted segments. For two
lines, `|p − q| ≥ max(|p − O|, |q − O|)·s1` holds along the whole lines
(constant tangents), so any meeting lies within `2η/s1 < ρ1/2` of `O`.

### 6.5 Nesting candidate points

`sectionaudit.Nesting` classifies each loop by the first of its walk starts the
ray test decides: once S7 has proven no crossing or contact, every point of a
loop classifies the whole loop, and a later start is read only where the ray
test cannot decide an earlier one (a start on another loop's carrier circle, as
a window's start on a concentric window's circle). Without it the wheel's union
audit, and any modify audit of the windowed disc, answered S9.

## 7. Lateral `Fillet` and `Chamfer` of a recorded receiver (PR 1)

### 7.1 Dispatch

In the prism path of `Fillet` and `Chamfer` (`internal/brepmodify/fillet.go`,
`chamfer.go`), a receiver with a nonzero `SectionDelta` and a valid record
(`prismrecord.CarrierRecordOf` on the receiver body) takes the carrier route
(`brepmodify.CarrierLateralBlend`) before `RequireExactSection`, when every
selected edge matches a lateral corner (`modifyselect.MatchCorner`) and the
chamfer is symmetric. Any other selection (a cap edge, a cap loop,
`WithTangentChain` reaching a cap, an asymmetric chamfer) is not matched; a
`Chamfer` of complete cap loops then tries §8.1's route, and every other
selection keeps today's routes. A receiver with no valid record keeps today's
behaviour exactly.

- `Fillet`: a Table CB refusal is returned (`ErrUnsupported`). Today's answer
  for these receivers was also a refusal.
- `Chamfer`: a Table CB refusal falls through to `RequireExactSection` and
  brep-modify §2.2's faceted view, exactly as for a receiver with no record, so
  no chamfer that builds today stops building. `ErrDegenerate` (S4, and S5 for
  a fillet) and cancellation are returned.

### 7.2 Corner walks and carriers

The route resolves corner walks with `offset2d.ProfileCornerLoopsBudget`, which
merges exactly collinear line walks (`boundarywalk.ModifyLoopsBudget`;
`SideWalk.Segs` lists the merged segment indices). Each walk's entries map
through `Segs`: a merged walk whose segments or internal junctions carry any
nonzero entry, in any loop that has a selected corner, is refused (CB0), since
`S*` may turn where the record does not and `RewriteLoop` emits one segment per
walk. Otherwise its carrier entry is zero, its start junction is the entry
before its first segment and its end junction the entry of its last. Its
segments lie on one exact line (`exactlyCollinear`), and the carrier is read
from the segment next to the corner: the last for an arriving walk, the first
for a leaving one.

The construction reads carriers from the record, not from walks:
`offset2d.FilletWithRoots` and `offset2d.Chamfer` take, per walk at the
corner, a carrier override built from the recorded segment: a line through the
recorded `Start` with direction `End − Start` normalized, or a circle of the
recorded `Center` and radius (`Radius`, or `|Start − Center|` for an
`ArcSeg`). `offset2d.carrierOf`'s walk-start anchor (for a narrowed flank, the
rounded `Lerp2` cut point) is not read on this route. The turn (S4, CB1) reads
the carriers' tangents at the walked corner from the same data: a line's
normalized `End − Start`, a circle's `rot90` of the normalized
`corner − Center` in the walk's sense. Chamfer feet are set back from the walked
corner point; §7.5 charges the corner's read position.

### 7.3 What the result denotes

The result denotes `S*_out`: `S*` with each selected corner replaced as follows.

- **Exact-carrier corner** (`ε_A = ε_B = 0`). The connector is the float
  evaluation of offset2d's closed forms on the recorded carrier data, which `S`
  and `S*` share: the arc `ArcSeg(ĉ, f̂A, f̂B)` as recorded. This is modify §5's
  convention, which brep-modify §4.4's Claim D already takes as every prism
  fillet's definition. A chamfer at a corner whose `δ_J` is also zero likewise
  takes today's float chord `LineSeg(f̂A, f̂B)`. `S*`'s two carriers end at the
  points `p_A`, `p_B` nearest the float feet, and each joins the connector by
  the chord from `p` to its foot (a junction path, §3.2).
- **Any other corner.** The exact blend of `S*`: the radius-`r` circle tangent to
  `S*`'s two carriers on the material side, centre `c*` the denoted offsets'
  crossing on the branch the construction took, feet the tangent points; or a
  chamfer whose foot on `A` lies at arc length `d` back from the end of `S*`'s
  segment `A`, and whose foot on `B` at arc length `d` forward from the start of
  `S*`'s segment `B`, joined by a chord. Each foot is measured from its own
  segment's end; where `S*` closes the corner with a junction path, there is no
  single corner point.

Exactly these entries rest on the convention and are zero: the connector's `ε`
at an exact-carrier fillet corner and at an all-zero chamfer corner. Each foot
there is charged its exact residual from its recorded carrier,
`r_f = |f̂ − p|` (rational for a line; `| |f̂ − C| − R |` from outward-rounded
enclosures for a circle), since the re-emitted wall ends on `S*`'s carrier and
the float foot does not lie on it. Every other entry charges the float
construction against the exact one (§7.5).

### 7.4 Corners on exact carriers (Claim D′)

**Claim D′.** At a selected corner with `ε_A = ε_B = 0`, under Table CB, the
fillet of `S` at that corner is `S*_out`'s at the matching corner. Its arc
carries no displacement, and each foot carries only its residual `r_f`.

Proof. `S*` has the same two carriers (CR2 with `ε = 0`), and §7.2 builds the
offsets from them alone. The construction reads the corner point only in four
decisions: the turn's sign (S4 and the offset side), which offset root is
nearest the corner, the connector's sense, and S6's cutback comparison. Every
point of `S*`'s junction path there (both of its segment ends included) lies
within `δ_J` of the matched point of the read path (CR3), and the walked corner
within `a_J = WalkEndBoundAllow` of the read end. CB1, CB3, CB4 and CB5 make
each decision the same for every corner point within `δ_J + a_J`, so the
construction on `S*` takes the same branch and evaluates the same closed forms
on the same data.

### 7.5 Corners with a displaced carrier or junction

Corner `J` joins walk `A` (arriving) to walk `B` (leaving), with entries `ε_A`,
`ε_B`, `δ_J`, fillet radius `r` or setback `d`. Every quantity is a float
rounded up by `proofbound` helpers, or exact over `big.Rat` rounded once.

**Offset displacement.** For a line of length `ℓ`, a point at parameter `s`
moves by at most `ε·(|1 − s| + |s|)`, and the unit normal by at most `4ε/ℓ`. Over
a ball of radius `ρ` about `ĉ`, whose projection falls `τ ≥ 0` outside the
recorded `[Start, End]`, the `r`-offset moves by at most
`e = ε·(1 + 2(ρ + τ)/ℓ) + 4rε/ℓ`. For a circle the offset circle's centre and
radius together move by at most `e = ε`.

**Centre (CB2).** Compute exactly the residuals `q_A`, `q_B` from `ĉ` to the
recorded offset carriers (outward-rounded square roots). Apply Lemma T to the
denoted offsets with `p = ĉ` and `q_i + e_i` as the two distances: start with
`ρ = 4·(q_A + q_B)` (at least `2⁻⁶⁰`); compute `e_A`, `e_B` at `ρ`; read `s` over
`B(ĉ, ρ + max ε)` and set `s* = s − θ_A − θ_B` (`θ` of §6.1). An offset
circle's radius `R ± r` is irrational for an `ArcSeg` (`R = |Start − Center|`),
so `s` is read by `prismcells.SinLowerBounds`, which takes each circle as its
exact centre and a radius enclosure `[R_lo, R_hi]`: it evaluates `sinLower`'s
bound at `R_hi` (every circle form falls as the radius grows) and its
half-radius test at `R_lo`. Then
`L = (q_A + e_A + q_B + e_B)/s* + min(q_A + e_A, q_B + e_B)`; accept when
`s* > ContactEps` and `2L ≤ ρ`, else set `ρ = 4L` and retry, at most four times.
On acceptance `c*` exists, is unique in `B(ĉ, ρ/2)`, and `D = |ĉ − c*| ≤ L`. The
float centre's own error (`|ĉ − c|`, with `c` the recorded offsets' exact
crossing) is inside `D`.

**Fillet feet.** With `ρ_f` the exact distance from the float foot `f̂` to the
exact foot of `ĉ` on the recorded carrier (rational for a line; radial and
tangential residuals from enclosures for a circle):

- line: `Δf = ρ_f + D + e + 4rε/ℓ` (projection is 1-Lipschitz, and the foot
  of `c*` on `S*`'s line is `c* − r·n*`);
- circle of recorded radius `R`, with `R_c` an exact lower bound on `|ĉ − C|`:
  `Δf = ρ_f + ε + (R + ε)·2(D + ε)/R_c`.

**Fillet arc.** `ε_arc = D + | |ArcSeg.Start − ĉ| − r |`, the second term from
outward-rounded enclosures; `ArcSeg.Start` is `f̂A` for a counter-clockwise arc
and `f̂B` for a clockwise one (§7.7).

**Chamfer feet.** Each foot is set back from its own walk's read end at the
corner (segment `A`'s read end, segment `B`'s read start), whose distance to
`S*`'s segment end there is at most `δ_J` (CR3). With `ρ_f` the exact distance
from `f̂` to the exact foot set back `d` from that read end along the recorded
carrier: a line foot
`Δf = ρ_f + δ_J + 4dε/ℓ`. A circle foot with `ε = 0` slides along the exact
circle by at most `R·δ_J/(R − δ_J)` (brep-modify §4.4's Claim C), so
`Δf = ρ_f + R·δ_J/(R − δ_J)`; with `ε > 0` and recorded radius `R ≥ 2ε` (else
CB0), `Δf = ρ_f + ε + (R + ε)·(π(δ_J + ε)/R + dε/(R(R − ε)))`.

### 7.6 Decision margins (Table CB)

Each row only refuses: failing it refuses with `ErrUnsupported`, naming the row,
the corner and the numbers (§9). `a_J` is the walked corner's
`WalkEndBoundAllow`. `θ` at the corner is `4ε/ℓ` for a line and
`2(δ_J + a_J + ε)/(R − ε)` for a circle: the tangent at `S*`'s corner equals the
recorded circle's tangent at the radial image of that corner, which lies within
`δ_J + a_J + ε` of the walked corner.

| CB | Condition |
|---|---|
| CB0 | A displaced walk at the corner covers several recorded segments (§7.2), a carrier is not a finite line or circle, or a displaced circle has `R < 2ε`. |
| CB1 | S4 turn: `|cross| > tol + θ_A + θ_B + 2⁻⁴⁸` (brep-modify PD5a with these `θ`; `2⁻⁴⁸` covers the float tangents of §7.2). |
| CB2 | Fillet: §7.5's centre acceptance at the chosen root and, where a second distinct root exists, at that root; skipped when `ε_A = ε_B = 0`. |
| CB3 | Fillet root: of two distinct roots at distances `d₀ < d₁` from the walked corner, `d₁ − d₀ > 2(δ_J + a_J) + D₀ + D₁ + 2⁻⁴⁸·(d₀ + d₁)` (PD5b), with `D₀ = D₁ = 0` under Claim D′. |
| CB4 | Fillet sense: `|rot90(f̂A − ĉ)·t_A| > Δf_A + D + r·(θ_A + 2⁻⁴⁸)` (PD5c); under Claim D′, `D = 0` and `Δf_A = r_f`. |
| CB5 | S6: every selected corner's cutbacks are widened by `2·T(δ′)`, `δ′ = max(δ_J + a_J, Δf_A, Δf_B, ε)` (under Claim D′, `Δf = r_f`), and every unselected junction with `δ > 0` at the end of a walk a selected corner trims widens that walk by `2·T(δ + a)`; `T(x) = x` on a line walk and `R_lo·x/(R_lo − x)` on a circular walk with `R_lo = R − ε` (PD6). |
| CB6 | S5 with displacement: an inward circular carrier needs `R − r > tol + ε + D`. |
| CB7 | Audit transfer, with `h_loop[l]` Lemma H's loop bound of the result record and `F_out` the rewritten section's contact floor (`sectionaudit.ContactFloor`): `4·max_l h_loop[l] < F_out`; every pair of non-adjacent segments of the rewritten section, pairs between kept (cloned) loops and rewritten loops included, has certified gap `G > F_out` (§6.2, `SegGapLower` with the bounding-box prefilter); and every rewritten loop's `|signed area|` exceeds `proofbound.SectionDisplacementArea(h_loop[l], walks, perimeter)`. |

CB5 holds because S6 compares a walk's remaining length with the cutbacks that
claim it. On `S*` each foot or end lies within its `δ′` of the recorded one, and
its position along the walk moves by at most `T(δ′)`; the carrier's own move
shifts positions on a circle by at most `T(ε)`. CB7 proves contact-freedom of
the rewritten section with the certified gap, not with S7's float
`segMinDist`, so it does not rest on that test's near-end crossing gap (§6.2).
A pair certified farther apart than `F_out` stays more than `F_out/2` apart under
a pointwise move of at most `h_loop` each (Lemma H, Corollary W), so no contact
appears on `S*_out`, nesting does not change (§6.5's candidate-point argument
then rests on this certified verdict), and no loop's area can change sign. Each
`h_loop[l]` is at most `SectionDelta_out` (§7.7); the row reads each loop's own
bound. Modify §5's own S7, used by every other modify op, keeps its float test;
fixing its near-end gap is a follow-up outside both PRs
(`agent-docs/missing-features.md`, §11).

### 7.7 Result record

`offset2d.RewriteLoop` emits, per loop with a blend, one segment per walk and
then the connector of the corner at that walk's end; a loop without a blend is
cloned and keeps its entries. Per emitted segment:

- A walk with no blend at either end whose single source segment is whole is
  re-emitted with its own data (`OriginalSegment` reproduces the pinned ends);
  it keeps its `ε`.
- Any other walk is re-emitted as a whole segment through its two float ends.
  Each emitted end's displacement `d` is `Δf` at a foot of a displaced corner,
  `r_f` at a foot of a convention corner (§7.3), or, at an untouched end, the
  receiver's junction `δ` plus the exact distance from the emitted read end to
  the receiver's read end there. A line takes `ε = max(d_s, d_e)`.
- **Arcs in record-field order.** `offset2d.OriginalSegment` and the fillet
  connector both record through `offset2d.ArcSegment`, which stores a clockwise
  walk reversed: `Start` = walk end, `End` = walk start, `TStart, TEnd = 1, 0`.
  So every arc rule below reads the ends in the `ArcSeg`'s field order, not the
  walk's. Let `d_Start` and `d_End` be the displacements of the ends stored in
  `ArcSeg.Start` and `ArcSeg.End`.
  - A re-emitted circular walk keeps its recorded centre and takes
    `ε = 2·ε_src + d_Start` (its new radius is `|ArcSeg.Start − C|`, within
    `d_Start + ε_src` of `R*`, and its centre within `ε_src`).
  - A fillet arc takes `ε_arc = D + | |ArcSeg.Start − ĉ| − r |` (§7.5; zero under
    Claim D′).
  - An `ArcSeg`'s read end on `ArcSeg.End`'s side is `ArcSeg.Start`'s radius on
    `ArcSeg.End`'s bearing (§3.2), so that side's displacement is
    `d_End + ρ`, with `ρ` the new segment's radial residual. For a clockwise walk
    that side is the walk's start.
- A chamfer connector `LineSeg(f̂A, f̂B)` takes `max(Δf_A, Δf_B)` (zero at an
  all-zero corner).
- A junction at a selected corner's foot takes the larger of the two adjacent
  segments' displacements there (including any `ρ` above). Every untouched
  junction takes the receiver's junction `δ` plus `τ`, the path rule's exact
  distance (CR3) between the new junction path, built from the emitted read
  ends, and the receiver's path there, each closed by the evaluator's own rule
  as §5.4 states: for two chords `τ` is the larger read-end distance, and an
  untouched line–line junction whose emitted read ends differ may close through
  the supports' corner. Both paths are recorded data, so no margin on the
  1024-chord test is needed; `S*`'s path enters only through the receiver's
  CR3.

**Arc read ends.** Every displacement of an emitted `ArcSeg`'s end is measured at
its read end (§3.2), never at the pinned field. On `ArcSeg.Start`'s side the
read end is the `Start` field. On `ArcSeg.End`'s side it is `Start`'s radius on
`End`'s bearing, so:

- at an untouched end, the displacement is the receiver's junction `δ` plus the
  distance between the emitted read end and the receiver's read end there.
  Where the two share the centre and the `End` field (a re-emitted walk), that
  distance is `|R_emit − R_recv|`, rounded outward from enclosures of the two
  radii (`RatSqrtDown`/`RatSqrtUp` of `|Start − Center|²`, as `circularRadius`
  reads them); otherwise it is the distance between the two read ends'
  enclosures, rounded up. It is not the walk end's `WalkEndBoundAllow`, which is
  zero at a natural-bound end (`boundarywalk.WalkOf` pins it), and it is not the
  new segment's radial residual `ρ_new` alone: the receiver's own `ρ_recv` moves
  the receiver's read end as well;
- at a foot or connector end, the displacement is `Δf` (or `r_f`) plus `ρ_new`,
  as the bullet above states;
- a segment re-emitted with its own data (`unchanged`) and not snapped has the
  receiver's read ends and adds nothing.

**Seam snap.** Before the record is made, `closeSeams`
(`internal/brepmodify/carrier_blend.go`) moves a rewritten segment's walk start
onto the previous segment's walk end where the two differ only by a walk end's
rounding (a circle's seam reads `sin(2π)·R` at parameter 1). It rewrites only a
natural-bound field (`seamStartAt`). Its `seamMove` records the distance each
read end of the moved segment moved, on both sides, from enclosures rounded up:
`Read` at the walk start and `ReadEnd` at the walk end. Moving a natural
`ArcSeg`'s `Start` changes its radius, so its `End`-side read end moves too;
moving a reversed `ArcSeg`'s `End` (its walk start) moves the read end that
reads `Start`'s radius on the new bearing, which carries the old residual
`ρ_old` and the new one. `seamMove.Carrier` stays how far the rewritten field
moved. `carrierRewriteRecord` adds `Read` to the moved segment's start
displacement (`emitted.sd`, a connector's included), `ReadEnd` to its end
displacement (`emitted.ed`), and `Carrier` to its `ε`. `junctionEntry` returns
the receiver's `endJ` alone only when neither neighbour moved on that side.

"Arc read ends" measures every read-end distance on the emitted segment after
the snap, the segment the record describes, and the snap's moves (`Read`,
`ReadEnd`, `Carrier`) are still added on top. A snapped segment's move is then
counted twice: once inside the post-snap distance and once through `seamMove`.
That overcharges by at most the snap's own move, a few ulps, and is sound;
`carrierRewriteRecord` needs no pre-snap profile.

On main the record charges `WalkEndBoundAllow` (zero there) at an untouched
`End`-side end, adds only `ρ_new` (and none on the `unchanged` arm), and
charges no snap move to `ed`. A published junction entry can then fall short by
`ρ_recv`, `ρ_old`, or a snap's `End`-side move, each a few ulps times `R`. PR 2
replaces those charges with the two rules above, with §10's tests that each
entry covers its read-end distance.

`SectionDelta_out` is the larger of the largest entry and `max_l h_loop[l]`
(CR4), rounded up, and zero when every entry is zero. Kept loops (cloned, not
rewritten) count with the receiver's entries, so a displaced kept loop keeps the
result displaced. The result is a `prismPayload` with that `SectionDelta`, the record (when
nonzero), and the receiver's frame, levels, level displacements and placement.

Proof. Claim D′ or §7.5 places every connector and foot of `S*_out`, and CR3
every untouched junction. Each emitted segment runs between two read ends
within the stated displacements of `S*_out`'s junctions, on a carrier within
the stated `ε` (CR2, CR3). CB1–CB7 make the rewrite of `S*` the same rewrite
(CR1), and each arc's Lemma H condition is checked before the record is set.

## 8. Second increment (PR 2): cap-loop `Chamfer` of exact loops

### 8.1 Admission and dispatch

The route takes a `Chamfer` whose selection is every edge of one or more
complete cap loops (reach §8.3) of a prism receiver with a nonzero
`SectionDelta` and a valid carrier record, when every selected loop is exact.
The motivating input is §1's windowed disc after the root fillet and the bore
`Cut`: its window loops are the disc's own loops, which §5.3, §7.7 and §5.4
carry with all-zero entries, and its outer loop carries the teeth's
displacement.

In `internal/brepmodify/chamfer.go` the route sits in the block that holds §7's
lateral carrier route: after route P's displaced arm (`route.Carriers`) and
before `pp.RequireExactSection`, when `pp.SectionDelta != 0`, `route.Prism ==
nil` and the receiver is not a straight `Sweep` reduction. The block reads the
record once, with `prismrecord.CarrierRecordOf(budget, b)` on the receiver body
`b`, and hands it to both routes. A record that is missing, stale, or owned by
another body (a `Transform` or `Copy` of the receiver, §3.4) answers false, and
both routes are skipped. The lateral route runs first, unchanged, and keeps its
`asym == nil` condition. The cap-loop route (`brepmodify.CarrierCapLoopChamfer`)
runs when the lateral route did not match or returned a `*CarrierRefusal` (a
Table CB refusal, which a `Chamfer` lets through), for a symmetric or an
asymmetric chamfer (reach §8.3.1):

1. A single straight cap edge (`straightPrismCapEdge`) does not match.
2. `ClassifyChamferSelection(ctx, pp, CapsOf(b), sel, edges)` runs. An error
   other than cancellation, or a lateral answer, does not match.
3. Every loop in `startLoops ∪ endLoops` must satisfy `cr.ExactLoop(l)`. Any
   other selected loop does not match.
4. The setbacks are the exact path's: one `capblend.Setback` on both caps, or
   `asym.CapSetbacks`. An error other than cancellation does not match.
5. `LoopDelta[l] = cr.LoopH(l)` for every loop `l`, in the index space the
   record and `capblend.Payload.Loops` share (0 the outer loop, `i+1` hole `i`).
   It is zero on every selected loop, and finite and at most `SectionDelta` on
   every loop (CR4, which `CarrierRecordOf` checks).
6. `capblend.BuildCheckedDisplaced(ctx, env, pp, start, end, startLoops,
   endLoops, loopDelta)` runs `BuildChecked`'s gates with §8.3's transfer
   condition and builds §8.4's payload through the shared builder.

A route that does not match continues to `RequireExactSection` and brep-modify
§2.2's faceted view exactly as today. Of the route's own refusals only
cancellation is returned. Every other refusal falls through to the faceted
view: each `ErrUnsupported` gate, both offset audits (their verdicts speak about
`S`, not `S*`), §8.3's transfer condition, and SX6's `ErrDegenerate`
(`capband.ChamferDropError` of `capblend.MixedOffsetProfile`). SX6 reads only
selected loops, so it does say the denoted chamfer does not exist; it falls
through all the same, so the route changes no answer except by building an
exact body, and the faceted view gives that selection its own answer as today.
So no chamfer that builds today stops building, and `BuildCheckedDisplaced`
needs no marker to tell SX6 from the audits' S8/S9 `ErrDegenerate`.

### 8.2 What the result denotes

Let `S` be the receiver's recorded section, `S*` the section it denotes, and
`B(Σ)` reach §8.3's cap-loop chamfer of a section `Σ` with the receiver's frame,
placement, levels and level displacements, the selected loops and the setbacks.
The result denotes `B(S*)`.

**Claim E.** Under §8.1 and §8.3's transfer condition, `B(S*)` passes every gate
`B(S)` passes. Its selected loops, with their bands, cap contours and trimmed
walls, are `B(S)`'s. Each other loop `l` of `B(S*)` is `S*`'s loop `l`, whose
walls run the whole sweep, and Lemma H's loop map moves every point of the
recorded loop `l` onto it by at most `LoopDelta[l]`.

Proof. A selected loop is exact, so `S*` holds it point for point (§3.4,
`ExactLoop`). Reach §8.3 builds a selected loop's band, cap contour and trimmed
wall from that loop, its setbacks and the two levels alone, which `B(S)` and
`B(S*)` share, so they are equal; `BandDelta`, the contour's own float
displacement, is charged as today. Every gate that reads only selected loops,
setbacks and levels (SX6, SX7's band reach, SX13 both halves, SX14) answers the
same on `S*`. A non-selected loop enters `B` only as a wall over `[Z0, Z1]`, as
a boundary of both caps, and in the two offset audits. Lemma H's "Loops"
paragraph (§3.3) parametrizes the recorded loop and `S*`'s loop over one circle
with matched points within `h_loop[l] = LoopDelta[l]`. §8.3 transfers the
audits. §8.5 to §8.7 charge `LoopDelta[l]` in every reading of loop `l`.

### 8.3 Transfer of the gates

| Gate in `BuildChecked` | Reads non-selected loops | Transfer |
|---|---|---|
| `MixedOffsetProfile` (SX6) | resolves every loop's corner walks (`offset2d.ProfileCornerLoopsBudget`), which can only refuse, and clones the non-selected loops | none: a refusal falls through (§8.1) |
| SX7 band reach | no: levels and side setbacks | none |
| `offset2d.AuditOffsetSectionBudget(budget, pp.Profile, mixed)` | S8, S7/SX12 and S9 over every loop of `mixed` | the transfer condition on `mixed` |
| `AuditSetbackSpan` | its own audit of the span-top mixed profile, when a cross setback carries a conversion rounding | the transfer condition on that profile |
| `RequireLevelsSeparate` (SX13 axial) | no: levels only | none |
| `RequireCornerLoci` (SX14) | no: selected loops only | none |
| `Build`: `capband.BandRadius` (SX13 radial) and each band | no: selected loops only | none |

**Transfer condition** on a mixed profile `M`, run right after the audit of `M`
passes, with `F` the contact floor of `M` (`sectionaudit.ContactFloor` over
`sectionaudit.EntriesOf` of its loops):

- **T1.** `4·max_l LoopDelta[l] < F`.
- **T2.** Every two segments of `M` that share no vertex, at least one of them on
  a loop with a nonzero `LoopDelta`, have certified gap `G > F` (§6.2:
  `sectionaudit.SegGapLower` of their walks less both allowances `c_e`, after
  the bounding-box prefilter). Every segment of `M` has `4·c_e < F`. A segment
  of a selected loop's cap contour is computed, not recorded, so its `c_e` is
  §6.2's walk allowance plus that loop's contour displacement from
  `capblend.Payload.LoopContourDelta`. On `BuildChecked`'s profile the
  arguments are `(ctx, l, loop, Dc, DcDelta)` of the loop's setback, the number
  `Build` later stores as the loop's `BandDelta` and `LoopBandDelta` reads. On
  `AuditSetbackSpan`'s span-top profile they are `(ctx, l, loop, top.Dc,
  DcDelta)`, with `top` from `setbackSpanTop`. That profile is the held offset
  at `top.Dc`, and the family argument below covers every smaller offset, so a
  zero span would suffice; keeping `DcDelta` encloses the contour over a wider
  span of offsets and is conservative. The term
  bounds the contour `MixedOffsetProfile` builds: both it and
  `capband.LoopContourDelta` offset the loop by `offset2d.BuildLoop(walks, 1,
  d, offset2d.ShellTol)`, and `capband.ContourDisplacement` bounds every point
  of that construction against the exact offset. The two read their walks
  separately (`offset2d.ProfileCornerLoopsBudget` against `capband.LoopWalks`),
  so T2 checks that each selected loop's two walk lists name the same segments
  and refuses otherwise.
- **T3.** Every loop with a nonzero `LoopDelta` has an area enclosure
  `prismcells.LoopAreaEnclosure(budget, loop, LoopDelta[l])` that excludes zero.

T2 is `prismcells.CertifiedContact`'s test with a per-loop displacement:
`CertifiedContactLoops(budget, loops, hLoop, extra []float64)` checks T1
against the largest `hLoop`, adds `extra[l]` to the `c_e` of every segment of
loop `l`, and skips a pair whose two loops both carry zero `hLoop`. Its
`4·c_e < F` check runs on every segment, those of zero-displacement loops
included: whenever some loop is displaced, every segment of every other loop
enters a tested pair with it (two loops' segments share no vertex), so a
narrower check would skip no segment that a tested pair reads.
`CertifiedContact(budget, loops, hMax)` stays CB7's and §5.3's call, with every
loop at `hMax` and no extra. T3 calls `prismcells.LoopAreaEnclosure` directly,
as `carrierAuditTransfer`'s area loop does (`capblend` cannot import
`brepmodify`). A failure falls through (§8.1).

Proof. A pair from two exact loops is the same pair on `S*`, so the audit's
verdict holds as it does for an exact receiver; its float S7 limit is that of
every modify audit (`agent-docs/missing-features.md`). A pair with a displaced
loop is certified more than `F` apart, and each of its segments moves by at most
`LoopDelta < F/4` (Corollary W), so on `S*` it stays more than `F/2` apart: no
crossing or contact appears (S7, SX12), every loop keeps its winding number
about every other (S9), and S9's candidate-point verdict (§6.5) rests on this
certified gap. S8 compares each mixed loop's orientation with the receiver's;
a non-selected loop is the receiver's own loop, so S8 passes on it trivially.
T3's job is the build: `Build` orients each loop's walls and signs its slab and
cap terms from the recorded loop (outer loop positive, holes negative, walls
facing out of the material), and T3 proves `S*`'s loop has the same orientation,
so those signs and wall orientations hold for `B(S*)`. SX12 audits the whole
offset family by its member at the
full setback: a selected loop's offset at fraction `s ≤ 1` is the same on `S`
and `S*`, and its distance to any fixed feature does not grow with `s` (reach
§8.3). At `s = 1` its distance to a displaced loop of `S*` exceeds
`G − LoopDelta > 3F/4`, so no member meets that loop.

### 8.4 The displaced payload

```go
type DisplacedPayload struct {
	Base      Payload   // the cap-loop chamfer record, as built today
	LoopDelta []float64 // per loop: the receiver record's h_loop, zero on every selected loop
}
```

**The type is the guard.** `Base` is a named field, never embedded, so no method
of `Payload` is promoted. `DisplacedPayload` has exactly these exported methods:
`Transform` and `Placed` (`topo.Payload`), `SurveyClass` (answering
`"displacedCapBlendPayload"`), `ExtentAlong` and `ExtentBoundedAlong` (§8.6). A
type switch or assertion on `capblend.Payload` never matches it, and each reader
not named in §8.8 gives its existing answer for a payload class it does not
know. An interface reader finds only those five methods: `extent.ThroughAll`
meets the refusing `ExtentAlong`, and no reader finds `AxialDelta`,
`DenotedCapLevels` or `ProvesSimple`. An unknown payload's existing answer is
not always a refusal: `booloperand.RequireVolumeProof`'s default arm admits it.
That reader fails closed only because §8.8 names it, and so does every reader
whose default answer would read the body as exact (`booleaneval.sourceGroup`,
rule DW).

**Construction.** `capblend.Build(ctx, env, cbp)` and `capblend.BuildDisplaced(ctx,
env, cbp, loopDelta)` share one builder. It refuses with `ErrUnsupported` a
`loopDelta` whose length is not the loop count, an entry that is negative, not
finite or nonzero on a selected loop, and a base with `Draft`, `Fillet` or
`LoftSource` set. With every entry zero it stores a plain `Payload`: `S*` then
equals `S` on every loop. Otherwise the body's payload is the
`DisplacedPayload`, whose `Base` carries `Patches` and `BandDelta` as today.
`LoopDelta` is stored once, and every reader charges the stored number, as every
reader of `BandDelta` does (`LoopBandDelta`).

**Placement.** `Placed` sets `Base.Xform` to the composed motion and rebuilds
through `BuildDisplaced` with the same `LoopDelta`. `LoopDelta` bounds a
plane-local displacement of the record, and the record moves with the body, so
a placement keeps it, as a prism's placement keeps `SectionDelta`. §3.4's
carrier record differs: `Placed` drops it because §7 and §8 admit a route on it,
while every reader of `LoopDelta` only charges it.

**Later modify ops.** `capblend.RequireNotReceiver` (SX10 in `Fillet`, `Chamfer`
and `Shell`) matches a `DisplacedPayload` as well as a `Payload`, so a later
modify op refuses with SX10's reason.

### 8.5 Measurements

The shared builder charges each loop `l` with `LoopDelta[l] > 0` (never a
selected loop) as `prismrecord.Build` charges `SectionDelta`. With
`n_l = len(loop.Segments)`:

- **Walls, junction vertices and wall lengths.** That loop's
  `prismrecord.BuildLoopSidesAs` call takes `ppFor.SectionDelta = LoopDelta[l]`,
  which charges every walk length `SectionDisplacementLength(LoopDelta[l], 1)`,
  every rim vertex `LoopDelta[l]` and every side face area through its walk
  length.
- **Perimeter.** `P_l` is the upper end of the `loopLen` that call returns (today
  `capblend.Build` discards it).
- **Enclosed area.** `A_l = proofbound.SectionDisplacementArea(LoopDelta[l], n_l,
  P_l)` is added to `loopArea.Bound` before the slab multiply, so `BoundedMul`
  carries it into the slab volume (times the straight height) and into the slab's
  axial moment (times the height and `zMid`).
- **In-plane first moments.** Before the height multiply, `loopMu.Bound` and
  `loopMv.Bound` each take `A_l·(M_l + LoopDelta[l])`, with `M_l` from
  `momentinput.CoordinateEnvelope(momentinput.Profile{Outer: loop}, ...)`.
- **Caps.** Each cap's area bound takes `A_l` once: a non-selected loop bounds
  both caps with its own boundary.
- **Box and centroid.** The box is §8.6's. The centroid's formula bound reads the
  charged moments and volume, and its geometry bound reads the charged box.

Every value is rounded up. Proof. The two sections' symmetric difference lies
in the union over `l` of the `LoopDelta[l]`-neighbourhoods of the recorded
loops (Corollary W: a point farther than `LoopDelta[l]` from loop `l` keeps its
winding number about it), which `SectionDisplacementArea` bounds per loop. A
first moment differs by at most the area of that difference times the largest
`|u|` or `|v|` in it, at most `M_l + LoopDelta[l]` (`M_l` bounds `|u| + |v|` on
the loop). A wall's walk ends and carrier lie within `LoopDelta[l]` of `S*`'s
(Lemma H), which `SectionDisplacementLength` and the rim vertex bounds charge.

### 8.6 Extent and box

`DisplacedPayload.ExtentBoundedAlong(ctx, g, work)` is the base reading with,
on each loop with a nonzero `LoopDelta`, `ProductUpper(inPlane, LoopDelta[l])`
added to that loop's `planeAllow` in both of its `take` calls. `inPlane` is the
reading's own upward-rounded in-plane norm of `g`, the factor it already applies
to a contour's displacement. Contour candidates belong to selected loops and
take nothing. The `coordUpper` that the reading hands
`prismrecord.PlacementCoeffAllow` is widened by `max_l LoopDelta[l]`, since a
denoted coordinate can exceed the recorded envelope by that much.
`DisplacedPayload.ExtentAlong` refuses with `ErrUnsupported`, as
`prismrecord.Payload.ExtentAlong` refuses a nonzero `SectionDelta`, so a
through-all stop with this body live refuses as it does for a displaced prism
(prism-boolean §12). `capblend.Bounds` reads the displaced reading on each axis,
so each displaced loop's candidates carry their own `LoopDelta[l]` into the box
bound.

### 8.7 Tessellation

`tessdispatch.Body` takes `capblend.DisplacedPayload` beside the plain payload,
through `capblend.TessellateDisplacedMesh`, which shares `TessellateMesh`'s core
with a `loopDelta` argument (nil for the plain payload). With `D = max_l
LoopDelta[l]`:

- **Reservation.** Before any chord, as `internal/prismtess` reserves
  `SectionDelta` (tessellation §5): `chord' =
  freeform.DownRound(freeform.DownRound(chord − D))`. `chord' ≤ 0` refuses with
  `ErrUnsupported` and prismtess's message; otherwise every walk is chorded
  against `chord'`.
- **Facet bounds.** The term each side face of loop `l` bumps into `faceExtra`
  (its sagitta plus the axial displacement) adds `LoopDelta[l]`, and each cap's
  term (trim, contour displacement and axial displacement) adds `D`, so
  `tessellation.ComposeCapBlendBounds` charges them into every facet bound there.
- **Area slack.** Per displaced loop, with `H` the wall height the wall slack
  reads (`|Z1 − Z0|` for a loop chamfered on neither cap), `P'_l` the upward sum
  of `Length + LengthBound` over the loop's chorded walks and `A'_l =
  SectionDisplacementArea(LoopDelta[l], n_l, P'_l)`: `ProductUpper(
  SectionDisplacementLength(LoopDelta[l], n_l), H)` for the walls and `A'_l` for
  each cap.
- **Occupied volume.** Where the proof is built (`proveVolume`, and
  `OccupiedVolumeAdmission(dp.Base)` admits), `terms` takes
  `Σ_l ProductUpper(A'_l, H_up)`, with `H_up` the sweep height `|Z1 − Z0|`
  rounded up plus `Z0Delta + Z1Delta`, rounded up. Where the admission refuses,
  `SymDiffOK` stays false, as today.

`booloperand.RequireVolumeProof` takes the plain payload's arm on `dp.Base`, so
it and `meshbool.OperandSymDiff` keep naming the same classes: a body the
admission passes composes in a mesh `Boolean`, and any other refuses before it
is meshed. `booloperand.sectionDisplacement`, which sets an operand's chord
floor (`ChordShare`), answers `max_l LoopDelta[l]` for a `DisplacedPayload`.

**Mesh Boolean source tags (rule DW, displaced walls).** A mesh `Boolean` copies
each operand face's surface onto its facets as a retained tag
(`booleaneval.sourceGroup`, `facetedrecord.SourceTag`), and `SourceTag` reads the
face alone. A side wall of a displaced loop has a recorded `Plane` or `Cylinder`
that lies only within the loop's bound of the denoted wall, both offset and
tilted (a line wall of length `ℓ` by up to about `2·b/ℓ`). Tagging it as
retained with `NormalBound` 0 breaks faceted-source-retention Claim R, and an
`AxialDelta` would not repair it: `facetedrecord` publishes the tag's normal
with no bound, and `MirrorFace` and the retained-blend plane model extrapolate
`AxialDelta` as a parallel offset.

Rule DW therefore declines the tag (no surface, not retained, no `AxialDelta`)
on every displaced wall of a `DisplacedPayload`, `Plane` and `Cylinder` alike: a
`side(l, j)` wall of a loop with `LoopDelta[l] > 0`, and any other non-cap face
whose roles name no single side loop and no band patch of a selected loop (the
band-patch roles are those `Base.Patches` lists). The rule is per loop: a
`DisplacedPayload` holds no per-segment record. Every other face keeps its tag.
`faceLoop` moves from `internal/brepmodify` into a package both `brepmodify`
and `booleaneval` can import (`brepmodify` imports `booleaneval`), which holds
the rule.

`sourceGroup` (defined in `internal/booleaneval/eval_mesh.go`) takes no context
or budget, so the decision is made once per operand where the operand's faces
are walked and handed to `sourceGroup` as a per-face lookup. It reads only the
payload's `LoopDelta` and the faces' roles. The rule runs at both `sourceGroup`
callers: the mesh `Boolean` evaluator's operand-face loop
(`internal/booleaneval/eval.go`) and `booleaneval.View` (`view.go`), the faceted
view the retained edge blend reads. A `DisplacedPayload` operand never reaches
the evaluator's replay path, since `booloperand.ChordShareReplay`'s default arm
refuses it, so the rule needs no replay metering.

**Displaced prisms keep today's tags.** A `prismrecord.Payload` with
`SectionDelta > 0` reaches `sourceGroup` as it does on main: its walls keep
their retained tags with zero departure, which Claim R does not support for a
wall whose carrier is displaced. Neither a per-loop nor a per-segment prism arm
ships in PR 2. A per-segment arm would keep a wall on an exact carrier
(record entry `ε = 0`, the reasoning of prism-boolean §7's "Exact carriers") and
decline the rest, but a fillet's feet leave ulp-sized carrier entries on the
adjacent line walls of a faceted retained blend's operand, so it declines tags
that `TestFacetedRetainedBlendConcaveArcEnd` and two other faceted-retained-blend
apitests need. The prism half stays an open gap in
`agent-docs/missing-features.md`, in the row on displaced classes that reach
`sourceGroup` with uncharged tags, beside the face-level readers outside the
mesh `Boolean` that read a displaced wall as exact.

### 8.8 Readers

| Reader | Answer for a `DisplacedPayload` |
|---|---|
| `tessdispatch.Body` | §8.7 |
| `booloperand.RequireVolumeProof` | the plain payload's arm on `Base` (§8.7) |
| `capblend.RequireNotReceiver` | SX10's refusal (§8.4) |
| `extent.ThroughAll` | `ExtentAlong` refuses with `ErrUnsupported` (§8.6) |
| `verifyrun` wall, undercut and minimum-radius surveys | the default arm: staged, `Suspect` |
| clearance kernel (`pairkernel.newBodyGeomBudget`, `payloadExtent`) | the default arm: no model, so a pair reads `Suspect` unless its boxes decide it |
| tolerance-gate witnesses (`gatewitness.Prisms`) | a new arm: `capblend.WitnessPrisms(dp.Base)` with displacement `AbsSumUpper(Base.AxialDelta(), max_l LoopDelta[l])`. Each witness point lies within that displacement of a point of the denoted body: its loop moves at most `LoopDelta[l]` in the plane, its level at most the axial term |
| tolerance-gate diameter (`gatediameter.DirectBodyGateDiameter`, then `FallbackGateDiameter`) | no direct arm; the clearance kernel builds no model, so `FallbackGateDiameter` reads the witness arm above |
| pair gate points (`gatediameter.BodyGatePoints`) | the default arm reads the witness arm above through `diameter.PrismGatePoints`, else the body's vertices with their published bounds, which carry `LoopDelta[l]` on every vertex of a displaced loop (§8.5) |
| `booloperand.sectionDisplacement` (an operand's chord floor) | `max_l LoopDelta[l]` (§8.7) |
| `booloperand.ChordShareReplay` (a replayed mesh `Boolean`) | the default arm: `ErrUnsupported` |
| mesh `Boolean` source tags and the faceted view (`booleaneval.sourceGroup`, called from the evaluator in `eval.go` and from `booleaneval.View`) | rule DW (§8.7): every wall of a loop with `LoopDelta[l] > 0`, `Plane` or `Cylinder`, keeps no tag; a displaced prism's walls keep today's tags (an open gap) |
| brep route (`brepmodify.brepModifyRecord`) | not a brep record: no route |
| `payloadclass.TypeName`, `verifypublish` | `SurveyClass` |

The packages that build `capblend.Payload` views internally (`breprecord`,
`breploop`, `brepbandmesh`, `brepfilletmesh`, `draftrecord`) never hold a
`DisplacedPayload`, and a draft view never builds one.

## 9. Refusals and fallbacks

| Where | Condition | Answer |
|---|---|---|
| §4 | the authoritative count exceeds the cap | `WithWorkLimit(ErrUnsupported)`, or the caller's silent miss |
| §5.3 | an extended-class set fails anything after U1 | silent miss to the next `Union` route |
| §5.4 | a record condition or the stability audit fails | the `Cut` builds as today, with no record |
| §7.1 | a receiver with a nonzero `SectionDelta` and no valid record | today's answer, unchanged |
| §7.6 | CB0–CB7 on `Fillet` | `ErrUnsupported`, naming `agent-docs/carrier-record-design.md`, the row, the corner's section point and the numbers |
| §7.6 | CB0–CB7 on `Chamfer` | today's route (`RequireExactSection`, brep-modify §2.2) |
| §7.4, §7.5 | S4 and S5 themselves | `ErrDegenerate`, before any CB row (modify §4) |
| §8.1 | a selection that is not complete cap loops, a selected loop with a nonzero entry, or no valid record | today's answer |
| §8.1, §8.3 | any refusal of the route other than cancellation: SX6, every gate, the offset audits and T1–T3 | today's route (`RequireExactSection`, brep-modify §2.2) |
| §8.4 | a malformed `LoopDelta`, or a base with `Draft`, `Fillet` or `LoftSource` | `ErrUnsupported` |
| §8.6 | `DisplacedPayload.ExtentAlong` | `ErrUnsupported` |
| §8.7 | a tolerance at or below the largest `LoopDelta` | `ErrUnsupported`, prismtess's message |

A carrier-route refusal is `ErrUnsupported` because the body exists and this
evaluator cannot prove it: a smaller displacement or a cleaner model builds.

## 10. Tests

PR 1, each beside the code it covers:

- `prismrecord`: `CarrierRecordOf` validity against a changed bit of
  `SectionDelta`, a changed segment, a stale shape, an `h_loop` above
  `SectionDelta`, and a record whose profile shares no slice with the payload;
  fail-closed ownership: `prismrecord.Build` of a recorded payload, a
  `&topo.Body{Payload: p}` copy and a `Placed` copy each answer no record;
  `ExactLoop`; Lemma H's `h` for a line, a circle, an arc and a narrowed circle, and
  `+∞` past its condition; a zero record of a windowed sketch section whose
  fillet arcs carry a radial residual of a few ulps stays valid (the read end is
  `Start`'s radius on `End`'s bearing).
- `prismcells` cap: `CircularSegmentCount` of a whole arc equals the builder's
  count from the same floats at sweeps near `0`, `π` and `2π`; the builder
  refuses a scene its recount puts over the cap; the fourteen-hole plate still
  fits; a wheel-like disc with windows and copied teeth fits.
- `prismcells` audit: Lemma T finds the crossing of two lines and of a line and a
  circle and refuses `2L > ρ`; `SegGapLower` answers 0 for two lines crossing
  `1e-8` of a length from an end (where `segMinDist` answers a positive gap) and
  stays at or below the exact distance of random line and arc pairs; the
  stability audit refuses a displaced edge `F/2` from another, refuses a shallow
  line–circle crossing that comes back within `F`, refuses a ball holding an own
  vertex, sizes `B_v` from the walked end's distance `w_v` (a walked end placed
  off `P̂_v` widens the ball), and passes the wheel-like teeth; V1, V2 and V3 each
  fail to refuse their scene when switched off; V3 pairs a whole circle's last
  vertex with the seam; an edge's `c_e` grows by the exact closing path of an
  own junction whose read ends differ; the stability prefilter's arc box holds
  the endpoint enclosures `SegGapLower` reads; the certified contact test
  refuses a web between two loops and a near-end crossing.
- `patternassembly`: a 17th-turn instance of a tooth carries a record whose
  entries equal its `SectionDelta`; an instance of a source with arcs carries
  `δ_End` at each arc end and a `SectionDelta` at least its `h_loop`; an
  instance of a source holding a clockwise arc (a hole-loop arc) carries
  `δ_End` at that arc's start junction `j − 1`, its `ArcSeg.End` in field order;
  an instance of a source with two narrowed lines meeting at distinct read ends
  carries none; an instance of a displaced source carries none.
- `prismcells` cap callers: an arc-heavy pair that used to exceed the cap
  reaches the pair route and answers its result or its own typed refusal.
- `prismboolean`: the extended `Union` of a disc with two windows and three teeth
  (one rotated copy) keeps two holes and the teeth, matches the volume of the
  same part drawn in one sketch within both bounds, and gives the windows
  all-zero entries; a triangle with a valid record whose own vertex lies on the
  disc circle misses (V1; the guard-disabled demonstration is in `prismcells`);
  a displaced block crossing a square's side 5e-5 from its corner misses (V2)
  and builds with V2 switched off; a near-coincident copy line misses (sketch
  merges it first); two copy flanks crossing one segment
  so close that their denoted crossings could swap miss (V3), including on an
  undisplaced circle; a holed rectangular plate united with a block and its
  translation-pattern copies (not disc-shaped) builds with a record; a holed
  loft set reaches the extended class through `StraightLoftUnionMany`; a
  copied-prism circular join still takes `prismpattern.UnionMany`; the result's
  `SectionDelta` is at least every loop's `h_loop`.
- `prismcells` `Cut`: a clean-nesting `Cut` of a displaced recorded target whose
  matched profile starts its loops elsewhere maps every entry through the tags;
  a narrowed target flank gains its walk allowance; two narrowed target lines
  whose re-recorded junction closes through a corner take the exact path
  distance `τ`; a tool `F/2` from a displaced loop builds with no record; a tool
  with a trimmed `CircleSeg` takes the mesh path.
- `sectionaudit`: `Nesting` decides a loop whose first walk start lies on
  another loop's carrier circle.
- `brepmodify`: Claim D′ (case 1's two-root fillet builds, matches the same
  filleted outline drawn in one sketch within both bounds, and records zero arc
  entries and tiny positive foot entries, each the foot's residual `r_f`); a
  displaced-carrier corner charges a positive `D` below
  `1e-12` mm for a 17th-turn copy; each of CB1–CB7 refuses a constructed
  near-boundary case and is shown to fail by disabling it (CB2: a corner whose
  denoted offsets meet too shallowly for Lemma T; CB6: an inward circle with
  `R − r` just above `r`'s tolerance and below `ε + D`); a CB refusal of a
  `Chamfer` falls through to the faceted view; a lateral chamfer of a recorded
  receiver states its foot entries from §7.5; a lateral fillet of a hole-loop
  corner on a displaced receiver, whose clockwise arc walk is blended at one end
  only, records `ε = 2·ε_src + d` at the `ArcSeg.Start` end and adds `ρ` on the
  `ArcSeg.End` side (§7.7), and a deliberately walk-order computation is shown
  to undercharge it; CB7's certified contact test refuses a rewritten pair whose
  float `segMinDist` reads a near-end crossing as a gap.
- `prismcells` pass-throughs: `MatchCutNoOp` with a tool inside a displaced hole,
  and a contained `Union` returning an operand, each return a body with no
  record.

PR 1, `apitest` (public API only):

- Case 1 (a hole-free disc and one tooth) and case 2 of §1 build analytically:
  every edge is `Line3`, `Arc3` or
  `Circle3`, an edge query `ParallelTo(+Z)` with `Concave()` asserted
  `Exactly(2·teeth)` selects the roots, and `Fillet` builds.
- A wheel-like part: a disc with two windows (each an inner and outer arc, two
  lines and four sketch-fillet arcs), a tooth and two rotated `PatternCopies`,
  the `Union`, the root `Fillet` through `Concave().Exactly(6)`, and a D bore
  `Cut` whose tool records its arc as a whole `ArcSeg`, with every edge
  analytic, a `VerifyAll` mesh whose boundary and volume verify, and a volume
  that matches the same outline drawn in place within both bounds.
- The faceted-loft attempt's shapes (commit `a6c77827d8`) where they fit the
  admitted class.

PR 2, each beside the code it covers. The twin of a displaced build is the same
`capblend.Payload` built with every `LoopDelta` zero.

- `capblend` build: a holed square receiver whose outer loop takes a positive
  `LoopDelta` and whose exact hole is chamfered on both caps builds a
  `DisplacedPayload` storing that `LoopDelta` and zero on the hole. Against its
  twin it holds the same volume value; a volume bound larger by at least
  `A_0·(Z1 − Z0)`; each cap's area bound larger by at least `A_0`; an
  in-plane moment bound larger by at least `A_0·M_0·(Z1 − Z0)`; and at least
  `LoopDelta[0]` on every outer-wall rim vertex. An all-zero `LoopDelta` builds
  a plain `Payload`. A wrong length, a nonzero entry on a selected loop and a
  non-finite entry each refuse. `Placed` keeps `LoopDelta` and answers a
  `DisplacedPayload`.
- `capblend` extent: `ExtentAlong` refuses. `ExtentBoundedAlong` along an
  in-plane axis whose extreme the outer loop holds states a bound at least
  `LoopDelta[0]` above its twin's, and along an axis whose extreme only the
  chamfered contour holds states its twin's bound. The box bound is at least
  `LoopDelta[0]`.
- `capblend` transfer: T1, T2 and T3 each refuse a constructed case and are
  shown to matter by disabling them, as Table CB's rows are. T2's case is a
  displaced outer loop certified less than `F` from a chamfered hole's contour
  while the float audit passes, including a near-end crossing that
  `segMinDist` reads as a gap, and a contour segment whose gap clears `F` only
  without its contour displacement in `c_e`. T3's case is a displaced loop whose
  segment pairs are all adjacent, so T2 tests none of them: a thin triangle of
  height below `8·LoopDelta`, whose area enclosure holds zero.
- `capblend` tessellation: a tolerance at or below `max LoopDelta` refuses.
  Above it, a `VerifyAll` mesh closes, every outer-wall facet and both caps state
  a bound of at least `LoopDelta[0]`, and `VolSymDiff` is at least
  `A'_0·H_up` above its twin's.
- `prismcells`: `CertifiedContactLoops` passes a pair of two zero-displacement
  loops closer than `F` and refuses the same pair when one loop carries a
  displacement; it refuses a zero-displacement loop's segment whose `c_e` is at
  least `F/4`; `CertifiedContact` answers as before.
- `brepmodify` arc read ends: a lateral blend of a recorded receiver whose arc's
  `End` sits a few ulps off `Start`'s radius (`ρ_recv > 0`), blended at its
  `Start` end only, records at the untouched `End`-side junction an entry of at
  least `δ + |R_emit − R_recv|`, the exact distance between the two read ends,
  checked against exact rationals.
- `brepmodify` seam snap: a snap that moves a natural `ArcSeg`'s `Start`, and one
  that moves a reversed `ArcSeg`'s `End`, each record an entry on that side of at
  least the exact distance the read end moved (`ρ_old` included for the
  reversed arc).
- `booleaneval` rule DW. Each case calls the `booleaneval` evaluator directly,
  so the mesh path runs even where a public `Cut` would take an analytic route.
  A mesh `Cut` by a box of a `DisplacedPayload` gives every facet from a
  displaced loop's `Plane` or `Cylinder` wall a group with no tag, while walls of
  exact loops, caps and selected-loop band patches keep their tags; without the
  rule the displaced walls read retained with zero departure. A displaced
  prism's walls (a disc united with a tooth and a rotated copy) keep today's
  tags. `booleaneval.View` of the same bodies decides the same faces.
- No regression: `apitest/chamfer_pattern_socket_test.go`,
  `apitest/faceted_retained_blend_cylinders_gaps_test.go` (two exact cylinders
  unioned, mesh-cut, then a retained-tag fillet or chamfer of the valley) and
  `TestFacetedRetainedBlendConcaveArcEnd` pass unchanged.
- `booloperand`: `sectionDisplacement` answers `max LoopDelta` for the result,
  and `ChordShare`'s floor is at least twice it.
- `brepmodify` dispatch: a cap-loop `Chamfer` of an exact hole loop on both caps
  of a recorded receiver builds a `DisplacedPayload`, and so does an asymmetric
  one. Each of these keeps today's answer, compared with the same call with the
  route switched off: a selected loop with a nonzero entry, a partial or mixed
  selection, a `Placed` copy of the receiver (it carries no record), a T2
  failure, and an SX6 drop (a hole loop whose setback reaches its radius), each
  of the last two falling through to the faceted view. Cancellation is returned.
- §8.8's readers: `Chamfer`, `Fillet` and `Shell` of the result refuse with
  SX10; `ThroughAll` with the result live refuses; the three surveys answer
  staged; the clearance kernel builds no model; `gatewitness.Prisms` answers
  one witness prism per loop with displacement at least `max LoopDelta` plus
  the axial term, and the tolerance-gate diameter reads it through
  `FallbackGateDiameter`; `booloperand.RequireVolumeProof` agrees with the mesh's `SymDiffOK`,
  and a mesh `Cut` of the result by a box at `VerifyAll` composes.

PR 2, `apitest` (public API only): `TestCarrierRecordWheelUnionFilletAndBore`'s
part (two windows, a tooth and two rotated `PatternCopies`, the six roots
filleted, the whole-`ArcSeg` D bore cut), then a `Chamfer` of every edge of both
windows' cap loops on both caps. `decadtest.PayloadClass` answers
`decad.displacedCapBlendPayload`; every edge is analytic
(`requireAnalyticEdges`); the volume matches `wheelInPlace(t, 2, 3, true,
true)` chamfered the same way within both bounds; and a `VerifyAll` mesh
verifies its boundary and volume. Full consumer models stay out of decad's tests
(`agent-docs/testing.md`, Test scope).

## 11. Documents to update in the same change

PR 1:

- `agent-docs/prism-boolean-design.md`: §4.1 (the bore `Cut` order), §4.6 (the
  extended class and its fallback), §8 row 3, §10 (the per-entity count and the
  builder's authoritative count), §12's modify row.
- `agent-docs/general-boolean-design.md` A6: that the readings this document
  defines (§5.3's vertex reading, ASA2, CB2) prove a crossing's existence and
  uniqueness through Lemma T, and that a recorded union result fillets. The A6
  text keeps its own pair reading as it is and does not claim that every A6
  reading has Lemma T's form.
- `agent-docs/brep-modify-design.md` Table RB row RB6, and §2.2 (a CB refusal of
  `Chamfer` falls through to it).
- `agent-docs/modify-design.md` §5 test 4 (the nesting candidate points).
- `agent-docs/mirror-pattern-design.md` §4.3 (the instance's record).
- `agent-docs/missing-features.md` where it lists the displaced-receiver refusal,
  and a new entry: modify §5's S7 contact test reads float `segMinDist`, which
  reports a crossing within `segCrossEps` of a segment end as a positive gap
  (§6.2); moving it to `SegGapLower` is a follow-up outside both PRs.
- `agent-docs/layout/design-docs.md` row for this document, and rows for any new
  root `.go` file.

PR 2:

- `agent-docs/modify-reach-design.md` §8.3 (`capblend.DisplacedPayload`) and
  §8.4 (its measurements and tessellation).
- `agent-docs/tessellation-design.md` §5 (the `LoopDelta` reservation).
- `agent-docs/prism-boolean-design.md` §8 row 3 (a displaced result with a valid
  record takes a cap-loop `Chamfer` of exact loops), §12: the `Fillet` / `Chamfer` row (a
  cap-loop `Chamfer` of exact loops builds a `DisplacedPayload`) and §8.8's
  readers, and "internal/capblend contour (boundary note)" (`LoopDelta` is the
  third term beside the contour displacement and `sectionDelta`).
- `agent-docs/brep-modify-design.md` Table RB row RB6 and §2.2 (a cap-loop
  `Chamfer` of exact loops of a recorded receiver reaches the faceted view only
  through §8.1's fall-through).
- `agent-docs/modify-reach-design.md` Table SX row SX10 (it also refuses a
  `DisplacedPayload` receiver) and Table DX's survey rows DX7 to DX9 (a
  `DisplacedPayload` is staged).
- `agent-docs/tessellation-reach-design.md` §7 (T7: the `LoopDelta` reservation,
  facet charges, area slack and volume term).
- `agent-docs/faceted-source-retention-design.md` §3 and §11 (rule DW declines
  the tag of every displaced wall of a `DisplacedPayload`, per loop, at both
  `sourceGroup` callers; a displaced prism's walls keep today's tags, an open
  gap).
- `agent-docs/missing-features.md`: the displaced-receiver modify row changes in
  the same commit as the code, since its "Today" cell names the cap-loop
  `Chamfer`'s current answer. Its rows for chorded loft plane normals, other
  displaced classes' source tags, face readers of displaced walls and
  displaced-junction convexity stay open.

## 12. Decisions the user may want to overturn

- **Copies stay displaced.** A frame-keeping instance denotes the source
  section under the exact motion (a rotation by exactly `i/Count` of a turn, or
  the exact translation; mirror-pattern §6.2), and its rounded record is
  displaced from that. Every reading of a part built from it stays
  `Approximate` and declines where prism-boolean §12 says a displaced prism
  declines (the clearance kernel and the surveys read `Suspect`). Treating the
  rounded record as the instance's denotation would make those parts exact, at
  the cost of every pattern's claim about its motion.
- **The stability audit and CB7 use a certified gap** (`SegGapLower`), not the
  modify audit's float `segMinDist`; so does the extended `Union`'s contact test
  and §8.2's transfer. Modify §5's own S7 keeps its float test until the
  follow-up `agent-docs/missing-features.md` lists.
- **A record is bound to its body.** `Placed` drops it, so a `Transform` or
  `Copy` of a recorded body cannot be blended analytically until the copy is
  rebuilt by a producer. Keeping it through placement would need `Placed` to see
  the source body, which `topo.Payload.Placed` does not take.
- **A `Chamfer` CB refusal falls through** to the faceted view; a `Fillet` CB
  refusal is returned.
- **An arc-bearing instance raises its `SectionDelta`** to its largest entry
  and `h_loop` (an arc-end junction entry `δ_End` of about `7e`, and through
  Lemma H's arc row an `h` of roughly `23e`, against the `3e` arc charge) so its
  record satisfies CR4. Withholding the record instead would keep today's value
  and stop every arc-bearing copy from filleting. An all-line copy, such as a
  tooth, keeps its `SectionDelta`.
- **Convention-corner feet are charged** their residual `r_f`, so case 1's
  fillet carries a `SectionDelta` of a few ulps instead of zero.
- **Every §8 refusal falls through** to the faceted view, SX6's `ErrDegenerate`
  included; only cancellation is returned. Returning §8's refusals would name its
  limit, at the cost of window chamfers that build faceted today refusing, and
  returning SX6 alone would change today's answer for a dropping hole loop.
- **`DisplacedPayload.ExtentAlong` refuses**, as a displaced prism's does, though
  its bounded reading already charges `LoopDelta`. The cost: a `ThroughAll` or
  `ThroughAllSide` extrude refuses while the chamfered part is live. Returning
  the bounded reading would let the stop charge it instead.
- **Rule DW declines a `DisplacedPayload`'s displaced walls** instead of charging
  them as an `AxialDelta`. A later retained edge blend or plane reading at such
  a wall then has no retained surface there and refuses; charging it would need
  a bounded tag normal that `facetedrecord` does not publish. The rule is per
  loop, which declines some exact-carrier walls of a displaced loop; a
  per-segment rule would need the payload to carry the receiver's carrier
  entries.
- **Displaced prisms keep today's tags in PR 2.** Declining them, per loop or
  per segment, would make three faceted-retained-blend apitests refuse
  (`TestFacetedRetainedBlendConcaveArcEnd` among them: a fillet's feet leave
  ulp-sized carrier entries on the adjacent line walls). The cost: a displaced
  prism's wall with a displaced carrier still enters a mesh `Boolean` as a
  retained tag with zero departure, which Claim R does not support. The remedy
  is a retained edge blend that accepts a bounded tag, so such a wall can be
  tagged with its displacement instead of being declined.
- **The tolerance gate reads witness prisms** (`capblend.WitnessPrisms` with the
  axial term plus `max LoopDelta`) rather than declining, so a displaced
  cap-loop chamfer gets a gate diameter, shrunk by twice that displacement.
- **The displaced cap-loop chamfer keeps a mesh volume proof.** Its tessellation
  charges the `LoopDelta` tube and `booloperand` admits it as it admits a plain
  cap-loop chamfer, so it composes in a mesh `Boolean` as a displaced prism
  does. Declining the proof would drop §8.7's volume term and the `booloperand`
  arm, and a later mesh `Cut` of the chamfered part would refuse.
- **Placement keeps `LoopDelta`**, while it drops §3.4's carrier record: no route
  admits on `LoopDelta`.
- **The transfer skips pairs of two exact loops.** Such a pair is the same on
  `S*`, and the float audit decides it as for an exact receiver. Certifying it
  too would close modify §5's S7 gap (`agent-docs/missing-features.md`) on this
  route alone, at the cost of refusing pairs the float audit admits.
- **The extended one-scene class never refuses.** A displaced or holed set it
  cannot prove takes the next route, ending at the mesh path or the pair fold, as
  A6 does for pairs. A caller who wants to know why reads the result's kind.
