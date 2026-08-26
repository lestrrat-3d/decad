# Loft Design

The increment-1 `Loft` feature: a solid between two recorded planar profiles on
distinct geometric planes, built by straight rules between corresponding boundary
points, with no guide rail and no centerline. Companion to `docs/api-design.md` (public surface,
"core §N"), `docs/sketch-seam-design.md` (the recording IR both profiles are
authenticated into, "seam §N"), `docs/evaluator-design.md` ("evaluator §N"),
`docs/modify-design.md` (the receiver/refusal/result/downstream table
discipline and the build-time simplicity-audit pattern this document reuses,
"modify §N"), `docs/spline-design.md` (the exactness-tier reasoning this
document's Table W follows, "spline §N"), `docs/tessellation-design.md`
("tessellation §N"), `docs/verification-design.md` ("verification §N"),
`docs/recipe-replay-design.md` ("replay §N"), and `docs/interference-design.md`
/ `docs/clearance-design.md` (the staged-consumer precedent Table D follows).

**This document changes an existing decision.** `docs/api-design.md` §8 and
§13 currently defer loft. This document un-defers it for the scope stated in
§1 below and records the minimal companion edits that keep the tree
consistent (§15).

Four tables are normative, in the discipline `docs/modify-design.md`
established:

| Table | States | Section |
|---|---|---|
| **P** | how the two profiles are paired into one correspondence | §3 |
| **S** | every refusal, once, with the existence test that picks its sentinel | §4 |
| **B** | the result: topology, faces, roles | §7 |
| **D** | one row per downstream consumer, and what it can prove today | §9 |

## 1. Scope — the increment-1 case, deferred reach, and permanent exclusions

The target case, stated by the consuming project
(`fusion360-gear-generator`'s 3D proof stage): two profiles, each already
closed and valid per `sketch`, recorded on distinct geometric planes, with the
same topology (same loop count, same segment count per loop), no guide rail,
no centerline, ruled between corresponding points. A helical gear tooth is one
such loft (bottom tooth loop to a twisted top loop); a bevel gear is two;
herringbone and spiral bevel compose those.

**Increment 1 admits exactly this shape, narrowed once more: every
corresponding segment pair MUST be same-kind — both `LineSeg`, both `ArcSeg`,
or both `CircleSeg`.** Any other pairing, including a mixed-kind or free-form
pair, is `ErrUnsupported` at the call — Table S row S3. A `LineSeg` pair walls
with an exact `Plane` (§5); a same-kind circular pair walls with flat
triangles over a chord chain built between the two recorded curves (§5.1) —
the body is the resulting polyhedron, never a curved ruled surface, and the
departure of that chord chain from the recorded arc or circle it approximates
publishes as a section displacement (§5.2), the same discipline
`docs/prism-boolean-design.md` §7 and `CLAUDE.md`'s "Section displacement"
note already state for a prism's re-expressed section. Same-kind is necessary
rather than sufficient for a circular pair: Table P row P5 carries one further
requirement on two `CircleSeg`s, that they agree in walk sense. A full-circle loop's
correspondence has exactly one segment (`record.go`), so its alignment offset
is confined to `[0, 1)` and forced to `0` (§3 S4): a rotated correspondence
between two full-circle loops is not reachable by this construction.

**Deferred reach, for reasons stated once:**

- **N-section lofts and guide-rail / centerline lofts.** Without a guide
  rail, ruling more than two sections needs an interpolation scheme this
  design has no closed-form, non-fitting answer for. The consumer does not
  need it either: a bevel gear is two 2-section lofts, not one 3-section
  loft. §12 defers this reach to PR 4; this design does not reserve a shape
  for it.

**Permanently out of scope, for reasons stated once:**

- **Free-form and mixed-kind correspondence.** The target needs only
  same-kind straight rules; every other correspondence needs its own surface
  and pairing design.
- **Reversed correspondence** (pairing segment `j` of one loop against
  segment `n-1-j` of the other). §3's alignment offset is a rotation only.
  Reversal changes which vertices are material-adjacent, which needs its own
  audit story; it is not needed for a twisted (rotated) top loop, which is
  what the target case is.

## 2. Public signature and options

```go
func (d *Document) Loft(s0 *sketch.Sketch, p0 *sketch.Profile, s1 *sketch.Sketch, p1 *sketch.Profile, opts ...LoftOption) (*Body, error)
func (d *Document) LoftContext(ctx context.Context, s0 *sketch.Sketch, p0 *sketch.Profile, s1 *sketch.Sketch, p1 *sketch.Profile, opts ...LoftOption) (*Body, error)

type LoftOption interface{ /* ... */ }

// WithLoftAlignment records, per loop, which recorded segment index of the
// SECOND profile pairs with segment index 0 of the FIRST profile's
// corresponding loop. offsets[0] is the outer loop; offsets[1+h] is
// p1.Holes[h]. Omitting the option means every offset is 0 — the natural
// case where both profiles were authored with matching segment order (the
// gear generator's top loop is the bottom loop's own points, rotated).
func WithLoftAlignment(offsets ...int) LoftOption
```

**Every `LoftOption` value MUST be non-nil and use decad's owned concrete
implementation.** A foreign type can embed the sealed marker, so `Loft` and
`LoftContext` check the concrete option before invoking any option callback.
They reject a nil or foreign value with `ErrDegenerate`, without changing the
document (Table S row S11).

**`WithLoftAlignment` is accepted at most once.** The variadic signature
type-checks a call that passes it twice, so the arity is a stated gate rather
than a compile-time one: zero occurrences mean every offset is 0, exactly one
supplies the whole per-loop payload, and two or more are `ErrDegenerate`
(Table S row S4) — never resolved by a last-wins or a merge rule. Two alignment
payloads are two different correspondences, so silently picking one builds a
body the caller did not ask for; this is the same ground modify-reach SX1
refuses a repeated contradictory option on.

`s0`/`p0` is the **from** section (`capStart`); `s1`/`p1` is the **to**
section (`capEnd`) — the same naming Extrude already uses for its two caps.
Two sketches are required, never one, because `sketch.Sketch` has one plane
(core §7): an admitted loft between two non-coplanar sections needs two.

**Both profiles pass through the unmodified seam gates of core §7 / seam
design, independently, in argument order.** `p0` MUST be a current,
unaltered, valid profile of `s0`; `p1` of `s1`. A foreign boundary entity,
a stale snapshot, a non-matching snapshot, or an unrecordable `Partial`
fragment on **either** profile is the seam's own sentinel
(`ErrForeignProfile` / `ErrStaleProfile` / `ErrInvalidProfile` /
`ErrUnrecordableProfile`) — this document adds no new gate here, and states
it once rather than doubling seam design's text (Table S row S9).

No `Extent`, no `AngularExtent`, no `Axis` — `OpLoft` is neither linear nor
angular in the §8.1 sense, and core §6.2's "every other Op leaves both nil"
already covers it without amendment. `Step.Inputs` is empty: increment 1
introduces no body-relative stop, so a loft consumes and depends on no body.

## 3. Table P — pairing and correspondence

**The correspondence is fixed entirely from recorded structural data — never
by sampling a curve, fitting a transform, or a nearest-point search between
the two boundaries.** `docs/api-design.md`'s hard rule against re-deriving a
2D answer, and the sibling rule that a decad-side check may only falsify an
upstream claim, both forbid inferring a correspondence sketch never stated:
decad reads the two `ProfileRecord`s' own segment order, and nothing else.

| P | Rule |
|---|---|
| **P1** | `p0.Outer` pairs with `p1.Outer`; `p0.Holes[h]` pairs with `p1.Holes[h]`, by **position** in the `Holes` slice — never by area, by nesting radius, or by any other geometric proximity |
| **P2** | `len(p0.Holes)` MUST equal `len(p1.Holes)`. A mismatch has no positional pairing (Table S, S1) |
| **P3** | For each paired loop, `len(loop0.Segments)` MUST equal `len(loop1.Segments)` — call it `n`. A mismatch has no one-to-one pairing (Table S, S2) |
| **P4** | Within a paired loop, segment `j` of `loop0` (for `j` in `[0, n)`) pairs with segment `(j + offset) mod n` of `loop1`, where `offset` is that loop's entry in `WithLoftAlignment`'s `offsets` (default 0) |
| **P5** | A paired segment's two sides MUST be same-kind: both `LineSeg`, both `ArcSeg`, or both `CircleSeg`. Any other pairing — mixed-kind or free-form — is `ErrUnsupported` (§1, Table S row S3). A same-kind `CircleSeg` pair MUST also agree in WALK SENSE: the two sides' recorded `CCW` flags MUST be equal, and a pair whose flags disagree is `ErrDegenerate` (Table S row S7's structural arm), decided from the two records before any triangle is built |
| **P6** | Every loop's own walk direction is intrinsic to its own plane (outer CCW, holes CW, seam §2) and is never reinterpreted for the pairing: P4's ordinal rule pairs walk-position `j` to walk-position `j`, in each loop's own sense, regardless of how the two profiles' planes are posed relative to each other |

**A wrong alignment choice is not a silent wrong body.** If the caller's
`offset` (or a caller's two sketches drawn without a shared segment-order
convention) produces a twisted, self-crossing correspondence, §6's audit
proves the resulting walls cross and refuses with `ErrDegenerate` — the
audit is the safety net a fitting-based correspondence would not have needed,
and would not have had.

**Vertex pairing follows from segment pairing.** Since `loop0` and `loop1`
are each closed loops of `n` segments, segment `j`'s start vertex is shared
with segment `j-1`'s end vertex; pairing every segment pairs every vertex:
vertex `k` of `loop0` (the start of its segment `k`) pairs with vertex
`(k + offset) mod n` of `loop1`.

## 4. Table S — refusals

The sentinel follows modify §1's existence test: **a body that does not
exist, at that correspondence, under any evaluator → `ErrDegenerate`; a body
that exists and this evaluator cannot build → `ErrUnsupported`.**

| S | The call asked for | Does that body exist? | Sentinel | Permanent from decad's side |
|---|---|---|---|---|
| **S1** | a hole-loop count mismatch (P2) | this evaluator has no positional pairing for it, though a smarter kernel could still loft a differing hole count by point-degenerate construction | `ErrUnsupported` | no — reach no increment in §12 claims |
| **S2** | a paired loop's segment-count mismatch (P3) | same — a smarter kernel could subdivide to match; this evaluator's ordinal correspondence cannot | `ErrUnsupported` | no — reach no increment in §12 claims |
| **S3** | a paired segment whose two sides are not same-kind (§1, P5), or a same-kind pair where either side is free-form. A same-kind `CircleSeg` pair whose two recorded `CCW` flags disagree (P5) is NOT this row's refusal: it is S7's `ErrDegenerate`, decided structurally beside this gate rather than by the audit | the ruled surface exists; this evaluator has no exact construction for a mixed-kind pairing, and none yet for a free-form one | `ErrUnsupported` | yes, §1 |
| **S4** | a `WithLoftAlignment` payload of the wrong length, an offset outside `[0, n)` for its loop, or the option passed more than once | no single intent (mirrors modify-reach SX1, which refuses a repeated contradictory option on the same ground) | `ErrDegenerate` | yes, §2 |
| **S5** | `p0` and `p1` represent the same geometric plane, regardless of which in-plane origin or right-handed `U`/`V` basis each `PlaneRecord` uses | no — every wall vertex then lies in one plane, so the solid is provably flat: the tetrahedron-sum volume (§8) is a structural zero, not a computed one | `ErrDegenerate` | yes, §4 |
| **S6** | a wall or cap triangle that collapses (coincident vertices, zero area) — every collapse S16's one-sided chord cell does not already claim, in either of two arms the existence test above splits: the RECORDED arm — EVERY vertex the collapse consumes is a coordinate the record states, which is a station §5.2 PINS (an untrimmed `LineSeg` pair's own endpoints; the two pinned ends of an `ArcSeg` pair recorded at ZERO RADIUS on BOTH sides) — or the COMPUTED arm, which takes every other collapse: one over GENERATED station vertices alone (§5.1's Table C) rounding to the same float64, one whose two stations DIFFER in provenance, and a cap triangle collapsing over either | the RECORDED arm: no — the modification consumed the region, the same existence answer modify §5 test 1 gives an inside-out loop. The COMPUTED arm: this evaluator cannot tell, and the row therefore never claims non-existence — where the two stations' recorded angles are distinct the denoted body exists at that correspondence and only this evaluator's float64 vertex table collapses it, and where any collapsing vertex is COMPUTED the record states no coordinate for it to be decided from | `ErrDegenerate` (RECORDED arm) / `ErrUnsupported` (COMPUTED arm) | yes, §4, for the RECORDED arm; no for the COMPUTED arm — a precision ceiling on this evaluator's float64 vertex table, the same reading S13 gives, and an undecided existence this row never converts into a permanent refusal |
| **S7** | either of two arms: the STRUCTURAL arm — a same-kind `CircleSeg` pair whose two recorded `CCW` flags disagree (P5), decided from the two records alone (§4's gate-order paragraph places both arms) — or the AUDIT arm, where the crossing audit (§6) finds contact other than the pair's own expected contact, whatever §5.1's Table C gives it | no — a self-intersecting or self-touching shell bounds no solid, and an opposite-sense circular correspondence walls each side against the other's reversed walk, which is that same crossing | `ErrDegenerate` | yes, §6 |
| **S8** | the crossing audit exhausts its fixed work budget (§6, §10) before every pair is decided, over the assembled triangle count `F` (§7), which a chorded pair grows past `2n` | this evaluator cannot tell | `ErrUnsupported` | no, §6 — a resource ceiling, not a shape rule |
| **S9** | either profile fails a seam gate (§2): foreign, stale, invalid, or an unrecordable `Partial` fragment | seam design's own answer, per profile | `ErrForeignProfile` / `ErrStaleProfile` / `ErrInvalidProfile` / `ErrUnrecordableProfile` | seam design's own answer, per gate; this document adds no permanence of its own (§2) |
| **S10** | a nil `*sketch.Sketch` or `*sketch.Profile` argument | no call at all | `ErrDegenerate` | yes, §2 |
| **S11** | a nil or foreign `LoftOption` value, including a foreign type that embeds the sealed marker | no well-defined decad operation can invoke an unowned callback | `ErrDegenerate` | yes, §2 |
| **S12** | ANY build — placed (`Placed`/`Duplicate`/`PlacedCopy`, §12 PR 2a), chorded (§5.1), or both — whose COMBINED proven volume allowance (§8) is not smaller than the held volume | yes — the body itself is sound; only its centroid's proven quotient bound has no positive denominator left to divide by | `ErrUnsupported` | no — a precision ceiling on this evaluator's centroid bound, not a shape rule |
| **S13** | a build whose lifted-and-placed coordinate, whose computed station coordinate (§5.1), or whose orientation anchor (§5), runs past the representable float64 range | yes — every input is finite (both records' coordinates, the plane origins, and a transform `r3` itself validated), and only decad's own float evaluation of the lift or the station computation overflows; a placed body is the rigid image of one this evaluator already built | `ErrUnsupported` | no — a range ceiling on this evaluator's float64 vertex table, not a shape rule |
| **S14** | ANY build for which a displacement term §5.2's table lists answers `+Inf` — a chorded circular pair's certified enclosure among them, a trimmed `LineSeg` end whose lerp the record cannot state as a rational, and a DERIVED term whose own outward-rounded composition saturates — decided in whichever of the row's two arms the gate-order paragraph below assigns that term | yes — the body exists; only one of its proven displacement terms has no derivation | `ErrUnsupported` | no — a derivation gap in this evaluator's certified circular enclosures, not a shape rule |
| **S15** | a paired segment whose chord target (§5.1) is not met inside the fixed station cap | yes — the ruled surface exists; this evaluator cannot chord it inside its own ceiling | `ErrUnsupported` (`errTooManyChords`, spline R8) | no — a resource ceiling, not a shape rule |
| **S16** | a chord cell (§5.1) whose two stations coincide on exactly ONE of the two sections. A cell collapsing on BOTH sections, and a collapsed cap triangle, are S6's two arms rather than this row, so every collapse is covered exactly once | yes — a collapsed piece is a recordable curve piece whatever the provenance of the two stations that produced it, and a point-degenerate correspondence is a body a smarter kernel could still loft; only the uniform two-faces-per-cell topology (§5) has no case for it | `ErrUnsupported` | no — an evaluator topology limit |

**S13 is `ErrUnsupported`, never `ErrNotFinite`.** Core §12 scopes
`ErrNotFinite` to a non-finite PARAMETER or a derived non-finite MEASUREMENT
or BOUND. A vertex coordinate is neither: every input reaching the lift is
finite and the derived measurement case is already the finiteness gate §8's
publication runs. The refusal states that this evaluator cannot hold the
body's vertex table in float64, the same reading spline design's R15 (an arc
length past `MaxFloat64`) and R16 (a fit interpolant's coefficient past it)
give a finite input whose derived magnitude runs off float64. **It is decided
before the first exact-rational lift.** §5's whole-shell orientation sum and
§8's accumulator both take every coordinate exactly, and the package's one
float-to-rational lift is defined only on a finite float, so a coordinate that
overflows must be refused while it is still a float.

**S6 is also reachable from a same-kind circular pair, and how many sides a
ZERO-RADIUS arc consumes decides which row answers.** A recorded `ArcSeg`
whose `Center`, `Start` and `End` are one point walks at radius zero, so
every station on that side lands on the centre. It is a recordable input:
`record.go`'s `validateSegment` checks an arc's three points for finiteness
and its range for emptiness alone, so such a record clears every gate a
decoded recipe runs, exactly as a degenerate `LineSeg` does. **A zero SWEEP
is not the trigger and cannot be one.** `extrude.go` folds an arc's
`math.Mod` sweep into `(0, 2π]` by adding `2π` to a non-positive result —
the branch `moments.go`'s own enclosure applies independently — so
coincident recorded endpoints walk a FULL turn rather than none, and
`record.go` refuses `TStart == TEnd` outright.

A zero-radius arc on exactly ONE side of a pair collapses that side's
stations alone, which is S16's one-sided chord cell and `ErrUnsupported`:
the correspondence denotes a point-to-arc fan, a body a smarter kernel could
still loft, and the gate order below reaches S16 first. A pair zero-radius on
BOTH sides collapses each of its cells on both sections, and that is this
row.

**Which arm a collapse takes is decided by §5.1's Table C, which states each
station vertex's provenance, and a cell whose two stations DIFFER in
provenance takes the COMPUTED arm.** The RECORDED arm needs EVERY vertex the
collapse consumes to be a coordinate the record states, so it is reached only
where every one of them is PINNED (§5.2's two kinds) — a zero-radius pair's
cell over two pinned ends, or a cap triangle over pinned stations alone,
reaching `ErrDegenerate` exactly as a degenerate `LineSeg` pair already does.
ONE computed station is enough to leave that arm, because `ErrDegenerate`
claims no body exists under ANY evaluator and nothing here proves that of a
computed station: the record states no coordinate for it, its enclosure may
be read only as an enclosure, and the collapse would otherwise be read off
the held float64s — which is the same zero §5.2 refuses to grant from the
shape of an enclosure. So a mixed cell reaches the COMPUTED arm and
`ErrUnsupported` beside a cell whose two stations are both computed, and
S6's permanence column claims nothing for either — the record's two station
angles may well be distinct, leaving a denoted body only this evaluator's
float64 vertex table collapses, the same reading S13 gives a coordinate that
runs past that table's range.

**A degenerate `LineSeg` pair is a recordable input, so the RECORDED arm
covers a live case rather than a hypothetical one.** `record.go`'s
`validateSegment` checks a `LineSeg`'s `Start` and `End` for finiteness
alone, and `validateSegmentRange` refuses only an empty or out-of-range
span, so a `LineSeg` whose `Start` equals its `End` over `[0, 1]` clears
every gate a decoded recipe runs; on the recording path `seam.go`'s
`segmentOf` copies both endpoints verbatim, adding no length check of its
own.

**S5 compares geometric planes, not `PlaneRecord` fields.** Its normal is
`U × V`; it refuses when the two normals are parallel and the displacement
between their origins lies in that plane. It returns `ErrDegenerate` before
construction for every coplanar section pair, even when their authenticated
records use distinct origins or bases.

**Gate order**, the same "ask what could be asked" discipline modify §4
states. **This paragraph is the single owner of where every gate sits
relative to construction and of what each gate's phase can already have
evaluated.** A site naming a gate's phase cites this paragraph and states no
phase of its own.

Pre-gates first (S10 nil check, S11 concrete option ownership without
invoking a callback, S4's ARITY half — a repeated `WithLoftAlignment` — decided
in that same option loop since it needs no record, S9 seam authentication of
both profiles — nothing downstream is safe to read before this), then the
shape gates that need only the two authenticated records (S1 hole count, S2
segment count, S4's PAYLOAD-SHAPE half — a wrong-length alignment or an offset
outside `[0, n)` for its loop — S3 segment kind, S7's STRUCTURAL arm
immediately beside it (a same-kind `CircleSeg` pair whose two recorded `CCW`
flags disagree, P5: one flag comparison over the same two records S3 already
reads, where S7's audit arm would reach the identical refusal only after
building every triangle and proving every pair), S5 geometric-plane
coincidence, and, for a same-kind circular pair, S15's station-cap decision
beside S14's DERIVATION arm below — the shared station count `m` and the
`mMax` it is compared against (§5.1) are each a closed-form function of the
two records alone, so both are decided here with the rest — all decidable
without building a single triangle), then construction (§5), whose own first
act is S13's coordinate-range gate on the anchor, on every computed station,
and on every placed vertex as it is emitted, then S16's
one-sided-collapsed-cell gate as stations are paired into chord cells, then
S14's CONSTRUCTION arm below, then the per-triangle existence gate S6 in
whichever arm the collapsed vertices' own provenance assigns it, then
S7's AUDIT arm with S8 beside it (§6) — the most expensive step, over
triangles already proven individually non-degenerate — and last of all S12,
which reads the COMBINED volume allowance §8 composes over that audited
triangle set and so cannot be asked before those measurements exist.

**S14 splits into two arms, the way S7 already does, because §5.2's table
lists terms of two kinds and no single phase can evaluate both.** A term is
judged in the phase that EVALUATES it — the earliest phase in which every
input its own arithmetic reads already exists — and the two arms together
cover every term that table lists, a term DERIVED from other rows included:

- the **DERIVATION arm** — record-only, decided beside S15 among the shape
  gates above. It asks whether the two authenticated records state every
  certified enclosure §5.2's table derives from them: the per-cell sagitta
  the walk-up compares against the chord target (§5.1), the `arcLenUpper_k`
  arc-length enclosure, the `circularEndpointInterval` enclosure a circular
  station's `stationRound` is measured against, and the exact `ratLerp` a
  trimmed `LineSeg` station's own `stationRound` is measured against. A
  candidate count whose certified sagitta has no derivation refuses here
  rather than walking on, and refuses before a single station is built. It
  also owns the DERIVED terms composed from those record-only rows alone —
  `sectionDelta`, the MAXIMUM of the per-cell sagittae, and
  `seamPerimeterUpper`, the SUM of the per-cell `arcLenUpper_k` over both cap
  loops. Beyond those, this arm decides DERIVABILITY and never a term's
  value; §5.2's table owns what each term's value is and what it is derived
  from;
- the **CONSTRUCTION arm** — decided after cells exist, since its terms read
  held coordinates rather than the record. It covers `maxTwistOffsetUpper`
  and `twistVolumeUpper` at each chorded cell's own four held corners, and
  each cap's `planeOffsetUpper` at that cap's own held vertices. These cannot
  be asked in the record-only phase at all: a cell's twist vector and a cap
  plane's offset from the anchor are functions of the vertex table, which
  does not yet exist there. It owns every remaining DERIVED term for the same
  reason, since each one's composition reads a held coordinate:
  `stationRound` and `placeAllow`, the `delta` over them, `matchedDelta`,
  `posUpper`, `wallAreaUpper`, `capAreaAllow`, `capVolumeUpper`, `seamAllow`,
  the facet departure, and `Bounds.Bound`.

**A DERIVED term answers `+Inf` on its OWN saturation as well as by
inheriting one, and S14 reaches it either way.** `absSumUpper` accumulates
through `upRound` (`moments.go`) and `bounds.go`'s product helpers round
outward at every step, so a composition of finite rows can itself run past
`float64`. S14's condition reads the term's PUBLISHED value and never its
inputs', so such a term refuses in whichever arm above evaluates it rather
than falling to neither.

**S14's condition is deliberately the broad one: ANY displacement term
§5.2's table lists answering `+Inf`, and not the per-station displacement
alone.** That table is what enumerates the terms S14 owns — each such term
names this row in its own Refusal column — so a reader checking S14's scope
reads that column and never a list restated here. The breadth is what makes
the two arms above necessary, since the terms it reaches are of both kinds,
and those two arms partition that column completely: no term it names is left
without a phase.

**A placement (`Placed`/`Duplicate`/`PlacedCopy`, §12 PR 2a) re-runs every
gate decided from the records rather than from the call — S1, S2, S3, S4's
payload-shape half, S5, S6, S7, S8, S12, S13, S14, S15, S16 — never a
reduced set of them.** The evaluator re-lifts both records under the
composed motion and rebuilds from scratch
(§7), so S6/S7/S8 are reachable from a placement too: the crossing audit
re-runs on the rounded vertex set every re-evaluation produces, and a
placement whose rounding closes a gap during this build is refused exactly as
a first build with the same geometry would be. S13 is judged on every build,
placement or first, since it reads the coordinate the lift emits rather than
the motion that produced it; a composed placement is how that coordinate
grows past the range in practice. S14, S15 and S16 are likewise decided fresh
on every build: the station generator (§5.1) reruns from the two records on
every re-evaluation, so a placement judges the identical station-derivation,
station-cap, and collapsed-cell questions a first build does, over the same
recorded curves. **S12 is judged on every build, because its condition is on
the COMBINED proven volume allowance §8 composes and not on how the build was
reached.** Each mechanism contributes its own term: a placement contributes
`placeAllow` through `delta`; a chorded build contributes `stationRound`
through that same `delta` (§5.2) and `sectionDelta` through the four-leg
`chordedBoundaryVolumeAllow` §8 composes; a build that is both contributes
all three. So a chorded build reaches S12 on its own terms, under
`r3.Identity()` included; §5.2's table owns which of those terms are
positive there and under what condition, and this paragraph states only that
S12 reads their combination.

**S9, S10, S11 and S4's ARITY half belong to the original call alone.** Each
judges an argument the caller passed to `Document.Loft` — the two live
`*sketch.Profile`s and their sketches (S9), a nil argument (S10), an option
value (S11), a repeated `WithLoftAlignment` (S4's arity half) — and a
placement receives none of them: the payload carries two already-recorded
`ProfileRecord`s and one normalized alignment slice, with no `*sketch.Sketch`
and no `*sketch.Profile` left to authenticate. So a profile that goes stale
after the loft is built refuses a FRESH `Document.Loft` on that profile while
`Placed`/`Duplicate`/`PlacedCopy` of the already-built body still succeed:
the record a placement rebuilds from was authenticated when it was recorded,
which is the same evaluate-from-the-record rule every other feature's
placement follows (`docs/evaluator-design.md` §1).

## 5. Construction — flat triangular walls, never a curved ruled surface

**Every wall is two flat triangles, split along the same fixed diagonal
`tessellate.go` already uses for a prism's lateral quad.** A loft reuses that
convention as its actual TOPOLOGY, not merely its tessellation. The local
order below fixes the diagonal and roles. The cap seed plus whole-shell rule
that follows fixes outward winding.

For paired segment `j` of a loop — `V_j -> V_{j+1}` on `p0`, `W_j -> W_{j+1}`
on `p1` (indices already rotated by §3's alignment) — the quad `V_j, V_{j+1},
W_{j+1}, W_j` splits into:

| Triangle | Vertices | Contains | Role |
|---|---|---|---|
| lower | `V_j, V_{j+1}, W_{j+1}` | that cell's own `p0`-side cap-boundary entity (§5.1's Table C) | `side(i,j,0)` |
| upper | `V_j, W_{j+1}, W_j` | that cell's own `p1`-side cap-boundary entity (§5.1's Table C), reversed | `side(i,j,1)` |

**Seed cap winding from shared-edge adjacency before orienting the complete
shell.** Retain each `capEnd` triangle from `p1`'s triangulation. Reverse each
`capStart` triangle from `p0`'s triangulation by swapping its second and third
vertices. Every cap-boundary edge then opposes its incident wall edge.

**Orient the already coherent complete shell once after constructing every
wall and both caps.** Compute §8's signed tetrahedron sum from the complete
triangle set, using `p0`'s `PlaneRecord.Origin` as its fixed anchor. Its sign
is nonzero after cap seeding and S5–S7. Retain every triangle when the sum is
positive. When it is negative, reverse every triangle's winding, including
both caps, by swapping its second and third vertices. Never orient an
individual wall or cap after cap seeding. This deterministic whole-shell step
makes Table B, mass properties, and `Tessellate` receive one positively
oriented material boundary.

**§7's Table B owns a loft's `side(i,j,k)` grammar** — what `i` and `j`
index and what `k` distinguishes. Evaluator §3 owns the provenance-role
MECHANISM this grammar instantiates and points at §7 for the grammar itself;
this section restates neither.

**Every Loft wall triangle is exempt from evaluator §3's
adjacent-coplanar-side-face canonicalization**, and evaluator §3 owns that
rule. No wall triangle merges with its mate or with a triangle in another
cell. A wall quad is generally non-planar — `V_j`, `V_{j+1}`, `W_{j+1}`,
`W_j` lie in one plane only where the two recorded segments happen to be
parallel and equally posed — while a split collinear side can make triangles
from neighboring cells coplanar. Merging either case would make the face
count, role grammar, diagonal incidence, or split-rung incidence depend on an
accident of the caller's two sections. The uniform two-face-per-cell topology
keeps Table B's roles and counts identical for every correspondence, which
makes §5's manifold argument and §6's adjacency checks read from recorded
pairing alone.

**Every vertex position is `V = Plane.Origin + p.U * Plane.U + p.V *
Plane.V`, the identical single float64 evaluation Extrude already performs
for a cap vertex.** Topology §3 grants a zero bound to a vertex whose
plane-local coordinates come from the RECORD rather than to every vertex a
feature builds, and `p` here is the recorded section's own point, so a loft
vertex carries the same standing a cap vertex does — no new rounding risk is
introduced; it is the same closed-form coordinate lift every other feature
already treats as truth. A station the walk COMPUTES is the exception — a
same-kind circular pair's (§5.1), and a `LineSeg` end at a trimmed parameter
(§5.2): §5.1's Table C states which station vertices the record states and
which this build generated, and a GENERATED one takes no zero-bound grant
here. §5.2's table states which two station kinds carry a zero and what
every other station's own displacement from the point the record denotes is.
This section restates neither and folds what they give into `delta` (§5.2).

**This claim holds where the payload's displacement `delta` is zero, and a
placement is only one of the two ways it stops being.** A placed,
duplicated, or placed-copied loft body (§7, §12 PR 2a) contributes the
placement term `placeAllow`, which is zero exactly at `r3.Identity()`;
a computed station contributes `stationRound`, which §5.2's table guarantees
zero at the kinds it pins and at no other — a chorded pair's station, and a
trimmed `LineSeg` end alike, carry whatever their own walk bound proves.
`delta` is `absSumUpper` of the two, so it is zero only where BOTH are, and
an unplaced pairing (§5.2's `placeAllow` row owns what unplaced means) whose
every station is PINNED — one of §5.2's own two kinds — is what guarantees
that. §5.2's table owns each term's source, rounding and refusal.

**Every consumer conditions on `delta > 0`, never on the body having been
placed.** A vertex position, an edge length, a face area, and each of the
four body measurements carry `delta` whenever it is positive, whatever
mechanism made it so — an unplaced chorded body included (§8).

**Edges get no new role mechanism.** Selector.go's existing rule already
covers loft: "`CreatedBy` matches an edge through its adjacent faces'
`Origins()` — an edge carries no roles of its own." A loft's three new edge
families — the cap-boundary entity on each side, the rung, and the
diagonal — need none. §5.1's Table C states each family's count, its
provenance, and which faces share it; this section restates none of that and
uses only what the table gives.

Every edge of the result bounds exactly two faces — §5.1's Table C states
which two for each family — so the payload is manifold and watertight **by
construction**, the same structural claim modify §2 makes for a rewritten
prism section —
**once §6's audit has proven no two non-adjacent faces cross.**

**`Edge.IsConvex` keeps the walked-boundary meaning core §6.1 and evaluator
§3 already define; this document narrows nothing and amends nothing.** The
two edge families a loft introduces are both JUNCTION edges, and evaluator
§3's junction rule already equates a junction's walk turn with its material
angle, so the two readings agree there and a loft states only how the turn is
computed:

- a **rung** edge and a **diagonal** edge are decided by
  `orient3d(A, B, C, D)` on the edge's two incident triangles — `A, B` the
  shared edge, `C` and `D` the two apex vertices, `(A, B, C)` the first
  triangle's own outward-wound vertex order — the edge convex exactly when
  the sign is negative, putting `D` on the MATERIAL side of `ABC`'s plane
  (`orientSign`'s own convention, `boolean_exact.go`, is positive on the
  outward-normal side, so the material side is negative). This is the
  identical adaptive exact-orientation predicate
  `boolean_exact.go` already implements for the mesh boolean's contact
  classification — reused here, not reinvented, because both faces are
  already exact `Plane`s, so the sign is always decidable without a
  tolerance. The predicate is needed because a prism's junction turn is read
  off the single recorded 2D section it sweeps rigidly (evaluator §3), while
  a loft has two sections whose own corner turns can disagree at the same
  paired vertex. A zero result retains a flat rung or diagonal as a decided
  non-convex edge: `IsConvex()` is false, `Convex()` does not select it, and
  `Concave()` does;
- a **cap-boundary** edge is a RIM edge, and takes evaluator §3's existing
  rim rule unchanged: a straight wall has no turn, so the edge takes the role
  of its loop — outer convex, hole concave. A loft therefore answers
  `Convex()` / `Concave()` on its rims exactly as a prism does, and `Concave`
  keeps picking a hole's rims (core §9).

**Every wall face is a `Plane`** (its `Frame` computed from its own three held
vertices — exact where the payload's own `delta` is zero, and within `delta`
wherever it is positive, whatever mechanism made it so (§5.2)), Exact by construction per core §6.1's surface-parameter
carve-out — the identical standing Extrude's `LineSeg` side walls already
have. That carve-out is what the vertices' own standing does not reach: the
SURFACE parameters are the exact answer for the vertices handed to them, while
the face's `Area()` and every vertex `Position()` on it carry `delta` (§8).
Cap faces (`capStart`, `capEnd`) are `Plane`s over a polygon-with-holes
region, exactly as an Extrude cap is.

### 5.1 Chord chain for a same-kind circular pair

**A same-kind `ArcSeg` or `CircleSeg` pair walls with a chain of chord
cells, never with the single quad §5's table shows for a `LineSeg` pair.**
Each side of the pair is sampled into `m` chord cells; the chain's
consecutive stations pair into cells exactly the way two recorded endpoints
pair into §5's quad, and each cell still splits into the same lower/upper
triangle pair Table B (§7) already defines — the topology above is
unchanged, only the number of cells a paired segment contributes grows from
one to `m`. **A chord cell is its own Table B cell, so `side(i,j,k)`'s
grammar needs no chorded exception** — §7 owns that grammar, and this
section changes nothing there.

**Table C — what one cell contributes, and which entities the record
states.** Every cell of a loop contributes exactly the entities below,
whether it is a `LineSeg` pair's single cell or one link of a chord chain.
Each entity is RECORDED — the record itself states its coordinates — or
GENERATED — this build computed them. **This table is the single owner of a
cell's entity inventory, its counts and its provenance marks.** A site naming
one of these entities cites this table and states no count or provenance mark
of its own.

| Entity | Per cell | Per loop | Provenance |
|---|---|---|---|
| lower triangle, role `side(i,j,0)` | 1 — `V_j, V_{j+1}, W_{j+1}` | `stations_i` (§7) | GENERATED — the triangle is this build's own facet |
| upper triangle, role `side(i,j,1)` | 1 — `V_j, W_{j+1}, W_j` | `stations_i` (§7) | GENERATED — same |
| `p0`-side cap-boundary entity | 1, held by the lower triangle and shared with `capStart`'s boundary | `stations_i` | RECORDED for a `LineSeg` pair — the recorded `LineSeg` itself, no new edge at all. GENERATED for a chord cell — a chord joining that cell's two `p0` stations, which the record does not state |
| `p1`-side cap-boundary entity | 1, held by the upper triangle reversed and shared with `capEnd`'s boundary | `stations_i` | same rule, on the `p1` side |
| rung edge `V -> W` | 2 — one at each of the cell's two station pairs, each shared with the neighbouring cell | `stations_i`, one per station pair, since a loop's cells form a closed cycle | GENERATED — no record states an edge between the two sections, whatever the provenance of its two endpoints |
| diagonal edge `V_j -> W_{j+1}` | 1, shared by this cell's own lower and upper triangle | `stations_i`, one per cell | GENERATED — same |
| station vertex, per side | 2 per side, each shared with the neighbouring cell | `stations_i` per side | RECORDED for a station §5.2's table PINS — an untrimmed `LineSeg` end or an untrimmed `ArcSeg` end, the two kinds that table names. GENERATED for every other station, a TRIMMED `LineSeg` end included, each carrying `stationRound` (§5.2) |

A cell's expected contact with another cell — what §6's audit admits and
refuses against — is the entity this table gives it, never a provenance mark
the audit decides for itself.

**`m` counts CHORD CELLS, and an OPEN side's station points are one more.**
`m` is `chordCount`'s own CHORD count, so a pair chorded at `m` contributes
exactly `m` chord cells to its loop. An OPEN side — an `ArcSeg`, or a
`CircleSeg` recorded over less than a full turn — holds `m + 1` station
POINTS: that side's own two END stations plus `m - 1` interior stations.
A pair at `m = 1` therefore has no interior station at all and walls with a
single cell, which is the case §8 and §12 both single out. **An end station
is not automatically a recorded coordinate**: only an untrimmed `ArcSeg`'s
two ends are read verbatim off the record, while a `CircleSeg` records no
endpoint coordinate at all and a trimmed `ArcSeg` end is computed like any
interior station — §5.2's table states which kinds are pinned and what the
rest carry. Wherever this document calls `m` a station count it names that
same chord-cell count, never the count of station points.

**A CLOSED side holds `m` CYCLIC station points, not `m + 1`, and its final
cell pairs the last station back to the first.** A `CircleSeg` recorded over
a full turn walks closed (`extrude.go`), and the station loop appends exactly
one point per chord cell and never the terminal one (`tessellate.go`),
because that point IS the walk's own first station. So a full-circle side has
ONE station where an open side has two ends, and `m` stations carry `m`
cells: cell `j` pairs station `j` to station `j + 1` for `j` in `[0, m-1)`,
and the last cell pairs station `m - 1` to station `0`. The open rule read
here would count the walk's single start station twice and add a closing cell
between a station and itself, so it is the CLOSED count that every derived
set takes: §5's vertex set, §8's assembled cap polygon and the triangle set
§8 integrates over, and §8's `Bounds` candidate set. `chordCount` forces
`n >= 3` on a closed walk, so a full-circle pair's smallest chorded body is
three cells over three stations a side, and `m = 1` is reachable only on an
open one. Table B's `stations_i` (§7) counts CELLS, so it reads the same for
either kind and needs no exception of its own.

**Station generation reuses the existing circular-walk primitives, never new
trigonometry, and the value it walks up against is the CERTIFIED one.** The
loft runs `chordCount`'s own walk-up SHAPE, the identical closed-form
recompute-and-increment `Document.Extrude`'s circular side walls already
use — evaluate the per-cell sagitta at a candidate station count, increment
until the value is at or below the target, so the depth is measured at every
step rather than sized from a rate. What it evaluates at each candidate is
§5.2's certified per-cell sagitta over the record's own radius and sweep
enclosures, NOT the held float `chordCount` itself returns. That float is
`chordSagitta`'s proven quadratic bound, the one
`docs/tessellation-design.md` §3 owns and publishes — never the true
`2r·sin²(Δθ/4)` closed form that section's own seed inverse solves — and it
is decided over a `math.Hypot` radius and a `math.Atan2` sweep neither of
which the record states, so it can only decide a COUNT and can never be the
sagitta this build publishes. Its OWN outward rounding does not change that:
what it over-states is the sagitta of the two held floats it was handed, and
that states nothing about the recorded curve. The value
the walk-up compares against the target is therefore the same over-stated
value §5.2 publishes as `s_k`, and a candidate count whose certified sagitta
has no derivation refuses at Table S row S14, in the arm and at the phase
§4's gate-order paragraph assigns it.

**Shared station count, one target.** For a paired segment, each side's own
minimum station count (`m0`, `m1`) is walked up independently at the shared
target below, and `max(m0, m1)` SEEDS a joint walk-up: at each candidate
count from there, both sides' certified per-cell sagittae are recomputed and
the count increments until BOTH are at or below the target. The pair's `m`
is the count that joint walk-up settles on, and both sides are chorded at
it, so each side's own published sagitta at `m` is at or below the target it
is judged against — established by recomputing at the shared count, never
inferred from how either side's own minimum compared. This is what keeps the
two sides' station counts identical without loosening either side's own
bound, and it needs no monotonicity claim about the certified sagitta as the
count grows.

**The chord target and its constant.**

```text
chordTarget = loftChordFraction * max(profileCoordinateUpper(p0), profileCoordinateUpper(p1))
```

`loftChordFraction` is one unexported package constant (§14 records its
calibration), and `profileCoordinateUpper` is the existing section-coordinate
envelope reader (`extrude.go`) — the target scales with the section's own
size rather than with one arc's own radius, so a Tier-A free-form pair that
has no radius can share the identical rule (§12 reach). **The target is not a
caller option.** `WithChordTolerance` is a tessellation/export render knob;
a loft's chording is TOPOLOGY — it decides the vertex set the payload holds —
so a caller-supplied tolerance would change body identity and demand a wire
field. The constant stays in source, and `LoftOpts` gains no new field for it
(§10).

**The station cap and the ceiling it answers to.** A build's total station
count is capped by one unexported package constant, `loftStationCap` (§14
names the increment that fixes its value). The cap exists to keep the chord
chain from being what carries §6's audit past the pair-test ceiling that
section already owns: §6 refuses under S8 unless the assembled triangle count
`F` (§7) has an `F*(F-1)/2` at or below `maxFacetPairTestsPerCall`.
`loftStationCap` is
fixed so that a build whose `Σstations` reaches it assembles an `F` whose
`F*(F-1)/2` is STRICTLY below that ceiling. A build chorded too finely for
the audit therefore refuses as S15, carrying the chord-count message, rather
than as S8 carrying the audit-budget one: **the cap is the soft limit and S8
the hard one, and the two are never merged.** A record whose own
paired-segment count already exceeds the cap is past chording altogether —
its `Σstations` is that segment count (§7) — and S8 is what refuses it,
exactly as for an all-`LineSeg` build.

**Allocating the cap.** `loftStationCap` bounds `Σstations`, the total over
every loop (§7), because the `F` above depends on that total and on nothing
per-loop. It is allocated per paired segment, which gives each loop a share
proportional to its own paired-segment count, and a share of the part
chording can spend proportional to its own circular-pair count. With `P` the
build's total paired-segment count and `C` the number of same-kind circular
pairs among them — both fixed by Table P from the two records alone — every
paired segment is entitled to its first station, which is a `LineSeg` pair's
whole entitlement (`m = 1`, §7), and each circular pair may take at most

```text
mMax = 1 + max(0, (loftStationCap - P) / C)      // integer division
```

stations. A circular pair whose own settled `m` (the joint walk-up above)
exceeds `mMax` is Table S row S15 (`ErrUnsupported`, `errTooManyChords` —
spline R8, the
identical sentinel `chordCount` itself already returns when its own walk-up
would exceed `maxChordsPerWalk`), and the refusal names that segment, since
the share it exceeded is that segment's own. A build with no circular pair
(`C = 0`) never consults the cap at all: its `Σstations` is `Σn_i` exactly
(§7), the count the record itself states, so an all-`LineSeg` build's only
resource refusal is S8.

**The per-pair share can never sum past the global cap.** `C` counts the
same-kind circular pairs AMONG `P`, so a circular pair's `m` stations
SUBSUME the first-station entitlement `P` already grants that segment rather
than adding to it. For a record whose `P` is within the cap — the paragraph
below carves out the one that is not — the build's total is therefore

```text
Σstations = (P - C)·1 + Σm  ≤  (P - C) + C·mMax
          =  P + C·floor((loftStationCap - P) / C)
          ≤  loftStationCap
```

because integer division only ever UNDER-allocates:
`C·floor((loftStationCap - P) / C) ≤ loftStationCap - P`. At
`P = loftStationCap - 3` with `C = 2`, `mMax` is `2`, and two circular pairs
both settling at `m = 2` total `(P - 2) + 2 + 2 = P + 2`, one station inside
the cap. That bound is why the per-pair share is sound: for such a record,
no build every one of whose pairs passes S15 can exceed `loftStationCap`.

**A record whose own `P` already exceeds the cap.** The `max(0, …)` term
clamps to zero there, so `mMax = 1`, and a circular pair whose joint walk-up
settles at `m = 1` passes S15 with `Σstations` already past the cap. That
admission is not an escape from the ceiling the cap answers to: the paragraph
above carves this record out as past chording altogether, and S8 is what
refuses it, over the assembled triangle count §6's `F*(F-1)/2` preflight
computes rather than over the cap. Nothing is left unchorded either, because
a pair settles at `m = 1` only when both sides' certified sagittae are at or
below the target at one cell — that is what the joint walk-up above settles
on. Refusing such a build at S15 instead would refuse a mixed build while
admitting an all-`LineSeg` build of the identical triangle count, which is
the shape rule Table S row S15 states S15 is not.

**Deciding S15 from the record.** `m` and `mMax` are each a function of the
two `ProfileRecord`s alone — the two sides' certified radius and sweep
enclosures (§5.2), the chord target above, `P` and `C` — so S15 is DECIDABLE
from the two records, with no station built. A pair whose certified sagitta
has no derivation refuses S14 beside it, since the walk-up that settles `m`
is what asks for it. §4's gate-order paragraph owns both placements and
S14's two arms; this section restates neither. Every product and sum
in the `mMax` comparison and in §6's own `F*(F-1)/2` preflight is evaluated
with checked arithmetic and refuses on overflow rather than wrapping, the
identical preflight-before-allocation discipline §6 states for the pair-test
ceiling itself.

### 5.2 The published displacement terms and their provenance

**Every displacement term this document publishes is listed once, in the
table below, with the quantity it bounds, the certified enclosure it is
derived from, the site that PROVES it dominates that quantity, the direction
it rounds, and what it does when that enclosure is underivable.** A site
naming one of these terms cites this table and states no condition of its
own.

| Term | The quantity it bounds | Certified source | Derivation | Rounding direction | Refusal |
|---|---|---|---|---|---|
| **`placeAllow`** | a LENGTH: the world-space displacement of one held vertex from the point the composed rigid motion denotes for it | `bounds.go`'s `rigidRoundAllow`, read at the pre-transform lifted point's own magnitude and the composed translation's magnitude — never at the result's, since that is where a general rigid motion's rounding is actually committed | `rigidRoundAllow`'s own doc comment (`bounds.go`), which derives a rigid motion's committed rounding at those pre-transform magnitudes; this document proves nothing of its own here | outward, inside `rigidRoundAllow`'s own rounding | exactly zero when the accumulated motion is `r3.Identity()`, decided by exact struct comparison and never by a tolerance. **That test is what UNPLACED means** wherever this document or a companion says it of a loft: a build is unplaced exactly when its accumulated motion is `r3.Identity()` — an identity `Duplicate`, which is `copyUnder(ctx, OpDuplicate, r3.Identity())` (`document.go`), included — and never by whether a placement call was made. This row owns that definition, and every site conditioning on an unplaced loft points here. A lifted or placed coordinate past the float64 range is Table S row **S13**, not a published bound |
| **`stationRound`** | a LENGTH: the world-space displacement of one held station from the point the record denotes for it, over EVERY station whose generator COMPUTES its coordinates — a circular-walk station, and a `LineSeg` end at a TRIMMED parameter, which `lerp2` evaluates in float instead of reading a recorded `Point2` | `extrude.go`'s `circularWalkEndBound` over `moments.go`'s `circularEndpointInterval` — a `CircleSeg`'s exact rational turn through `quarterTurnSinCos` / `turnSinCosInterval`, an `ArcSeg`'s `ratSqrtDown` / `ratSqrtUp` radius and `atan2Interval` swept angle — carried into world space by `bounds.go`'s `walkEndBoundAllow`. For a trimmed `LineSeg` end, `extrude.go`'s `lineWalkEndBound` over `moments.go`'s `ratLerp` — the exact rational lerp of the two recorded endpoints at the recorded parameter, which is the denoted point itself rather than an enclosure of it — carried into world space by that same `walkEndBoundAllow` | this section's closing paragraph: ONLY the point the record denotes lies in that enclosure. The held station is an independent `math.Sincos` evaluation (`circularWalk`) and can sit OUTSIDE it, so the enclosure's own width bounds nothing here. `intervalFloatError` measures the OUTWARD GAP from the held station to the enclosure — `max(\|held − lo\|, \|held − hi\|)`, which dominates the held station's distance from EVERY point of that interval and so from the denoted point wherever in it that point lies — and `walkEndBoundAllow` carries that gap through the payload's ORTHONORMAL frame without growing it. The trimmed-`LineSeg` arm needs no enclosure step: `lineWalkEndBound`'s own doc comment states the denoted point exactly as `ratLerp`, and `rationalFloatError` reports the held `lerp2` float's own per-component gap from it | outward at every step: `intervalFloatError` takes the FARTHER of the held station's two gaps from the enclosure's ends, `walkEndBoundAllow` widens the wider plane-local component through `radius3D`, and `rationalFloatError` rounds the trimmed-line arm's gap out the same way. The build-wide value is the MAXIMUM over stations and never a sum, for the reason this section's `delta` paragraph gives | `+Inf` wherever the record cannot state the enclosure, or cannot state a trimmed `LineSeg` end's own lerp as a rational, refused `ErrUnsupported` at Table S row **S14**. Exactly zero at the two PINNED station kinds below, and PINNING is the only thing that GUARANTEES it: a computed station whose own arithmetic happens to be exact — a trimmed `LineSeg` end whose `lerp2` reproduces `ratLerp` bit for bit — reports zero too, a tighter bound this term is free to publish and never a zero the kind proves, so every site reads a build's `stationRound` at the value this term publishes and takes only the GUARANTEE from the kind |
| **`delta`** | a LENGTH: the world-space displacement of one held vertex from the point the record and the motion together denote for it | `absSumUpper(stationRound, placeAllow)` — the two rows above and no third mechanism | the triangle inequality over the two rows above: the two displacements are committed at independent stages — the station is computed, then the motion is applied — so the vertex's total departure is at most their sum | outward, in `absSumUpper` | inherits both rows'. Zero exactly when both terms are zero, which an unplaced pairing whose every station is PINNED — one of the two kinds below — is what GUARANTEES |
| **per-cell sagitta `s_k`** | a LENGTH: the in-section-plane distance from one chord to the recorded curve piece it chords, on side `k` of one chord cell | `2·r·sin²(Δθ/4m)` evaluated over side `k`'s own enclosures — the RADIUS enclosure (`ratSqrtDown` / `ratSqrtUp` of the exact squared `Start`-to-`Center` distance for an `ArcSeg`; the recorded `Radius` converted to millimetres, exactly rational, for a `CircleSeg`) and the SWEEP enclosure (`atan2Interval`'s difference under the same `+2π` branch correction `circularLengthInterval` applies for an `ArcSeg`; the exact rational turn `2π·(TEnd − TStart)` for a `CircleSeg`), with `radSinCosSpan` supplying the sine of the enclosed angle | elementary and stated here: a circular arc's distance from its own chord is `r·(1 − cos(half the cell's sweep))`, taken at the cell's midpoint where the two are farthest apart, and `1 − cos x = 2·sin²(x/2)` turns that into the form the row publishes | interval arithmetic to the last step, then ONE outward rounding of the interval's upper end into the published float | `+Inf` wherever an enclosure has no derivation — the `In(units.Millimeter)` conversion, `floatRat`, `ratSqrtUp` or `radSinCosSpan` answering no — refused `ErrUnsupported` at Table S row **S14** |
| **`sectionDelta`** | a LENGTH: the largest single `s_k` over every chord cell and both sides of the whole build, a MAXIMUM and never a sum | the row above | this section's maximum-not-a-sum paragraph: a boundary point lies in exactly one cell, so no point is displaced by two cells' sagittae | none of its own — a maximum of values already rounded outward is already an over-statement | inherits the row above's `+Inf` and its **S14**. Exactly zero when every paired segment is a `LineSeg` |
| **`matchedDelta`** | a LENGTH: how far one point of a HELD chord sits from the point the recorded curve denotes at the SAME arc-length parameter — the PARAMETER-MATCHED departure every chorded leg charges, and a strictly stronger claim than the SET distance a sagitta states | `absSumUpper(sectionDelta, delta)` — the `sectionDelta` row above and the `delta` row above that, and no third mechanism | this section's parameter-matched paragraph, in two steps: the sagitta is the IDEAL chord's own matched departure for the two kinds a paired segment may carry here, and the HELD chord sits within `delta` of that ideal chord at every matching parameter, since a segment's displacement is the convex combination of its two endpoints' and each held station sits within `delta` of the point the record and the motion denote for it | outward, in `absSumUpper` | inherits both rows' `+Inf` and their **S14**. Exactly zero only where BOTH are, which an unplaced `LineSeg`-only pairing whose every station is PINNED is what GUARANTEES, the `delta` row's own zero test |
| **per-cell `arcLenUpper_k`** | a LENGTH: the arc length of side `k`'s own recorded curve piece over one chord cell, never below that cell's own chord length on that side | `moments.go`'s `circularLengthInterval` over the same radius and sweep enclosures the sagitta row names | `bounds.go`'s `cellChordCurveAreaUpper` doc comment, whose derivation parametrizes each side at CONSTANT ARC-LENGTH speed and reads this bound as that side's own constant tangent magnitude; the same comment states why a bound below the chord it subtends is a broken claim rather than a tighter one | outward: the enclosure's upper end, rounded out once | `+Inf` wherever the record cannot state the enclosure, refused `ErrUnsupported` at Table S row **S14** |
| **`maxTwistOffsetUpper`** | a LENGTH: how far one point of a CHORDED wall cell's bilinear ruled patch sits from the built triangle pair at the matching parameter, over the WHOLE build — a MAXIMUM over the build's CHORDED wall cells and never a sum, and exactly zero on a build that holds none (a `LineSeg`-only pairing, whose walls this term never reads) | `bounds.go`'s `cellTwistOffsetUpper`, read at each CHORDED cell's own twist vector `T = vLo − vHi − wLo + wHi` as `\|T\|/4`, and over no other cell | `cellTwistVolumeAllow`'s own derivation part (a), which solves that deviation exactly as `r·(s−1)·T` and `s·(r−1)·T` and maximises it at `\|T\|/4`; `cellTwistOffsetUpper`'s doc comment owns the maximum-not-a-sum rule, since the term bounds how far a SINGLE point sits from its nearest held vertex rather than an accumulation over cells. **The chorded scoping is proven rather than a convenience**: §5 builds a `LineSeg` pair's wall AS the held triangle pair, and that pair IS the boundary the body has there — §5's polyhedron rule and §8's `Volume`-`Exact` rule both read it as the true solid — so no ruled patch stands between such a facet and the surface it stands for, and a `LineSeg` cell charges nothing here however its four corners twist. A CHORDED cell is the only cell whose facet stands for a piece of a solid the record denotes and the build does not hold, and its bilinear ruled patch is the intermediate surface §8.1's twist leg starts from | outward, in `upRound` | `+Inf` on a non-finite CHORDED-cell corner, refused **S14** — the chorded cells that row reaches; a build with no chorded cell publishes the exact zero above and reaches no refusal here |
| **cap `planeOffsetUpper`** | a LENGTH: `\|h\|`, one cap plane's own perpendicular offset from the mass accumulator's anchor (§8) | the exact rational distance from that anchor to a held vertex of that cap, bracketed by `ratSqrtUp` | a plane's own perpendicular offset from a point never exceeds the distance to any single point ON that plane, and every held cap vertex lies on that cap's plane exactly | outward, in `ratSqrtUp` | `+Inf` where the assembly states no such vertex, refused **S14** |
| **cap `capAreaAllow`** | an AREA: how far one cap's ASSEMBLED chord polygon region differs in area from the region its recorded boundary denotes | `bounds.go`'s `sectionDisplacementArea(matchedDelta, walks, perimeterUpper)` over that cap's own recorded boundary, its `perimeterUpper` summed from the `arcLenUpper_k` row | `sectionDisplacementArea`'s own doc comment: the two regions' symmetric difference lies inside the `matchedDelta`-neighbourhood of the recorded boundary, covered by a `2·matchedDelta`-wide tube along the walks plus a disk of that radius at each joint. The held polygon's own vertices are displaced as well as chorded, which is why the neighbourhood is the matched term and not the sagitta | outward, in `productUpper` and one closing `upRound` | inherits the `matchedDelta` and `arcLenUpper_k` rows' `+Inf` and their **S14** |
| **`posUpper`** | a LENGTH: the distance from the anchor to any point of EITHER cap loop's TRUE recorded curve | the held vertex set's own maximum distance from that anchor, widened by `matchedDelta` | `chordedBoundarySeamAllow`'s own doc comment: every true curve point sits within `matchedDelta` of its own held chord at the matching parameter, and every point of that chord lies in the convex hull of the held vertex set, so the curve point sits at most that much further from the anchor than the farthest held vertex does | outward, in `absSumUpper` | inherits the `matchedDelta` row's |
| **`seamPerimeterUpper`** | a LENGTH: the total arc length of BOTH cap loops' true recorded curves | the SUM over both loops of every wall cell's own `arcLenUpper_k` for that side — the identical quantities that row already states, read a second time rather than derived again | `chordedBoundarySeamAllow`'s own doc comment, whose line integral runs over exactly those two loops | outward, in `absSumUpper` | inherits the `arcLenUpper_k` row's |
| **`wallAreaUpper`** | an AREA: the area of EVERY surface the wall's chord-to-curve homotopy visits, summed over wall cells — an ABSOLUTE bound, never a held facet area plus an excess | `bounds.go`'s `cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, arcLenUpperA, arcLenUpperB, matchedDelta)` per wall cell | that helper's own doc comment, whose `eA·eB` product bounds the homotopy's own area at every time; the same comment gives the counterexample an excess reading misses — a cell can hold almost no triangle area while its own ruled patch carries substantial area | outward: every factor through `absSumUpper` / `productUpper` | inherits the `arcLenUpper_k` and `matchedDelta` rows' `+Inf` and their **S14** |
| **`twistVolumeUpper`** | a VOLUME: the gap between a wall cell's HELD pair of flat triangles and the BILINEAR RULED patch the wall leg's own homotopy starts from, summed over wall cells | `bounds.go`'s `cellTwistVolumeAllow` per wall cell | that helper's parts (a), (b) and (c): the pointwise deviation `\|T\|/4`, the homotopy's own `eA·eB` area at every time, and the flux identity closing them — with no seam term of its own, since the deviation vanishes on all four edges of the parameter square | outward, in `productUpper` and `absSumUpper` | `+Inf` on a non-finite corner, refused **S14** |
| **`capVolumeUpper`** | a VOLUME: the volume one cap contributes when its held chord polygon is replaced by the region its recorded curve denotes, summed over the (at most two) caps | `bounds.go`'s `capAreaVolumeAllow(planeOffsetUpper, capAreaAllow)` per cap, over the two rows above | that helper's EXACT planar identity: a planar face's own signed-tetrahedron sum is `2·h·Area(cap)` whatever the triangulation, so replacing the held polygon's area with the denoted region's changes it by exactly `2·h·ΔArea`, giving `\|ΔVolume_cap\| ≤ \|h\|·\|ΔArea\|/3`. It is never `perturbedAreaUpper`, whose per-facet argument is about vertices that MOVE and a cap's never do | outward, in `productUpper` and one closing `upRound` | inherits the two rows above |
| **`seamAllow`** | a VOLUME: the line-integral residue the wall leg's flux identity drops by treating the wall as CLOSED when it is an OPEN patch whose `r=0`/`r=1` seam moves under the same homotopy | `bounds.go`'s `chordedBoundarySeamAllow(matchedDelta, posUpper, seamPerimeterUpper)` | that helper's own doc comment: Cauchy-Schwarz on the by-parts boundary term, `matchedDelta · posUpper · seamPerimeterUpper / 3` | outward, in `productUpper` and one closing `upRound` | inherits the `matchedDelta`, `posUpper` and `seamPerimeterUpper` rows' |
| **facet departure** | a LENGTH: how far one point of a HELD facet sits from the true boundary surface that facet stands for | `absSumUpper(delta, sectionDelta, maxTwistOffsetUpper)` — the three rows above, and no fourth mechanism | the triangle inequality over three independent departures: the facet's own vertices sit within `delta` of the points the record and the motion denote, a CHORDED cell's bilinear ruled patch through those corners departs from the recorded curve by `sectionDelta`, and that cell's held flat triangle pair departs from the ruled patch by at most `maxTwistOffsetUpper`. **The last two terms are charged by chorded cells alone**, and both are exactly zero on a `LineSeg`-only build, whose held triangle pair IS the boundary §5 gives it (the two rows above): the sum is the same three-term reading for every build, and it collapses to `delta` there by its own terms rather than by a consumer choosing a shorter one. Every per-facet consumer reads this term (§9 D1, D2) and no consumer reads a two-term subset of it | outward, in `absSumUpper` | inherits all three rows' |
| **`Bounds.Bound`** | a LENGTH: the radius by which the axis-aligned box the payload holds may fall short of the box the true recorded boundary occupies | `absSumUpper(delta, sectionDelta)` — the two published terms above, summed | §8's `Bounds` paragraph: the recorded boundary can exceed the held box both by a held vertex's own displacement and by the recorded curve's bulge outside the station polygon, and the two act on the same face of the box, so the shortfall is at most their sum. **This reading takes two terms where the facet-departure row takes three, and the difference is proven rather than an omission**: the held triangle pair and the bilinear ruled patch both lie in the convex hull of a cell's own four held corners, so a cell's twist moves no face of the box and `maxTwistOffsetUpper` has no term here | outward, in `absSumUpper` | inherits both rows'. `Bounds` is `Exact` only where that sum is exactly zero (§8) |

**`matchedDelta` is the PARAMETER-MATCHED displacement the chorded allowance
requires, and no single mechanism supplies it.**
`cellChordCurveAreaUpper`'s displacement argument is a bound on `|curve(s) −
chord(s)|` at the SAME `s` under one constant-arc-length parametrization,
which is a strictly stronger claim than the SET distance a sagitta states: a
curve can hug its chord within an arbitrarily small sagitta while packing
almost all of its arc length into one short span, so its arc-length-matched
point sits far from the chord point at the same `s`
(`TestCellChordCurveAreaUpperRefusesTheSagittaZigzag` pins that
counterexample). **A caller that cannot PROVE the matched bound must pass
`+Inf`, and the sagitta may never stand in for it** — that helper's own rule,
and `CLAUDE.md`'s reject-only discipline at this seam.

**The sagitta is the IDEAL chord's matched departure, and the chord this
build HOLDS is a different segment.** Call a cell's IDEAL chord the one
joining the two points the record and the motion together denote for its two
stations. That chord's matched departure from the recorded curve is at most
`sectionDelta` for exactly the two kinds a paired segment can carry here: a
`LineSeg` side's chord IS its curve, deviation zero, and a circular arc under
its own uniform-angle parametrization has matched deviation exactly
`2·r·sin²(Δθ/4m)` — the number the sagitta row publishes, maximised over
cells by the `sectionDelta` row
(`TestArcMatchedDeltaEqualsSagitta`, over a 5°–170° sweep). **That is an
EQUALITY for a circular arc, so it leaves no slack a second mechanism could
hide in.** The chord the build HOLDS joins two held stations, each sitting
within `delta` of the point the record and the motion denote for it —
guaranteed zero only at one of the two PINNED station kinds below, and
elsewhere whatever the station's own walk bound proves; a segment's
displacement at parameter `s` is the convex combination `(1−s)·(h₀−d₀) + s·(h₁−d₁)` of its
two endpoints' displacements, of magnitude at most `delta` at every `s`. The
triangle inequality over the two closes it: the held chord departs from the
recorded curve at the matching parameter by at most
`absSumUpper(sectionDelta, delta)`, which is what the `matchedDelta` row
publishes. **Reading `matchedDelta` as `sectionDelta` alone leaves the
computed station's own displacement uncharged on every chorded leg**, and no
finite value read off the geometry may stand in for that term either — the
achieved sagitta `tessellate.go`'s `chordCount` returns is not the sagitta
this table publishes (the rounding rule below), so its incidental slack
proves nothing here.

**Four rules govern every row, and they are stated here once.**

- **A term names the quantity it bounds, and never stands in for a quantity
  of another dimension.** A length excess is not an area (§8's `Area` term),
  and a boundary displacement is not a point motion (`sectionDelta` and
  `delta` below).
- **Every row cites a DERIVATION a reader can check, and a term whose
  derivation is not written is not a proven bound.** The `Certified source`
  column states where a term's inputs come from and the `Rounding direction`
  column states which way they are rounded; neither can state WHY the
  published value dominates the quantity the row names, so a term of the
  wrong functional form satisfies both columns while bounding nothing. The
  `Derivation` column closes that gap: it names the site that proves the
  domination — this document's own derivation, or the source helper whose
  doc comment owns it — and a term with no such site is not published at
  all. It answers `+Inf` and the build refuses at Table S row **S14**,
  exactly as an underivable enclosure does. Extending this table means
  writing the new row's derivation, never only its source.
- **Every term is evaluated over the certified source its row names and
  rounded OUTWARD**, so what it publishes over-states the true displacement
  in the direction its consumer needs. No term is read off a held float the
  record's own enclosure did not produce — in particular the achieved
  sagitta `tessellate.go`'s `chordCount` returns is `chordSagitta`'s proven
  bound (`docs/tessellation-design.md` §3) over a held `math.Hypot` radius
  and a held `math.Atan2` sweep, with no enclosure of either behind it.
  Rounding that bound outward over-states the sagitta of those two held
  floats and no quantity the record states, so it decides a station COUNT
  (§5.1) and is never the sagitta this table publishes.
- **An enclosure the record cannot state answers `+Inf`, and the build
  REFUSES** — `ErrUnsupported`, Table S row S14 — never a finite substitute
  and never a published zero. This is `CLAUDE.md`'s reject-only rule at this
  document's own seam: a residual measured against a recorded curve can
  falsify a claim but never admit one, so no term here is admitted by
  measuring the geometry it was handed.

**TWO station kinds GUARANTEE a zero `stationRound`, and the kind is what
the record proves — never the enclosure the generator happened to reach.
Both are NATURAL BOUNDS, `t == 0` or `t == 1`, and this list is what PINNED
names wherever this document or a companion says it of a station.** The
`stationRound` row above owns why a guarantee is all this list claims, and
the zero-`delta` paragraph below owns what may be read from it.

- an **untrimmed `ArcSeg` end**: `extrude.go`'s `arcWalkEnd` PINS the held
  pair to the recorded `Start` / `End` verbatim and stamps a zero bound, at
  `t == 0` and `t == 1` ALONE, so that station IS a recorded coordinate;
- an **untrimmed `LineSeg` end**: `lerp2` and `ratLerp` (`moments.go`) each
  special-case those same two parameters to the recorded `Point2` verbatim,
  so the held pair and the point the record denotes are one coordinate and
  `lineWalkEndBound` answers zero, which is what that function's own doc
  comment states. The pin lives in those two lerps: `lineWalkEndBound`
  carries no natural-bound test of its own and stamps whatever gap its two
  `rationalFloatError` calls measure.

**A station whose generator COMPUTES its coordinates is never granted a zero
by the shape of the record's own enclosure.** `circularEndpointInterval`
encloses the point the record denotes, and at a whole quarter turn
(`quarterTurnSinCos`, `4t` an integer) that enclosure is a POINT interval —
but `circularWalk` still reaches the held pair through its own
`math.Sincos`, and `walkOf` stamps `circularWalkEndBound` over the result
without pinning it. What that bound reports is the held station's own gap
from the enclosure, which a point interval leaves free to be positive; a
zero there is read off the value `circularWalkEndBound` publishes and is
never proven from the kind. The `stationRound` row above owns that value,
and every site reads it as published.

**Every station outside the two pinned kinds above carries whatever its own
walk bound proves for it, the two ends of a chord chain included.** A
`CircleSeg` records no endpoint coordinate at all — `walkOf` stamps
`circularWalkEndBound` on both of its ends unconditionally, quarter turns
among them — and a TRIMMED `ArcSeg` end takes the same computed bound an
interior station does. **A `LineSeg` pair is not exempt from this rule**: a
TRIMMED `LineSeg` end is computed too — `walkOf` fills the walk's own end
from `lerp2` at the recorded parameter, §5's pairing reads that end as the
station, and `lineWalkEndBound` stamps whatever gap its own
`rationalFloatError` calls measure there, a bound no kind proves zero.
Every one of these records is caller-reachable rather than a corner
case: `seam.go`'s `recordEdge` records a certified `Partial` line, circle or
arc fragment over a non-natural range, and no Table S row excludes one. So
pinning BOTH of a pair's stations by the kinds above is what GUARANTEES a
zero-`delta` build — a `LineSeg` pair as much as a pair chorded at `m = 1` —
and only a pair that clears that test is GUARANTEED one. Every other pair
carries whatever its own walk bounds prove, a proven zero included, and
every site in this document reads that published value rather than the kind.

**`sectionDelta` and `delta` bound different objects, and neither ever
stands in for the other.** `delta` bounds the displacement of a HELD VERTEX
from the point the record denotes for it; `sectionDelta` bounds the
displacement of a BUILT CHORD, in the section plane, from the recorded curve
it approximates — a boundary-surface quantity, not a point motion. A reading
that both terms displace — `Bounds`' box, or `Volume` / `Centroid` on a body
that is both displaced and chorded (§8) — sums the two into ITS OWN bound;
the two source terms are never added into one another or substituted for
each other anywhere upstream of that composition.

**`sectionDelta` is a maximum rather than a sum because a boundary point
lies in exactly one cell** — the identical reasoning
`docs/prism-boolean-design.md` §7 and `CLAUDE.md`'s "Section displacement"
cross-cutting note already state for `prismPayload.sectionDelta`, which this
field mirrors by name and by contract.

**`delta` gains exactly one new term over the placement one, `stationRound`.**
The walk that produces a station evaluates `math.Sincos` on a computed angle,
so neither the trig nor its argument is a quantity the walk itself can enclose
while holding floats alone; the enclosure comes from the recorded curve
instead, which is what the table's `stationRound` row names.
`circularWalkEndBound` reports each plane-local component's own gap from that
enclosure and `walkEndBoundAllow` carries the wider component through the
payload's orthonormal frame into one 3D world-space displacement;
`stationRound` is the MAXIMUM of those per-station allowances over the
build and never their sum: the term bounds ONE held vertex's own
displacement, and every consumer spends it as a PER-VERTEX bound —
`perturbedTriangleAreaAllow` over a facet's three corners,
`sweptVolumeAllow` over a held mesh whose every vertex moves by it, §12's
diameter shrink over the two witnesses — so the widest station's own
allowance already covers every vertex, and an accumulation over stations
would scale it with the station count while bounding no quantity this
document reads.
A pairing whose every station is PINNED (the two kinds above) has every
station at a recorded coordinate, so its `stationRound` is exactly zero and
an unplaced such loft's `delta` is exactly `0`. A `LineSeg`-only pairing
earns that only where the record leaves every one of its stations
untrimmed. Unlike `delta`, `sectionDelta` composes from the recorded
curves alone and carries no placement term at all.

## 6. The build-time simplicity / crossing audit

**A ruled wall can cross another wall the two 2D profiles alone would never
reveal** — extreme twist between the two sections is exactly the shape the
target case (a helical tooth) invites. decad never builds an unproven
solid (modify §1), so this is a build-time gate, not a `Verify` question.

The audit tests every pair among the assembled triangle set — the wall
triangles §5.1's Table C gives every cell of every loop, plus the two
triangulated caps (`triangulate.go`'s existing ear-clipping triangulation of
each polygon-with-holes cap). That set's size is §7's `F`, and the audit
reads no count of its own.

**The audit decides CONTACT, never PROVENANCE.** Table C states each cell's
expected contact entity and whether the record or this build produced it;
the audit asks only whether a pair's actual contact IS that entity. So an
admission rule below never says "recorded" — the expected entity is
whatever Table C gives the pair, generated chords and station vertices
included. A shared entity does not prove it is the pair's only contact.
**Adjacency states the expected contact; every pair is classified against
that expectation:**

- **A pair sharing an EDGE is admitted only when the edge-adjacency check
  reports that pair's own expected common edge as its whole shared segment.**
  This applies to a rung, a diagonal, a cap-boundary entity, and an internal
  edge of one cap's own triangulation. The matching segment proves the
  triangle interiors are disjoint; a point, an extra segment, a 2-D region,
  or a crossing refuses.
- **A pair sharing exactly one VERTEX and no edge is admitted only when its
  sole contact is that pair's own expected shared vertex.** A point
  elsewhere, a shared segment, a 2-D region, or a transversal crossing
  refuses. Vertex-sharing pairs need this check because they can cross away
  from that vertex.

The vertex rule is what every consecutive wall pair needs: the lower
triangles of consecutive cells `j` and `j+1` share only the rung's `V` at
the station between them, and their upper triangles share only that rung's
`W`, so each pair must pass the expected-vertex check.
`triangulate.go` produces an interior-disjoint conforming
triangulation of one planar region, so two triangles of the SAME cap meet
only in shared edges and shared vertices — each must produce the expected
classification above. Every cap triangle is also tested against every wall
triangle and every triangle of the opposite cap.

Every pair is tested with `boolean_exact.go`'s existing adaptive
triangle/triangle predicate and `boolean_mesh.go`'s `triTriClassify` — the
identical exact machinery the mesh boolean already uses to decide whether two
triangles are disjoint, share a point, share a segment, or overlap in a 2-D
region. Two flat triangles need no bracket, no interval subdivision, and no
certified polynomial isolation the way two curved bilinear patches would: the
predicate is exact and total.

**`triTriClassify` remains the boolean classifier.** It continues to report
`contactRegion` for every coplanar positive-area intersection and every
positive-length coplanar shared boundary. The audit adds
`triTriCoplanarSharedEdge` for an edge-adjacent pair when that classifier
reports `contactRegion`. The helper receives both exact triangles and the
pair's own expected common edge (Table C). It projects them exactly onto
their common plane and accepts only when that edge's endpoints are an edge of
both triangles and the two opposite vertices lie strictly on opposite sides
of its supporting line. Those conditions prove that the closed-triangle
intersection is exactly that segment. Every other result refuses under S7.
The helper is audit-only and MUST NOT change `triTriClassify` or
mesh-boolean contact classification.

- **empty** (disjoint) → admitted only for a pair Table C gives no shared
  edge or vertex;
- **exactly the pair's own expected common-edge segment** → admitted only for
  the pair Table C gives that edge;
- **a point contact at the pair's own expected shared vertex** → admitted
  only for that vertex-sharing pair;
- **every other classification** — a missing expected contact, a point or
  segment elsewhere, a shared area, or a genuine transversal crossing →
  proven self-contact or self-intersection, `ErrDegenerate` (S7);
- exhausting the fixed work budget before every pair is decided →
  `ErrUnsupported` (S8), never a guess.

**The work budget reuses tessellation design §3's own ceiling.** The
predicate under test here is the same one tessellation's boolean pre-pass
runs, so the audit charges every invocation against
`maxFacetPairTestsPerCall = 8_000_000` (tessellation §3) rather than minting
a second constant for the identical quantity. Before running a single pair
test, compute the conservative `F*(F-1)/2` upper bound (`F` the total
triangle count) with checked arithmetic and refuse before allocation if it
would exceed the ceiling — the same preflight-before-allocation discipline
tessellation §3 states.

`LoftContext` threads a shared `workBudget` (`budget.go`) through the audit,
polling at `workPollInterval` exactly as `FilletContext` / `ChamferContext`
/ `ShellContext` already do (modify §5). Cancellation returns `ctx.Err()`
before commit; the document and recipe stay unchanged. `Loft` is the
`context.Background()` compatibility wrapper.

**This audit is unchanged in kind for a chorded pair.** It still tests every
pair among the assembled triangle set exactly as stated above; only the
triangle count grows with the station chain (§5.1, §7). §5.1's station cap,
which §4's gate-order paragraph places before construction, is what keeps the
CHORDING from carrying `F` past this audit's own `F*(F-1)/2` ceiling: the
cap is the soft limit, S8
above stays the hard one, and the ceiling here is the quantity the cap is
fixed against (§5.1). The two are never merged, and S8 remains the only
resource refusal a build with no circular pair can reach, since such a build
never consults the cap at all.

## 7. Table B — the result

| Face | Count | Surface | Roles |
|---|---|---|---|
| `capStart` | 1 | `Plane`, over `p0`'s region | `capStart` |
| `capEnd` | 1 | `Plane`, over `p1`'s region | `capEnd` |
| lower wall triangle | `stations_i` per loop `i` | `Plane` | `side(i,j,0)` |
| upper wall triangle | `stations_i` per loop `i` | `Plane` | `side(i,j,1)` |

`stations_i` is loop `i`'s own chord-cell count — the sum, over that loop's
paired segments, of each pair's own station count `m` (§5.1). `n_i` (the
loop's segment count) is `stations_i`'s special case: `m = 1` for a
`LineSeg` pair, so a loop with no curved pair has `stations_i = n_i` exactly.

**The assembled triangle count is `F = 2·Σstations + cap triangles`** — the
`2·Σstations` wall triangles above, plus the two caps' own
polygon-with-holes triangulations (§6). **This section is the single owner of
`F`.** A site naming `F` cites this section and states no count of its own.

**This section is the single owner of a loft's `side(i,j,k)` grammar.** `i`
is the loop index — `0` for `Outer`, `1+h` for `Holes[h]`, matching Table
P's own indexing. `j` indexes that loop's flattened CHORD-CELL sequence
(§5.1) — one entry per `LineSeg` pair, `m` entries per curved pair — and
never one entry per recorded segment. `k` is `0` for a cell's lower triangle
and `1` for its upper. A site naming this grammar cites this section and
states no index meaning of its own.

**Lump count is always 1.** The two caps and `2*sum(stations_i)` wall
triangles form one connected, manifold, watertight shell once §6's audit has
passed — there is no shape in increment 1's admitted correspondence that
produces a second lump, unlike a modify op's holed both-caps shell (modify
Table B, row B4).

**`Body.Origin()`** is the loft step, role `"body"`, the same uniform rule
every other feature follows (modify §11).

**Placement is landed (§12 PR 2a).** It needs no geometry-specific payload
case: every loft surface is a `Plane`, and evaluator §8 already states "every
v1 surface variant maps to itself under an isometry (plane→plane, …)." A
placed, duplicated, or placed-copied loft body re-evaluates from the same two
recorded profiles: it re-lifts every vertex from the record and applies the
composed motion ONCE — never moving an already-built mesh incrementally, so
`delta` does not accumulate across repeated placements — reproducing the same
roles (modify §11's "roles derive from the record and the deterministic walk
order") and the same pairing, and re-running the Table S gates §4's placement
paragraph names and §6's crossing audit on the rounded vertex set. §5's
whole-shell orientation step re-decides the sign from the placed triangle
set on its own, so a mirror
flips `reversed` and needs no separate winding-flip case — unlike
`facetedPayload.placed`'s `IsReflection()` handling, which moves a held mesh
rather than re-lifting one.

## 8. Mass properties — derived, not asserted

Write `T` for the set of `2*sum(stations_i)` wall triangles (§7) plus the two
caps' own triangulations, each triangle `(A, B, C)` outward-oriented by §5's
cap seeding and complete-shell rule (material on the left, the same
walk-order convention every payload already uses).

**Volume is a signed sum of tetrahedron volumes from a fixed anchor** —
the standard divergence-theorem reduction for a closed triangulated
boundary, valid for any non-self-intersecting polyhedron regardless of
convexity or hole count, which §6's audit is what makes admissible to use
at all:

```text
Volume = (1/6) * sum_{(A,B,C) in T} (A - anchor) . ((B - anchor) x (C - anchor))
```

**Centroid** is the matching closed form (the standard polyhedron centroid:
a weighted sum of each tetrahedron's own centroid by its signed volume,
divided by the total). Both are **polynomial in the vertex coordinates —
no square root anywhere** — so both reduce to `moments.go`'s existing
discipline: every vertex coordinate is a float, taken exactly as a
`math/big.Rat` (the same "take the floats exactly" rule `clearance_poly.go`
and `spline_bezier.go` already follow), accumulated into the rational volume
and three rational centroid coordinates (anchored at `p0`'s own
`PlaneRecord.Origin`, mirroring `moments.go`'s own
anchor-then-publish-once discipline, §3). At publication, the volume rounds
once and each centroid coordinate rounds once — the identical `addExact` /
`translateExactMoments` / `publishExact` shape `moments.go` already
implements for a 2D region's exact rational accumulator, extended here to a
3D triangulated boundary rather than invented as new machinery.

**Each cap's own contribution is the exact rational shoelace area of the cap
polygon this construction actually ASSEMBLED (§5), never `moments.go`'s
region integral read directly off the record.** **This paragraph is the
single owner of what a cap's assembled polygon IS**, over the per-cell
cap-boundary entity §5.1's Table C states. A site naming that polygon cites
this paragraph and states no definition of its own. For a
`LineSeg`-only loop whose every station is PINNED (§5.2) the assembled cap
polygon is the region boundary itself, so the shoelace rational equals
`moments.go`'s own region rational there — no new 2D integration for that
case. A TRIMMED `LineSeg` station is a computed point rather than a recorded
one (§5.2), so that equality is not stated for it: the assembled polygon
sits within `delta` of the region the record denotes, which is the term
§8 already composes into every reading it feeds. For a same-kind circular
pair the assembled cap polygon is instead the chord chain §5.1 built: `addCircular`
(`moments.go`) calls `dropExact()` unconditionally, so an arc-bearing
`ProfileRecord`'s own region integral is never an exact rational, while the
ASSEMBLED chord polygon's vertices are the same held float64 points the
triangulation already holds, taken exactly as `math/big.Rat` — the identical
"take the floats exactly" lift the wall sum already uses. Reading the cap
term from the built polygon rather than the record's region integral is what
keeps `Volume`/`Centroid` exact-rational for a chorded loft: the two caps
and the chord-chain wall triangles then integrate over the SAME assembled
boundary, with no region-versus-chord mismatch at the cap seam.

**`Volume` is `Exact` exactly when its published rational is representable in
the `units.Value` magnitude it carries, AND the payload's displacement
`delta` is zero, AND its section displacement `sectionDelta` (§5.2) is zero —
never unconditionally.** This is spline design §3's Tier A rule, verbatim:
"the reported bound is a SINGLE rounding of that rational into that
magnitude, and it is zero — hence `Exact` — exactly when the rational is
representable in the magnitude the value ACTUALLY CARRIES." A loft's volume
earns that ceiling for the same reason a Tier A free-form region's area does:
the integral is exactly rational; only its final publication rounds. A body
whose `delta` is positive — placed (§12 PR 2a), chorded past §5.2's one
pinned station kind, or both — composes `bounds.go`'s
`sweptVolumeAllow(delta, areaUpper)` on top of that single rounding, so
`delta` alone is enough to make the reading `Approximate` however exactly
any placement's own rotation or reflection is representable. The condition
is `delta > 0` and never that the body was placed: an unplaced chorded body
whose computed stations carry a positive `stationRound` (§5.2) takes this
allowance exactly as a placed one does. A chorded (same-kind circular)
body's volume additionally composes `bounds.go`'s
`chordedBoundaryVolumeAllow(matchedDelta, wallAreaUpper, twistVolumeUpper,
capVolumeUpper, seamAllow)` — a twin over the chord-to-curve homotopy rather
than the placement's rigid one, and **never a `sweptVolumeAllow`-shaped
`(sectionDelta, areaUpper)` pair**, which charges the wall leg alone and
understates a twisted pairing by about five orders of magnitude
(`TestChordedBoundaryVolumeAllowTwistLegIsLoadBearing`). The helper composes
FOUR legs by `absSumUpper`, each its own mechanism with its own charge: the
wall chord-to-curve leg `matchedDelta · wallAreaUpper`, the twist leg
`twistVolumeUpper`, the cap leg `capVolumeUpper`, and the seam leg
`seamAllow`. §5.2's table states every one of those terms, `matchedDelta`
among them — `absSumUpper(sectionDelta, delta)`, the sagitta the ideal chord
commits plus the displacement the held chord's own computed endpoints carry;
§8.1 states which mechanism each leg answers for and where its derivation
lives. So `sectionDelta` alone is enough to make the
reading `Approximate` even where `delta == 0`, which is the `m = 1` pair
whose two end stations §5.2's table pins (§12). A body that is both placed
and chorded composes both terms, since each bounds a
displacement committed at an independent stage of the construction — the
section chording, then the rigid placement.

**`Centroid` publishes three exact rational coordinates as a
`VecMeasurement`, not a `units.Value`.** Round each coordinate once into the
returned `r3.Vec`. Its `Bound` is the length radius enclosing all three
coordinate-rounding errors, and it is `Exact` only when every coordinate has
zero rounding error AND the payload's displacement `delta` and section
displacement `sectionDelta` are also zero. This is the existing `moments.go`
centroid publication pattern, extended from the plane-local two-coordinate
result to this 3D triangulated boundary. A body whose `delta` is positive
(§5.2) widens each coordinate's bound by the same quotient composition
`moments.go`'s `boundedQuotient` states, using `sweptVolumeAllow` as the
denominator's own allowance and `sweptMomentAllow` as the numerator's — a
placed body (§12 PR 2a) and an unplaced chorded one alike, since the pair is
keyed to `delta` and not to what produced it. A body whose `sectionDelta` is
positive widens it again by the matching `chordedBoundaryVolumeAllow` /
`chordedBoundaryMomentAllow` pair (`bounds.go`), each doing for
`sectionDelta` what the swept pair does for `delta`. **The moment twin takes
the volume twin's four legs and TWO further arguments, and both of them widen
a radius the volume reading never needs.**
`chordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper,
capVolumeUpper, seamAllow, maxTwistOffsetUpper, coordUpper)` reads the
relation a region of proven volume `V` inside radius `R` obeys, `|∫p dV| ≤
V·R`, and `R` is `coordUpper` WIDENED rather than `coordUpper` itself:
`R := coordUpper + matchedDelta + maxTwistOffsetUpper`, composed by
`absSumUpper`. The symmetric difference whose moment it bounds extends
outside every held vertex — by `matchedDelta` on the two chord-to-curve legs,
since a curve point sits that far from its own chord at the matching
parameter, and by `maxTwistOffsetUpper` on the twist leg (§5.2). The seam
leg's own displaced material sits inside that same neighbourhood and takes no
term of its own. A build whose COMBINED volume allowance — the placement term,
the chording term, or the two composed — is not smaller than the held volume
leaves that quotient's denominator non-positive, and the centroid is
unstateable — refused `ErrUnsupported` (Table S, S12) rather than published
with a bound nobody could use.

**`Area` is never Exact.** A triangle's own area is `(1/2) * |(B-A) x
(C-A)|` — a square root of a rational, generically irrational — so a wall
triangle's area contribution carries a proven outward bound derived at that
triangle's OWN scale: `(B-A) x (C-A)` is taken over exact rationals (the same
"take the floats exactly" lift the volume sum already uses), its squared norm
is therefore an exact rational, and `spline_length.go`'s outward-rounded
`ratSqrtDown` / `ratSqrtUp` bracket that rational's square root. The published
bound sums those per-triangle enclosure widths beside the summation loop's own
slop (`bounds.go`'s `sumSlop`). **Both of those two terms are upper bounds
nudged outward once per triangle, so each can SATURATE at `+Inf` on a wall set
whose areas approach `float64`'s own ceiling — while the plain sum they speak
for stays finite by rounding whole triangles away. A saturated term states no
scale, so the published bound is `+Inf`, never the zero `sumSlop` reports for a
non-finite `absSum`**: the enclosure widths are exactly zero whenever every
triangle's own area is representable, so the two together would otherwise leave
a saturated sum claiming `Exact` over mass it has already swallowed, with a true
error that here runs past `MaxFloat64`. Any finite substitute would be an
unproven guess. A bound scaled off the held TOTAL bounds only
the loop: a float cross product's forward error scales with its products
rather than with its result, so on a thin triangle it exceeds any such bound by
roughly one over the triangle's aspect ratio — and Table B's diagonal split
makes thin walls the ordinary case for a short loft over long recorded
`LineSeg`s. Spline design §3 states the identical asymmetry for arc length:
"Arc length is never exact in ANY tier… a Tier A body's `Area` always
carries a positive bound even where its `Volume` does not." A loft's `Area`
is the two caps' own exact rational SHOELACE area over the polygon this
construction assembled (above — never `moments.go`'s region integral read off
the record, which `addCircular`'s own unconditional `dropExact()` leaves
non-rational for an arc-bearing record), contributing no bound of its own,
plus the wall triangles' proven-bound sum — so the total is
`Approximate` with a proven bound whenever at least one wall triangle has
nonzero area, which increment 1's admitted correspondence always does.

A body whose `delta` is positive (§5.2) adds one further term to that total:
the per-triangle area allowance `bounds.go`'s `perturbedTriangleAreaAllow`
states, summed over every wall AND cap triangle. The caps need it as much as
the walls do — a cap's contribution is the assembled chord polygon's exact
rational area (above), and the built cap triangles are within `delta` of the
points that polygon's own vertices denote. The trigger is `delta > 0`, so an unplaced chorded body whose
computed stations carry a positive `stationRound` (§5.2) adds the term too; a
placement (§12 PR 2a) is one way to reach it, not the condition.

A CHORDED (same-kind circular) body's `Area` reaches for TWO further terms,
and only one of them has a proven owner. **The quantity each bounds is an
AREA, and an arc-minus-chord LENGTH excess is neither** — it carries one
length dimension too few to sit in an `Area` sum at all.

- **The cap term is owned.** A cap's held reading is the assembled chord
  polygon's shoelace area, and the region that cap's recorded boundary
  denotes differs from it by at most `capAreaAllow` — `sectionDisplacementArea`
  over that cap's own boundary (§5.2), the same term the cap VOLUME leg
  charges one dimension up. The two caps' `capAreaAllow`s enter `Area`'s bound
  directly, with no plane-offset division of any kind: a cap's area gap is an
  area gap whether or not its own plane passes through the anchor.
- **The wall term is NOT owned, and this design refuses rather than invent
  one.** The quantity is `|Area_held − Area_true|` over the wall: how far the
  flat chord facet pair this construction builds over one chord cell sits from
  the area of the curved ruled surface between the two recorded curve pieces
  that cell approximates. `cellChordCurveAreaUpper` does NOT bound it — that
  helper bounds the true surface's own area from ABOVE (§5.2), which is what
  the volume leg's flux identity needs and is not a bound on a DIFFERENCE of
  areas. `cellTwistVolumeAllow` bounds a volume, not an area. No shipped
  helper owns the difference, and substituting either of those two would
  publish a number that bounds a different quantity than the one the reading
  states.

**So the chorded increment does not land its `Area` until a helper owns that
difference.** §12's own rule is that all four measurements land with the
operation — a `Body` caches `Volume` / `Centroid` / `Area` / `Bounds` at build
with no staging error path — so there is no accessor behind which a missing
`Area` bound could be staged, and the consequence falls on the INCREMENT
instead: §12 PR 3 lands the chorded correspondence only once `bounds.go` owns
a wall cell's `|Area_held − Area_true|` with a written derivation, and until
then a same-kind circular pair stays unlanded — `ErrUnsupported`, the staging
refusal every §12 row before PR 3 already carries for it (§14). A `LineSeg`-only build is untouched — its walls chord
nothing, so its `Area` is the held triangle sum this section already derives.
An invented wall term would be the one outcome `CLAUDE.md`'s
proven-or-refused rule forbids, whatever margin a fixture measured for it.

**`Bounds` is Exact only when BOTH the payload's displacement `delta` and its
section displacement `sectionDelta` (§5.2) are zero.** Every vertex is
already treated as exact where `delta` is zero (§5, §5.2); the axis-aligned box is
the componentwise min/max over that vertex set, the same componentwise-extreme
reasoning Extrude's `Bounds` applies to its own candidate set — no new
rounding is introduced by comparing exact numbers. Extrude's candidates are
not all exact (a computed walk endpoint or arc radius carries a bound there),
and the reasoning shared is the min/max step, never a claim about the other
feature's inputs. **`Bounds.Bound` is `absSumUpper(delta, sectionDelta)`, the
SUM of the two terms, never `delta` alone.** A chorded curve bulges OUTSIDE
the station polygon that holds its vertices, so a box that carried only
`delta` would understate the box the true recorded curve actually occupies —
the one direction that is unsound, since `Verify`'s box-disjointness proof
(Table D row D3) reads `Bounds` to certify two bodies apart, and a box too
small for its own body can certify a pair apart that is not. Composing both
terms is what keeps that certificate sound for a chorded body exactly as it
already is for a `LineSeg`-only one, where `sectionDelta` is zero and the sum
reduces to `delta` unchanged. **A chorded body's `Bounds` is `Approximate`
whatever its `delta` is**, since `sectionDelta` alone is positive on every
chorded pair: even at `m = 1` on a pair whose two end stations are both
pinned (§5.2), where `delta` is exactly zero and every held vertex is a
recorded coordinate, the box still widens, because the box is about the TRUE
recorded curve between those two stations and not about the vertices alone.
A chorded pair whose end stations are NOT pinned carries whatever its own
`stationRound` proves (§5.2), and `Bounds.Bound` sums both terms as the rule
above states at whatever value that is.

**Vertex position, edge length, and face area follow the standing rules
already governing every other analytic payload, each composed with the
payload's own displacement `delta`.** Where `delta` is exactly zero — an
unplaced pairing whose every station is pinned by §5.2's table — a position
is Exact by construction (§5), and a straight edge's length and a triangle's
own `Area()` need a square root and are `Approximate` with a proven bound,
Exact only when that particular evaluation happens to be exactly
representable — the same standard Extrude's own `LineSeg` walls and edges
already carry. Where `delta` is positive, all three readings carry it: a
vertex position publishes `delta` itself as its bound, and an edge length
and a face area each add a strictly positive `delta` term to the bound they
would otherwise publish, so none of the three is Exact however exactly its
own evaluation happens to come out. **A placement (§12 PR 2a) is one way
`delta` becomes positive and a computed station's own `stationRound` (§5.2) is
the other, and the rule reads `delta` rather than the mechanism** — so an
unplaced chorded body whose computed stations carry a positive `stationRound`
publishes no Exact position either. This
document introduces no new per-accessor rule beyond what §8 already derives
for the body-level quantities.

### 8.1 The four legs of the chorded volume allowance

**`chordedBoundaryVolumeAllow` composes four legs by `absSumUpper`, and every
one of them is derived in `bounds.go` rather than here.** This section states
which mechanism each leg answers for and which helper owns its derivation;
§5.2's table states each term's certified source, rounding direction and
refusal, and §8's `Volume` and `Centroid` paragraphs state where the two
composed readings spend them.

The gap the composition bounds is between the HELD flat-triangle polyhedron —
the two triangles per wall cell §5's Table B assembles and §8's accumulator
actually sums tetrahedra over, never a ruled patch — and the TRUE solid the
two paired curved sections denote.

| Leg | The mechanism it answers for | Where its derivation lives |
|---|---|---|
| **wall chord-to-curve** | `matchedDelta · wallAreaUpper`. Every point of a wall cell's BILINEAR RULED patch moves along the straight path to the curve point AT ITS OWN PARAMETER, a motion of at most `matchedDelta`; a parametrized patch's signed volume is a polynomial in its boundary, so `\|dV/dt\| ≤ matchedDelta · A(t)` along that path | `cellChordCurveAreaUpper`'s own doc comment, whose `eA·eB` product bounds `sup_t A(t)` ABSOLUTELY — the area of every surface the homotopy visits, never a held facet area plus an excess |
| **twist** | `twistVolumeUpper`. The ruled patch is not the surface the evaluator holds: the held pair of flat triangles is a different surface, and the gap between the two is its own mechanism the wall leg's homotopy never visits | `cellTwistVolumeAllow`'s parts (a), (b) and (c): the deviation is exactly `r·(s−1)·T` and `s·(r−1)·T`, at most `\|T\|/4`; the homotopy's own area is at most `eA·eB` at every time; the flux identity closes them, with no seam correction needed because that deviation vanishes on all four edges of the parameter square |
| **cap** | `capVolumeUpper`. A cap has no second section to rule toward, and its own vertices never move under this homotopy — they are boundary points of the same recorded profile the wall cells chord — so only its 2-D region's shape changes | `capAreaVolumeAllow`'s EXACT planar identity: a planar face's own signed-tetrahedron sum is `2·h·Area(cap)` whatever the triangulation, so replacing the held polygon's area with the denoted region's changes it by exactly `2·h·ΔArea`, giving `\|ΔVolume_cap\| ≤ \|h\|·\|ΔArea\|/3` |
| **seam** | `seamAllow`. The wall leg's flux identity is the formula for a CLOSED surface, but the wall is an OPEN patch whose `r=0`/`r=1` seam moves under the SAME homotopy, leaving a by-parts line integral the wall leg never charges | `chordedBoundarySeamAllow`'s own doc comment: Cauchy-Schwarz on that residue, `matchedDelta · posUpper · seamPerimeterUpper / 3` |

**Summing the four is sound because the difference telescopes exactly.**
Writing `W_true`, `W_ruled` and `W_tri` for the wall's true, ruled-patch and
held-triangle contributions and `C_true`, `C_held` for the cap's,

```text
(W_true - W_ruled) + (W_ruled - W_tri) + (C_true - C_held)
  = (W_true + C_true) - (W_tri + C_held) = V_true - V_held
```

so the triangle inequality bounds `|V_true − V_held|` by the three magnitudes.
The wall and seam legs TOGETHER bound the first — never either alone, since
the by-parts split is what separates them — the twist leg is the second, and
the cap leg is the third.

**No leg may be dropped for another's incidental slack.** The twist leg is
load-bearing on its own measurement: deleting it understates a twisted
pairing's true gap by about five orders of magnitude
(`TestChordedBoundaryVolumeAllowTwistLegIsLoadBearing`), which is why the
composition takes four terms and not the wall leg's own `(sectionDelta,
areaUpper)` pair. Whether the WALL leg could be deleted given the other three
is an open question `bounds.go` records and this document does not settle;
leaving it in can only make the published total larger, and DOMINATION — that
the total bounds the true gap — is proven leg by leg above whatever that
question's answer turns out to be.

**The moment twin reads this total as a region measure, and only two of the
four legs license that.** The wall and twist legs ARE measures and already
dominate the swept symmetric difference on their own; the cap leg is an exact
signed identity whose material never leaves its own plane, and the seam leg is
a contour residue attached to no region at all, so the two can only add to a
total that already sits at or above the measure.
`chordedBoundaryMomentAllow`'s own doc comment owns that argument, and §8's
`Centroid` paragraph states the widened radius `R` it needs.


## 9. Table D — downstream

| D | Consumer | Reads | Increment-1 status |
|---|---|---|---|
| **D1** | `Tessellate` / `STL` / `OBJ` | the payload | works from the first PR that wires it in (§12 PR 2b), and the returned `Bound` is **the payload's own facet departure `absSumUpper(delta, sectionDelta, maxTwistOffsetUpper)`** (§5.2, §8, §12 PR 2a), not unconditionally zero and never `delta` alone: that sum is zero only for a `LineSeg`-only loft under an identity motion, and pinning that build's every station is what GUARANTEES it zero (§5.2), a tessellation that is still restatement with a zero bound. Every wall and cap face of a body whose `delta` is positive — placed, holding a station §5.2's table does not pin, or both — is a flat triangle over held vertices that are no longer provably exact; every wall facet of a CHORDED body chords a recorded curve it departs from by `sectionDelta` (§5.2) whatever the placement; and where a CHORDED cell twists, that held flat triangle pair is not the bilinear ruled patch through its own four corners either, a further departure of at most `maxTwistOffsetUpper` (§5.2, §8.1) — a term a `LineSeg`-only build charges nothing to, since §5 makes its held triangle pair the boundary itself. So tessellation restates exactly what the payload holds, all three terms included. `Bounds.Bound` is the one loft reading that stays the two-term `absSumUpper(delta, sectionDelta)`, for the reason §5.2's own row gives |
| **D2** | the mesh boolean (`Union`/`Cut`/`Intersect`, evaluator §9) | the tessellation | a first-class operand once D1 lands — no new boolean code, a loft body is just another all-planar operand. A loft whose facet departure is exactly zero — which an unplaced `LineSeg`-only loft whose every station is PINNED (§5.2) is what GUARANTEES — is admitted through the existing all-planar zero-bound path (`docs/evaluator-design.md` §2 — "the VOLUME of an all-planar pair whose contact points round exactly"); every loft whose facet departure is positive — one under a non-identity motion, a CHORDED one, one holding a station §5.2's table does not pin whose own walk bound proves nonzero, or any combination of the three — hands the boolean its own facet departure `absSumUpper(delta, sectionDelta, maxTwistOffsetUpper)` as the operand displacement every other nonzero-bound operand already carries (`bounds.go`'s `rimDelta`), so the result's volume is `Approximate` like any other. That is D1's own term and never a two-term subset of it: what a boolean intersects is the FACET, and a twisted CHORDED cell's facet departs from the surface it stands for by `maxTwistOffsetUpper` beyond the two section terms (§5.2, §8.1). **A chorded loft is not a zero-bound operand however it is placed**: at `m = 1` on a pair whose two end stations §5.2's table pins, its `delta` is exactly zero while its `sectionDelta` is positive (§5.2, §8), so admitting it on `delta` alone would hand the boolean a zero bound for a boundary §8 states departs by `sectionDelta` |
| **D3** | Interference (`docs/interference-design.md`) | box separation (D6-style) reads `Bounds` directly; the read-only mesh-boolean path reads D2's tessellation | box-disjoint pairs prove only their disjoint-interior interference relation (`Bounds` carries the payload's own two-term `absSumUpper(delta, sectionDelta)`, §5.2, §8). `Verify` is `Sound` only when every other required or requested body and pair check is decided and trusted; a pair needing the mesh boolean works once D2 lands; a pair needing the analytic containment/pair kernel stays `Suspect` until a loft case is added to `clearance_geom.go`'s payload switch — identical staging to the cup's own D6 row in `docs/modify-design.md` |
| **D4** | Clearance (`WithClearances`, `docs/clearance-design.md`) | the analytic pair kernel's payload switch | `WithClearances` stays `Suspect`, even for a box-disjoint pair: box separation proves disjoint interiors but does not measure the gap. No loft case exists in the kernel yet. |
| **D5** | `MinWallThickness` / `Undercuts` / `MinRadius` (verification §6, `survey2d.go`) | one constant 2D cross-section (a prism's section, a revolve's meridian) | The corresponding requested survey is `Suspect` until its loft implementation lands. In increment 1, a loft's cross-section varies continuously between the two profiles, so the existing spanning-disk / meridian-walk reduction does not reach it; `docs/modify-reach-design.md` DX9 states the identical cap-blend reason: "not one constant section at one height… the existing 2D spanning-disk proof does not decide them" |
| **D6** | `Verify` — structural audit + tolerance gate | topology + measurements | valid by construction once §6's audit has passed (modify §1's standard; §4's gate-order paragraph owns where that audit sits); the tolerance gate judges `Volume`/`Area`/`Centroid`/`Bounds` on the terms §8 derives — wherever the payload's `delta` is positive (a placement, §12 PR 2a, or a computed station whose own walk bound proves nonzero, §5.2) all four carry it, so the gate judges four readings that all carry that displacement |
| **D7** | `Placed` / `Duplicate` / `PlacedCopy` | the payload | landed (§12 PR 2a): `Placed` retires the receiver; `Duplicate`/`PlacedCopy` leave it live. No geometry-specific payload case is needed (§7) — every reading composes the payload's own proven displacement `delta` (§5, §8). |

## 10. Recipe, provenance, and replay

**The `Step`.** A loft step reuses the existing `Profile` / `Plane` fields
for the **from** section (their doc comments extend to name Loft alongside
Extrude/Revolve, §15) and carries the **to** section and the alignment
inside a new sealed `StepOpts` variant — the established mechanism for
adding op-specific recorded data without widening `Step`'s own field list
(`ShellOpts` landed the same way for `modify-design.md`):

```go
type LoftOpts struct {
    Profile2  ProfileRecord // required — the "to" section
    Plane2    PlaneRecord   // required — the "to" section's plane
    Alignment []int         `json:"alignment,omitempty"` // per-loop rotation offset; absent means every offset is 0
}
```

| Field | Value |
|---|---|
| `Op` | `OpLoft` (wire token `"loft"`) |
| `Inputs` | `[]` (empty) |
| `Profile` / `Plane` | the **from** section, exactly as Extrude/Revolve record theirs |
| `Extent` / `Angular` / `Axis` | absent — `OpLoft` falls under core §6.2's "every other Op leaves both nil" |
| `Selectors` / `Values` | empty |
| `Opts` | `LoftOpts{Profile2, Plane2, Alignment}` |
| `Placement` | absent |

`Profile2` and `Plane2` are REQUIRED wire content, exactly as
`ExtrudeOpts.Taper` and `ShellOpts.Sense` are required today (core §6.2): a
missing or explicit-null `LoftOpts`, or one missing either field, rejects.
`Alignment` is the one optional field, `omitempty`, decoding to "every
offset 0" when absent — never distinguished from an explicit all-zero list,
since the two mean the same intent.

`OpLoft`, `Profile2`, and `Plane2` are version-2 wire content. Replay §2.1
owns the version-1/version-2 decode, canonical encode, and migration rules.

**Recipe validation (replay §3) independently re-proves BOTH profiles**,
exactly as it already re-proves the one profile an Extrude or Revolve step
carries — reconstructing each in a private `sketch` arrangement and matching
it exactly to the stored record (replay §3.1). It additionally checks Table
P's own shape (P2, P3) and Table S's own malformed-alignment gate (S4)
structurally, with no geometry construction needed for any of them —
they are checks on recorded slice lengths and integers, the same class of
check replay §3.1 already runs for every other closed-set field.

**Replay is deterministic for the same reason every other feature's is**
(evaluator §1, modify §11): the evaluator reads only the two validated
records, never a live sketch. The pairing (Table P), the construction (§5),
and the audit (§6) are closed-form functions of that recorded data: a
same-kind circular pair's station chain (§5.1) is a closed-form walk-up over
the two records' own certified radius and sweep enclosures (§5.2), evaluated
against the fixed `loftChordFraction` constant (§5.1, §14) — the same two
records and the same constant always produce the same station count and the
same station coordinates, with no live sketch consulted and no caller input
threaded through the wire (§10). A replay reproduces the same triangles, the
same roles, and the same measurements every time.

## 11. Cancellation and work budget

Covered in full by §6, which owns the audit's budget, its ceiling and its
cancellation. §13's fixture wall-clock budget paragraph owns the build cost
model — which phase costs what, and what each phase is linear or quadratic
in. This section restates neither and adds no discipline of its own.

## 12. Increments

These PR labels are the count-free Loft delivery plan. They do not consume a
global evaluator increment.

| PR | Lands | Still refused after it |
|---|---|---|
| 1 | `OpLoft` wire/recipe plumbing (`LoftOpts` codec, `Op` token, `Step.Profile`/`Plane` reuse), Table P pairing + Table S gates S1–S5/S9–S11, the flat-triangle wall construction (§5), the crossing audit (§6, Table S S6's RECORDED arm, S7's audit arm, S8), `Document.Loft` / `LoftContext`, `Volume` / `Centroid` (§8's rational accumulator) / `Area` / `Bounds`, `Verify` (D6: the structural audit and the tolerance gate over all four) | same-kind `CircleSeg`/`ArcSeg` correspondence; N-section/guide-rail/centerline loft; `Placed`/`Duplicate`/`PlacedCopy`; reversed correspondence; surveys, clearance, interference beyond box-disjoint |
| 2a | `Placed` / `Duplicate` / `PlacedCopy` (D7): the payload's own proven displacement term `delta` (§5), composed into every vertex, edge length, face area, and all four body measurements; Table S gains S12 and S13 | D1/D2 (`Tessellate`/`STL`/`OBJ`, mesh-boolean admission); D3/D4's analytic-kernel case; D5 |
| 2b | `Tessellate` / `STL` / `OBJ` (D1), mesh-boolean admission (D2) | D3/D4's analytic-kernel case, D5 |
| 3 | same-kind `CircleSeg`/`ArcSeg` correspondence (§1): the chord-chain construction and its shared station generator (§5.1), every term §5.2's table lists that a chorded build reaches — the certified per-cell sagitta and the `sectionDelta` it publishes, the `stationRound` term `delta` gains, the `matchedDelta` those two compose, and the four legs of the chorded volume allowance with the moment twin's own widened radius (§8.1) — composed into `Volume`/`Centroid`/`Bounds`, Table S gates S14–S16, S6's COMPUTED arm, and S7's structural walk-sense arm (P5). **This row lands only once `bounds.go` owns a wall cell's `Area` difference** (§8, §14): until then a chorded body has no `Area` bound to publish, and a same-kind circular pair keeps the `ErrUnsupported` staging refusal the rows above carry | free-form and mixed-kind correspondence (§1); N-section and guide-rail/centerline lofts; a loft case in `clearance_geom.go`; a non-constant-cross-section wall survey kernel |
| 4 (reach, not committed by this document) | N-section and guide-rail/centerline lofts, a loft case in `clearance_geom.go`, a non-constant-cross-section wall survey kernel | — |

**The four measurements land with the operation, never after it.** A `Body`
caches `Volume` / `Centroid` / `Area` / `Bounds` at build and its accessors
return that cache with no staging error path, and `Exact` is `Exactness`'s
zero value, so a PR that published `Document.Loft` before §8's derivations
would answer `Volume()` with a proven-exact zero and `Verify` would call it
`Sound` — a confidently wrong number, which is the one outcome core §1 exists
to prevent. §8 derives all four in closed form from the same triangle set the
construction already builds, so there is nothing to stage on the measurements
themselves. The tolerance gate does need one piece of per-payload wiring
beside them, though, landed in the same PR: `bodyGateDiameter` (verification
§3) reads a `loftPayload`'s reference off its own held vertex set, because
`Area` is always `Approximate` (§8) and a body with no reference diameter can
never clear the gate's relative tolerance.

On an uncurved payload whose `delta` is zero (every paired segment a
`LineSeg` whose two stations §5.2 PINS, and the accumulated motion
`r3.Identity()`, which is what UNPLACED means — §5.2's `placeAllow` row owns
that test) the vertex set's own maximum
IS the body's true diameter — every vertex is exact (§5), so a convex-hull
diameter realized at vertices is that diameter, not an envelope — and the
arm reports the shared
reader's answer unchanged, with no subtraction and no rounding of its own. That answer is the
largest `float64` at or below the true diameter, because the reader publishes
every witness maximum rounded toward zero (verification §3), so the arm
carries the tightest lower bound a `float64` can state on a quantity that is
exact. **For a same-kind circular pair whose stations are all PINNED (§5.2)
the vertex set's own maximum is at or below the body's true diameter, never
necessarily equal to it**: a pinned station is a coordinate the record
states, so it lies ON the recorded curve rather than at an extreme of it,
the true boundary can bulge past the station polygon (§5.2), and the held
diameter can only understate the true one. The arm's own reasoning holds
there without change, because it only ever needs the held reading to be a
sound LOWER bound on the true diameter, never an exact one, and a
pinned-station diameter is that: every witness is a real point of the true
boundary, so no true pairwise distance the arm could be missing exceeds the
diameter it would have measured directly. **A COMPUTED station's KIND earns
it no such standing, and wherever its own `stationRound` is positive the
unshrunk reading is not a lower bound over one**: the walk reaches it
through `math.Sincos` and it sits only within that bound of the point the
record denotes (§5.2), so it can then sit OUTWARD of the true boundary and
the raw station-set maximum can EXCEED the true diameter — the unsound
direction, since an overstated reference loosens the relative tolerance gate
toward a false `Sound`. Only the shrunk reading is a lower bound there, and
the rule that follows is what supplies it — conditioned on the value `delta`
takes and never on the station's kind. **A
payload whose `delta` is positive holds every vertex only within `delta` of
its true position, so each of the two farthest points can move by `delta`
and the reported reference is the held diameter minus `2*delta`, rounded
down**: an understated reference can only tighten the gate into a false
`Suspect`, never loosen it into a false `Sound`. The arm applies that
subtraction on `delta > 0` and never on the body having been placed, so a
placement whose `placeAllow` is positive (PR 2a) and an unplaced chorded
build whose computed stations carry a positive `stationRound` (§5.2) both
take it, while an identity
placement of a `LineSeg`-only pairing whose every station is PINNED (§5.2)
leaves `delta` zero and reports the unshrunk reading. A shrink that
collapses to zero or below reports no diameter at all, the same answer any other unusable magnitude gets. That last
branch is defensive rather than a reachable reference-less `Suspect`: the
divergence theorem bounds a closed boundary's own volume by `d*A/3` for `d`
the vertex-set diameter and `A` the held surface area, so a `delta` at or
above `d/2` makes `sweptVolumeAllow`'s `delta*A` at least `3/2` of the held
volume, and S12 refuses that placement before any gate reads it.

**A curved pair chorded at `m = 1` (§5.1) has no interior station, and only
pinning BOTH of its two end stations by §5.2's table guarantees it a zero
`stationRound`** — which for a curved pair means an untrimmed
`ArcSeg`'s two recorded ends, the one pinned kind that table names. On such
a pair `delta` is zero, so the gate arm reports the unshrunk held-diameter
reading, exactly as the uncurved zero-`delta` case above. That is sound
rather than an oversight: every vertex there is a coordinate the record
states, so the held diameter's witnesses are exact points on the true
boundary, and the pinned-station half of the arm's soundness argument above
covers the case without needing `sectionDelta` at all. `sectionDelta`
bounds how far the BUILT WALL departs from the curve between those two exact
stations, a question the diameter-witness argument never asks. **Every other
`m = 1` pair — every `CircleSeg` end, quarter turns included, and a trimmed
`ArcSeg` end — is a COMPUTED station carrying whatever `stationRound`
§5.2 proves for it, and the arm shrinks by `2*delta` at that proven value,
as on any other build.** A `CircleSeg` end is never exempted by its
enclosure being a point interval (§5.2), so no quarter-turn pairing is
GUARANTEED a zero `stationRound` by its kind — it stays a COMPUTED station
either way, and where its own `circularWalkEndBound` proves that zero the
arm reads the resulting zero `delta` by value and reports the unshrunk
reading, exactly as it does for any other `delta`.

## 13. Required tests

Every test asserts on computed geometry — coordinates, volumes, residuals —
never merely that a call ran (project rule).

**Every fixture below is an UNPLACED body unless its own bullet says
otherwise** — its accumulated motion is `r3.Identity()`, the test §5.2's
`placeAllow` row owns. Unplaced is not the same as zero-`delta`, and only the
second licenses an exactness or zero-bound assertion: an unplaced chorded
fixture is granted a zero `delta` only where every one of its stations is
pinned by §5.2's table, so each such assertion below is read at `delta` zero
and names why its own fixture is there. §8 owns the rule every reading follows —
`delta` enters every vertex, edge length, face area, and all four body
measurements wherever it is positive — so no fixture here may be reused
against a placed or a chorded body without carrying it.

**The fixture wall-clock budget.** Every fixture in this section builds its
loft in 2 seconds or less, and at most three of them chord a curve at the
calibrated `loftChordFraction` (§14) rather than at a coarse or explicitly
forced station count; a fixture whose build needs longer runs behind
`testing.Short()` rather than shipping in the default `go test ./...` run.
**This paragraph is the single owner of the build cost model.** §6's audit
is the phase this budget governs — it tests every pair among the assembled
triangle set, so its cost grows with the square of `F` (§7), while pairing
and construction are linear in `Σstations`, which chording puts above `Σn_i`
(§5.1). A site naming the build cost model cites this paragraph and states
no cost of its own. The budget bounds which
fixtures ship, never what the evaluator admits: a station count that misses
it is a fixture this section excludes, never a reason to loosen §5.1's chord
target or its cap. §14 records the reference fixture's own measured build
against this budget.

- **Pairing**: hole-count mismatch → S1; segment-count mismatch → S2;
  mixed-kind or free-form segment pair → S3; a same-kind `CircleSeg` pair
  whose two recorded `CCW` flags disagree → S7's `ErrDegenerate` at the
  structural gate, asserted to refuse before construction rather than from
  the audit — the position §4's gate-order paragraph gives it — so the fixture
  pins the gate's position and not only its sentinel; malformed
  `WithLoftAlignment` (wrong length, out-of-range offset, or duplicate
  option) → S4; nil and foreign `LoftOption` values (including a type with a
  promoted sealed marker) → S11 before their callbacks run and with the
  document unchanged; geometrically coplanar sections → S5, including two
  distinct `PlaneRecord`s with the same plane but rotated `U`/`V` bases; a
  nonzero alignment offset pairs the expected rotated vertex, asserted on the
  built wall's own coordinates. A
  two-hole fixture whose recorded `Holes` order is swapped between the two
  profiles MUST assert the two ordinal pairings by wall coordinates, or S7 for
  the resulting crossed correspondence; a nearest-hole matcher MUST fail it.
- **Construction**: every wall/cap edge bounds exactly two faces; every
  triangle has positive area; each cell contributes exactly the entities
  §5.1's Table C gives it, at that table's own counts; the two caps'
  triangulation matches `triangulate.go`'s existing polygon-with-holes output
  over the cap polygon §8's cap-contribution paragraph says this construction
  ASSEMBLED — the recorded region boundary for a `LineSeg`-only loop, the
  chord chain for a chorded one — and never over the record's region for a
  chorded profile; matching counter-clockwise square `LineSeg` profiles on
  identity frames at `z=0` and `z=1` first assert that `capStart` reverses `p0`'s
  triangulation while `capEnd` retains `p1`'s, with every cap-boundary edge
  opposite its wall neighbor; that fixture then normalizes all wall and cap
  windings to a positive signed tetrahedron sum before mass properties and
  tessellation read the payload; a junction's `Edge.IsConvex` matches its
  hand-computed walked-boundary turn; an outer rim is convex and a hole rim is
  concave. An
  untwisted congruent parallel loft with one outer side deliberately split
  into two collinear `LineSeg`s retains exactly two `Plane` faces per wall
  cell. At the flat split rung, the preceding cell's lower triangle and the
  next cell's upper triangle remain distinct faces. Every retained flat
  diagonal and that rung report `IsConvex() == false`, are selected by
  `Concave()`, and are not selected by `Convex()`. A collapsed
  (coincident-vertex) segment pair → S6's RECORDED arm, asserted on
  `ErrDegenerate`; a same-kind circular pair recorded at ZERO RADIUS on
  exactly ONE side reaches S16 ahead of that arm and is asserted on
  `ErrUnsupported`, while one at zero radius on BOTH sides whose collapsing
  cell consumes two PINNED stations (§5.2) reaches that same arm and that
  same sentinel; a pair whose collapsing cell holds a COMPUTED station on
  either section — two computed stations rounding to the same float64 (§5.1),
  and a cell whose two stations differ in provenance alike — reaches S6's
  COMPUTED arm instead, asserted on `ErrUnsupported` and not merely on being
  refused.
- **Audit**: a deliberately over-twisted correspondence (e.g. an intentional
  wrong `WithLoftAlignment` offset on a non-convex profile) proves a
  crossing → S7, asserted against the specific triangle pair the crossing
  predicate found; two matching square `LineSeg` profiles on parallel planes
  pass when their coplanar cap-triangulation pairs and coplanar wall-diagonal
  pairs are admitted through `triTriCoplanarSharedEdge`. The same pairs remain
  `contactRegion` in `triTriClassify`, proving that the adjacency helper does
  not change mesh-boolean contact classification. Consecutive wall pairs
  classify as the expected shared vertex §5.1's Table C gives them, whether
  that station is recorded or generated. A vertex-sharing wall pair that crosses
  away from that vertex → S7. Two valid square `LineSeg` profiles with `p0` frame
  `U=(1,0,0)`, `V=(0,1,0)`, origin `(0,0,0)` and `p1` frame
  `U=(-1,0,0)`, `V=(0,1,0)`, origin `(1,0,1)`, carrying the same local
  square, make cell 0's two triangles share their recorded diagonal but
  overlap in area; the audit rejects them as S7. A synthetic profile pair
  sized to exceed the fixed pair-test budget → S8, refused before any pair
  result is trusted.
- **Mass properties**: a scaled cube-like loft (two congruent squares,
  parallel planes, no twist) reproduces the closed-form prism volume and all
  three centroid coordinates exactly. The fixture asserts `Volume` is `Exact`
  only when its rational is representable, and `Centroid` is `Exact` only when
  every coordinate is representable. A non-representable volume carries its
  single-rounding bound; a non-representable centroid coordinate makes the
  centroid `Approximate` with a length-radius bound enclosing all three
  coordinate-rounding errors (mirroring spline design Table F's own 293/18 vs
  293/2 worked example). `Area` is `Approximate` whenever any wall triangle
  has nonzero area, and its reported bound ENCLOSES `|held - true|` against a
  high-precision reference, never merely asserted present — over a table of
  slivers at aspect 1e-2, 1e-3 and 1e-6 and at least one large-coordinate
  case, and over a many-triangle wall set so the summation loop is covered
  beside the per-triangle brackets. One fixture drives the summation SCALE to
  `+Inf` while the summed value stays finite and every triangle's own bracket
  has zero width — two congruent rectangles of width 2^-27 and height
  2^27 − 2^-26 on the planes z = 0 and z = 2^996, whose four long wall
  triangles have area exactly `MaxFloat64`/4 and whose four short ones have
  area exactly 2^968 — and asserts the published bound is `+Inf` and the
  reading `Approximate`, never a zero bound over a value that has swallowed
  whole triangles. `Bounds` matches the exact per-vertex componentwise extreme.
- **Same-kind circular pairs (§5.1, §5.2)**: a 90° radius-5 arc paired with
  itself between `z=0` and `z=10` at the calibrated `loftChordFraction` (§14)
  builds, and `|Volume.Value - (π·25/4)·10| <= Volume.Bound` against the
  closed-form quarter-wedge volume; `Document.Extrude` on the same untwisted
  congruent section is an independent oracle whose measurement interval
  overlaps this one. A pair with different radii and sweeps on its two sides
  takes the shared `m` the joint walk-up settles on, and each side's
  CERTIFIED sagitta at that shared `m` is at or below its own target, checked
  over `math/big.Rat`. `sectionDelta` on a loop with two curved pairs of different
  curvature equals the LARGER pair's own measured sagitta, never their sum,
  over `big.Rat`. **A THRESHOLD FIXTURE pins that the published sagitta is
  the certified one and not `chordCount`'s held-float estimate**: an
  `ArcSeg` whose `Start`, `End` and `Center` are chosen so the exact
  Start-to-Center distance falls strictly between two consecutive float64s,
  making the held `math.Hypot` radius round BELOW it, and whose sweep is
  chosen so the same happens to the held `math.Atan2` difference. Its exact
  per-cell sagitta, computed over `math/big.Rat` from the recorded
  coordinates, then EXCEEDS `2*heldRadius*sin²(heldSweep/4m)`. The fixture
  asserts the published `sectionDelta` is at or above that exact sagitta,
  and strictly above the held-float estimate — a build that adopted
  `chordCount`'s value verbatim would publish a bound smaller than the
  displacement it claims to bound and fails here. A paired `CircleSeg` loop builds a full-circle-to-full-circle
  frustum whose volume encloses the closed-form frustum volume at
  `chordCount`'s closed-walk floor of three CYCLIC stations a side, walled by
  the three cells §5.1's closed rule pairs from them — a fourth station, or a
  closing cell between one station and itself, fails it; its alignment offset
  is forced to `0` (§1), and a nonzero `WithLoftAlignment` entry for that
  loop is S4. A fixture sized past the station cap → S15 (`errTooManyChords`),
  asserted to fire BEFORE S8 on a construction that would otherwise reach the
  audit ceiling. A synthetic pair whose two sides are forced to different
  station parities at one cell → S16. A recorded pair whose station enclosure
  the record cannot state (`circularWalkEndBound` answering `+Inf`, §5.2) →
  S14, asserted on the sentinel and on the refusal happening instead of a
  published zero bound. Two further underivable terms get their own fixtures
  over the same rule: a pair whose certified per-cell SAGITTA has no
  derivation (its radius or sweep enclosure answering no, §5.2), and a pair
  whose cap `planeOffsetUpper` or per-cell `arcLenUpper_k` enclosure runs past
  `MaxFloat64` (§5.2), each refuse S14 too, each asserted to refuse rather
  than fall back on a finite estimate, and each asserted to refuse in the S14
  arm §4's gate-order paragraph assigns its term. A DERIVED term that
  saturates on its OWN arithmetic gets a fixture of the same shape: a build
  whose `matchedDelta` composition runs past `float64` while both source rows
  it reads stay finite refuses S14 in the CONSTRUCTION arm, asserted on the
  sentinel and on the arm rather than on the source rows' own finiteness.
  **The chorded volume allowance's four legs are fixtured in `bounds.go`'s own
  internal tests, and this section names those fixtures rather than restating
  their assertions over a built body.**
  `TestChordedBoundaryVolumeAllowComposesAllFourLegs` pins the composition's
  shape; `TestChordedBoundaryVolumeAllowCapLegIsLoadBearing` pins that
  deleting the CAP leg understates a measured gap, and
  `TestCapAreaVolumeAllowIsExactForAPlanarFace`,
  `TestCapAreaVolumeAllowIsZeroAtZeroOffsetOrZeroAreaGap` and
  `TestCapAreaVolumeAllowRefusesOnBrokenClaims` pin that leg's own exact
  planar identity, its two zero cases and its `+Inf` refusal;
  `TestChordedBoundaryVolumeAllowTwistLegIsLoadBearing` pins the twist leg;
  `TestChordedBoundarySeamAllowScalesWithItsThreeOperands` and
  `TestChordedBoundarySeamAllowRefusesOnBrokenClaims` pin the seam leg; and
  `TestChordedBoundaryMomentAllowWidensPastTheHeldCoordEnvelope` pins that the
  moment twin's own radius widens past the held coordinate envelope (§8).
  **Two counterexample fixtures pin why the wall leg's area obligation is
  ABSOLUTE rather than a held area plus an excess.**
  `TestCellChordCurveAreaUpperEnclosesTheFlatTriangleCounterexample` holds a
  cell whose flat-triangle area vanishes while its own ruled patch still
  carries area `1/3`, and
  `TestCellChordCurveAreaUpperEnclosesTheCrossedCellCounterexample` holds a
  crossed quad; a held-area-plus-excess reading encloses neither.
  `TestCellChordCurveAreaUpperRefusesTheSagittaZigzag` and
  `TestArcMatchedDeltaEqualsSagitta` together pin §5.2's parameter-matched
  paragraph: a sagitta is not a matched bound in general, and IS the IDEAL
  chord's matched departure for the circular kinds this design admits. **A
  third fixture pins the step those two do not reach**: on a chorded pair
  whose computed stations carry a positive `stationRound`, the published
  `matchedDelta` is strictly greater than `sectionDelta` and equals
  `absSumUpper(sectionDelta, delta)`,
  and a dense sample of the recorded curve is enclosed at the matching
  parameter by the `matchedDelta`-neighbourhood of the HELD chord while the
  `sectionDelta`-neighbourhood of that same held chord is asserted NOT to be
  the term the legs charge. A build that fed `sectionDelta` to any chorded leg
  fails it.
  `Bounds` widened by its own `Bound`
  (`absSumUpper(delta, sectionDelta)`) CONTAINS a dense sample of both true
  recorded arcs lifted through their planes — a box that did not widen fails
  it. An untrimmed `ArcSeg` pair chorded at exactly `m = 1` — both end
  stations pinned to recorded coordinates (§5.2) — publishes `delta == 0` and
  `bodyGateDiameter`'s unshrunk reading, with `sectionDelta > 0` (§12).
  **A PARTIAL CIRCLE is the companion fixture and asserts the opposite**: a
  certified `Partial` `CircleSeg` fragment recorded over a non-natural range
  (`recordEdge`, `seam.go`), paired with itself and chorded at `m = 1`,
  publishes the `stationRound` `circularWalkEndBound` proves for its own two
  ends — strictly positive on that fixture's recorded geometry, a value its
  own walk bound states and no station kind grants — and therefore a strictly
  positive `delta` under `r3.Identity()`, and `bodyGateDiameter` shrinks its
  reading by `2*delta` — a fixture that read `delta == 0` there would pin the
  false generalisation from the untrimmed-`ArcSeg` case. **That fixture is
  run at a QUARTER-TURN range as well, and that run asserts the value the
  walk bound proves rather than a sign.** The quarter-turn run is what pins
  §5.2's no-exemption rule from the other side: `circularEndpointInterval`
  answers a POINT interval at each of those two ends, and the held pair is
  `circularWalk`'s own `math.Sincos` result, which can land ON that point — a
  `CircleSeg` of radius 1 about `(4, 4)` recorded over `[0.25, 0.5]` publishes
  `stationRound == 0` at both ends, each held coordinate absorbing its own
  `math.Sincos` residue and equalling the denoted rational bit for bit. What
  the kind never grants is the ZERO, not the reading: the run asserts the
  build takes that value from the walk bound and treats the station as
  GENERATED throughout (§5.1's Table C), and a run that read such a station as
  PINNED would pin the exemption §5.2 refuses to grant. A TRIMMED `ArcSeg`
  pair at `m = 1` asserts its own walk bound the same way. A
  mixed line-to-arc pair, and an arc-to-fit-spline pair, still refuse S3.
  Replay of a recorded circular-pair step reproduces the same station count,
  the same triangle set, and bit-identical measurements.
- **Downstream**: D1's `Bound` is the payload's own facet departure
  `absSumUpper(delta, sectionDelta, maxTwistOffsetUpper)` (§5.2), asserted
  against a CHORDED fixture whose cells TWIST so that a two-term reading
  fails it (a `LineSeg`-only twisted fixture charges no twist term, §5.2, so
  it decides nothing there), and `Bounds.Bound` on that same body is
  asserted to stay the two-term
  `absSumUpper(delta, sectionDelta)` — exactly zero for an admitted unplaced
  `LineSeg`-only loft whose every station §5.2 PINS, positive for a placed
  one, positive for an UNPLACED CHORDED one, and, for an unplaced
  `LineSeg`-only loft holding a TRIMMED `LineSeg` station, equal to that
  station's own `lineWalkEndBound` reading rather than to a positive value
  the kind grants, asserted on a placed fixture, on an unplaced chorded
  fixture, and on a trimmed-`LineSeg` fixture beside the pinned
  `LineSeg`-only one. **That
  pinned `LineSeg`-only fixture is itself TWISTED** — its wall quads' four
  held corners are not parallelograms, so a twist term read over EVERY wall cell would be
  positive there — **and it is asserted to publish a facet departure of
  exactly zero**, which is what pins the chorded scoping §5.2's own row
  states. A D2 boolean between an
  unplaced `LineSeg`-only loft whose every station §5.2 PINS and a prism
  succeeds through the existing all-planar zero-bound path, while one
  between a prism and either a PLACED loft or an unplaced CHORDED one succeeds with an `Approximate` volume whose
  bound composes that same facet departure; a box-disjoint loft/loft pair
  proves only its disjoint-interior interference relation under D3, and its
  `Verify` report is `Sound` only with no other undecided required or
  requested check; a box-disjoint pair with `WithClearances` reads `Suspect`
  until the analytic kernel adds loft; each requested D5 survey reads
  `Suspect`, never absent or a silently wrong number.
- **Recipe/replay**: round-trip a `LoftOpts` payload including a non-zero
  `Alignment` through a version-2 envelope; a missing `Profile2`/`Plane2` on
  the wire rejects; a version-1-only decoder rejects that complete version-2
  envelope before it dispatches `"loft"`; replay reproduces the same triangles,
  roles, and measurements as the immediate call; a failed call leaves the
  document and recipe unchanged.
- **Placement (§12 PR 2a)**: `r3.Identity().Apply(v)` is bit-identical for a
  subnormal, `1e308` and `1/3`, and moves a `-0.0` coordinate by exactly
  zero while not preserving its sign bit (identity's own zero-cross terms
  sum `-0.0 + 0.0`, which IEEE 754 rounds to `+0.0`) — the identity fast
  path's premise is the zero DISPLACEMENT, not the sign of zero, and both
  halves are pinned. A test that a stale profile refuses a fresh
  `Document.Loft` while `Duplicate`/`PlacedCopy`/`Placed` of the
  already-built body each succeed pins §4's original-call-only S9.
  `Duplicate` of a fresh loft reproduces bit-identical vertices, Exact
  16000 mm³ volume, Exact centroid, Exact bounds, leaves the source live, and
  gives the document two bodies. Rotating the 16000 mm³ square-square loft 37
  degrees about X is the regression this PR exists for: assert
  `|Volume.Value - 16000| <= Volume.Bound` (the naive implementation misses by
  1.819e-12 against a 7.203e-13 bound), the centroid against the closed-form
  rotated centroid within its bound, every `Vertex.Position().Bound >= 0`, and
  `Bounds.Exactness == Approximate`. A `PlacedCopy` by `(100,0,0)` moves
  `Bounds` and `Centroid` by exactly 100 in x, keeps the volume value,
  publishes a positive bound enclosing 16000, and leaves the source live. A
  reflection (a herringbone mirror) produces positive volume, outward face
  normals (spot-checked against `Face.NormalAt` on one wall), unchanged
  face/edge/vertex counts, and a centroid that mirrors the source's within
  bound. Ten successive `PlacedCopy` motions keep the volume bound within a
  small constant factor of one placement's own, proving the re-lift-from-record
  path charges `delta` once rather than accumulating it. The placed body's
  faces carry `side(i,j,k)`/`capStart`/`capEnd` under the NEW `StepRef`;
  `FaceCreatedBy(CapStart(b2))` selects exactly one face; manifoldness and
  per-edge `IsConvex` match the source. The sum of `Face.Area()` equals
  `Body.Area().Value` within the summed bounds, catching a per-face bound that
  forgot the perturbation term. `Placed` retires the receiver; `Duplicate`/
  `PlacedCopy` do not; a refused placement (an invalid transform, an S12
  fixture, and an S13 fixture whose composed translation carries a far-plane
  section past `MaxFloat64`) leaves the recipe and document untouched, and the
  S13 fixture returns `ErrUnsupported` rather than panicking inside the exact
  lift. A canceled
  `PlacedContext` returns `ctx.Err()` with the receiver live and the recipe
  unchanged. A placed loft is `Sound` at the default tolerance under `Verify`;
  two lofts placed apart read box-disjoint `Sound`; an internal test asserts
  `bodyGateDiameter` shrinks by `2*delta`, and a second asserts that shrink is
  rounded OUTWARD — `2*delta` is exact (a power-of-two scaling), so the
  difference is the arm's one rounding, and round-to-nearest lands it above the
  exact `d - 2*delta` at some placement translations and below at others, so the
  test judges the reported reference against the exact difference over
  `math/big.Rat` at several translations rather than at one witness.
  `perturbedTriangleAreaAllow`
  encloses the area change over a brute-force sweep of perturbed vertices at
  `delta`, at aspect ratios 1, 1e-3 and 1e-6. With `delta == 0` every
  published measurement is bit-identical to PR 1's, and so is the gate
  reference: a zero-`delta` loft's `bodyGateDiameter` must equal the shared
  reader's own answer over its vertex set under `==`, since neither the
  subtraction nor the outward rounding above may run on a zero allowance.

## 14. Open questions

**The chord-target constant.** `loftChordFraction = 3.76491e-05` (§5.1),
used as `chordTarget = loftChordFraction * max(profileCoordinateUpper(p0),
profileCoordinateUpper(p1))`. The number comes from two calibration fixtures
measured at a FORCED `m = 64`, each with its own implied fraction at that
count: the arc wedge, this document's reference fixture — a 90° radius-5
quarter-arc lofted between `z=0` and `z=10` — reaches a `sectionDelta` of
3.76491e-4 there, an implied fraction of 3.76491e-05, and the matching
fit-spline wedge reaches a `sectionDelta` of
4.73591e-4, an implied fraction of 6.69759e-05. **The shipped constant is the
finer of those two implied fractions**, the arc wedge's 3.76491e-05, so one
constant serves both fixtures rather than each kind carrying its own.

**The binding reading is `Volume`, not `Centroid`, and the two measured
margins belong to that forced `m = 64` run.** There, `Verify` at the default
`1e-3` tolerance clears with a 2.39x margin on the arc wedge (a gate ratio of
4.18e-4) and with a 1.90x margin on the fit-spline wedge. The arc wedge's
`m = 64` is a FORCED count, not the count the shipped constant yields on the
production path: `chordCount` (§5.1) proves its own bound through
`chordSagitta`'s `r·sweep²/(8n²)`, which is conservative against the exact
`2r·sin²(Δθ/4)` the calibration measured, so at `m = 64` it reads 3.764955e-4
against a 3.764910e-4 target — over by 4.53e-9 — and steps to `m = 65`, where
it reads 3.650002e-4 and clears. The production `sectionDelta` at 65 is
therefore TIGHTER than the calibration's own at 64, so the 2.39x is a
conservative LOWER bound on the margin the shipped constant reaches on this
fixture, never an overstatement of it. The fit-spline wedge's 1.90x
is NOT: 6.69759e-05 is a coarser target than the shipped constant, so a
fit-spline wedge chorded to meet 3.76491e-05 takes more than 64 chord cells,
and the margin it reaches there is a reading this document does not state.

**The constant does not clear a 4x margin inside the wall-clock budget, and
this design accepts that rather than widen either.** A 4x margin needs 128
stations, whose build measures about 4.3 seconds; the fixture wall-clock
budget §13 states caps that build at 2 seconds, which the 64-station build
meets at about 1.4 seconds. What ships is the chord-target fraction that
64-station run implies — `loftChordFraction`, a dimensionless number and not
a station count, whose own count is whatever each build's walk-up settles on
(65 on this fixture, above). An
arc loft at an aspect ratio more extreme than the reference fixture, judged
at a tolerance tighter than the default, can read `Suspect` rather than
`Sound` — a correct, non-silent outcome under a tight enough tolerance, not
a wrong answer, and the reject-only discipline `CLAUDE.md` requires: no
two-pass rebuild reads its own published measurement and rebuilds to chase a
tighter margin, since that would make the topology a function of a published
float and a new determinism obligation for replay (§10).

**`chordedBoundaryVolumeAllow` and `chordedBoundaryMomentAllow` (§5.2,
`bounds.go`) are TWINNED helpers with their own derivations, never an
extension of `sweptVolumeAllow`/`sweptMomentAllow`.** The two pairs speak for
different mechanisms: `sweptVolumeAllow`/`sweptMomentAllow` bound a MESH
whose vertices moved under a rigid motion; `chordedBoundaryVolumeAllow`/
`chordedBoundaryMomentAllow` bound a boundary REPLACED by a nearby non-mesh
surface — the recorded curve a chord chain approximates. **The chorded pair
takes FOUR legs and never a `(sectionDelta, areaUpper)` pair** (§8, §8.1),
and `perturbedAreaUpper` discharges nothing for it: that helper's per-facet
argument is about vertices DISPLACED by `delta`, which is the swept pair's own
mechanism and stays its discharge alone. The wall leg's area obligation is
`wallAreaUpper`, the SUM over wall cells of `cellChordCurveAreaUpper` — an
ABSOLUTE bound on the area of every surface the chord-to-curve homotopy
visits, never a held facet area plus a per-cell excess, because there is no
fixed held quantity an excess could add to: a cell can hold almost no
triangle area while its own ruled patch already carries substantial area
(`TestCellChordCurveAreaUpperEnclosesTheFlatTriangleCounterexample`), and
containment inside a thin neighbourhood of the chord facets bounds no
surface's area at all, since a surface can carry unbounded area inside an
arbitrarily thin slab. The moment twin adds `maxTwistOffsetUpper` and
`coordUpper` on top of those four legs, and widens its own radius by both
(§8).

**The wall leg's displacement argument is a PARAMETER-MATCHED departure, and
`matchedDelta` is the term that states it.** §5.2's parameter-matched
paragraph states the obligation and its two-step proof — the sagitta is the
ideal chord's own matched departure for the kinds this design admits, and the
held chord adds `delta` on top of it; a caller that cannot prove the
composition passes `+Inf`, and the sagitta never stands in.

**A chorded wall's `Area` difference has no proven owner, and §12 PR 3 waits
on one.** `cellChordCurveAreaUpper` bounds the true surface's area from ABOVE,
which is what the volume allowance's flux identity needs; the `Area` reading
needs `|Area_held − Area_true|` over a wall cell, a different quantity, and no
shipped helper bounds it. §8 therefore refuses rather than substitute the one
that is available. This is an open question about a DERIVATION and not about a
design variable: the reading's shape, its trigger and its consumers are all
settled above, and what is missing is a helper in `bounds.go` carrying a
written proof of that difference.

**`loftStationCap`'s value is this document's one open variable.** §5.1
states the rule the cap obeys and everything an implementation needs to
decide S15 from the record — the per-segment share, the `mMax` comparison,
and the checked arithmetic — but not the number itself. The number is fixed
by the increment that lands the station generator (§12 PR 3), inside two
constraints §5.1 already states: a build whose `Σstations` reaches the cap
must assemble an `F` whose `F*(F-1)/2` is strictly below
`maxFacetPairTestsPerCall` (§6), and the cap must leave room for every
fixture §13 requires. Nothing else in this document reads the number: the
reference fixture's FORCED 64 stations, and every other station count named
here, are stated against the chord target above rather than against the cap.

Every other design variable this document depends on is resolved above. The
reach items in §12's PR 4 row are future work, not open questions of this
design: N-section and guide-rail/centerline lofts, and a loft case in the
analytic clearance kernel are each named and deferred with a one-line
reason, not left undecided.

## 15. Companion edits

This document changes `docs/api-design.md`'s current decision, so it makes
the following edits, in `docs/api-design.md` and `CLAUDE.md`, alongside
landing this file:

- **§8's vocabulary line**: `Sweep and Loft are deferred.` becomes `Sweep is
  deferred.`, and `Loft` joins the vocabulary list.
- **§13's non-goals list**: `sweep and loft` becomes `sweep` — loft is no
  longer a v1 non-goal; it is design-only until its PRs land (the same
  standing every other not-yet-shipped capability in this package already
  has, per `CLAUDE.md`'s own opening paragraph).
- **`CLAUDE.md`'s Layout table** gains a row for this document.
- **§6.2's `Step.Op` comment** lists `Loft` among the ops.
- **§6.2's `Step.Profile` and `Step.Plane` comments** name Loft beside
  Extrude and Revolve, and say the recorded section is the **from** one (§10).
- **§6.2's sealed `StepOpts` block** carries `LoftOpts` — the **to**
  section's `ProfileRecord`, its `PlaneRecord`, and the `Alignment` offsets
  (§10) — beside `ShellOpts`. `StepOpts` is a closed variant set decad owns
  (core §6.2), so a variant this document requires belongs in that block.

**`doc.go`'s support-and-refusal map is a dependent of §12's PR rows.** Its
Loft entries state what this evaluator builds and what it refuses, so §12's
increments — the same-kind circular correspondence of §12 PR 3 among them —
are what change them, and the map is correct once the increment that changes
it has landed. §12's rows own what each increment admits; that map restates
none of it, and this document assigns no PR for the edit itself.

§5.1's chord chain also gives a loft under an identity motion a positive
boundary displacement (§5.2, §8). Five companion documents state a zero bound
or an exact boundary for a loft, so each is corrected to condition that on
BOTH `delta` and `sectionDelta` being zero — a `LineSeg`-only loft whose
every station §5.2 PINS and whose accumulated motion is `r3.Identity()`, the
two tests §5.2's `stationRound` and `placeAllow` rows own — rather than on
`delta` alone or on the body never having been placed.
**Which term each site names depends on what it reads.** A site that reads a
per-FACET departure names the payload's
facet departure `absSumUpper(delta, sectionDelta, maxTwistOffsetUpper)`
(§5.2), because a twisted CHORDED cell's held triangle pair is not the
ruled patch through its own four corners — a term a `LineSeg`-only build
charges nothing to (§5.2); a site that reads the payload's `Bounds` names
the two-term `absSumUpper(delta, sectionDelta)`, for the reason §5.2's own row
gives; and a site that composes a chorded VOLUME allowance names
`chordedBoundaryVolumeAllow`'s four legs (§8.1), never a `(sectionDelta,
areaUpper)` pair:

- **`docs/tessellation-design.md`**: the `loftPayload` row of §2's proof-term
  table and the exact-restatement text under it, §13's T6 row, and §14's
  `loftPayload` test obligation. Its `sourceBound(face)` and `Bound` are the
  facet departure, its `volSymDiff` composes the four-leg chorded allowance,
  and its `areaSlack` has no chorded term to state while a chorded `Area`
  itself has no owner (§8). A chorded loft is not an all-planar
  zero-bound mesh-boolean operand however it is placed.
- **`docs/clearance-design.md`**: §2's `loftPayload` sentence, which decides a
  box-disjoint partition on the `Bounds` reading and names the wider facet
  departure for the distance a future adapter would measure against a held
  facet.
- **`docs/payload-verification-design.md`**: §2's `loftPayload` bullet, which
  calls the held boundary exact only where all three terms are zero.
- **`docs/evaluator-design.md`**: §3's per-vertex bound rule, whose swept-
  vertex clause names a section displacement and a sweep-level one and has no
  loft arm, and whose loft clause names the placement term alone. It gains
  the chorded station's own `stationRound` (§5.2) beside that placement term,
  and names §5.2's table as where a loft's terms and their conditions live.
  §3 also states the `side(i,j,k)` provenance-role mechanism, and points at
  §7 for what a loft's `j` indexes rather than glossing it as a segment
  index.
- **`docs/verification-design.md`**: §3's `bodyGateDiameter` prose, which
  earns the vertex maximum as the true diameter for a `LineSeg`-only loft at
  `delta` zero, as a lower bound on it for a chorded one whose every station
  is pinned, and as the `2*delta`-shrunk reading wherever `delta` is
  positive — a condition on `delta` and never on the body having been placed
  (§12).
