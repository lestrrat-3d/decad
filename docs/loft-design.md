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
| **S6** | a wall or cap triangle whose three recorded points collapse (coincident vertices, zero area) | no — the modification consumed the region, the same existence answer modify §5 test 1 gives an inside-out loop | `ErrDegenerate` | yes, §4 |
| **S7** | either of two arms: the STRUCTURAL arm — a same-kind `CircleSeg` pair whose two recorded `CCW` flags disagree (P5), decided from the two records before construction — or the AUDIT arm, where the crossing audit (§6) finds contact other than the pair's recorded expected contact | no — a self-intersecting or self-touching shell bounds no solid, and an opposite-sense circular correspondence walls each side against the other's reversed walk, which is that same crossing | `ErrDegenerate` | yes, §6 |
| **S8** | the crossing audit exhausts its fixed work budget (§6, §10) before every pair is decided, over the triangle count `F = 2·Σstations + cap triangles` (§7) a chorded pair contributes rather than `2n` | this evaluator cannot tell | `ErrUnsupported` | no, §6 — a resource ceiling, not a shape rule |
| **S9** | either profile fails a seam gate (§2): foreign, stale, invalid, or an unrecordable `Partial` fragment | seam design's own answer, per profile | `ErrForeignProfile` / `ErrStaleProfile` / `ErrInvalidProfile` / `ErrUnrecordableProfile` | seam design's own answer, per gate; this document adds no permanence of its own (§2) |
| **S10** | a nil `*sketch.Sketch` or `*sketch.Profile` argument | no call at all | `ErrDegenerate` | yes, §2 |
| **S11** | a nil or foreign `LoftOption` value, including a foreign type that embeds the sealed marker | no well-defined decad operation can invoke an unowned callback | `ErrDegenerate` | yes, §2 |
| **S12** | ANY build — placed (`Placed`/`Duplicate`/`PlacedCopy`, §12 PR 2a), chorded (§5.1), or both — whose COMBINED proven volume allowance (§8) is not smaller than the held volume | yes — the body itself is sound; only its centroid's proven quotient bound has no positive denominator left to divide by | `ErrUnsupported` | no — a precision ceiling on this evaluator's centroid bound, not a shape rule |
| **S13** | a build whose lifted-and-placed coordinate, whose computed station coordinate (§5.1), or whose orientation anchor (§5), runs past the representable float64 range | yes — every input is finite (both records' coordinates, the plane origins, and a transform `r3` itself validated), and only decad's own float evaluation of the lift or the station computation overflows; a placed body is the rigid image of one this evaluator already built | `ErrUnsupported` | no — a range ceiling on this evaluator's float64 vertex table, not a shape rule |
| **S14** | a chorded circular pair whose per-station displacement this evaluator cannot derive | yes — the body exists and the chord set is built; only the station's own proven displacement has no derivation from this record | `ErrUnsupported` | no — a derivation gap in this evaluator's circular walk bound, not a shape rule |
| **S15** | a paired segment whose chord target (§5.1) is not met inside the fixed station cap | yes — the ruled surface exists; this evaluator cannot chord it inside its own ceiling | `ErrUnsupported` (`errTooManyChords`, spline R8) | no — a resource ceiling, not a shape rule |
| **S16** | a chord cell whose two stations coincide on exactly one of the two sections | yes — a collapsed piece is a recordable curve piece; only the uniform two-faces-per-cell topology (§5) has no case for it | `ErrUnsupported` | no — an evaluator topology limit |

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

**S6 is also reachable from a same-kind circular pair.** A zero-sweep arc, or
two computed stations that round to the same float64 (§5.1), collapses a wall
or cap triangle exactly as a degenerate `LineSeg` pair already can. The rule
(S6) is unchanged; a circular pair only widens the set of recorded shapes
that can reach it.

**S5 compares geometric planes, not `PlaneRecord` fields.** Its normal is
`U × V`; it refuses when the two normals are parallel and the displacement
between their origins lies in that plane. It returns `ErrDegenerate` before
construction for every coplanar section pair, even when their authenticated
records use distinct origins or bases.

**Gate order**, the same "ask what could be asked" discipline modify §4
states: pre-gates first (S10 nil check, S11 concrete option ownership without
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
coincidence, and, for a same-kind circular pair, S15's station-cap decision —
the shared station count `m` and the `mMax` it is compared against (§5.1)
are each a closed-form function of the two records alone, so it is decided
here with the rest — all decidable without building a single triangle), then
construction (§5), whose own first act is S13's coordinate-range gate on the
anchor, on every computed station, and on every placed vertex as it is
emitted, together with S14's per-station displacement-derivation gate for a
circular pair, then S16's one-sided-collapsed-cell gate as stations are
paired into chord cells, then the per-triangle existence gate S6, then S7's
AUDIT arm with S8 beside it (§6) — the most expensive step, run last, over
triangles already proven individually non-degenerate.

**A placement (`Placed`/`Duplicate`/`PlacedCopy`, §12 PR 2a) re-runs every
record-only gate — S1, S2, S3, S4's payload-shape half, S5, S6, S7, S8, S13,
S14, S15, S16 — plus S12, never a reduced set of them.** The evaluator
re-lifts both records under the composed motion and rebuilds from scratch
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
through that same `delta` (§5.2) and `sectionDelta` through
`chordedBoundaryVolumeAllow` (§8); a build that is both contributes all
three. A same-kind circular pair's `stationRound` and `sectionDelta` are each
positive under `r3.Identity()`, so an unplaced chorded body reaches S12
through both of them.

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
| lower | `V_j, V_{j+1}, W_{j+1}` | the full `p0` segment (shared with `capStart`'s boundary) | `side(i,j,0)` |
| upper | `V_j, W_{j+1}, W_j` | the full `p1` segment, reversed (shared with `capEnd`'s boundary) | `side(i,j,1)` |

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

Evaluator §3 owns the `side(i,j,k)` grammar. Here `i` is the loop index
(`0` for `Outer`, `1+h` for `Holes[h]`), matching Table P's own indexing.

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
already treats as truth. A same-kind circular pair's interior stations
(§5.1) are the one exception: a station beyond a segment's own two recorded
endpoints is a COMPUTED point, not one the record states directly, so it
takes no zero-bound grant here — its own displacement from the point the
record denotes is `stationRound`, folded into `delta` rather than treated as
exact (§5.2).

**This claim holds for the identity transform.** A placed, duplicated, or
placed-copied loft body (§7, §12 PR 2a) carries the payload's own proven
displacement term `delta`: zero exactly when the accumulated placement is
`r3.Identity()` (an exact struct comparison, never a tolerance), and
otherwise `bounds.go`'s `rigidRoundAllow`, read at the pre-transform lifted
point's own magnitude and the composed translation's magnitude — never at
the result's, since that is where a general rigid motion's rounding is
actually committed. A placed body's every vertex, every edge length, every
face area, and each of the four body measurements carries that same
`delta`.

**Edges get no new role mechanism.** Selector.go's existing rule already
covers loft: "`CreatedBy` matches an edge through its adjacent faces'
`Origins()` — an edge carries no roles of its own." A loft's three new edge
families need none:

- a **cap-boundary edge** is the recorded `LineSeg` itself — no new edge at
  all, shared between the cap face and its one incident wall triangle;
- a **rung edge** `V_k -> W_k` (one per paired vertex) — shared by the two
  wall triangles that meet at vertex `k` from opposite sides;
- a **diagonal edge** `V_j -> W_{j+1}` (one per paired segment) — shared by
  that segment's own lower and upper triangle.

Every edge of the result therefore bounds exactly two faces (one cap +
one wall triangle for a boundary edge; two wall triangles for a rung or a
diagonal), so the payload is manifold and watertight **by construction**,
the same structural claim modify §2 makes for a rewritten prism section —
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
vertices — exact on an unplaced body, and within the payload's own `delta` on
a placed one), Exact by construction per core §6.1's surface-parameter
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
grammar (§5) is unchanged**: for a chorded pair, `j` ranges over that loop's
flattened chord-cell sequence rather than over one entry per recorded
segment, and each cell still resolves to exactly the `side(i,j,0)` /
`side(i,j,1)` pair §5's table states.

**`m` counts CHORD CELLS, and the station points are one more.** `m` is
`chordCount`'s own CHORD count, so a pair chorded at `m` contributes exactly
`m` chord cells to its loop and holds `m + 1` station POINTS on each side of
the pair — that side's two recorded endpoints plus `m - 1` interior stations.
A pair at `m = 1` therefore has no interior station at all and walls with a
single cell, which is the case §8 and §12 both single out. Wherever this
document calls `m` a station count it names that same chord-cell count, never
the count of station points.

**Station generation reuses the existing circular-walk primitives, never new
trigonometry.** Each side's own station count and achieved sagitta come from
`tessellate.go`'s `chordCount`, the identical closed-form walk-up
`Document.Extrude`'s own circular side walls already use: it recomputes the
sagitta `2r·sin²(Δθ/4)` at each candidate station count and increments until
the recomputed value is at or below the target, so the depth is measured at
every step rather than sized from a rate.

**Shared station count, one target.** For a paired segment, each side's own
minimum station count (`m0`, `m1`) is computed independently by `chordCount`
at the shared target below, and the pair uses `m = max(m0, m1)` stations on
BOTH sides. `chordCount`'s achieved sagitta strictly decreases as the station
count increases, so the side whose own minimum is smaller than `m` still
meets its own target at the shared `m` — every side's achieved sagitta at
`m` is at or below the target that side alone would have needed. This is
what keeps the two sides' station counts identical without loosening either
side's own bound.

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
section already owns: the assembled triangle count is
`F = 2·Σstations + cap triangles` (§7), and §6 refuses under S8 unless
`F*(F-1)/2` is at or below `maxFacetPairTestsPerCall`. `loftStationCap` is
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

stations. A circular pair whose own `m = max(m0, m1)` exceeds `mMax` is
Table S row S15 (`ErrUnsupported`, `errTooManyChords` — spline R8, the
identical sentinel `chordCount` itself already returns when its own walk-up
would exceed `maxChordsPerWalk`), and the refusal names that segment, since
the share it exceeded is that segment's own. A build with no circular pair
(`C = 0`) never consults the cap at all: its `Σstations` is `Σn_i` exactly
(§7), the count the record itself states, so an all-`LineSeg` build's only
resource refusal is S8.

**Deciding S15 from the record.** `m0`, `m1` and `mMax` are each a
closed-form function of the two `ProfileRecord`s — the two sides' radii and
sweeps, the chord target above, `P` and `C` — so S15 is decided with the
record-only gates (§4's gate-order paragraph) and a build that would exceed
the cap is refused before a single station is built. Every product and sum
in the `mMax` comparison and in §6's own `F*(F-1)/2` preflight is evaluated
with checked arithmetic and refuses on overflow rather than wrapping, the
identical preflight-before-allocation discipline §6 states for the pair-test
ceiling itself.

### 5.2 `sectionDelta` — the in-plane chord displacement

**`sectionDelta` bounds the in-section-plane displacement of a BUILT CHORD
from the recorded curve it chords, as a MAXIMUM over chord cells, never a
sum.** For each chord cell, the achieved sagitta on side 0 and on side 1
(§5.1) are each an in-plane displacement of that cell's chord from the arc or
circle it approximates; `sectionDelta` is the largest of those per-cell,
per-side values over the whole loft. It is a maximum rather than a sum
because a boundary point lies in exactly one cell — the identical reasoning
`docs/prism-boolean-design.md` §7 and `CLAUDE.md`'s "Section displacement"
cross-cutting note already state for `prismPayload.sectionDelta`, which this
field mirrors by name and by contract. `sectionDelta` is zero exactly when
every paired segment is a `LineSeg`.

**`sectionDelta` and `delta` bound different objects, and neither ever
stands in for the other.** `delta` (§5) bounds the displacement of a HELD
VERTEX from the exact point the record denotes for it; `sectionDelta` bounds
the displacement of a BUILT CHORD, in the section plane, from the recorded
curve it approximates — a boundary-surface quantity, not a point motion. A
reading that both terms displace — `Bounds`' box, or `Volume`/`Centroid` on a
body that is both placed and chorded (§8) — sums the two into ITS OWN bound;
the two source terms are never added into one another or substituted for
each other anywhere upstream of that composition.

**`delta` gains exactly one new term, `stationRound`** — the world-space
displacement between a HELD circular-walk station and the CERTIFIED INTERVAL
enclosing the point the record denotes for it. The walk that produces a
station evaluates `math.Sincos` on a computed angle, so neither the trig nor
its argument is a quantity the walk itself can enclose while holding floats
alone; the bound comes from the recorded curve instead.
`extrude.go`'s `circularWalkEndBound` builds the enclosure of the denoted
point through `circularEndpointInterval` and reports each plane-local
component's own gap from that enclosure, and `bounds.go`'s
`walkEndBoundAllow` carries the wider component through the payload's
orthonormal frame into one 3D world-space displacement. `stationRound` is
those per-station allowances accumulated over the build. **A station whose
enclosure the recorded data cannot state answers `+Inf`, and the build
refuses `ErrUnsupported` (Table S row S14): a `+Inf` is a refusal and never a
published zero.**

So `delta = absSumUpper(stationRound, placeAllow)`, where `placeAllow`
is the existing placement-rounding term this document already names (§5,
"This claim holds for the identity transform"). A `LineSeg`-only pairing has
every station at a recorded endpoint, so its `stationRound` is exactly zero
and an unplaced `LineSeg`-only loft's `delta` is still exactly `0`. A
same-kind circular pair chorded at more than one station (`m > 1`) has a
positive `stationRound` for its interior stations, so `delta` is positive
even at `r3.Identity()` — unlike `sectionDelta`, which composes from the
recorded curves alone and carries no placement term.

## 6. The build-time simplicity / crossing audit

**A ruled wall can cross another wall the two 2D profiles alone would never
reveal** — extreme twist between the two sections is exactly the shape the
target case (a helical tooth) invites. decad never builds an unproven
solid (modify §1), so this is a build-time gate, not a `Verify` question.

The audit tests every pair among the `2n` wall triangles and the two
triangulated caps (`triangulate.go`'s existing ear-clipping triangulation of
each polygon-with-holes cap). A recorded shared edge or vertex does not prove
that it is the pair's only contact. **Adjacency states the expected contact;
every pair is classified against that expectation:**

- **A pair sharing an EDGE is admitted only when the edge-adjacency check
  reports the exact recorded common edge as its whole shared segment.** This
  applies to a rung, a diagonal, a cap-boundary edge, and an internal edge of
  one cap's own triangulation. The matching segment proves the triangle
  interiors are disjoint; a point, an extra segment, a 2-D region, or a
  crossing refuses.
- **A pair sharing exactly one VERTEX and no edge is admitted only when its
  sole contact is that recorded vertex.** A point elsewhere, a shared segment,
  a 2-D region, or a transversal crossing refuses. Vertex-sharing pairs need
  this check because they can cross away from their recorded vertex.

The vertex rule is what every consecutive wall pair needs: the lower
triangles of paired segments `j` and `j+1` share only `V_{j+1}`, and their
upper triangles share only `W_{j+1}`, so each pair must pass the recorded-
vertex check. `triangulate.go` produces an interior-disjoint conforming
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
recorded edge. It projects them exactly onto their common plane and accepts
only when the recorded endpoints are an edge of both triangles and the two
opposite vertices lie strictly on opposite sides of that edge's supporting
line. Those conditions prove that the closed-triangle intersection is exactly
the recorded segment. Every other result refuses under S7. The helper is
audit-only and MUST NOT change `triTriClassify` or mesh-boolean contact
classification.

- **empty** (disjoint) → admitted only for a pair with no recorded shared
  edge or vertex;
- **the exact recorded common-edge segment** → admitted only for a pair that
  records that edge;
- **a point contact at the pair's own recorded shared vertex** → admitted
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
decided before construction, is what keeps the CHORDING from carrying `F`
past this audit's own `F*(F-1)/2` ceiling: the cap is the soft limit, S8
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
`j`, in `side(i,j,k)`, indexes that loop's flattened chord-cell sequence
(§5.1) — one entry per `LineSeg` pair, `m` entries per curved pair.

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
order") and the same pairing, and re-running §4's record-only Table S gates
and §6's crossing audit on the rounded vertex set — the set §4 names, which
is every gate but S9/S10/S11 and S4's arity half. §5's whole-shell orientation step
re-decides the sign from the placed triangle set on its own, so a mirror
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
region integral read directly off the record.** For a `LineSeg`-only loop the
assembled cap polygon is the region boundary itself, so the shoelace rational
equals `moments.go`'s own region rational there — no new 2D integration for
that case. For a same-kind circular pair the assembled cap polygon is
instead the chord chain §5.1 built: `addCircular` (`moments.go`) calls
`dropExact()` unconditionally, so an arc-bearing `ProfileRecord`'s own region
integral is never an exact rational, while the ASSEMBLED chord polygon's
vertices are the same held float64 points the triangulation already holds,
taken exactly as `math/big.Rat` — the identical "take the floats exactly"
lift the wall sum already uses. Reading the cap term from the built polygon
rather than the record's region integral is what keeps `Volume`/`Centroid`
exact-rational for a chorded loft: the two caps and the chord-chain wall
triangles then integrate over the SAME assembled boundary, with no
region-versus-chord mismatch at the cap seam.

**`Volume` is `Exact` exactly when its published rational is representable in
the `units.Value` magnitude it carries, AND the payload's displacement
`delta` is zero, AND its section displacement `sectionDelta` (§5.2) is zero —
never unconditionally.** This is spline design §3's Tier A rule, verbatim:
"the reported bound is a SINGLE rounding of that rational into that
magnitude, and it is zero — hence `Exact` — exactly when the rational is
representable in the magnitude the value ACTUALLY CARRIES." A loft's volume
earns that ceiling for the same reason a Tier A free-form region's area does:
the integral is exactly rational; only its final publication rounds. A
placed body's volume (§12 PR 2a) composes `bounds.go`'s
`sweptVolumeAllow(delta, areaUpper)` on top of that single rounding, so
`delta` alone is enough to make the reading `Approximate` however exactly
the placement's own rotation or reflection is representable. A chorded
(same-kind circular) body's volume additionally composes
`chordedBoundaryVolumeAllow(sectionDelta, areaUpper)` (`bounds.go`) — a
`sweptVolumeAllow` twin over the chord-to-curve homotopy rather than the
placement's rigid one — so `sectionDelta` alone is enough to make the
reading `Approximate` even at `delta == 0` (the `m = 1` case, §12). A body
that is both placed and chorded composes both terms, since each bounds a
displacement committed at an independent stage of the construction — the
section chording, then the rigid placement.

**`Centroid` publishes three exact rational coordinates as a
`VecMeasurement`, not a `units.Value`.** Round each coordinate once into the
returned `r3.Vec`. Its `Bound` is the length radius enclosing all three
coordinate-rounding errors, and it is `Exact` only when every coordinate has
zero rounding error AND the payload's displacement `delta` and section
displacement `sectionDelta` are also zero. This is the existing `moments.go`
centroid publication pattern, extended from the plane-local two-coordinate
result to this 3D triangulated boundary. A placed body (§12 PR 2a) widens
each coordinate's bound by the same quotient composition `moments.go`'s
`boundedQuotient` states, using `sweptVolumeAllow` as the denominator's own
allowance and `sweptMomentAllow` as the numerator's; a chorded body widens it
again by the matching `chordedBoundaryVolumeAllow` / `chordedBoundaryMomentAllow`
pair (`bounds.go`), each doing for `sectionDelta` what the swept pair does
for `delta`. A build whose COMBINED volume allowance — the placement term,
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
is the two caps' own exact rational area (from `moments.go`, contributing no
bound) plus the wall triangles' proven-bound sum — so the total is
`Approximate` with a proven bound whenever at least one wall triangle has
nonzero area, which increment 1's admitted correspondence always does.

A PLACED body's `Area` (§12 PR 2a) adds one further term to that total: the
per-triangle area allowance `bounds.go`'s `perturbedTriangleAreaAllow` states,
summed over every wall AND cap triangle. The caps need it as much as the walls
do — a cap's contribution is its recorded region's exact rational area, and
under a placement the built cap triangles are within `delta` of what that
rational denotes.

A CHORDED (same-kind circular) body's `Area` adds a different further term,
to the wall sum only. **The quantity that term bounds is an AREA: the area
between the flat chord facet pair this construction actually builds over one
chord cell and the curved ruled surface between the two recorded curve pieces
that cell approximates.** An arc-minus-chord LENGTH excess is not that
quantity and never stands in for it — it carries one length dimension too few
to sit in an `Area` sum at all. `bounds.go`'s `cellRuledExcessUpper` (§14) is
the one owner of the term, publishes it as an outward bound in the direction
`Area` needs, and states it per cell as a length excess multiplied by a
ruling length:

```text
cellRuledExcessUpper = rulingUpper * ((arc0 - L0) + (arc1 - L1))
```

- `arc_k - L_k` is side `k`'s own arc-minus-chord length excess over that
  cell: `r·Δθ − 2r·sin(Δθ/2)` at that side's own recorded radius `r` and the
  cell's own sweep `Δθ` (that side's recorded sweep divided by the shared
  station count `m`, §5.1). It is a closed form of recorded data
  rather than an asymptotic estimate, it is non-negative because a chord is
  the shortest path between its own two endpoints, and it is evaluated with
  outward rounding, so what the term carries is an upper bound and not a
  leading-term approximation to one.
- `rulingUpper` is the stated upper bound on that cell's own RULING LENGTH —
  the distance across the two sections that one ruling of the cell spans. It
  is the larger of the cell's two rung edge lengths `|V_j - W_j|` and
  `|V_{j+1} - W_{j+1}|` (§5's rung edges, both held edges of the built body),
  widened by the two sides' achieved sagittae `s0 + s1` (§5.1). A ruling joins
  the point at parameter `t` on side 0's recorded curve piece to the point at
  the same `t` on side 1's; each of those two points sits within its own
  side's achieved sagitta of the chord point at that `t`, and the
  chord-to-chord distance at any `t` is the norm of a convex combination of
  the cell's two rung vectors, hence at most the larger of their two lengths.

Charging BOTH sides' length excesses against that one ruling bound, rather
than their mean, is deliberate slack in the outward direction. The bound is
per cell and is never fit against a particular curve. The two caps carry no
such term: their own contribution is the assembled chord polygon's exact
rational shoelace area (above), not a bound on a curved region.

**`Bounds` is Exact only when BOTH the payload's displacement `delta` and its
section displacement `sectionDelta` (§5.2) are zero.** Every vertex is
already treated as exact when `delta` is zero (§5); the axis-aligned box is
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
reduces to `delta` unchanged. At `m = 1` (§12) `delta` is zero but
`sectionDelta` is positive, so `Bounds.Bound` is positive and
`Bounds.Exactness` is `Approximate` even though every held vertex is a
recorded endpoint — the box still widens, because the box is about the TRUE
recorded curve between those endpoints, not about the vertices alone.

**Vertex position, edge length, and face area follow the standing rules
already governing every other analytic payload, each composed with the
payload's own displacement `delta`.** On an unplaced body (`delta` zero) a
position is Exact by construction (§5), and a straight edge's length and a
triangle's own `Area()` need a square root and are `Approximate` with a
proven bound, Exact only when that particular evaluation happens to be
exactly representable — the same standard Extrude's own `LineSeg` walls and
edges already carry. A placed body's `delta` is positive (§5, §12 PR 2a) and
all three readings carry it: a vertex position publishes `delta` itself as
its bound, and an edge length and a face area each add a strictly positive
`delta` term to the bound they would otherwise publish, so none of the three
is Exact however exactly its own evaluation happens to come out. This
document introduces no new per-accessor rule beyond what §8 already derives
for the body-level quantities.

## 9. Table D — downstream

| D | Consumer | Reads | Increment-1 status |
|---|---|---|---|
| **D1** | `Tessellate` / `STL` / `OBJ` | the payload | works from the first PR that wires it in (§12 PR 2b), and the returned `Bound` is **the payload's own combined displacement `absSumUpper(delta, sectionDelta)`** (§5.2, §8, §12 PR 2a), not unconditionally zero and never `delta` alone: that sum is zero only for an unplaced `LineSeg`-only loft, whose tessellation is still restatement with a zero bound. Every wall and cap face of a PLACED body is a flat triangle over held vertices that are no longer provably exact, and every wall facet of a CHORDED body chords a recorded curve it departs from by `sectionDelta` (§5.2) whatever the placement, so tessellation restates exactly what the payload holds, both terms included (`triangulate.go`'s existing polygon-with-holes triangulator for the two caps; no chording anywhere) |
| **D2** | the mesh boolean (`Union`/`Cut`/`Intersect`, evaluator §9) | the tessellation | a first-class operand once D1 lands — no new boolean code, a loft body is just another all-planar operand. An unplaced `LineSeg`-only loft — the one case where `delta` and `sectionDelta` are BOTH zero — is admitted through the existing all-planar zero-bound path (`docs/evaluator-design.md` §2 — "the VOLUME of an all-planar pair whose contact points round exactly"); every other loft, placed or chorded or both, hands the boolean its combined displacement `absSumUpper(delta, sectionDelta)` as the operand displacement every other nonzero-bound operand already carries (`bounds.go`'s `rimDelta`), so the result's volume is `Approximate` like any other. **A chorded loft is not a zero-bound operand however it is placed**: at `m = 1` its `delta` is exactly zero while its `sectionDelta` is positive (§5.2, §8), so admitting it on `delta` alone would hand the boolean a zero bound for a boundary §8 states departs by `sectionDelta` |
| **D3** | Interference (`docs/interference-design.md`) | box separation (D6-style) reads `Bounds` directly; the read-only mesh-boolean path reads D2's tessellation | box-disjoint pairs prove only their disjoint-interior interference relation (`Bounds` carries the payload's own displacement, §8). `Verify` is `Sound` only when every other required or requested body and pair check is decided and trusted; a pair needing the mesh boolean works once D2 lands; a pair needing the analytic containment/pair kernel stays `Suspect` until a loft case is added to `clearance_geom.go`'s payload switch — identical staging to the cup's own D6 row in `docs/modify-design.md` |
| **D4** | Clearance (`WithClearances`, `docs/clearance-design.md`) | the analytic pair kernel's payload switch | `WithClearances` stays `Suspect`, even for a box-disjoint pair: box separation proves disjoint interiors but does not measure the gap. No loft case exists in the kernel yet. |
| **D5** | `MinWallThickness` / `Undercuts` / `MinRadius` (verification §6, `survey2d.go`) | one constant 2D cross-section (a prism's section, a revolve's meridian) | The corresponding requested survey is `Suspect` until its loft implementation lands. In increment 1, a loft's cross-section varies continuously between the two profiles, so the existing spanning-disk / meridian-walk reduction does not reach it; `docs/modify-reach-design.md` DX9 states the identical cap-blend reason: "not one constant section at one height… the existing 2D spanning-disk proof does not decide them" |
| **D6** | `Verify` — structural audit + tolerance gate | topology + measurements | valid by construction once §6's audit has passed at build time (modify §1's standard); the tolerance gate judges `Volume`/`Area`/`Centroid`/`Bounds` on the terms §8 derives — for a placed body (§12 PR 2a) all four now carry the payload's own `delta`, so the gate judges four readings that all carry the placement term |
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
same-kind circular pair's station chain (§5.1) is `chordCount`'s own
closed-form walk-up over the two records' own radius and sweep, evaluated
against the fixed `loftChordFraction` constant (§5.1, §14) — the same two
records and the same constant always produce the same station count and the
same station coordinates, with no live sketch consulted and no caller input
threaded through the wire (§10). A replay reproduces the same triangles, the
same roles, and the same measurements every time.

## 11. Cancellation and work budget

Covered in full by §6 (the audit is the only expensive phase; pairing and
construction are `O(sum n_i)`, cheap). No separate discipline is needed
beyond what §6 already states.

## 12. Increments

These PR labels are the count-free Loft delivery plan. They do not consume a
global evaluator increment.

| PR | Lands | Still refused after it |
|---|---|---|
| 1 | `OpLoft` wire/recipe plumbing (`LoftOpts` codec, `Op` token, `Step.Profile`/`Plane` reuse), Table P pairing + Table S gates S1–S5/S9–S11, the flat-triangle wall construction (§5), the crossing audit (§6, Table S S6–S8), `Document.Loft` / `LoftContext`, `Volume` / `Centroid` (§8's rational accumulator) / `Area` / `Bounds`, `Verify` (D6: the structural audit and the tolerance gate over all four) | same-kind `CircleSeg`/`ArcSeg` correspondence; N-section/guide-rail/centerline loft; `Placed`/`Duplicate`/`PlacedCopy`; reversed correspondence; surveys, clearance, interference beyond box-disjoint |
| 2a | `Placed` / `Duplicate` / `PlacedCopy` (D7): the payload's own proven displacement term `delta` (§5), composed into every vertex, edge length, face area, and all four body measurements; Table S gains S12 and S13 | D1/D2 (`Tessellate`/`STL`/`OBJ`, mesh-boolean admission); D3/D4's analytic-kernel case; D5 |
| 2b | `Tessellate` / `STL` / `OBJ` (D1), mesh-boolean admission (D2) | D3/D4's analytic-kernel case, D5 |
| 3 | same-kind `CircleSeg`/`ArcSeg` correspondence (§1): the chord-chain construction and its shared station generator (§5.1), `sectionDelta` (§5.2) composed into `Volume`/`Centroid`/`Area`/`Bounds`, Table S gates S14–S16 and S7's structural walk-sense arm (P5) | free-form and mixed-kind correspondence (§1); N-section and guide-rail/centerline lofts; a loft case in `clearance_geom.go`; a non-constant-cross-section wall survey kernel |
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

On an unplaced, uncurved payload (`delta` zero, every paired segment a
`LineSeg`) the vertex set's own maximum IS the body's true diameter — every
vertex is exact (§5), so a convex-hull diameter realized at vertices is that
diameter, not an envelope — and the arm reports the shared reader's answer
unchanged, with no subtraction and no rounding of its own. That answer is the
largest `float64` at or below the true diameter, because the reader publishes
every witness maximum rounded toward zero (verification §3), so the arm
carries the tightest lower bound a `float64` can state on a quantity that is
exact. **For a same-kind circular pair the vertex set's own maximum IS AT OR
BELOW the body's true diameter, never necessarily equal to it**: a station
lies ON the recorded curve rather than at an extreme of it, so the true
boundary can bulge past the station polygon (§5.2) and the held diameter can
understate the true one. The arm's own reasoning still holds without change,
because it only ever needs the held reading to be a sound LOWER bound on the
true diameter, never an exact one, and a station-set diameter is that: every
station sits on the true boundary, so no true pairwise distance the arm could
be missing exceeds the diameter it would have measured directly. A placed
payload (PR 2a) holds every vertex only within `delta` of its true position,
so each of the two farthest points can move by `delta` and the reported
reference is the held diameter minus `2*delta`, rounded down: an understated
reference can only tighten the gate into a false `Suspect`, never loosen it
into a false `Sound`. A shrink that collapses to zero or below reports no
diameter at all, the same answer any other unusable magnitude gets. That last
branch is defensive rather than a reachable reference-less `Suspect`: the
divergence theorem bounds a closed boundary's own volume by `d*A/3` for `d`
the vertex-set diameter and `A` the held surface area, so a `delta` at or
above `d/2` makes `sweptVolumeAllow`'s `delta*A` at least `3/2` of the held
volume, and S12 refuses that placement before any gate reads it.

**A curved pair chorded at `m = 1` (§5.1) has no interior station, so
`stationRound` and `delta` are both exactly zero even though `sectionDelta`
is positive.** The gate arm reads only `delta`, so it reports the unshrunk
held-diameter reading in this case — no `2*delta` subtraction, exactly as the
uncurved unplaced case above. That is sound rather than an oversight: every
vertex at `m = 1` is still a recorded endpoint (the segment's own two ends),
so the held diameter's witnesses are exact points on the true boundary, and
the arm's own soundness argument above (a station-set diameter is always a
lower bound on the true one) covers this case without needing `sectionDelta`
at all. `sectionDelta` bounds how far the BUILT WALL departs from the curve
between those two exact endpoints, a question the diameter-witness argument
never asks.

## 13. Required tests

Every test asserts on computed geometry — coordinates, volumes, residuals —
never merely that a call ran (project rule).

**Every fixture below is an UNPLACED body unless its own bullet says
otherwise**, so every exactness and zero-bound assertion in this section is
read at `delta` zero. A placed body's readings are the Placement bullet's own
subject, and §8 owns the rule each of them follows: `delta` enters every
vertex, edge length, face area, and all four body measurements, so no fixture
here may be reused against a placed body without carrying it.

**The fixture wall-clock budget.** Every fixture in this section builds its
loft in 2 seconds or less, and at most three of them chord a curve at the
calibrated `loftChordFraction` (§14) rather than at a coarse or explicitly
forced station count; a fixture whose build needs longer runs behind
`testing.Short()` rather than shipping in the default `go test ./...` run.
§6's audit is the phase this budget governs — it tests every pair among the
assembled triangle set, so its cost grows with the square of `F`, while
pairing and construction are linear in `Σstations`. The budget bounds which
fixtures ship, never what the evaluator admits: a station count that misses
it is a fixture this section excludes, never a reason to loosen §5.1's chord
target or its cap. §14 records the reference fixture's own measured build
against this budget.

- **Pairing**: hole-count mismatch → S1; segment-count mismatch → S2;
  mixed-kind or free-form segment pair → S3; a same-kind `CircleSeg` pair
  whose two recorded `CCW` flags disagree → S7's `ErrDegenerate` at the
  structural gate, asserted to refuse before construction rather than from
  the audit, so the fixture pins the gate's position and not only its
  sentinel; malformed
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
  triangle has positive area; the two caps' triangulation matches
  `triangulate.go`'s existing polygon-with-holes output for each profile in
  isolation; matching counter-clockwise square `LineSeg` profiles on identity
  frames at `z=0` and `z=1` first assert that `capStart` reverses `p0`'s
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
  (coincident-vertex) segment pair → S6; a same-kind circular pair with zero
  sweep, and one whose two computed stations round to the same float64
  (§5.1), each also reach S6.
- **Audit**: a deliberately over-twisted correspondence (e.g. an intentional
  wrong `WithLoftAlignment` offset on a non-convex profile) proves a
  crossing → S7, asserted against the specific triangle pair the crossing
  predicate found; two matching square `LineSeg` profiles on parallel planes
  pass when their coplanar cap-triangulation pairs and coplanar wall-diagonal
  pairs are admitted through `triTriCoplanarSharedEdge`. The same pairs remain
  `contactRegion` in `triTriClassify`, proving that the adjacency helper does
  not change mesh-boolean contact classification. Consecutive wall pairs
  classify as their recorded vertices. A vertex-sharing wall pair that crosses
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
  takes the shared `m = max(m0, m1)`, and each side's achieved sagitta at
  that shared `m` is at or below its own target, checked over
  `math/big.Rat`. `sectionDelta` on a loop with two curved pairs of different
  curvature equals the LARGER pair's own measured sagitta, never their sum,
  over `big.Rat`. A paired `CircleSeg` loop builds a full-circle-to-full-circle
  frustum whose volume encloses the closed-form frustum volume at
  `chordCount`'s closed-walk floor of three stations; its alignment offset is
  forced to `0` (§1), and a nonzero `WithLoftAlignment` entry for that loop is
  S4. A fixture sized past the station cap → S15 (`errTooManyChords`),
  asserted to fire BEFORE S8 on a construction that would otherwise reach the
  audit ceiling. A synthetic pair whose two sides are forced to different
  station parities at one cell → S16. A recorded pair whose station enclosure
  the record cannot state (`circularWalkEndBound` answering `+Inf`, §5.2) →
  S14, asserted on the sentinel and on the refusal happening instead of a
  published zero bound. `Area.Bound` on a chorded body ENCLOSES
  `|held - true|` against a high-precision reference for the true ruled
  surface between the two recorded curves, so §8's per-cell ruled-excess term
  is asserted in the outward direction rather than merely present.
  `Bounds` widened by its own `Bound`
  (`absSumUpper(delta, sectionDelta)`) CONTAINS a dense sample of both true
  recorded arcs lifted through their planes — a box that did not widen fails
  it. A pair chorded at exactly `m = 1` publishes `delta == 0` and
  `bodyGateDiameter`'s unshrunk reading, with `sectionDelta > 0` (§12). A
  mixed line-to-arc pair, and an arc-to-fit-spline pair, still refuse S3.
  Replay of a recorded circular-pair step reproduces the same station count,
  the same triangle set, and bit-identical measurements.
- **Downstream**: D1's `Bound` is the payload's own combined displacement
  `absSumUpper(delta, sectionDelta)` — exactly zero for an admitted unplaced
  `LineSeg`-only loft, positive for a placed one, and positive for an
  UNPLACED CHORDED one, asserted on a placed fixture and on an unplaced
  chorded fixture beside the `LineSeg`-only one; a D2 boolean between an
  unplaced `LineSeg`-only loft and a prism succeeds through the existing
  all-planar zero-bound path, while one between a prism and either a PLACED
  loft or an unplaced CHORDED one succeeds with an `Approximate` volume whose
  bound composes that combined displacement; a box-disjoint loft/loft pair
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
  reference: an unplaced loft's `bodyGateDiameter` must equal the shared
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
`m = 64` is the count the shipped constant itself yields on that fixture, so
its 2.39x is a margin at the shipped constant. The fit-spline wedge's 1.90x
is NOT: 6.69759e-05 is a coarser target than the shipped constant, so a
fit-spline wedge chorded to meet 3.76491e-05 takes more than 64 chord cells,
and the margin it reaches there is a reading this document does not state.

**The constant does not clear a 4x margin inside the wall-clock budget, and
this design accepts that rather than widen either.** A 4x margin needs 128
stations, whose build measures about 4.3 seconds; the fixture wall-clock
budget §13 states caps that build at 2 seconds, which the 64-station build
meets at about 1.4 seconds. The fixed 64-station constant is what ships. An
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
surface — the recorded curve a chord chain approximates. Their `areaUpper`
obligation is discharged by `perturbedAreaUpper(verts, tris, sectionDelta)`
plus a per-wall-cell `cellRuledExcessUpper` term, because containment inside
a thin neighbourhood of the chord facets does not, on its own, bound a
surface's area: the arc-minus-chord ruled surface can carry more area than
the flat facets it stays close to, so the excess needs its own term rather
than following from proximity alone. `cellRuledExcessUpper` is the same
per-cell helper §8's `Area` wall term reads, in the same length-excess-times-
ruling-length form, and §8 states that form once for both consumers.

**`loftStationCap`'s value is this document's one open variable.** §5.1
states the rule the cap obeys and everything an implementation needs to
decide S15 from the record — the per-segment share, the `mMax` comparison,
and the checked arithmetic — but not the number itself. The number is fixed
by the increment that lands the station generator (§12 PR 3), inside two
constraints §5.1 already states: a build whose `Σstations` reaches the cap
must assemble an `F` whose `F*(F-1)/2` is strictly below
`maxFacetPairTestsPerCall` (§6), and the cap must leave room for every
fixture §13 requires. Nothing else in this document reads the number: the
reference fixture's 64 stations, and every other station count named here,
are stated against the chord target above rather than against the cap.

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

§5.1's chord chain also gives an unplaced loft a positive boundary
displacement (§5.2, §8). Four companion documents state a zero bound or an
exact boundary for an unplaced loft, so each is corrected to condition that
on BOTH `delta` and `sectionDelta` being zero — an unplaced `LineSeg`-only
loft — rather than on `delta` alone, and to name the combined displacement
everywhere else:

- **`docs/tessellation-design.md`**: the `loftPayload` row of §2's proof-term
  table and the exact-restatement text under it, §13's T6 row, and §14's
  `loftPayload` test obligation. A chorded loft is not an all-planar
  zero-bound mesh-boolean operand however it is placed.
- **`docs/clearance-design.md`**: §2's `loftPayload` sentence, whose bounds
  inflate by that combined displacement.
- **`docs/payload-verification-design.md`**: §2's `loftPayload` bullet, which
  calls the held boundary exact only where both displacements are zero.
- **`docs/verification-design.md`**: §3's `bodyGateDiameter` prose, which
  earns the vertex maximum as the true diameter for a `LineSeg`-only loft and
  as a lower bound on it for a chorded one (§12).
