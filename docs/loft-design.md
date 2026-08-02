# Loft Design

The increment-1 `Loft` feature: a solid between two recorded planar profiles,
built by straight rules between corresponding boundary points, with no guide
rail and no centerline. Companion to `docs/api-design.md` (public surface,
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

## 1. Scope — the increment-1 case, and what is deferred permanently

The target case, stated by the consuming project
(`fusion360-gear-generator`'s 3D proof stage): two profiles, each already
closed and valid per `sketch`, each recorded on its own plane, with the same
topology (same loop count, same segment count per loop), no guide rail, no
centerline, ruled between corresponding points. A helical gear tooth is one
such loft (bottom tooth loop to a twisted top loop); a bevel gear is two;
herringbone and spiral bevel compose those.

**Increment 1 admits exactly this shape, narrowed once more: every
corresponding segment pair MUST be `LineSeg`.** A profile with a curved
boundary (`CircleSeg`, `ArcSeg`, or any free-form kind) is `ErrUnsupported`
at the call — Table S row S3 — even where both sides carry the *same* curved
kind. This is the one deliberate narrowing beyond what the consumer described
as its easy case, and it is what makes every wall face an exact `Plane`
(§5) rather than a new ruled-surface `Surface` variant this evaluator does not
otherwise have a construction for. The reason is stated once, not per row:
representing a ruled surface between two circular arcs exactly needs a
rational-quadratic-Bézier-per-arc construction this evaluator does not build,
and approximating it would be exactly the confidently-wrong failure
`docs/api-design.md` §1 exists to prevent. Lifting it is future reach, listed
in §12 as a decad-side capability, not an upstream ask.

**Permanently out of scope, for reasons stated once:**

- **N-section lofts and guide-rail / centerline lofts.** Without a guide
  rail, ruling more than two sections needs an interpolation scheme this
  design has no closed-form, non-fitting answer for. The consumer does not
  need it either: a bevel gear is two 2-section lofts, not one 3-section
  loft. A future design may add it; this one does not reserve a shape for it.
- **Free-form and curved-segment correspondence** (§1 above).
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
(core §7): a loft between two differently-posed sections needs two.

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
| **P5** | A paired segment's two sides MUST both be `LineSeg`. Any other pairing — including a matching curved kind — is `ErrUnsupported` (§1, Table S row S3) |
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
| **S3** | a paired segment where either side is not `LineSeg` (§1, P5) | the ruled surface exists; this evaluator has no exact construction for it | `ErrUnsupported` | no, §12 PR 3 |
| **S4** | a `WithLoftAlignment` payload of the wrong length, an offset outside `[0, n)` for its loop, or the option passed more than once | no single intent (mirrors modify-reach SX1, which refuses a repeated contradictory option on the same ground) | `ErrDegenerate` | yes, §2 |
| **S5** | `p0`'s and `p1`'s `PlaneRecord`s are exactly equal (`Origin`, `U`, `V` all equal) | no — every wall vertex then lies in one plane, so the solid is provably flat: the tetrahedron-sum volume (§8) is a structural zero, not a computed one | `ErrDegenerate` | yes, §4 |
| **S6** | a wall or cap triangle whose three recorded points collapse (coincident vertices, zero area) | no — the modification consumed the region, the same existence answer modify §5 test 1 gives an inside-out loop | `ErrDegenerate` | yes, §4 |
| **S7** | the crossing audit (§6) finds contact other than the pair's recorded expected contact | no — a self-intersecting or self-touching shell bounds no solid | `ErrDegenerate` | yes, §6 |
| **S8** | the crossing audit exhausts its fixed work budget (§6, §10) before every pair is decided | this evaluator cannot tell | `ErrUnsupported` | no, §6 — a resource ceiling, not a shape rule |
| **S9** | either profile fails a seam gate (§2): foreign, stale, invalid, or an unrecordable `Partial` fragment | seam design's own answer, per profile | `ErrForeignProfile` / `ErrStaleProfile` / `ErrInvalidProfile` / `ErrUnrecordableProfile` | seam design's own answer, per gate; this document adds no permanence of its own (§2) |
| **S10** | a nil `*sketch.Sketch` or `*sketch.Profile` argument | no call at all | `ErrDegenerate` | yes, §2 |

**Gate order**, the same "ask what could be asked" discipline modify §4
states: pre-gates first (S10 nil check, S9 seam authentication of both
profiles — nothing downstream is safe to read before this), then the shape
gates that need only the two authenticated records (S1 hole count, S2
segment count, S4 malformed alignment, S3 segment kind, S5 plane identity —
all decidable without building a single triangle), then construction (§5),
then the per-triangle existence gate S6, then the crossing audit (S7/S8,
§6) — the most expensive step, run last, over triangles already proven
individually non-degenerate.

## 5. Construction — flat triangular walls, never a curved ruled surface

**Every wall is two flat triangles, split along the same fixed diagonal
`tessellate.go` already uses for a prism's lateral quad**: "split it along
the fixed (bottom-start, top-end) diagonal into two outward triangles." A
loft reuses that convention as its actual TOPOLOGY, not merely its
tessellation.

For paired segment `j` of a loop — `V_j -> V_{j+1}` on `p0`, `W_j -> W_{j+1}`
on `p1` (indices already rotated by §3's alignment) — the quad `V_j, V_{j+1},
W_{j+1}, W_j` splits into:

| Triangle | Vertices | Contains | Role |
|---|---|---|---|
| lower | `V_j, V_{j+1}, W_{j+1}` | the full `p0` segment (shared with `capStart`'s boundary) | `side(i,j,0)` |
| upper | `V_j, W_{j+1}, W_j` | the full `p1` segment, reversed (shared with `capEnd`'s boundary) | `side(i,j,1)` |

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
for a cap vertex.** Per topology §3's "feature-built vertices are exact
(bound zero)," a loft vertex carries the same standing — no new rounding
risk is introduced; it is the same closed-form coordinate lift every other
feature already treats as truth.

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
  shared edge, `C` and `D` the two apex vertices — the edge convex exactly
  when the sign puts `D` outside the half-space `ABC`'s outward normal
  defines. This is the identical adaptive exact-orientation predicate
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

**Every wall face is a `Plane`** (its `Frame` computed from its own three
exact vertices), Exact by construction per core §6.1's surface-parameter
carve-out — the identical standing Extrude's `LineSeg` side walls already
have. Cap faces (`capStart`, `capEnd`) are `Plane`s over a polygon-with-holes
region, exactly as an Extrude cap is.

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

- **A pair sharing an EDGE is admitted only when `triTriClassify` reports the
  exact recorded common edge as its whole shared segment.** This applies to a
  rung, a diagonal, a cap-boundary edge, and an internal edge of one cap's own
  triangulation. The matching segment proves the triangle interiors are
  disjoint; a point, an extra segment, a 2-D region, or a crossing refuses.
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

## 7. Table B — the result

| Face | Count | Surface | Roles |
|---|---|---|---|
| `capStart` | 1 | `Plane`, over `p0`'s region | `capStart` |
| `capEnd` | 1 | `Plane`, over `p1`'s region | `capEnd` |
| lower wall triangle | `n_i` per loop `i` | `Plane` | `side(i,j,0)` |
| upper wall triangle | `n_i` per loop `i` | `Plane` | `side(i,j,1)` |

**Lump count is always 1.** The two caps and `2*sum(n_i)` wall triangles
form one connected, manifold, watertight shell once §6's audit has passed —
there is no shape in increment 1's admitted correspondence that produces a
second lump, unlike a modify op's holed both-caps shell (modify Table B,
row B4).

**`Body.Origin()`** is the loft step, role `"body"`, the same uniform rule
every other feature follows (modify §11).

**Placement.** `Body.Placed` / `Duplicate` / `PlacedCopy` need no new case:
every loft surface is a `Plane`, and evaluator §8 already states "every v1
surface variant maps to itself under an isometry (plane→plane, …)." A
placed, duplicated, or placed-copied loft body re-evaluates from the same
`Step` record, reproducing the same roles (modify §11's "roles derive from
the record and the deterministic walk order").

## 8. Mass properties — derived, not asserted

Write `T` for the set of `2*sum(n_i)` wall triangles plus the two caps'
own triangulations, each triangle `(A, B, C)` consistently outward-oriented
(material on the left, the same walk-order convention every payload already
uses).

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
and `spline_bezier.go` already follow), accumulated into ONE rational sum
per body (anchored at `p0`'s own `PlaneRecord.Origin`, mirroring
`moments.go`'s own anchor-then-publish-once discipline, §3), and rounded to
float64 **once**, at publication — the identical `addExact` /
`translateExactMoments` / `publishExact` shape `moments.go` already
implements for a 2D region's exact rational accumulator, extended here to a
3D triangulated boundary rather than invented as new machinery.

**Each cap's own contribution reuses `moments.go` unchanged.** A `LineSeg`-
only `ProfileRecord`'s area and first moment are already exact rationals
there (evaluator §4); the two caps' contribution to the 3D volume/centroid
sum is that same 2D exact rational, lifted through the cap's own
`PlaneRecord` — no new 2D integration is written.

**Therefore `Volume` and `Centroid` are `Exact` exactly when their published
rational is representable in the `units.Value` magnitude actually carried —
never unconditionally.** This is spline design §3's Tier A rule, verbatim:
"the reported bound is a SINGLE rounding of that rational into that
magnitude, and it is zero — hence `Exact` — exactly when the rational is
representable in the magnitude the value ACTUALLY CARRIES." A loft's volume
earns that ceiling for the same reason a Tier A free-form region's area
does: the integral is exactly rational; only its final publication rounds.

**`Area` is never Exact.** A triangle's own area is `(1/2) * |(B-A) x
(C-A)|` — a square root of a rational, generically irrational — so a wall
triangle's area contribution is a float evaluation with a proven outward
bound (`bounds.go`'s `sumSlop`: "a PROVEN bound for the NAIVE float sum
that produced the value — never zero for a nonzero float-computed
quantity"). Spline design §3 states the identical asymmetry for arc length:
"Arc length is never exact in ANY tier… a Tier A body's `Area` always
carries a positive bound even where its `Volume` does not." A loft's `Area`
is the two caps' own exact rational area (from `moments.go`, contributing no
bound) plus the wall triangles' proven-bound sum — so the total is
`Approximate` with a proven bound whenever at least one wall triangle has
nonzero area, which increment 1's admitted correspondence always does.

**`Bounds` is Exact.** Every vertex is already treated as exact (§5); the
axis-aligned box is the componentwise min/max over an already-exact set, the
same per-vertex-extreme reasoning Extrude's `Bounds` already relies on — no
new rounding is introduced by comparing exact numbers.

**Vertex position, edge length, and face area follow the standing rules
already governing every other analytic payload** — a position is Exact by
construction (§5); a straight edge's length and a triangle's own `Area()`
need a square root and are `Approximate` with a proven bound, Exact only
when that particular evaluation happens to be exactly representable — the
same standard Extrude's own `LineSeg` walls and edges already carry. This
document introduces no new per-accessor rule beyond what §8 already derives
for the body-level quantities.

## 9. Table D — downstream

| D | Consumer | Reads | Increment-1 status |
|---|---|---|---|
| **D1** | `Tessellate` / `STL` / `OBJ` | the payload | works from the first PR that wires it in, and the returned `Bound` is **zero**: every wall and cap face is already a flat triangle with exact vertices, so tessellation is restatement, not chording (`triangulate.go`'s existing polygon-with-holes triangulator for the two caps; no chording anywhere) |
| **D2** | the mesh boolean (`Union`/`Cut`/`Intersect`, evaluator §9) | the tessellation | a first-class operand once D1 lands, admitted through the existing all-planar zero-bound path (`docs/evaluator-design.md` §2 — "the VOLUME of an all-planar pair whose contact points round exactly") — no new boolean code, a loft body is just another all-planar operand |
| **D3** | Interference (`docs/interference-design.md`) | box separation (D6-style) reads `Bounds` directly; the read-only mesh-boolean path reads D2's tessellation | box-disjoint pairs prove only their disjoint-interior interference relation (`Bounds` is Exact, §8). `Verify` is `Sound` only when every other required or requested body and pair check is decided and trusted; a pair needing the mesh boolean works once D2 lands; a pair needing the analytic containment/pair kernel stays `Suspect` until a loft case is added to `clearance_geom.go`'s payload switch — identical staging to the cup's own D6 row in `docs/modify-design.md` |
| **D4** | Clearance (`WithClearances`, `docs/clearance-design.md`) | the analytic pair kernel's payload switch | `WithClearances` stays `Suspect`, even for a box-disjoint pair: box separation proves disjoint interiors but does not measure the gap. No loft case exists in the kernel yet. |
| **D5** | `MinWallThickness` / `Undercuts` / `MinRadius` (verification §6, `survey2d.go`) | one constant 2D cross-section (a prism's section, a revolve's meridian) | The corresponding requested survey is `Suspect` until its loft implementation lands. In increment 1, a loft's cross-section varies continuously between the two profiles, so the existing spanning-disk / meridian-walk reduction does not reach it; `docs/modify-reach-design.md` DX9 states the identical cap-blend reason: "not one constant section at one height… the existing 2D spanning-disk proof does not decide them" |
| **D6** | `Verify` — structural audit + tolerance gate | topology + measurements | valid by construction once §6's audit has passed at build time (modify §1's standard); the tolerance gate judges `Volume`/`Area`/`Centroid`/`Bounds` on the terms §8 derives |
| **D7** | `Placed` / `Duplicate` / `PlacedCopy` | the payload | works unchanged (§7) |

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
records, never a live sketch: the pairing (Table P), the construction (§5),
and the audit (§6) are closed-form functions of that recorded data, with no
step that samples, fits, or searches. A replay reproduces the same
triangles, the same roles, and the same measurements every time.

## 11. Cancellation and work budget

Covered in full by §6 (the audit is the only expensive phase; pairing and
construction are `O(sum n_i)`, cheap). No separate discipline is needed
beyond what §6 already states.

## 12. Increments

These PR labels are the count-free Loft delivery plan. They do not consume a
global evaluator increment.

| PR | Lands | Still refused after it |
|---|---|---|
| 1 | `OpLoft` wire/recipe plumbing (`LoftOpts` codec, `Op` token, `Step.Profile`/`Plane` reuse), Table P pairing + Table S gates S1–S5/S9/S10, the flat-triangle wall construction (§5), the crossing audit (§6, Table S S6–S8), `Document.Loft` / `LoftContext`, `Volume` / `Centroid` (§8's rational accumulator) / `Area` / `Bounds`, `Verify` (D6: the structural audit and the tolerance gate over all four) | curved-segment correspondence; N-section/guide-rail loft; reversed correspondence; surveys, clearance, interference beyond box-disjoint |
| 2 | `Tessellate` / `STL` / `OBJ` (D1), mesh-boolean admission (D2), `Placed` / `Duplicate` / `PlacedCopy` (D7) | D3/D4's analytic-kernel case, D5 |
| 3 (reach, not committed by this document) | curved (`CircleSeg`/`ArcSeg`) same-kind correspondence, a loft case in `clearance_geom.go`, a non-constant-cross-section wall survey kernel | — |

**The four measurements land with the operation, never after it.** A `Body`
caches `Volume` / `Centroid` / `Area` / `Bounds` at build and its accessors
return that cache with no staging error path, and `Exact` is `Exactness`'s
zero value, so a PR that published `Document.Loft` before §8's derivations
would answer `Volume()` with a proven-exact zero and `Verify` would call it
`Sound` — a confidently wrong number, which is the one outcome core §1 exists
to prevent. §8 derives all four in closed form from the same triangle set the
construction already builds, so there is nothing to stage: the tolerance gate
is generic over `Measurement` (verification §4) and needs no per-payload
wiring once the readings exist.

## 13. Required tests

Every test asserts on computed geometry — coordinates, volumes, residuals —
never merely that a call ran (project rule).

- **Pairing**: hole-count mismatch → S1; segment-count mismatch → S2;
  mixed/curved segment pair (including same-kind circular) → S3; malformed
  `WithLoftAlignment` (wrong length, out-of-range offset, or duplicate
  option) → S4; identical planes → S5; a nonzero alignment offset pairs the
  expected rotated vertex, asserted on the built wall's own coordinates. A
  two-hole fixture whose recorded `Holes` order is swapped between the two
  profiles MUST assert the two ordinal pairings by wall coordinates, or S7 for
  the resulting crossed correspondence; a nearest-hole matcher MUST fail it.
- **Construction**: every wall/cap edge bounds exactly two faces; every
  triangle has positive area; the two caps' triangulation matches
  `triangulate.go`'s existing polygon-with-holes output for each profile in
  isolation; a junction's `Edge.IsConvex` matches its hand-computed
  walked-boundary turn; an outer rim is convex and a hole rim is concave. An
  untwisted congruent parallel loft with one outer side deliberately split
  into two collinear `LineSeg`s retains exactly two `Plane` faces per wall
  cell. At the flat split rung, the preceding cell's lower triangle and the
  next cell's upper triangle remain distinct faces. Every retained flat
  diagonal and that rung report `IsConvex() == false`, are selected by
  `Concave()`, and are not selected by `Convex()`. A collapsed
  (coincident-vertex) segment pair → S6.
- **Audit**: a deliberately over-twisted correspondence (e.g. an intentional
  wrong `WithLoftAlignment` offset on a non-convex profile) proves a
  crossing → S7, asserted against the specific triangle pair the crossing
  predicate found; a rectangular loft passes when its cap-triangle pairs
  classify as their recorded internal diagonals and consecutive wall pairs
  classify as their recorded vertices; a vertex-sharing wall pair that crosses
  away from that vertex → S7. Two valid square `LineSeg` profiles with `p0` frame
  `U=(1,0,0)`, `V=(0,1,0)`, origin `(0,0,0)` and `p1` frame
  `U=(-1,0,0)`, `V=(0,1,0)`, origin `(1,0,1)`, carrying the same local
  square, make cell 0's two triangles share their recorded diagonal but
  overlap in area; the audit rejects them as S7. A synthetic profile pair
  sized to exceed the fixed pair-test budget → S8, refused before any pair
  result is trusted.
- **Mass properties**: a scaled cube-like loft (two congruent squares,
  parallel planes, no twist) reproduces the closed-form prism volume/
  centroid exactly, asserted `Exact` when the rational happens to be
  representable and `Approximate` with a correctly-signed one-ulp bound when
  scaled to force a non-representable rational (mirroring spline design
  Table F's own 293/18 vs 293/2 worked example); `Area` is `Approximate`
  whenever any wall triangle has nonzero area, with the bound checked
  against a high-precision reference sum, never merely asserted present.
  `Bounds` matches the exact per-vertex componentwise extreme.
- **Downstream**: D1's `Bound` is exactly zero for an admitted loft; a D2
  boolean between a loft and a prism succeeds through the existing
  all-planar path; a box-disjoint loft/loft pair proves only its
  disjoint-interior interference relation under D3, and its `Verify` report
  is `Sound` only with no other undecided required or requested check; a
  box-disjoint pair with `WithClearances` reads `Suspect` until the analytic
  kernel adds loft; each requested D5 survey reads `Suspect`, never absent or
  a silently wrong number.
- **Recipe/replay**: round-trip a `LoftOpts` payload including a non-zero
  `Alignment`; a missing `Profile2`/`Plane2` on the wire rejects; replay
  reproduces the same triangles, roles, and measurements as the immediate
  call; a failed call leaves the document and recipe unchanged.

## 14. Open questions

None — every design variable this document depends on is resolved above.
The reach items in §12's PR 3 row are future work, not open questions of
this design: curved correspondence, guide-rail/N-section loft, and a loft
case in the analytic clearance kernel are each named and deferred with a
one-line reason, not left undecided.

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
