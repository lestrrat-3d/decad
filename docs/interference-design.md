# Interference Design

How `Document.Verify` proves pairwise overlap and reports its volume without
changing the document: the pair relation (§1), stable report walk (§2), proof
paths (§3), full-containment certificate (§4), read-only boolean evaluation
(§5), positive-volume gate (§6), errors, cost and cancellation (§7), tolerance
gate (§8), evaluator coverage (§9), tests (§10), and increment plan (§11).
Companion to `docs/verification-design.md`, which owns what an
`Interference` row means and how its `Volume` is judged; to
`docs/clearance-design.md`, which owns analytic pair classification; and to
`docs/evaluator-design.md`, which owns the mesh boolean and its proven bounds.
Nothing here adds a public API or a recipe operation. Nothing here changes a
public boolean's consuming semantics.

## 1. Pair relation

Every unordered pair of proven solids has one evaluator relation:

| Relation | Proven claim | Report effect |
|---|---|---|
| **disjoint** | interiors share no point | no `Interference` row; emit `Clearance` only when asked and measured |
| **touching** | interiors are disjoint and boundaries meet through a certified contact | no `Interference` row; emit `Exact` zero `Clearance` only when asked |
| **overlapping** | interiors share a non-empty open region | emit `Interference` only when overlap volume is also bounded; otherwise `Suspect` |
| **undecided** | none of the three claims is proven | no pair row; `Suspect` |

`pairOverlapping` MUST be distinct from `pairUndecided`. A nesting witness,
an admitted transversal boundary crossing, or a certified same-side contact
proves overlap even when the evaluator cannot yet measure it. Losing that fact
inside `pairUndecided` prevents the next proof stage from using it and makes
diagnostics lie about what remains unknown.

The relations concern **interiors**. Boundaries may touch while the pair stays
disjoint. A positive-volume `Interference` row is never emitted for contact
alone.

The pair result also records the strongest certificate already obtained:

```go
type pairResult struct {
    verdict pairVerdict // undecided / disjoint / touching / overlapping
    // gap interval + exactness + pair diameter when disjoint or touching
    // containment classification when a strict nesting proof ran
}
```

The concrete fields stay private. The relation and its certificate MUST be
structural values, never inferred from an error string.

## 2. Stable, non-mutating report walk

`Verify` walks live bodies in `Document.Bodies()` order, keeps only proven
solids for pair work, and enumerates `i < j`. Both pair lists preserve that
order. No spatial index, worker completion order, or proof-path order may
reorder rows.

For each pair:

1. Check `ctx`.
2. Use bounds-inflated box separation. If it proves disjoint interiors and
   clearances were not requested, finish the pair.
3. Run the analytic pair kernel when a gap is requested or box separation did
   not settle the partition.
4. Finish immediately on `pairDisjoint` or `pairTouching`; append a clearance
   row only when `WithClearances()` asked and the gap was measured.
5. On `pairOverlapping`, try its containment-volume certificate first, then
   read-only intersection measurement when needed.
6. On `pairUndecided`, try read-only intersection measurement. A positive
   volume may prove overlap even when the analytic kernel could not classify
   the contact arrangement.
7. If neither volume path proves and bounds the overlap, mark the pair
   undecided for aggregation.

`Verify` MUST NOT call public `Intersect`. Public `Intersect` retires both
operands, appends a recipe step, and registers a result. Verification is
non-mutating and must leave all three unchanged:

- `Document.Bodies()` membership and order;
- every body's live/retired state;
- `Document.Recipe()` contents.

## 3. Proof paths

The paths run cheapest and strongest first. A later path may add a measurement;
it may never weaken an earlier relation.

### 3.1 Box separation

Bounds-inflated boxes with disjoint interiors prove the bodies' interiors
disjoint. Touching boxes are included: box contact cannot prove body contact,
but it does prove no positive-volume overlap.

Box separation does not measure the true gap. Under `WithClearances()`, the
analytic clearance kernel still runs.

### 3.2 Analytic pair kernel

The clearance kernel owns boundary distance, certified contact, boundary
crossing, and nesting classification. It returns all four §1 relations:

- positive boundary clearance plus nesting excluded → `pairDisjoint`;
- a certified opposite-material contact with no other crossing →
  `pairTouching`;
- an admitted transversal crossing, certified same-side contact, or nesting
  witness → `pairOverlapping`;
- insufficient trim, contact, or containment proof → `pairUndecided`.

Boundary distance alone MUST NOT turn nesting into disjointness. A nesting cast
must preserve three outcomes — outside, inside, undecided — instead of returning
one `bool` that merges inside with failed classification.

### 3.3 Full containment

A strict full-containment certificate proves both the relation and the overlap
volume without a boolean. §4 defines it.

### 3.4 Read-only intersection

The exact-predicate mesh boolean computes the held intersection. Its exact
rational volume integral and symmetric-difference bound yield a proven interval
for the true overlap. §5–§6 define when that interval proves a row.

## 4. Full-containment certificate

If one entire body lies in the other's material, their intersection is the
contained body. Reuse that body's existing `Volume` measurement; do not
tessellate it again and do not manufacture a new bound.

The certificate requires all of these:

1. Both operands are proven solids.
2. Their boundaries are proven disjoint with a **strictly positive** boundary
   clearance. A zero-distance or undecided contact cannot enter this path.
3. Both bodies are shipped analytic payloads represented by `bodyGeom`, and
   each has exactly one material lump. Multi-lump/faceted bodies skip this
   certificate and proceed to §5.
4. Choose one deterministic witness from **every** shell of both operands,
   void shells included.
5. `pointInBody` proves **both** directions: every witness of `inner` lies in
   `outer` material, **and** every witness of `outer` — its void shells among
   them — lies outside `inner` material. A witness proven in an `outer` void is
   outside `outer` material and fails full containment. A cast that grazes or
   exhausts its direction ladder is undecided.

With disjoint boundaries, membership is constant on each connected component of
one body minus the other body's shells — not throughout the material lump,
which the other body's shells can cut into several components. Each shell is
connected and misses the other boundary, so it lies wholly in one such
component and its single witness decides that whole shell.

Proving every `outer` witness outside `inner` is what makes `inner`'s own
witnesses speak for all of it: `outer`'s boundary then misses `inner`
entirely, so `inner` lies in one component and is wholly inside or wholly
outside `outer`. Without that direction, an `outer` void shell can sit inside
`inner`'s material and carve away a part of it that no `inner` witness reports
— and the reused volume would overstate the true overlap. `inner`'s own voids
need no separate treatment: they remove material from `inner`, and the
intersection with `outer` removes the same void.

If A's lump is inside B, report A's volume. If B's lump is inside A, report B's.
Strict boundary separation prevents both directions from holding for non-empty
solids.

**All multi-lump analytic containment is staged.** Shipped multi-lump bodies are
faceted Boolean results, and `bodyGeom` does not represent their held facets.
They skip analytic nesting entirely and proceed to read-only intersection; an
unsupported or unmeasurable intersection reads `Suspect`. Never reuse a whole
multi-lump body's volume from a partial witness set, and never sum only known
inside lumps.

Containment volume keeps the contained body's `Value`, `Exactness`, and
`Bound` unchanged. The containment proof itself establishes positive overlap,
so this path may emit the row even when that existing volume interval reaches
zero. The coarse measurement still fails §8's tolerance gate; the report
remains `Interfering`, which outranks `Suspect`.

### 4.1 Coincident analytic bodies

Strict containment cannot classify equal bodies because their boundaries
coincide. An exact set-equality certificate may reuse one operand's volume on
the same grounds: `A = B` proves `A ∩ B = A`.

The shipped certificate is analytic and exact. It compares the payload fields
that determine the represented point set with exact structural equality, never
samples, bounding boxes, tolerances, or topology counts. Derived metadata that
does not change that set — `cupPayload.thickness`/`sense`,
`prismPayload.blendSegs`/`blendKind`, and the section displacement bound
`docs/prism-boolean-design.md` §7 puts on `prismPayload` — is deliberately
excluded, as is every later derived field that leaves the boundary where it is.
Equal represented-set fields in `prismPayload`, `cupPayload`, or
`revolvePayload` prove identical represented sets.

Broader equality is staged. A future normalizer may prove harmless
representation choices equivalent:

- a closed loop's cyclic start point;
- hole order;
- an axis line's direction sign;
- a full revolution's angular start;
- a sweep interval's endpoint spelling.

That broader normalizer MUST compare by payload:

| Payload | Equality certificate |
|---|---|
| `prismPayload` | same world plane and sweep direction; same signed world sweep interval; same normalized line/arc material region |
| `cupPayload` | same outer region and interval; same cavity region and interval; same accumulated rigid placement |
| `revolvePayload` | same world axis line; same normalized meridian material region; same angular point set — equal partial interval or both full |

Current coordinates and curve parameters are compared as exact stored values.
A near match or structurally different record stays undecided. A future
normalizer may add adaptive-exact comparisons only when it proves set identity.

Do not apply this certificate to `facetedPayload`. Equal held polygons do not
prove the two bounded source parts equal. Their read-only intersection still
uses §5–§6.

When equality is certified, emit A's existing volume measurement, where A is
the first operand in stable pair order. Do not choose opportunistically by
bound width; stable pair order makes repeated reports byte-stable. The set
identity proves positive overlap independently of the volume interval, exactly
as strict containment does.

## 5. Read-only boolean evaluation

Factor the mesh boolean into geometry evaluation and document commit:

```go
func evaluateBoolean(
    ctx context.Context,
    op OpKind,
    a, b *Body,
) (booleanEvaluation, error) // no document write

func performBoolean(
    ctx context.Context,
    op OpKind,
    a, b *Body,
) (*Body, error) // public commit path
```

Names are illustrative; the split is normative.

`evaluateBoolean` performs only geometry work:

1. derive pair chord tolerance and pair diameter;
2. tessellate operands;
3. map source faces;
4. prepare exact-predicate meshes;
5. run the hidden-tangency refusal;
6. classify and cut the mesh intersection;
7. stitch, conform, weld, and audit the held result;
8. compose chord and rounding bounds;
9. integrate held volume in exact rational arithmetic.

It MUST NOT call `nextStepRef`, append a `Step`, retire an operand, register a
body, or expose a transient result through the document. Its result contains
the held facets and every bound input needed by either caller.

The public context variants keep consuming behavior:

1. gate nil, foreign, retired, and identical operands;
2. call `evaluateBoolean(ctx, op, a, b)`;
3. build and audit the public faceted body with `ctx` and the next real step reference;
4. append the boolean step and retire/register atomically.

`Union` / `Cut` / `Intersect` call their context variants with
`context.Background()` for compatibility. Cancellation before step 4 returns
`ctx.Err()` unchanged and leaves the document and operands unchanged.

`Verify` calls `evaluateBoolean(ctx, OpIntersect, a, b)` and consumes only the
volume result. It does not build or register a transient `Body`.

### 5.1 One volume integrator

Extract the exact rational volume loop from faceted-body construction into one
private helper used by both callers. Given the stitched, oriented closed mesh
and the composed symmetric-difference allowance, it returns:

```go
Measurement{
    Value:     units.CubicMillimeters(Vheld),
    Exactness: exactnessOf(E),
    Bound:     units.CubicMillimeters(E),
}
```

`E` is the existing boolean volume allowance:

- operand A chord symmetric difference;
- operand B chord symmetric difference;
- volume swept by final float rounding over the **pre-round** stitched area;
- exact-rational-to-float rounding of `Vheld`.

The boolean rim displacement does not replace this allowance. The
symmetric-difference identity is the volume proof; the trim-amplified rim bound
continues to govern the public result's boundary measurements.

Both callers MUST use the same held facets, orientation audit, volume integral,
and bound helpers. A verification-only approximate volume formula is forbidden.

### 5.2 Positive-area coplanar patches

The current symmetric triangle classifier refuses a two-dimensional coplanar
intersection. That refusal stays correct until this complete replacement
lands; it must not be weakened one pair at a time. `docs/prism-boolean-design.md`
supersedes this section for the narrower case of two co-directional coplanar
prisms — an admitted pair gets an analytic overlap volume through
`evaluateBoolean`'s analytic dispatch, exact under that design's §7 rule and
otherwise bounded by that rule's own terms, never reaching this mesh-side
arrangement; this section still governs every coplanar pair that design does
not admit.

Coplanar breadth support constructs one exact 2D arrangement per coplanar face
patch in the dominant-axis projection already used by the boolean's rational
cutting code:

1. Intersect projected triangle boundaries with exact rational predicates.
2. Split them into maximal positive-area cells. Edge-only and vertex-only cells
   remain contact cases.
3. For each cell, read both operands' outward material side from the oriented
   adjacent facets. Never infer it from triangle input order alone.
4. Opposing material sides describe boundary-only contact at that cell. Keep it
   out of the intersection volume and pass the complete contact set to the
   touching proof.
5. Matching material sides describe a coincident intersection boundary. Keep
   exactly one copy, choosing operand A in stable pair order, and connect its
   cell edges to the non-coplanar cut regions.
6. Audit every arrangement edge for two-sided adjacency after the merge. A
   branching edge, an unclassified neighboring cell, mixed side evidence, or a
   cell the rational arrangement cannot close is an expected undecided contact
   outcome, never a guessed keep/drop.

The patch relation does not decide the whole pair by itself. Opposing cells are
`pairTouching` only when the analytic/contact pass proves no overlap elsewhere.
Matching cells may bound a positive-volume intersection only after the complete
stitched result passes §5 and its volume clears §6.

## 6. Positive-volume gate

Let an intersection evaluation return `V = abs(Volume.Value)` and
`E = Volume.Bound`. The true overlap volume lies in the non-negative interval

```text
[max(0, V - E), V + E].
```

Emit an `Interference` row from this path only when `V - E > 0`. That strict
inequality proves the true intersection has positive volume. Copy the
measurement unchanged into the row.

These outcomes do **not** emit a row:

| Outcome | Why | Pair result |
|---|---|---|
| held intersection empty | an empty approximation plus a nonzero symmetric-difference allowance does not prove true emptiness | `Suspect` unless an earlier path proved disjoint/touching |
| `V - E <= 0` | interval admits zero; held overlap is below this evaluator's proof reach | `Suspect` |
| tangent/grazing/coplanar contact refused | topology classifier did not prove a positive-volume intersection | `Suspect` unless clearance certified touching |
| operand/result unsupported | evaluator lacks a complete path | `Suspect` |

Do not turn a mesh witness point, an intersection segment, a nesting witness,
or a positive held volume alone into a row. Every row carries the bounded
overlap volume the public contract promises.

The set-identity paths are the only exceptions to `V - E > 0`: strict full
containment proves `inner ∩ outer = inner`, and analytic equality proves
`A ∩ B = A`. Each independently proves positive overlap, and the existing body
volume is the quantity, not the proof of existence.

## 7. Errors and cancellation

### 7.1 Expected undecided outcomes

The read-only evaluator MUST distinguish expected geometric non-results from
internal failures with private typed outcomes, not sentinel identity alone and
never message matching:

- empty held intersection;
- contact/graze/coplanar arrangement the mesh boolean refuses;
- evaluator staging (`ErrUnsupported`);
- a valid operand whose chording is too coarse to prove its own topology —
  loops that close within their chord bounds of each other, a bridge that
  pinches, a chorded boundary whose ear clipping stalls. The evaluator owns the
  chord tolerance here, so the caller cannot act on the refusal;
- positive held intersection whose bound does not clear zero.

`performBoolean` maps these to its existing public errors (`ErrBooleanFailed`,
`ErrDegenerate`, or `ErrUnsupported`). `Verify` maps them to an undecided pair.
For diagnostics, preserve the private reason: pre-contact operand staging emits
`DiagUnsupportedPairPayload` and names the operand; contact policy emits
`DiagUnsupportedPairContact`; later pipeline reach emits
`DiagUnsupportedPairPipeline`. Each message states the matching corrective
action. Keep the broad `DiagUnsupportedPair` signal alongside these codes for
compatibility, while callers should branch on the cause-specific code.

An invariant failure — inconsistent source mapping, an unclosed stitched mesh,
an impossible shell relation, a failed exact predicate, or a non-positive
signed volume on a supposedly non-empty oriented result — is not ordinary
uncertainty. `Verify` returns that error and no report. It MUST NOT hide a broken
evaluator as `Suspect`.

### 7.2 Context and work

`Verify` passes its context through the entire read-only path. After document
and option validation, return `ctx.Err()` unchanged once canceled.

Check cancellation:

- before and after every §5 phase;
- in every O(facet²), O(edge²), component, shell, and containment loop;
- at every branch-and-bound/refinement subdivision;
- while cutting facets and while building/stitching adjacency.

Leaf exact predicates remain context-free. Their callers use a shared bounded
work counter and check `ctx` at least once per 256 candidate operations. Phase
boundaries check unconditionally. This gives prompt cancellation without
threading context through arithmetic primitives or changing deterministic
geometry decisions.

`UnionContext` / `CutContext` / `IntersectContext` propagate their caller
context through evaluation and faceted-body construction. `Union` / `Cut` /
`Intersect` supply `context.Background()` as compatibility wrappers.

The mesh fallback has no constant-work promise. `meshBoolean` checks every
pair of operand facets, using the facet boxes to skip exact predicates only
after each pair-box check. Its pair-box work is therefore the number of facets
in A multiplied by the number in B. `Verify` can pay that cost for every
proven-solid pair that box, analytic, containment, and equality proofs leave
unresolved. Verification §1.2 owns the public deadline guidance. The existing
context is the public control for this work; do not add a separate work-limit
or progress API.

## 8. Interference tolerance gate

An emitted row makes the report `Interfering` regardless of measurement
coarseness. The row's measurement is still judged because nothing in a report
is exempt.

Use verification's ordinary scalar rule:

```text
Bound <= rel × Ref
Ref = max(abs(Volume.Value), Quantum)
Quantum = δ × (AreaA + AreaB)
δ = 1e-9 × Dpair
```

`Dpair` is the diameter of the union of the two operand point sets. `AreaA` and
`AreaB` are the operands' own surface-area readings. The overlap boundary lies
on operand skins, so their summed area bounds the surface displaced at the
noise floor.

Keep this reference pair-local. Do not use either body's volume, either body's
box volume, the document diameter, or the transient intersection's faceted
area. The transient area can omit or add facets exactly where the approximation
is uncertain; the operand skins are the proof reference.

The gate affects the pair's trust finding, but status precedence remains
`Interfering > Suspect`. A caller can inspect the row's `Exactness` and `Bound`
even though `Trustworthy()` is already false because the bodies overlap.

## 9. Coverage and refusal

The proof path is capability-based, not operation-history-based:

| Operand capability | Available path |
|---|---|
| bounds | box disjointness |
| analytic boundary model + certified casts | clearance/contact/full-containment proof |
| tessellation accepted by mesh boolean | read-only intersection volume |
| neither boundary model nor tessellation | undecided → `Suspect` |

A prism, a filleted/chamfered prism, and a tube share `prismPayload` and use the
same paths. A cup uses box proofs and the read-only boolean once its shipped cup
tessellation is accepted; analytic clearance/full containment remain staged
until `bodyGeom` supports `cupPayload`. A faceted boolean result uses its held
mesh and carried bounds. A revolve uses analytic clearance/full containment
where supported; read-only intersection stays staged until revolve
tessellation lands.

Do not claim curved coverage from planar facets alone. The hidden-tangency gate
and every operand chord allowance remain mandatory. If a curved pair lies
inside those bounds and no analytic proof settles it, the pair is `Suspect`.

## 10. Required tests

Every increment asserts geometry and report state, not only successful return.

### 10.1 Non-mutation and order

- Snapshot recipe encoding, live body pointers/order, and retired state before
  `Verify`; assert exact equality afterward on overlap, disjoint, undecided,
  error, and cancellation paths.
- Build at least three bodies; assert `Interferences` and `Clearances` follow
  document pair order, independent of proof path.
- Call `Verify` repeatedly; assert byte-stable report ordering and identical
  measurements.

### 10.2 Relation

- separated boxes → no interference, `Sound` without asked clearances;
- certified touching caps → no interference, optional `Exact` zero clearance;
- transversal overlap → one interference row with `Value - Bound > 0`;
- nested single-lump body → row volume equals contained body volume exactly as
  a `Measurement` value;
- a single-lump body whose **void** shell lies inside the other body, whose
  boundaries are disjoint → no containment reuse: the outer body's void
  witness is proven inside, so the pair reports only what the read-only
  Boolean proves, else `Suspect`. A hole-free fixture cannot exercise this;
- fully or partially contained multi-lump faceted body → no analytic
  containment reuse; read-only Boolean proves the complete overlap or the pair
  stays `Suspect`;
- held overlap at or below its bound → no fabricated row, `Suspect`;
- identical/coincident and broad coplanar pairs stay `Suspect` until their
  stated increments land, then report the proved volume.

### 10.3 Bounds and errors

- perturb analytic fixtures within the carried operand chord allowances; true
  overlap remains inside `Value ± Bound`;
- all-planar exact-rational case reaches zero volume bound only when its float
  conversion is exact;
- an expected unsupported/contact/empty outcome yields a report with
  `Suspect`;
- a forced closure/source/invariant failure makes `Verify` return an error;
- cancellation during tessellation, facet-pair classification, cutting,
  stitching, and containment returns `context.Canceled` or
  `context.DeadlineExceeded` and leaves the document unchanged.

### 10.4 Gate

- judge `Interference.Volume` against pair diameter and summed operand areas;
- never use document size or transient intersection area;
- a row beyond tolerance remains `Interfering` by precedence while preserving
  its approximate measurement.

## 11. Increments

Each row is a PR-sized stage. An unanswered verification question reads
`Suspect`; no row claims work that has not landed.

| PR | Lands | Still `Suspect` |
|---|---|---|
| 1 | four-way pair relation; read-only `evaluateBoolean`; shared shell audit and rational volume/bound helper; transversal intersections for operands the mesh boolean accepts; stable ordering; context propagation | containment without a measurable mesh intersection; coincident/broad-coplanar arrangements; operands without tessellation |
| 2 | strict full-containment certificate for shipped single-lump analytic payloads; whole-contained-body volume reuse | all multi-lump containment unless the Boolean measures the complete intersection |
| 3 | exact structural equality certificate for analytic payloads, reusing the first equal body's volume | harmless alternate record spellings; non-identical broad coplanar overlap |
| 4 | coplanar breadth in the mesh classifier: classify material sides over every positive-area coplanar patch, keep crossing/overlap patches, and retain pure opposite-side contact as touching | unsupported curved operands and unresolved curved tangencies |
| 5 | curved read-only intersection coverage after revolve tessellation, with chord bounds and the hidden-tangency refusal intact | contact or overlap whose proven interval still admits both zero and positive volume |

## 12. Decisions

- Keep the existing public `Interference{A, B, Volume}` shape. Add no option,
  witness, selector, or recipe step.
- Keep public booleans consuming. Share only their read-only geometry evaluator.
- Reuse the boolean's exact rational volume and symmetric-difference bounds.
- Require strict positive lower overlap volume except under a certified
  containment or equality set identity.
- Treat empty, contact, unsupported, and coarse results as undecided; propagate
  internal failures.
- Preserve document pair order.
- Stage all multi-lump analytic containment; use read-only intersection and
  never report a partial sum as total overlap.
- Check cancellation at phase boundaries and every 256 expensive candidates.

No design decision remains open in this increment. Revolve tessellation,
analytic cup boundary support, and broader contact certificates are explicit
prerequisites or staged coverage, not unresolved contract choices.
