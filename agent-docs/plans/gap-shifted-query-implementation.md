# Gap-shifted query implementation tasks

Status: unexecuted task plan. Source baseline: `5ed2045c6`.
This plan is non-authoritative. [The design](../gap-shifted-query-design.md) and its named owners win.
Use [shared execution rules](../exact-arithmetic-task-plans.md) before starting work.
No implementation or proof review described below has been completed by writing this plan.

## Navigation

This router is navigation, not authority.

| Question | Read |
|---|---|
| What can the implementer do immediately? | Task 1 and Task 2 |
| What prevents enabling point queries? | Task 3 |
| Which code follows an accepted proof? | Task 4 through Task 8 |
| What happens to Section? | Task 9 |
| What establishes completion? | Task 10 and Acceptance |

## Scope and dependency order

Implement only these conditional implications from an unchanged exact first input `A`:

| Public operation | Exact first-input answer | Permitted output answer |
|---|---|---|
| Union | Inside | Inside |
| Cut | Outside | Outside |
| Intersect | Outside | Outside |

Every other combination must miss this new route and retain the existing fallback.
Do not return OnBoundary through this certificate.
Do not enable original-operand replay for the changed second input.
Do not change `boolchain.Expandable`, regrouping, Section admission or measurement bounds.

Execute Tasks 1–3 before adding permanent certificate infrastructure.
Tasks 4–8 are blocked until Task 3's proof review accepts a concrete producer relation.
Task 9 is a separate blocked Section investigation; point-query acceptance does not unblock it.
Task 10 records either completed capability or a precise prerequisite stop.

## Task 1 — Reproduce the current public case

Read these existing symbols before editing:

- `apitest/point_containment_history_test.go`: `TestPointLocationHistoryRefusals`.
- `point_containment.go`: `Body.LocatePoint` and `locateExact`.
- `internal/pointquery/history_location.go`: `LocateBoolean` and `historyWalk.locate`.
- `internal/boolchain/chain.go`: `Expandable`.

Run and save the baseline:

```sh
go test ./apitest -run '^TestPointLocationHistoryRefusals$' -count=1
```

This command currently passes because the test expects Undecided. Its passing result is not a new capability.
Use its existing `gap-shifted union` subtest as the real fixture; do not substitute a fabricated mesh:

1. Create `doc := decad.New()`.
2. Build `plate := boxBody(t, doc, 0, 0, 30, 30, 4)`.
3. Call `decad.Copy` on the plate with `WithCopyRotation` about `(15,15,2)`, axis `(1,0,0)`, 45 degrees.
4. Union the original plate and the returned full-width copy in that order.
5. Require `tookMeshPath(joined)` and the accepted `OpDiagGapShiftApplied` diagnostic.
6. Read its volume and require a positive bound; never substitute a zero bound to simplify classification.
7. Query `(1,1,3.999)` and `(1,1,4)` with the existing `requireLocations` helper.

Current expected answers are Undecided for both points.
After accepted implementation, only the first expectation changes to Inside.
The top-face point remains on the ordinary fallback; do not require a new boundary answer.

The first point has strict interior coordinates in `[0,30] × [0,30] × [0,4]`.
That box membership is the independent oracle, not the result's parity or a diagnostic.
Reuse existing `boxBody`, `tookMeshPath` and `requireLocations` helpers in the same test package.
Their implementations, not this plan, own Sketch construction and query chord tolerance.

Completion evidence: baseline log plus recorded point answers, shift diagnostic and scalar volume bound.
Failure response: stop if the producer no longer takes the retry or fails; report the changed boundary before planning code.

## Task 2 — Capture one accepted evaluator attempt

Keep this work confined to a temporary prototype in the implementation worktree.
Use a Go test overlay or temporary test instrumentation; remove it before committing production changes.
Do not add global hooks, exported debug APIs, mutable cross-call caches or persistent exact facet arrays.

Read the actual data flow:

| Existing file and symbol | Capture point |
|---|---|
| `internal/booleaneval/eval.go`, `evaluate` | `ma` and `mb` after their VerifyAll tessellations |
| Same function | `ma` immediately before `PrepBoolMesh`; record source identity before preparation |
| `internal/booleaneval/shift.go`, `shiftOperandMesh` | Accepted `pieceShift` map, decision/charged second meshes |
| `evaluate` | `bmA`, `bmB` after preparation, before `MeshBoolean` |
| `evaluate` | Exact `kept []meshbool.KeptFacet` after `MeshBoolean` succeeds |
| `evaluate` | `stitched` after `StitchFacetsContext`, and accepted `EmbeddingReach` |
| `evaluate` | Final payload charges and `VolumeReading` after its audit |

Keep only the accepted retry in the evidence. Failed earlier attempts cannot certify the output.
Connect every captured value to the same two source bodies, operation and call; matching counts is insufficient.

Require these first-target facts on actual `ma`, before `PrepBoolMesh`:

- `BoundaryOK` and `SymDiffOK` are true.
- `Bound == 0` and `VolSymDiff == 0` exactly.
- The exact record class is one `pointquery.LocateRecord` already understands.
- The record and mesh name the same original first input in the same world frame.

Record `CoordBound`, per-vertex and per-face bounds too; unexpected contradictions stop the investigation.
Do not treat zero numeric fields without the corresponding validity flags as proof.
The first-target gate excludes a chorded circular target even when its analytic source is exact.

For the crossed-plate fixture, demonstrate the acceptance point is inside the original first input.
Use `pointquery.LocateRecord` and, separately, existing exact held-mesh parity for `ma`.
`meshbool.MeshParityContext` accepts float vertices and triangles; do not round exact `KeptFacet.V` to feed it.
Capturing exact kept facets is for the next task's proof audit, not permission to invent a rational parity kernel.
If inspecting their occupancy needs an absent exact consumer, record that missing consumer as a blocker.

Record final stitched held parity separately from membership in the body's denoted region.
A favorable sampled point cannot establish the whole unchanged-target relation.
Preserve all existing geometry, retry selection, charges and warning production during this prototype.

Completion evidence: bounded scalar trace and a source-to-preparation-to-kept-to-stitch call map.
Report counts, origin references, validity flags, bounds, point answers and at most a capped facet sample.
Failure response: stop on unexpected geometry movement, failed target gates, unsupported exact inspection or resource failure.

## Task 3 — Obtain the producer proof before coding certificates

Write `docs/handoff-gap-shifted-target-proof.md` in the implementation worktree for review.
Use the repository agent-document/prose pre-reads before writing it.
This is a local engineering handoff; do not open a GitHub issue or comment.

The implementer must fill the following evidence fields, not invent the missing theorem:

1. Name the exact first-input region supplied by its record and its zero-bound tessellation.
2. Identify precisely what solid the prepared changed second mesh denotes.
3. Trace `MeshBoolean`'s kept-facet selection to the regularized operation over those two solids.
4. Trace exact welding/conforming and rounding to the public result's existing denotation contract.
5. Enumerate every allowed additional mechanism and show its effect on the first-input implication.
6. State why the resulting implication applies to the denoted body, not merely the held stitched mesh.
7. Name concrete admission predicates and their source symbols for each necessary condition.
8. Include the one real fixture, logs, known negative branches and unresolved facts.

Reject these substitutes for that evidence:

- `ShiftSummary`, a warning, an operation name or a caller-set boolean alone.
- Zero target bounds without a valid occupied-volume and source correspondence proof.
- `StitchFacetsContext` closure as proof of original-input occupancy.
- `RefuseRimPastPair` as proof that rounding preserved an input-derived set implication.
- `AuditFacetedMesh` validity or `VolumeReading`'s charged interval as local membership proof.
- Small sampled residuals, equal volume/counts or successful held parity at one point.

Request a design/proof review of this bounded artifact from the supervising agent.
The review must supply a written acceptance naming the admitted producer path and predicates, or a rejection reason.
This is a proof gate for future execution; this task plan does not claim that acceptance exists.

If accepted, first update the owning proposal's Producer certificate section with the exact relation and predicates.
Only then continue to Task 4. If the review changes the certificate shape below, update this plan before coding it.
If rejected or incomplete, stop capability implementation and retain current query behavior.
Do not repair unrelated tessellation, stitching or dependency defects under this task.

Completion evidence: reviewed producer proof, exact admission checklist and one real captured component boundary.
Failure response: deliver the handoff and stop; all following point-enablement tasks remain blocked.

## Task 4 — Define one immutable point-only record

Execute only after Task 3 accepts the unchanged-target relation.
The following new names/signatures are proposed; none exist at the baseline.
Add `internal/facetedrecord/shifted_point.go` and its adjacent test file.
Add `ShiftedPoint *ShiftedPointRecord` to `facetedrecord.Payload` in `payload.go`.

Store only constant-size immutable data:

```go
type ShiftedPointRecord struct {
    operation meshbool.OperationKind
    output, first, second provenance.Ref
    placement r3.Transform
}
```

Keep fields private. Store full `provenance.Ref` values, including roles.
The initial placement is `r3.Identity()`; nonidentity descendants are outside this slice.
Store no `*topo.Body`, document, mesh, mutable slice, context, map or query result.

Proposed metadata operations:

```go
func NewShiftedPointRecord(operation meshbool.OperationKind,
    output, first, second provenance.Ref) (*ShiftedPointRecord, error)
func (r *ShiftedPointRecord) Matches(operation meshbool.OperationKind, output, first, second provenance.Ref,
    placement r3.Transform) bool
```

The constructor accepts only Union, Cut or Intersect and requires each reference's producer to be positive
and role to equal `provenance.RoleBody`. Require the new output reference to differ from both inputs.
`Matches` rejects nil/zero records, stale references, reversed inputs and nonidentity placement.
Use the existing enum from `internal/meshbool/identity.go` throughout the record, evaluator proof and reader.
Validate it with `switch operation { case meshbool.OpUnion, meshbool.OpCut, meshbool.OpIntersect: ... }`;
every other value is invalid. Pass the typed value from `BooleanProducer` directly; do not convert it to an int
or define another enum. No numeric mapping or serialization is needed.
The dependency direction already exists: `facetedrecord/point_plane_scale.go` imports `meshbool`, and meshbool's
production code does not import `facetedrecord`, `booleaneval` or `pointquery`. Preserve those directions.
`booleaneval` may import both lower packages; neither lower package may import `booleaneval` for this record.
These methods validate metadata only. Their call is not the geometric proof from Task 3.
Document that only the accepted producer path can invoke the constructor to issue usable evidence.
Do not add a generic `geometryWasUnchanged bool` argument or a setter that can upgrade an existing payload.

Unit tests compare scalar fields through behavior, not full graph equality.
Test each mismatched reference independently, operation mismatch, reversed order, zero record and placement change.
Completion evidence: focused new-record tests pass; a record cannot match a different output.
Failure response: stop on a dependency cycle or identity ambiguity; revise the record design with its owner.

## Task 5 — Mint evidence only on the accepted original output

Add the accepted-path evidence in `internal/booleaneval/eval.go`; keep it private until final result validation.
Use the exact admission predicates approved in Task 3, including pre-preparation target checks.
Retain an unforgeable-by-diagnostics route distinction through the accepted attempt's local control flow.
Do not publish evidence from a failed attempt or recreate it by reading a warning after success.

Propose a new `FirstOperandProof` type in `booleaneval`, with private `meshbool.OperationKind` operation and
ordered input references,
and a new optional `Evaluation.FirstOperandProof` field. Its zero value must bind nothing.
Its proposed `Bind(output provenance.Ref) (*facetedrecord.ShiftedPointRecord, error)` method freezes the
validated metadata using Task 4's constructor. Only the reviewed successful path initializes its private state.
Review acceptance must settle any additional proof data this type needs before implementing that API.

Suggested implementation placement, conditional on the reviewed relation:

1. Save target eligibility and original input references before `PrepBoolMesh`.
2. Require accepted shifts on operand 1 only; reject unknown mechanisms or source substitutions.
3. Run all existing evaluation steps and propagate their errors unchanged.
4. Exclude positive embedding reach and special point-cone/cap substitution paths.
5. After the final audit, volume reading and cancellation check, expose a typed accepted-path result.
6. In `internal/booleandispatch/dispatch.go`, `Body`, run `BuildWithProof`, warning aggregation and all existing
   accepted-output hooks. Keep the proof unbound until those hooks and the final context check succeed.
7. Call `Bind(body.Origin)` exactly once at that point. Copy the built `facetedrecord.Payload`, set its
   `ShiftedPoint` field, and assign the updated value back to `body.Payload` before returning it.
8. Require the later committed public producer and ordered inputs to match when reading the record.

Do not bypass `BuildWithProof`, warning aggregation, `RecordConeToe`, `LowerSupport` or the final context check.
If a post-build hook changes accepted geometry, it must prevent this certificate; a code name alone does not suffice.
Private pattern/retry steps without a matching public two-input producer cannot pass reader admission.
Refused operations must leave document/history unchanged through the existing atomic commit path.

Build the single crossed-plate Union and query the acceptance point through a temporary internal point reader.
Do not expand to Cut/Intersect or lifecycle matrices until this actual producer-to-consumer case succeeds.
Assert its prior geometry measurements, diagnostic and bounds are unchanged; assert Inside from the new record.
Completion evidence: one real original output and reader agree with the independent box-membership oracle.
Failure response: stop at the first failed real case; repair within accepted scope or return the proof issue to review.

## Task 6 — Add a separate point-only reader

Add proposed `internal/pointquery/shifted_location.go` and adjacent tests.
Use `boolchain.Source` only to obtain the committed producer and retained inputs; do not call `Expandable` to bless it.
Proposed entry point:

```go
func LocateShiftedFirst(ctx context.Context, p r3.Vec, n *topo.Body,
    src boolchain.Source) (Location, bool, error)
```

Perform checks in this order:

1. Check context and create one `proofbound.WorkBudget`.
2. Require a solid faceted node with the new record and its original identity placement.
3. Read the committed producer; require exactly two ordered inputs in the same document.
4. Validate record operation/output/input references against that producer and payload.
5. Require the expected gap-shift warning for operand 1 and reject unknown operation-change mechanisms.
   A chain-allowance warning with `ChangeUnknown` is the only additional no-geometry-change warning allowed.
   These warning checks can reject a record; they can never create or admit one without producer evidence.
6. Call `LocateRecord` on input 0 at the unchanged world point.
7. Return Inside only for Union/Inside; return Outside only for Cut-or-Intersect/Outside.
8. Poll cancellation before any decided return. Every other answer returns `Undecided, false, nil`.

Do not recurse through the changed input or use result tags to refine target boundary answers.
Missing/mismatched evidence is a route miss; preserve ordinary mesh fallback.
Never turn cancellation into a route miss.

In `point_containment.go`, `locateExact`, keep existing tolerance validation before either fast path.
Try ordinary `LocateBoolean` first for a faceted result, then this new route only when it misses without error.
Call the public result's mesh fallback once through unchanged `Body.LocatePoint` after all exact routes miss.
Do not add the record to distance queries, `sectionhistory` or `boolchain`.

Completion evidence: three allowed table entries decide; every other entry misses; existing Boolean algebra tests pass.
Failure response: stop on changed legacy answers, accepted invalid tolerance or widened expansion eligibility.

## Task 7 — Clear evidence on every rewrite

Add explicit resets beside existing dropped computational records in:

- `internal/facetedrecord/placed.go`, `Payload.PlacedWith`, immediately after `next := fp`.
- `internal/facetedrecord/point_plane_scale.go`, its `next := fp` similarity copy.

Search all other faceted payload copies and rewrites before concluding the audit:

```sh
rg -n 'next := fp|:= fp|ExactCopyMoments|Payload\{' internal/facetedrecord
rg -n 'facetedrecord.Payload|facetedPayload' internal/retainedblend internal/booleandispatch
```

Fresh generic Boolean payloads start without inherited certificates and can issue only their own new evidence.
Copy, identity Copy, Transform, positive scale, retained blend and every other rewrite must clear the old record.
Reader reference binding is a second gate; it does not replace producer-side clearing.
Retirement alone is different: the original body and its original consumed inputs remain readable.

Completion evidence: rewrite tests observe no usable old record; original retired body still answers through its record.
Failure response: stop on an unclassified copying path instead of assuming the record is harmless there.

## Task 8 — Expand focused tests after the first real case

Add proposed `TestPointLocationShiftedFirstOperand` to `apitest/point_containment_history_test.go`.
Retain baseline helpers and use real public producers for every acceptance test.

| Fixture | Required assertion |
|---|---|
| Full-width crossed plates from Task 1 | Shift diagnostic, positive bound and Inside at `(1,1,3.999)` |
| Same Union at `(1,1,4)` | New route misses; preserve baseline fallback result |
| Fresh 10 mm target; translated 6 × 6 × 5 tool at `(2,2,5)`; Cut | Real shifted result; Outside at `(-0.001,3,3)` |
| Same fresh inputs; Intersect | Nonempty shifted result; same exterior point is Outside |
| Target-boundary/input-Undecided cases | New route never invents Inside/Outside/OnBoundary |
| Chorded circle as first operand | Pre-preparation zero-bound gate withholds evidence |
| Any positive embedding-reach or unknown mechanism | Evidence absent or unusable |
| Copy/Transform/ScaledAt descendants | Old record does not decide any point |
| Retired original source/result | Original supported query remains readable |

Use `gapBoxes(t, 2, 2, 5, 6, 6, 5)` from `apitest/boolean_gap_tolerance_test.go` for the flush cases.
Call it anew for Cut and Intersect because each successful Boolean consumes its inputs.
Outside answers can also come from the mesh; internal tests must show the intended certificate route executed.
Do not add a public debug accessor merely to distinguish routes.

Add internal table/lifecycle tests in the new record and reader test files, with proposed descriptive test names.
Include swapped Cut roles, wrong document/producer, unknown warning, pre-cancellation and concurrent reads.
Assert scalar answers, individual references and capped diagnostics; never `require.Equal` on body or mesh graphs.
Use existing `TestPointLocationRejectsInvalidRequests` to preserve public request validation.

Focused commands after the proposed test names exist:

```sh
go test ./internal/facetedrecord -run '^TestShiftedPointRecord' -count=1
go test ./internal/pointquery -run '^TestLocateShiftedFirst' -count=1
go test ./internal/pointquery -run '^TestLocateBoolean' -count=1
go test ./apitest -run '^TestPointLocationShiftedFirstOperand$' -count=1
go test ./apitest -run '^TestPointLocationRejectsInvalidRequests$' -count=1
go test -race ./internal/pointquery -run '^TestLocateShiftedFirst' -count=1
```

The first, second, fourth and last filters are proposed names; an empty matching run does not verify anything.
Completion evidence: named tests execute and pass, prior scalar bounds remain enclosed, no legacy route widened.
Failure response: stop on behavior/proof failure; do not relax expected answers or reduce coverage to make the run green.

## Task 9 — Keep Section blocked pending a separate local proof

Do not connect the point-only record to `booleanHistorySection` or `internal/sectionhistory.Section`.
Their full source-set equality and exact Sketch reconstruction obligations are stronger.
Create a separate local handoff only if this follow-on is selected after the first point case succeeds.

Reproduce the existing `gap-shifted result` fixture in `TestSectionMeshBooleanHistoryRefusals`:

1. Build `historyCube(t, doc)`, the `[0,10]³` cube.
2. Cut `boxBodyAtZ(t, doc, 5, 2, 15, 8, 5, 5)` and require its real shift diagnostic.
3. Keep the existing `z = 7` exact-history refusal.
4. Investigate `historyLevel(t, 2)`, below the tool's lower cap; the candidate section is the full 10 mm square.

The required producer artifact is a local-set-equality certificate for a neighborhood of that entire plane.
It must associate all changed pieces with their original sources, bound their entire accepted motion and
approximation, and prove stitching introduces no unrecorded change near the plane.
Per-piece support boxes need strict separation from the plane; a point-only witness or global volume bound is insufficient.
Any box touching the plane is a miss. Do not infer separation from a few sampled vertices.

Ask for review of that local producer relation before defining its permanent record or reader.
Without accepted relation, record the blocker and leave `Body.Section` on current refusal/outline behavior.
After acceptance, first feed the real source through existing exact section and Sketch reconstruction machinery.
Use `sectionclip.ReconstructExactSections`; every partial Sketch fragment must carry upstream `TExact`.
Keep current exact-junction and zero-cut-displacement requirements from the section owner.
An uncertified fragment remains refused; do not project or fit it to make the section reconstructible.

The first acceptance case must return one hole-free square, feed its actual profile to a 1 mm `Extrude`,
and publish exactly 100 mm³ with zero bound. Verify the generated solid's boundary using `Tessellate(VerifyAll)`.
Keep `z = 7` refused, then stop before curved sections or a larger fixture matrix.
This plan supplies investigation and acceptance tasks; it does not claim the required local certificate exists.

Completion evidence: either reviewed local proof plus real Section→Extrude evidence, or a precise unimplemented prerequisite.
Failure response: keep Section disabled; a failed prerequisite does not authorize a wider reconstruction repair.

## Task 10 — Update owners and report the actual state

Update current-state owners only for behavior implemented and proven:

- `agent-docs/point-containment-design.md`: new point-only producer admission and three limited implications.
- `agent-docs/operation-history-design.md`: certificate lifetime, bindings and committed-source lookup.
- `agent-docs/gap-shifted-query-design.md`: implemented class versus still-blocked follow-ons.
- Matching query qualification in `agent-docs/missing-features.md`; general shifted Section gap stays until Task 9 passes.
- Public `LocatePoint` comment/reference only when its description needs the new qualified class.
- Applicable layout rows for new internal files; register new public tests in the apitest shard manifest.

Run focused tests above and the repository shard/layout checks required by shared execution rules.
Let remote CI establish the full-suite result when a later implementation is pushed.
Do not call the capability complete while the first end-to-end producer proof or any required check is pending.

## Acceptance

- The unchanged-target relation has written review acceptance and its exact predicates are implemented.
- One real crossed-plate producer reaches the new consumer before broader test expansion.
- Only the three permitted strict point implications decide through the new record.
- Bounds, geometry, diagnostics, ordinary mesh fallback and complete Boolean expansion are unchanged.
- Record lifetime, invalid requests, cancellation and concurrent reads have focused evidence.
- General Section remains disabled unless its independent local proof and actual reconstruction consumer pass.
- Any prerequisite stop names the missing evidence and leaves unsupported behavior conservative.

## Plan validation

The source paths and existing symbols above were checked against `5ed2045c6`.
New record/reader files, function signatures and their focused test names are explicitly proposed.
This planning task ran only the existing point-history refusal baseline; its log is
`.tmp/plan-shifted-query/baseline.log`. It establishes today's conservative result, not any proposed proof.
