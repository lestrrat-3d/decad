# Boolean BRep point-query implementation tasks

## Status and authority

This plan is unexecuted and non-authoritative. It was checked against source revision `5ed2045c6`.
The [BRep point-containment design](../brep-point-containment-design.md) owns admission and proof requirements.
Follow [shared execution rules](../exact-arithmetic-task-plans.md) before starting any task.
If source or design disagrees with this plan, stop and report the named difference.

Implementation is blocked at Task 1. The real whole-line A7 fixture carries a nonzero section displacement.
Its current design requires zero face displacement. Do not invent another fixture, relax that gate, or erase its charge.
Tasks 3–9 are conditional instructions; they are not permission to continue past Tasks 1–2.

## Navigation

This router is navigation, not authority.

| Question | Section |
|---|---|
| Why can implementation not start? | Task 1: reproduce the failed admission gate |
| What must the reviewer resolve? | Task 2: hand off the source-set proof |
| What record and binding are proposed? | Task 3: add the immutable identity record |
| Where may evidence be attached? | Task 4: attach evidence on the approved route |
| How should the query consume evidence? | Task 5: add point-only history eligibility |
| What real result must improve first? | Task 6: complete one producer-to-query checkpoint |
| When may other constructions be enabled? | Task 7: widen one approved producer at a time |
| What must stay refused? | Task 8: verify lifecycle, failure and concurrency |
| What completes the work? | Task 9: update contracts and run focused checks |

## Scope and prerequisites

- Read `CLAUDE.md`, `agent-docs/layout.md`, `agent-docs/testing.md`, and the linked design first.
- Read `point-containment-design.md`, `general-boolean-design.md`, and `operation-history-design.md` before code edits.
- Read `evaluator-design.md` before changing topology, evaluator dispatch or node evidence.
- Read `sketch-seam-design.md` before reading any partial arrangement edge.
- Read the user's Go and debugging pre-reads before writing probes or implementation.
- Work in the implementation worktree selected under the shared rules; preserve these planning files.
- Use the existing root public API and `internal/pointquery` predicates. Add no public API.
- Leave `boolchain.Expandable`, Section and Boolean regrouping eligibility unchanged.
- Leave mesh fallback, source geometry, bounds, and existing producer refusals unchanged.
- Treat all new symbol names below as proposed. Existing names and paths are identified explicitly.

## Task 1: reproduce the failed admission gate

Prerequisite: none beyond the pre-reads. Make this the first real operation, before infrastructure or a case matrix.

Use the `crossing-corners` geometry of existing
`apitest/crossing_blind_pocket_test.go`, `TestCrossingBlindPocketOnThroughHole`:

1. Create a document with `decad.New(decad.ExactOnly())`.
2. Sketch and extrude the plate `[-10,10] × [-10,10] × [0,10]`.
3. Sketch and extrude a through tool `[-4,0] × [-2,2] × [0,10]`.
4. Public `Cut(plate, through)` produces the holed prism.
5. Sketch and extrude the pocket `[-1,3] × [-1,3] × [6,10]`.
6. Public `Cut(holed, pocket)` produces the A7 BRep.
7. Inspect only its class, face count, `SectionDelta()`, `AxialDelta()` and presence of `Stack`.

Construct each box from a fixed Sketch rectangle on `World.XY()` or a real offset plane.
Use `World.CreateOffsetPlane`, `CreateSketch`, `CreateRectangle`, `Fix(rect.A)`, `Solve`, then `Extrude`.
Use millimetre `Distance` and `Along`; do not replace either public Cut with handwritten payloads.

The planning probe used a root-package test supplied by a Go overlay so it could read `part.n.Payload`.
Its expected-zero assertion FAILED before any descendant operation ran:

```text
payload = breprecord.Payload
face count = 15
SectionDelta() = 7.105427357601007e-15
AxialDelta() = 0
Stack != nil = false
```

That is a failed admission experiment, not an implementation test pass.
The prior fixture's `Exact` volume does not establish zero section displacement or source-set equality.
The failed probe and log are investigation evidence under `.tmp/plan-brep-point/`; the recipe above is durable.

For a repeatable diagnostic, use proposed name `TestBrepPointA7AdmissionDiagnostic` in a scratch root test.
Assert the class and observed nonzero charge; report only the five scalars/booleans above.
Keep temporary probes out of tracked test/shard files.

Completion evidence: save the diagnostic command, source revision and five bounded readings.
Failure response: if construction fails or readings differ, stop with that result; do not search for easier inputs.
On the documented result, STOP at Task 2. No certificate or point-query changes are allowed yet.

## Task 2: hand off the source-set proof

Prerequisite: Task 1's failed zero-displacement gate. This task prepares a bounded reviewer handoff, not a repair.

Write a local evidence note containing the fixture, readings and the following source obligations.
Use the exact filenames and symbols below so the reviewer can inspect the disputed boundary directly.

| Existing producer or consumer | Required reviewer answer |
|---|---|
| `internal/prismboolean/pair.go`, `TryCrossingBlindCut` | Which exact input sets are preserved by admission and interval clipping? |
| `internal/stackedunion/brep.go`, `ubBuild.scene`, `charge`, `cutSlabRegion` | Do selected Sketch cells define the original difference or an approximating replacement? |
| `internal/stackedunion/brep.go`, `ubBuild.run` | What does `AbsSumUpper(b.cutDelta, b.geom.Allow())` enclose? |
| `internal/stackedbrep/record.go`, `Engine.junction`, `FaceRecord`, `UnitSegments` | Which canonical vertices preserve a source carrier exactly, and which only approximate it? |
| `internal/breprecord/build.go`, `Build` | Is the represented solid defined by source regions or by the emitted face records? |
| BRep volume, centroid and tessellation readers | Which denoted set do their published intervals enclose? |

Ask for one of two explicit outcomes, with the source proof attached:

- Keep the zero-displacement design gate. Identify a useful admitted class and its real producer-to-query fixture.
  The current A7 fixture remains excluded; the lesser implementer does not pick a replacement independently.
- Approve a source-defined identity extension. Specify its immutable record, exact scene/canonicalization conditions,
  represented-set semantics and required consumer-enclosure changes. Preserve every existing charge.

The reviewer must update the owning design with an `Approved producer identities` section before implementation.
That section must name the exact producer branch, input gates, output gates, set identity, and first fixture.
It must state whether nonzero charges bound only a representation of that same set or a different accepted set.
A successful mesh audit, operation name, zero volume bound or residual check cannot replace this answer.

Do not implement that larger representation change under this task plan if the reviewer selects it.
Its new consumer work needs its own scoped implementation tasks before Tasks 3–9 can be used.

Completion evidence: a local handoff plus the reviewer's source-backed design decision and approved first fixture.
Failure response: without that decision, report this plan blocked and stop. Do not add unused certificate scaffolding.

## Task 3: add the immutable identity record

Prerequisite: Task 2 approves the existing zero-displacement route and supplies a first fixture.
The sketches here are conditional on that outcome. A different design decision requires revising them first.

Create proposed `internal/prooflineage/boolean_set.go` and `boolean_set_test.go`.
Keep the representation below the topology dependency boundary:

```go
type BooleanKind uint8
const (
    BooleanInvalid BooleanKind = iota
    BooleanCut
    BooleanUnion
    BooleanIntersect
)
type BooleanSetIdentity struct {
    kind BooleanKind
    output, target, tool provenance.Ref
}
func NewBooleanSetIdentity(kind BooleanKind, output, target, tool provenance.Ref) (*BooleanSetIdentity, error)
func (r *BooleanSetIdentity) Matches(kind BooleanKind, output, target, tool provenance.Ref) bool
```

Use private fields and copied plain `provenance.Ref` values. Return no mutable slices or topology pointers.
Reject unknown kinds, nonpositive producers, empty roles, and an output reference equal to either input.
`Matches` must return false on nil and compare the full references, including roles and original Cut input order.
The constructor validates binding structure only. Its call site must supply the approved geometric proof.

Do not import `meshbool`, `topo`, `breprecord` or root `decad` into `prooflineage`.
Map the three operation kinds explicitly at the evaluator/query boundary; never cast unrelated enums.
This avoids extending topology's transitive imports above its permitted dependency set.

Add proposed `BooleanSet *prooflineage.BooleanSetIdentity` to existing `internal/topo/body.go`, beside evidence fields.
Update the exact field-name expectation in existing `internal/topo/floor_test.go`, `TestBodyMovedFields`.
Preserve `TestTopoDependsOnlyOnFloor`; do not widen its allowed dependency table to permit an accidental import.

Test kind/ref rejection, role mismatches, swapped Cut inputs, nil matching and independent records with equal values.
Compare booleans, integers and small references only. Do not compare complete topology nodes.

Completion evidence: focused new record tests and both topology-boundary tests pass.
Failure response: an import-boundary failure requires moving plain data downward, not relaxing the boundary test.

## Task 4: attach evidence on the approved route

Prerequisite: Task 3 passes and Task 2 identifies a concrete approved producer branch.

In existing `internal/booleandispatch/dispatch.go`, replace only that route's `return found(AnalyticBody(...))`
with explicit build, error handling, approved-evidence validation, attachment and return.
Use original arguments `a`, `b` and the built node's `Origin`; preserve caller order even if a symmetric builder swaps.
Check cancellation before returning. Never attach in `found`, `breprecord.Build` or generic analytic success handling.

For the original zero-displacement design, check all these conditions before attachment:

- Output payload is a solid `breprecord.Payload` with no stack/loop-band/shell substitution outside approval.
- `SectionDelta() == 0`, `AxialDelta() == 0`, and all actual face fields satisfy the design's zero-displacement gate.
- Source and result maps are exactly those named by the approved producer proof.
- Neither source has an unapproved band, displacement, unknown geometry change or stale source view.
- The successful construction branch has the approved source-set identity; output field checks alone cannot prove it.

The current A7 and A8 APIs return only payload/hit/error. Do not infer a route proof merely from that triple.
If approved admission needs a new plain proof result from `TryCrossingBlindCut` or `TryOpposedBlindCut`, add it only
as specified in Task 2's design decision, update all callers, and preserve existing miss/error behavior.

In root `document.go`, validate any present identity against the pending public operation inside `validateOperation`.
Use full output/input origins, Cut roles `target` then `tool`, and the operation kind.
A nil identity preserves existing behavior. A present mismatched identity returns `ErrBooleanFailed` before commit.
Keep validation before the last context check, producer advancement, retirement and publication.
Private steps without a matching public Boolean producer receive no query permission.

Completion evidence: producer test checks actual attachment; malformed pending binding fails without changing live
bodies, producer state or operation history. Compare bounded IDs/counts instead of full snapshots.
Failure response: missing producer proof declines attachment; malformed present proof fails atomically.

## Task 5: add point-only history eligibility

Prerequisite: the approved real producer attaches valid evidence under Task 4.

In `internal/pointquery/history_location.go`, add proposed method:

```go
func (w historyWalk) expandable(n *topo.Body) (meshbool.OperationKind, []*topo.Body, bool)
```

Keep `LocateRecord` first in `historyWalk.locate`. Replace only its call to `boolchain.Expandable` with the new helper.
The helper must first try the existing faceted eligibility unchanged, then try the new BRep evidence branch.
For that branch:

1. Require nonnil node/document, solid BRep payload and nonnil `BooleanSet`.
2. Charge the visit through the existing walk budget; do not start a second unbounded history traversal.
3. Reject any warning except the existing allowed `CodeChainAllowanceApplied` with `ChangeUnknown` combination.
4. Obtain operation and ordered inputs from existing `boolchain.Source.BooleanProducer`.
5. Require exactly two inputs for this first record, both nonnil and in the output's document.
6. Map the kind explicitly and call `Matches` with output and both original full origins.
7. Return those inputs only on success. Missing/mismatched proof leaves the route undecided.

Keep `boolchain.Source` unchanged; root `historyProducers.BooleanProducer` already reads retained committed inputs.
Its mapping is keyed by the output handle. Do not recover a producer from an origin number alone.
Do not expand uncertified BReps merely because that adapter reports `OperationCut`.

In root `point_containment.go`, `locateExact`, keep its current payload and tolerance validation.
For a BRep or faceted payload, enter `LocateBoolean`; other analytic types may keep their direct record path.
The common walk already tries exact records first, so stack BReps keep their existing answers.
Keep the world query point unchanged. Reuse existing `join`, `meet`, `complement`, `cut`, `union`, `intersect`.

Check cancellation before returning a decided result. Keep exactly one root mesh fallback when history is undecided.
Never tessellate source operands from inside the history walk or change both-boundary table entries.

Completion evidence: a source spy proves expansion reads the certified BRep and both needed original operands.
Also prove `boolchain.Expandable` still declines the same BRep, preserving Section/regrouping eligibility.
Failure response: provenance, warning or leaf uncertainty returns undecided; context cancellation returns its error.

## Task 6: complete one producer-to-query checkpoint

Prerequisite: Task 2's approved first fixture and Tasks 3–5. Do not build the wider matrix before this succeeds.

Add proposed `apitest/point_containment_brep_test.go` with the approved public construction.
For the A7 route, Task 1's geometry is usable only after the reviewer explicitly resolves its nonzero charge.
The following descendant is a candidate, not an established passing experiment:

1. Query the A7 source at `q = (-4,0,5)`, strictly below its pocket floor and on the original through-hole wall.
2. Construct a circular tool centred at `(7,7)` with radius 1, extruded from `z=2` to `z=8`.
3. Public `Cut(A7, cylinder)` should force a mesh result with a real curved interior cavity away from `q`.
4. Require faceted payload, no geometry-change diagnostic, verified mesh and nonzero approximation bound.
5. Require the new history route to return `PointOnBoundary` at `q` and material/void answers at x offsets ±1e-9.

For this through hole, `x=-4-1e-9` is material and `x=-4+1e-9` is void at y=0, z=5.
The holed-prism leaf is on its boundary, the pocket is outside, and the cylinder is outside at `q`.
Their unchanged algebra therefore gives boundary. Confirm these three leaf answers independently.
The later tool must not alone decide the result; prove expansion of the BRep is required using a source spy
or a narrowly scoped removal-of-certificate mutation. Record baseline and candidate scalar answers.

The planning probe STOPPED at Task 1. It did not construct or validate this descendant.
If it refuses, becomes analytic, shifts geometry, or already decides without the new route, stop and report that
specific boundary. Do not replace it with a hand-built history fixture as end-to-end acceptance.

Completion evidence: one approved real construction, bound/provenance checks, point answers and certificate-removal
mutation all succeed. A removed certificate must lose the new proof; do not demand a false result from fallback.
Failure response: stop before Task 7 and request a fixture/design revision with the actual measurements.

## Task 7: widen one approved producer at a time

Prerequisite: Task 6 passes. Every row needs its own approved source identity and real first checkpoint.

| Branch and exact code path | Allowed work after approval |
|---|---|
| A7 `TryCrossingBlindCut` → `stackedunion.Cut` → `ubBuild.run` | Retain exact scene selection and canonicalization evidence required by Task 2. |
| A8 `TryOpposedBlindCut` → `stackedrecord.ApplyOpposedBlindCut` → `stackedunion.OpposedCut` | Prove interface floor/ceiling selection and source slab identities; keep touching holes refused. |
| Class B `OfPayloads` → `ThroughReach` → `buildClassB` | Prove exact face restatement and tool interval clipping before attachment. |
| Class B rooted path through `buildClassB` | Test the added floor and original input order independently of through reach. |

Use existing `apitest/stacked_opposed_overlap_test.go` for a whole-line A8 source and
`apitest/classb_rooted_pin_test.go` for rooted source construction.
The current class-B entry exposes no branch identity to dispatch. Add route evidence only as the approved design
specifies; do not label every successful `classB` result a through/rooted construction.

Keep `tryClassBCrossingCut`, rounded curved A7/A8, `WalkReach`, `FlushReach`, and Revolve/stepped views uncertified
unless separate reviewer decisions cover their exact original-source identities and downstream enclosures.
The prior round-hole A7/A8/class-B probes remain negative controls until their own proof is approved.
Do not infer class-B reach from nonzero/zero output charge or face count.

Completion evidence: each newly enabled branch has its own exact admission test and public point checkpoint.
Failure response: leave that branch unattached; do not weaken the already accepted branch or its bound.

## Task 8: verify lifecycle, failure and concurrency

Prerequisite: the first accepted branch is complete. Add proposed unit tests beside the code they exercise.

- In `internal/pointquery/history_location_test.go`, test absent proof, foreign documents, wrong output role,
  swapped Cut inputs, changed producer, unknown warnings and both-boundary answers.
- In `internal/boolchain/chain_test.go`, extend `TestFlattenEligibility` with a certified BRep that still declines.
- In `internal/prooflineage/boolean_set_test.go`, verify immutable binding structure and nil/malformed reads.
- In root `operation_history_test.go`, verify pending mismatches and pre-commit cancellation publish no result.
- Audit `Copy`, `Transform`, `ScaledAt`, Fillet, Chamfer and Shell outputs. They must not inherit node evidence.
  Existing payload rebuilds must create fresh nodes; do not copy the optional field from the old node.
- Query a retired original body and a live descendant whose consumed BRep operand is retired. Both are readable.
- Remove a source through its public operation only where the public lifecycle permits that operation.
  Missing committed history must decline; a user-visible diagnostic must never mint replacement evidence.
- Use `t.Parallel`/race mode for independent concurrent reads of one immutable result.
- Exercise pre-canceled and mid-budget contexts; report location enums, bounded counts and error identities only.

Completion evidence: all focused negative tests pass; no geometry-change or placement descendant gains admission.
Failure response: preserve the existing fallback or return cancellation. Do not suppress the failing case.

## Task 9: update contracts and run focused checks

Prerequisite: Tasks 6 and 8 pass for the exact enabled scope. State only that scope in current-state documentation.

Update `point-containment-design.md`, `operation-history-design.md`, `missing-features.md`, `docs/api.md`,
and `LocatePoint`/`locateExact` comments. Register a new root source file only if one was actually needed.
Register new public tests in `.github/test-shards-apitest.txt`; root tests go in `.github/test-shards.txt`.
Use shard 10 for unmeasured new names. Keep other design proposals marked unimplemented where applicable.

Run these focused commands after the corresponding tests exist; every `Brep` name below is proposed:

```sh
go test ./internal/prooflineage -run '^TestBooleanSetIdentity' -count=1
go test ./internal/topo -run '^(TestTopoDependsOnlyOnFloor|TestBodyMovedFields)$' -count=1
go test ./internal/pointquery -run '^(TestSetAlgebraTables|TestLocateBoolean|TestBrep)' -count=1
go test ./internal/boolchain -run '^TestFlattenEligibility$' -count=1
go test . -run '^TestBrepIdentityCommit' -count=1
go test ./apitest -run '^TestBrepPoint' -count=1
go test -race ./internal/pointquery -run '^TestBrep' -count=1
go test -race ./apitest -run '^TestBrepPoint' -count=1
go test . ./apitest -run '^TestCI' -count=1
go test . -run '^TestCLAUDEMDLayoutStaysCompact$' -count=1
```

Also run existing `TestCrossingBlindPocketOnThroughHole` and existing point-history regressions in `apitest`.
Save outputs under the implementation worktree's `.tmp/` before reading them.
Follow shared execution rules for lint, message checks and CI; do not run the full suite locally.

Completion requires the approved producer proof, one real improved query, unchanged fallback/Section/regrouping
behavior, bounded geometry assertions, lifecycle tests and required CI results. Record unsupported branches plainly.
If Task 2 remains unresolved, the only complete deliverable is its evidence/handoff; no implementation is ready.
