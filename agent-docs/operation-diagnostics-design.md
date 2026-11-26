# Operation diagnostics design

Navigation only; the linked sections own their contracts.

| Question | Section |
|---|---|
| Where do callers read diagnostics? | [Delivery API proposal](#delivery-api-proposal) |
| Which typed details and codes exist? | [Record proposal](#record-proposal) and [Failure catalog and compatibility](#failure-catalog-and-compatibility) |
| Which successful changes warn? | [Successful-call warnings](#successful-call-warnings) |
| When are records published? | [Atomicity, order, and storage](#atomicity-order-and-storage) |
| How do diagnostics differ from verification? | [Verification and other result types](#verification-and-other-result-types) |
| What must implementation prove? | [Focused repository proof fixtures](#focused-repository-proof-fixtures) |
| Which files and tests pin each part? | [Ordered implementation tasks](#ordered-implementation-tasks) |

## Status and ownership

Delivered: public diagnostic values, typed `BooleanError` and `SelectionError` accessors, mapped
`OperationError` branches, and warnings for accepted Boolean shifts, raised chain allowances, tolerant Stitch
merges, and accepted positive-reach repairs. Three-input Union chains, Patterned, private tool composites, direct
modify routes, tangent trim, and mirrored assembly paths aggregate accepted summaries. `arrangement_segments` and fourteen `proofbound` resource names
(`proof_steps`, `replay_bytes`, `multiway_work`, `multiway_records`, `facet_pairs`, `pair_enumeration_steps`,
`mesh_facets`, `facet_allocations`, `freeform_work_units`, `reconstruction_work_units`, `sweep_span_pairs`,
`sweep_faces`, `sweep_spans`, `shell_candidate_visits`) retain exact counts where their producers know them; overflowed counts, saturated estimates and opaque refusals
stay unclassified. Arrangement-cap refusals retain their observed and maximum segment counts.

- `TestBooleanGapTolerance` proves that a real accepted shift can return an empty Intersect with both codes.
- `TestCopyReportsAcceptedEmbeddingReach` proves that a public notched-prism placement publishes an accepted
  repair, keeps one solid lump, and covers its volume change with the published bound. `ExactOnly` refuses the
  same placement without publishing a body.

These contracts stay authoritative:

- [Public API design](api-design.md), especially architecture, bounded readings, booleans, selectors, and errors.
- [Evaluator design](evaluator-design.md), especially document state and re-evaluation.
- [Prism boolean design](prism-boolean-design.md), especially admission and refusals.
- [General boolean design (Part G)](prism-boolean-design.md), especially route admission and proof requirements.
- [Verification design](verification-design.md), especially diagnostics and report status.
- [Gap tolerance](gap-tolerance-design.md), especially charged displacement and shifted retries.
- [Interference design](interference-design.md), especially expected outcomes and invariant failures.

This document owns operation diagnostic delivery, record shape, warning semantics, and proof fixtures.
It does not change geometric admission, tolerance defaults, error classifications, or verification verdicts.

## Problem and intended behavior

Callers can branch on `BooleanError.Code` or inspect `SelectionError`. Many other failures expose their
operand, failed proof, or bound comparison only through text. A successful operation can use a gap tolerance
to move geometry without recording that choice apart from its resulting measurement bounds.

Expose the reason at the call that produced it. Preserve existing `(T, error)` signatures, sentinel matches,
typed errors, and immutable bodies. Warnings attach to the returned body; failure diagnostics attach to the returned
error. `Message` is for people; stable codes are for programs.

The caller's Go function is the feature tree. Store no operation graph, failed-attempt registry, replay program,
timeline, or document-level last error. A caller may keep its own call label beside an error or warning; that
allocates no Decad producer identity.

## Delivery API proposal

The public types live in package `decad`, per Public API design's type-ownership contract. Internal packages
carry private plain-data records; root adapters produce public records and handles. `OperationKind` is the shared
vocabulary owned by [Operation history design: Operation kinds](operation-history-design.md#operation-kinds);
it describes calls independently of history snapshot lookup.

| Surface | Behavior |
|---|---|
| `OperationDiagnosticError` | An interface embedding `error` and declaring `OperationDiagnostics() []OperationDiagnostic`. |
| `(*BooleanError).OperationDiagnostics()` | Returns the failed boolean's structured reasons while preserving `Code` and existing sentinel behavior. |
| `(*SelectionError).OperationDiagnostics()` | Describes the failed selection while preserving its existing query, counts, residuals, and identity. |
| `OperationError` | Wraps an otherwise unstructured Decad operation error. It implements `Error`, `Unwrap`, and `OperationDiagnostics`. |
| `(*Body).OperationDiagnostics()` | Returns warnings from the successful public call that created this body. It returns no failure entries. |

Find `OperationDiagnosticError` with `errors.As`, including through a caller's `%w` wrapper.
`errors.Is` and existing `errors.As` branches stay the authoritative failure classification.
Wrap only otherwise unstructured Decad errors within this document's modeling scope.
`OperationError.Unwrap` returns the original error and never rebuilds a sentinel from its message.
Its `Error` preserves the original text; add context only when the owning API explicitly needs it.
The wrapper can break direct equality and concrete-type assertions on those previously unstructured errors;
callers use the supported `errors.Is` and `errors.As` branches instead.

Every accessor returns newly owned slices and copies of optional detail records. A caller's edits to one
returned record cannot affect a later read, the error, another body, or a concurrent reader.
Body handles in error subjects stay immutable shared identities.

Keep these exceptions explicit:

- Return `ctx.Err()` unchanged on cancellation or deadline. Those errors do not gain an operation diagnostic.
- Return an existing `BooleanError` directly, without an `OperationError` wrapper or changed classification.
- Return a propagated `SelectionError` unchanged, as the selector contract requires. Its diagnostic names
  `SelectEdges` or `SelectFaces`; it does not claim to name the enclosing `Fillet`, stop, or axis call.
- Preserve panic behavior for caller-created typed-nil interface values. Diagnostics add no reflection or recovery.
- Do not wrap arbitrary third-party or writer errors merely to make this interface universal.

An empty diagnostic slice means no event from this document's catalog was recorded. It proves nothing about
exactness, validity, verification coverage, or future warnings.

## Record proposal

`OperationDiagnostic` is one root-owned value with the fields below. Optional details are typed; add no
`map[string]any`, arbitrary attributes, embedded topology dumps, or message parsing.

| Field | Type and meaning |
|---|---|
| `Code` | `OperationDiagnosticCode`; the stable reason. Zero means unknown, never success. |
| `Severity` | `OperationDiagnosticSeverity`; zero is unknown, followed by warning and failure. |
| `Operation` | The shared `OperationKind` defined by Operation history design. |
| `Stage` | `OperationStage`; unknown, input, recording, evaluation, or result. Stages do not expose route helper names. |
| `Subject` | `OperationSubject`; an argument location and, on errors only, an optional original body handle. |
| `Chain` | Optional `OperationChainDetail`; the touched held bound, pair chord limit, and configured chain allowance. |
| `Change` | Optional `OperationChange`; the tolerance mechanism, displacement upper bound, and allowed reach. |
| `WorkLimit` | Optional `OperationWorkLimit`; a named resource and integer observed and maximum counts. |
| `Message` | Human-readable explanation. Its text is not a branch key or a serialized identity. |

`OperationSubject` names a public argument, such as target, tool, bodies, body, profile, or selector.
A variadic argument has an optional zero-based ordinal within that argument. The ordinal is caller input order,
never a face, edge, vertex, facet, or private transient body. An absent subject concerns the whole operation.
A nil offending body can still name its argument.

Error subjects reference only bodies the caller supplied. Successful-body warnings store argument locations
without input body handles, so they retain no chain of consumed bodies. Never publish private temporary handles,
speculative producer references, internal topology indices, or uncommitted result bodies. Selection residuals
stay in the existing `SelectionError` fields; do not copy their schema into this record.
Omit an uncertain subject rather than blaming one original input for a failure of a transient composite.

`OperationChainDetail` carries `TouchedBound units.Value`, `PairChordLimit units.Value`, and
`ChainAllowance *units.Value`. Every value is a length. A nil allowance means none was configured.
`TouchedBound` is the certified upper bound of the held geometry this contact touches. It is not a measured
distance to the true surface or the body's global maximum bound. All three gate inputs are preserved so a
caller can inspect the ordinary threshold and any configured allowance without reconstructing text.
`ChainAllowanceApplied` requires a non-nil allowance: the touched bound exceeds the pair chord limit and
passes the configured allowance. `OperandBoundExceeded` carries this detail too when that gate refuses.

`OperationChange` carries a mechanism enum, `DisplacementBound units.Value`, and `AllowedReach units.Value`,
both lengths. The displacement is a certified upper bound for the change this call made. It is neither the
configured reach nor the result's total accumulated bound. Compose multiple changes with the owning geometry
proof, or a conservative sum when only per-stage upper bounds exist. An upper bound is never presented as an
exact measured maximum.

Any actual measurement added to a detail record uses `Measurement`, `VecMeasurement`, or `Box`, with its own
exactness and bound. Scalar parameters, limits, angles, and error bounds use `units.Value`. The coordinate and
curve-parameter exceptions are the ones Public API design defines.

Each diagnostic-owned enum has a stable lower-snake `String()` token and a total unknown-value rendering.
Never serialize the enum's integer as its meaning. New codes never renumber published values. Callers MUST
handle unknown codes without treating them as success.

## Failure catalog and compatibility

Publish only reasons the evaluator already knows; a broad reason beats an invented distinction.
Every constant name below carries the prefix `OpDiag`.

| Suffix | Concrete trigger | Existing classification |
|---|---|---|
| `InputRejected` | A public parameter or ownership gate rejects an input. The error chain identifies the specific sentinel. | Existing parameter, foreign-body, retired-body, or profile error. |
| `SelectionFailed` | A query fails its asserted cardinality or its unasserted nonempty requirement. | The same `SelectionError`, with its existing sentinel and residuals. |
| `ProfileUnrecordable` | The sketch seam cannot record a boundary under its exactness contract. | `ErrUnrecordableProfile`; no tolerance-based admission. |
| `ResultEmpty` | A boolean encloses no volume. | `BooleanEmpty`, wrapping `ErrBooleanFailed`. |
| `UnsupportedContact` | The existing contact classifier cannot decide the contact. | `BooleanUnsupportedContact`, wrapping `ErrUnsupported`. |
| `OperandProofUnavailable` | A particular operand lacks the occupied-volume or other proof needed by this operation. | Existing plain `ErrUnsupported`; no `BooleanError`. |
| `OperandBoundExceeded` | Touched held facets exceed the admitted bound at this contact. | Existing plain `ErrUnsupported`, with a structured local bound comparison. |
| `TessellationUnresolved` | A valid operand's requested chording cannot prove the required topology. | Existing retryable `ErrDegenerate`; no `BooleanError`. |
| `WorkLimitReached` | An existing bounded arrangement or evaluator resource is exhausted. | The existing error mapping at that gate. |
| `UnsupportedIntent` | The requested feature class or admitted construction exceeds the current evaluator's reach. | Existing `ErrUnsupported` or its established typed mapping. |
| `EvaluatorFailure` | A proved internal invariant breaks. | Existing invariant error; booleans preserve `BooleanEvaluatorFailure`. |

- Keep malformed inputs distinct from chording refusals even when both match `ErrDegenerate`.
- Never classify every `ErrUnsupported` as contact, or every `ErrBooleanFailed` as empty.
- An analytic prism admission miss stays a silent fallback. An admitted route's refusal keeps the behavior
  Prism boolean design specifies; a diagnostic never converts a refusal into a fallback or the reverse.

Carry private typed detail to root before it is rendered into text. `meshbool.StagingError` renders a
`CoarseHeldContactError` into operand/bound/tolerance prose, so the private error path preserves those values and
root fills `OperandBoundExceeded` directly. The public boolean mapping stays at `asBooleanError`; records
extend there without reclassifying outcomes.

## Successful-call warnings

Emit a warning only when its event actually affects the accepted result. Ordinary floating-point bounds, a mesh
route, a nonzero measurement bound, and `Approximate` alone create none; their measurement and surface contracts
already expose them. `MatchingFaces` is outside the warning catalog until its real producer supplies typed
actual-use and deviation data; its existing geometry and bound contracts apply.

| Suffix | Trigger and detail |
|---|---|
| `GapShiftApplied` | A shifted boolean attempt produces the accepted outcome. `Change` names shifted retry and bounds the actual accepted displacement. |
| `EmbeddingReachApplied` | Held-embedding repair accepts a position from its positive-reach stage. `Change` excludes ordinary float-box rounding. |
| `StitchMergeApplied` | Tolerant Stitch actually merges distinct vertex positions. `Change` describes the accepted merge, using its existing charged bound. |
| `ChainAllowanceApplied` | A local held bound exceeds the ordinary pair chord gate and passes using a configured chain allowance. `Chain` records all three gate inputs. |

Each mechanism contributes at most one summary per public call and public argument location. A summary
conservatively covers all affected pieces or internal steps at that location. It exposes no private component
IDs and retains no per-facet data.

- Record only the accepted retry's changes. Discard warnings from rejected trial positions and abandoned routes.
- If a shifted attempt ends in `BooleanEmpty`, return that existing error with its accepted shift warning beside
  `ResultEmpty`; no body commits.
- If every shifted attempt refuses, return the original refusal and no applied-shift warning.

For multi-input `Union` or a composite feature, collect internal-step summaries locally and publish them only
with the outer successful body. If a later step fails, discard the earlier summaries and report the failing
step's original caller input locations. Never expose a partial result or claim earlier inputs were consumed.
A terminal failure may carry its own accepted-outcome warning, as the empty-result case above shows.

A copied or transformed body starts a new creation record and does not copy its source's warning list. Its
geometry still carries all inherited bounds, and any new repair produces its own warning. Warnings on retired
bodies stay readable because they describe the immutable call result. `Remove` neither edits that record nor
allocates a warning record.

Warnings never decide body validity or a verification verdict. There is no warning-as-error mode: the operation
has already committed when the caller reads the body. A caller needing strict geometry picks construction and
tolerance options before the call, including `ExactOnly`.

## Atomicity, order, and storage

Collection is local to one public operation. Final warning records are built before commit and attached to the
immutable result that commits; never append warnings to a published body. Failure or cancellation preserves
live-body membership and the producer identity under Evaluator design. Level and curve counters keep that
design's failed-build semantics.

- Never allocate a producer identity for diagnostics.
- Never update an operand to record a failed attempt.
- Add no mutable collector to `Document`, no context value, and no public callback.
- Private evaluator result fields and typed error detail carry records across internal boundaries.
- Each parallel worker returns its own records; the owning operation merges them in deterministic input order.
  This grants no permission for concurrent document mutation.

Order diagnostics by public argument order, stage order, then stable code token; keep the existing selection
residual order. Aggregate repeated mechanism summaries at one public location with the owning proof's bound
composition. Keep one primary failure reason unless the evaluator establishes independent reasons. Never
continue geometry work only to collect more failures.

Records are bounded by the catalog and input locations, never by facets, candidate pairs, trial positions, or
failed intermediate routes. Retain no meshes, ASTs, or arbitrary error graphs in detail. Error messages and
debug formatters summarize bounded scalar data.

## Verification and other result types

Operation diagnostics stay separate from `Diagnostic` and `Report.Diagnostics`. Verification codes explain a
requested proof verdict; operation warnings explain how one modeling call obtained its accepted outcome. An
operation record has no `Status`, `Passed`, or error field, and a creation warning never enters the verification
inventory.

Sound still equals an empty verification diagnostic inventory. Verification judges a repaired body's published
bounds, so a body with an operation warning can be Sound and a body without one can be Suspect. The
cause-specific pair diagnostic mapping in Interference design stands; shared private reason records must not
force the two public code sets to coincide.

Coverage is failures of modeling operations and their built-in selectors, plus the body warnings cataloged
here. No modeling warning list attaches to `Mesh`, export writers, `Report`, motion results, or dynamics results.
A read-only boolean inside Verify must not mutate either body's creation warnings. Query-only tolerance changes
need a future proposal that gives them an explicit result location without weakening the verification inventory
contract.

## Failure cases that must stay explicit

- A nil context fails at the existing entry gate, without inspecting an operand or allocating provenance.
- A selector failing inside a modifier returns the same `SelectionError`, not an outer wrapper.
- A held-bound refusal names the touched operand and comparison and suggests no boolean chord option.
- A canceled bounded worker finishes under its existing cancellation contract; diagnostics add no worker lifetime.
- A tolerance cannot turn an uncertified sketch fragment into an accepted input or an exact reading.
- A message-format change cannot alter any branch, diagnostic code, operand role, or bound field.
- A warning cannot hide an invariant failure or replace the operation's returned error.
- Retained errors keep their referenced input bodies alive; success warning storage must not.

## Focused repository proof fixtures

Start with one real path: the small face-touching box construction in `apitest/boolean_gap_tolerance_test.go`.
The real boolean evaluator produces a charged shift that appears through the real public body accessor. Prove
its volume bound against 2000 mm³ and keep its single-lump assertion. Stop expanding if private evaluation
cannot deliver this record through commit; a handcrafted warning fixture never replaces this boundary.

After it passes, add these focused assertions:

| Fixture | Required proof |
|---|---|
| The same touching boxes under `ExactOnly` | The original contact code and sentinel survive; operands and producer numbering are unchanged; no shift warning appears. |
| Touching boxes passed to `Intersect` | `BooleanEmpty` and `ErrBooleanFailed` survive, with the accepted retry's warning and no committed body. |
| A flush tool blocked by both slot walls | The original unsupported contact returns, with no applied-change warning and both operands live. |
| `TestBooleanChainDepthGateReadsOnlyTheTouchedRims` | The structured subject and bounds equal the real local gate's values; an untouched coarse region does not cause a refusal. |
| `TestImplicitExactlyOneStopFailedAssertionPassesThrough` | The same `SelectionError` reaches the caller; query, counts, and residuals remain inspectable through caller wrapping. |
| Existing nil-context and cancellation boolean fixtures | Cancellation is the identical `ctx.Err()` value, with no body, producer advance, or warning publication. |
| A three-input Union whose final pair refuses | All original operands remain live; no earlier successful-step warning leaks into a partial body or unrelated later call. |
| `internal/booleaneval` shifted-piece fixture | Each piece's real charge contributes conservatively to the public summary; an untouched piece contributes no fictitious movement. |
| `internal/facetproof` placement-reach fixture | A real positive-reach repair emits a warning, while its float-box-only counterpart does not. Preserve the existing geometric audit. |
| `TestCopyReportsAcceptedEmbeddingReach` | A public notched-prism Copy emits the accepted repair warning, keeps one solid lump, and covers 999.98 mm³; `ExactOnly` refuses the same placement. |
| A minimal planar Stitch pair | A real tolerant merge warns and its volume remains within its bound; exact matching positions and `ExactOnly` do not warn. |
| A normal analytic rectangular operation | Existing exact readings stay exact and the warning slice is empty. |
| Accessor ownership and order | Changing returned slices and detail pointers does not change later reads. Parallel reads preserve stable tokens, ordering, and bounds. |
| Verify on a shifted result | A loose request can be Sound with operation warnings; a stricter request produces existing bound diagnostics. Warnings do not enter `Report.Diagnostics`. |

Exported-API fixtures go in `apitest/`; private transport tests sit beside the transport implementation.
Use existing geometry constructors and real producer/consumer paths. Compare fields, sentinels, counts, and
bounded numerical invariants; never deep-dump bodies, errors with body subjects, or topology. Check bounded
failure formatting with a small deliberate mismatch. [Test guidance](testing.md) owns test names and CI shards.
Full consumer models stay in their consumer repositories.

## Settled scope and API decisions

- Warnings are read through the body-local accessor in Delivery API proposal. Operation signatures stay as they
  are; there are no parallel detailed-result entry points.
- The warning catalog is the one in Successful-call warnings. The `MatchingFaces` exclusion is settled;
  adding it needs the producer evidence and typed detail specified there.
- The chain record is the one in Record proposal; the touched bound, pair chord limit, and chain allowance are
  exposed together, never reduced to one effective threshold.
- Error wrapping follows Delivery API proposal: `BooleanError`, `SelectionError`, and cancellation delivery are
  preserved, and only otherwise unstructured Decad errors are wrapped with their chains intact.

## Ordered implementation tasks

Each phase below describes what exists now and which files and tests pin it. The contract sections above win if
wording differs. Root records and accessors live in `operation_diagnostics.go`.

### Phase 0: Resolve prerequisites and map entry points

The shared `OperationKind` vocabulary lives in `operation_kind.go`, owned by
[Operation history design: Operation kinds](operation-history-design.md#operation-kinds). Diagnostics depends on
that vocabulary and on nothing else from history. `booleanOperandLocation` in `operation_diagnostics.go` maps each
Boolean kind's operand index to its public argument name and ordinal (`target`/`tool`, `a`/`b`,
`bodies` with the operand's ordinal). No other entry-point checklist is kept in the repository.

### Phase 1: Add records, error transport, and immutable accessors

- `operation_diagnostics.go` defines the enums with total `String()` methods, `OperationSubject`,
  `OperationChange`, `OperationChainDetail`, `OperationWorkLimit`, `OperationDiagnostic`, `OperationDiagnosticError`
  and `OperationError`. `OperationError.Unwrap` returns the original error and `Error` returns its text.
- Plain-data records and aggregation live in the leaf package `internal/opdiagnostic`. `internal/topo/body.go`
  stores them as `Body.OperationWarnings` without input body references. Root adapts them to public values.
- Accessors return newly owned slices and copies of optional details.
- `operation_diagnostics_test.go` pins enum tokens and unknown values, `BooleanError` diagnostics, optional-detail
  copies and `Unwrap` (`TestOperationErrorCopiesOptionalDetailsAndUnwrapsOriginal`), and the pass-through of known
  typed failures (`TestOperationErrorPreservesKnownFailureAndSkipsTypedErrors`).
  Synthetic records appear only in these value and ownership tests.

### Phase 2: Prove one real shifted Boolean end to end

The accepted retry's shift summary travels from `internal/boolshift`, `internal/booleaneval` and
`internal/booleandispatch` through `boolean.go` to `(*Body).OperationDiagnostics()` before `document.go` commits
the body. The displacement is the upper bound of the actual accepted per-piece charges, not the configured reach.
`TestBooleanGapTolerance` in `apitest/boolean_gap_tolerance_test.go` pins the real path: the warning, a positive
displacement within the allowed reach, one lump, and a volume bound covering 2000 mm³. Rejected-attempt records
stay local and are discarded.

### Phase 3: Preserve failure classifications and add typed reasons

- `BooleanError` and `SelectionError` carry `OperationDiagnostics()` and keep their concrete identity, fields,
  sentinels, `Code` and residual order. A selector diagnostic names `SelectEdges` or `SelectFaces` even when an
  enclosing modifier propagates it.
- `operationError` in `operation_diagnostics.go` wraps otherwise unstructured Decad errors in `OperationError`. It
  returns cancellation and deadline errors unchanged, returns errors that already carry diagnostics, and maps typed
  private errors before falling back to `operationErrorCode`, which classifies by sentinel.
  The typed private errors are `tessellation.ExpectedError`, `meshbool.CoarseHeldContactError`,
  `meshbool.BooleanExpectedError`, `prismcells.WorkLimitError` and `proofbound.WorkLimitError`.
- `WorkLimitReached` carries `arrangement_segments` counts from `internal/prismcells` and the resource names
  listed in Status and ownership, from `internal/proofbound` and the evaluators that call `proofbound.WithWorkLimit`.
  A proof-limit count above the platform `int` range leaves `WorkLimit` nil.
- Tests: `TestOperationWorkLimitHasTypedArrangementCounts` and `TestSelectionErrorHasSelectorDiagnostic` in
  `operation_diagnostics_test.go`, `apitest/selection_error_test.go` (including
  `TestImplicitExactlyOneStopFailedAssertionPassesThrough`) and `apitest/boolean_test.go`.

### Phase 4: Carry the local chain gate's three inputs

`meshbool.CoarseHeldContactError` keeps the touched bound, the pair chord limit and the configured chain
allowance past `StagingError`'s text rendering. `operationError` fills `OperandBoundExceeded` from it, with
`ChainAllowance` nil when none was configured. `ChainAllowanceApplied` is emitted only when a touched gate passes
using the allowance. Credit already charged by a shifted retry does not emit it by itself. `TestBooleanChainDepthGateReadsOnlyTheTouchedRims` (`apitest/boolean_test.go`) and
`TestChainToleranceAdmitsAContactOnACoarserRim` (`apitest/chain_tolerance_test.go`) read these structured fields;
the latter has no regexp or `ParseFloat` extraction.

### Phase 5: Report accepted positive-reach embedding repairs

`internal/meshbool/held_embedding_diagnostics.go` defines `EnforceHeldEmbeddingWithSummary` and
`RepairHeldEmbeddingWithSummary`, which return an `*EmbeddingReach` describing the accepted positive-reach use.
The summary reaches the public body through `internal/meshbool/mesh_stitch.go` and `internal/facetproof`, and `EmbeddingReachApplied` is published at the outer operation. `TestCopyReportsAcceptedEmbeddingReach`
(`apitest/embedding_reach_test.go`) pins the public placement path. `internal/facetproof/placement_reach_test.go`
pins the placement-reach producer.

### Phase 6: Report actual tolerant Stitch merges

`stitch.go` and `internal/stitchrecord` publish `StitchMergeApplied` for distinct positions the accepted tolerant
plan merged, using the plan's charged separation bounds. When `loopsKeepTheirShape` (`internal/stitchrecord/plan.go`)
selects the exact plan, the tolerant plan's records are discarded.
`apitest/stitch_gap_tolerance_test.go` pins the merge warning, and `ExactOnly` producing none.

### Phase 7: Finish composite publication, order, and storage bounds

`internal/opdiagnostic` holds `UnionSummaries`, `ReplayUnionSummaries` and `CompositeSummaries`, which aggregate one
summary per mechanism and public location. `boolean_group_blind.go`, `boolean_group_history.go`,
`boolean_regroup.go`, `boolean_group_tessellate.go`, `pattern.go`, `tool_combine.go`, `stitch.go` and
`point_plane_scale.go` attach them to the outer result before `document.go`'s `commitOperation*In` helpers commit
it. A failed later step discards earlier summaries. `apitest/union_many_diagnostics_test.go`
(`TestUnionManyOperationWarnings`) pins the Union path, and `internal/opdiagnostic/composite_test.go` and
`union_test.go` pin aggregation. `TestEmptyBooleanDiagnosticsKeepAnAcceptedShift` pins the accepted-shift empty
Intersect.

### Phase 8: Prove verification separation and complete API documentation

`TestVerifyGapToleranceDecidesAFlushPair` (`apitest/verify_gap_tolerance_test.go`) pins the separation: a shifted
result is Sound under a loose request, an exact construction has an empty warning slice, and read-only Verify
leaves the creation warnings unchanged. `docs/api.md`, `docs/data-structures.md`,
`examples/operation_diagnostics_example_test.go` and the `agent-docs/layout/` rows cover the public surface.
No warning-as-error mode, report verdict field, or warning on Mesh, export, motion or dynamics results exists.
`MatchingFaces` has no warning code.

### Phase 9: Run focused checks and prepare review

New test names belong in the shard manifest `.github/test-shards.txt`; unmeasured tests go in the last numbered shard and shards
are rebalanced only from measured `_shardgen` input (see [Test guidance](testing.md)). `.github/workflows/ci.yml`
runs `golangci-lint` v2.15.0 with `--timeout=5m` and `go vet ./...`, and the full suite runs in remote CI.
Focused local runs of the touched package or named fixtures stand in for the suite only before CI reports.

## Behavioral reference boundary

Read-only reference inspection used the sibling OCCT checkout's
`src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Options.hxx` and
`src/FoundationClasses/TKernel/Message/Message_Report.hxx`.
The observed behavior was per-operation inspection of typed failures and warnings, including results from
nested work. This design specifies that behavior independently from Decad's existing Go errors, immutable body
results, and proof contracts. No OCCT implementation, declaration, or text is copied or adapted. OCCT is not a
dependency, a runtime oracle, or an interoperability test requirement.
