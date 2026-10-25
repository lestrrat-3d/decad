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
| What should a coding agent implement first? | [Ordered implementation tasks](#ordered-implementation-tasks) |

## Status and ownership

Implementation delivers public diagnostic values, typed `BooleanError` and
`SelectionError` accessors, mapped `OperationError` branches, and warnings for
accepted Boolean shifts, raised chain allowances, tolerant Stitch merges, and
accepted positive-reach repairs. Three-input Union chains, Patterned,
PatternJoin, private tool composites, direct modify routes, tangent trim, and
mirrored assembly paths aggregate accepted summaries. Eleven resource names
retain exact counts where their producers know them; overflowed counts,
saturated estimates and opaque refusals remain unclassified. Arrangement-cap
refusals retain their observed and maximum segment counts.
`TestBooleanGapTolerance` proves that a real accepted shift can return an empty
Intersect with both diagnostic codes. `TestCopyReportsAcceptedEmbeddingReach`
proves that a public notched-prism placement publishes an accepted repair,
keeps one solid lump, and covers its volume change with the published bound.
The same placement under `ExactOnly` refuses without publishing a body.

The existing contracts remain authoritative:

- [Public API design](api-design.md), especially architecture, bounded readings, booleans, selectors, and errors.
- [Evaluator design](evaluator-design.md), especially document state and re-evaluation.
- [Prism boolean design](prism-boolean-design.md), especially admission and refusals.
- [General boolean design](general-boolean-design.md), especially route admission and proof requirements.
- [Verification design](verification-design.md), especially diagnostics and report status.
- [Gap tolerance](gap-tolerance-design.md), especially charged displacement and shifted retries.
- [Interference design](interference-design.md), especially expected outcomes and invariant failures.

This document owns operation diagnostic delivery, record shape, warning semantics, and proof fixtures.
It does not change geometric admission, tolerance defaults, error classifications, or verification verdicts.

## Problem and intended behavior

A caller can branch on `BooleanError.Code` or inspect `SelectionError`, but many other failures expose their
operand, failed proof, or bound comparison only through text. A successful operation can use a gap tolerance
to move geometry without recording that choice separately from its resulting measurement bounds.

Expose the reason at the call that produced it. Preserve existing `(T, error)` signatures, sentinel matches,
typed errors, and immutable bodies. Provide warnings on the successfully returned body. Provide failure
diagnostics through the returned error. Keep `Message` for people and stable codes for programs.

The caller's Go function remains the feature tree. Store no operation graph, failed-attempt registry,
replay program, timeline, or document-level last error. A caller can save its own call label beside an error
or warning without allocating a Decad producer identity.

## Delivery API proposal

Define the new public types in package `decad`, following Public API design's type-ownership contract.
Internal packages carry private plain-data records; root adapters produce public records and handles.
Use the shared `OperationKind` vocabulary owned by
[Operation history design: Operation kinds](operation-history-design.md#operation-kinds).
The type describes calls independently of history snapshot lookup.

| Proposed surface | Behavior |
|---|---|
| `OperationDiagnosticError` | An interface embedding `error` and declaring `OperationDiagnostics() []OperationDiagnostic`. |
| `(*BooleanError).OperationDiagnostics()` | Returns the failed boolean's structured reasons while preserving `Code` and existing sentinel behavior. |
| `(*SelectionError).OperationDiagnostics()` | Describes the failed selection while preserving its existing query, counts, residuals, and identity. |
| `OperationError` | Wraps an otherwise unstructured Decad operation error. It implements `Error`, `Unwrap`, and `OperationDiagnostics`. |
| `(*Body).OperationDiagnostics()` | Returns warnings from the successful public call that created this body. It returns no failure entries. |

Use `errors.As` to find `OperationDiagnosticError`, including through a caller's `%w` wrapper.
Keep `errors.Is` and existing `errors.As` branches as the authoritative failure classification.
Wrap only otherwise unstructured Decad errors within this document's modeling scope.
`OperationError.Unwrap` returns the original error, rather than rebuilding a sentinel from its message.
Its `Error` preserves the original text. Add contextual text only when the owning API explicitly needs it.
The new wrapper can change direct equality and concrete-type assertions on those previously unstructured
errors. Callers use the supported `errors.Is` and `errors.As` branches instead.

Return newly owned slices and copies of optional detail records from every accessor. A caller's edits to
one returned record cannot affect a later read, the error, another body, or a concurrent reader.
Body handles in error subjects remain immutable shared identities.

Keep these exceptions explicit:

- Return `ctx.Err()` unchanged on cancellation or deadline. Those errors do not gain an operation diagnostic.
- Return an existing `BooleanError` directly, without an `OperationError` wrapper or changed classification.
- Return a propagated `SelectionError` unchanged, as the selector contract requires. Its diagnostic names
  `SelectEdges` or `SelectFaces`; it does not claim to name the enclosing `Fillet`, stop, or axis call.
- Preserve panic behavior for caller-created typed-nil interface values. Diagnostics add no reflection or recovery.
- Do not wrap arbitrary third-party or writer errors merely to make this interface universal.

An empty diagnostic slice means no event from this document's catalog was recorded. It is not a proof of
exactness, validity, verification coverage, or the absence of every possible future warning.

## Record proposal

Use one root-owned `OperationDiagnostic` value with the fields below. Keep optional details typed; do not
introduce `map[string]any`, arbitrary attributes, embedded topology dumps, or message parsing.

| Field | Proposed type and meaning |
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

`OperationSubject` identifies a named public argument, such as target, tool, first, others, body, profile,
or selector. A variadic argument has an optional zero-based ordinal within that argument. This ordinal
identifies caller input order, never a face, edge, vertex, facet, or private transient body.
An absent subject concerns the whole operation. A nil offending body can still name its argument.

Error subjects may reference only bodies actually supplied by the caller. Successful-body warnings store
argument locations without input body handles. They therefore do not retain a chain of consumed bodies.
Do not publish private temporary handles, speculative producer references, internal topology indices, or
uncommitted result bodies. Use existing `SelectionError` fields for selection residuals rather than copying
their complete schema into the new record.
Omit an uncertain subject rather than blaming one original input for a failure of a transient composite.

`OperationChainDetail` carries `TouchedBound units.Value`, `PairChordLimit units.Value`, and
`ChainAllowance *units.Value`. Every value is a length. A nil allowance means none was configured.
`TouchedBound` is the certified upper bound of the held geometry this contact touches, not a measured
distance to the true surface or the body's global maximum bound. Preserve all three gate inputs so the
caller can inspect both the ordinary threshold and any configured allowance without reconstructing text.
`ChainAllowanceApplied` requires a non-nil allowance: the touched bound exceeds the pair chord limit and
passes the configured allowance. `OperandBoundExceeded` also carries this detail when that gate refuses.

`OperationChange` carries a mechanism enum, `DisplacementBound units.Value`, and `AllowedReach units.Value`.
Both quantities are lengths. The displacement is a certified upper bound for the change made by this call,
not the configured reach and not the result's total accumulated bound. Compose multiple changes using the
owning geometry proof; use a conservative sum when only per-stage upper bounds are available.
Do not imply that an upper bound is an exact measured maximum.

Every actual measurement added to a future detail record must use `Measurement`, `VecMeasurement`, or `Box`,
including its own exactness and bound. Scalar parameters, limits, angles, and error bounds use `units.Value`.
The coordinate and curve-parameter exceptions remain the ones Public API design defines.

Each diagnostic-owned enum has a stable lower-snake `String()` token and a total unknown-value rendering.
Never serialize the enum's integer as its meaning. Add new codes without renumbering old published values.
Code availability is additive: callers must handle unknown codes without treating them as success.

## Failure catalog and compatibility

Publish only reasons the evaluator already knows. A broad reason is preferable to an invented distinction.
The proposed constant names below all have the prefix `OpDiag`.

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

Keep genuinely malformed inputs distinct from chording refusals even when both match `ErrDegenerate`.
Do not classify every `ErrUnsupported` as contact, or every `ErrBooleanFailed` as empty.
An analytic prism admission miss remains silent fallback. An admitted route's refusal keeps the behavior
specified by Prism boolean design; a diagnostic never converts it into a fallback or vice versa.

Carry private typed detail to root before it is rendered into text. For example,
`meshbool.StagingError` currently renders a `CoarseHeldContactError` into operand/bound/tolerance prose.
Preserve those values in the private error path so root can populate `OperandBoundExceeded` directly.
Keep the public boolean mapping at `asBooleanError`; extend its records without reclassifying its outcomes.

## Successful-call warnings

Emit a warning only when the corresponding event actually affects the accepted result.
Ordinary floating-point bounds, a mesh route, a nonzero measurement bound, and `Approximate` alone do not
create warnings. Their existing measurement and surface contracts already expose them.
Exclude `WithMatchingFaces` from the initial warning catalog until its real producer supplies typed
actual-use and deviation data. Its existing geometry and bound contracts continue to apply.

| Proposed suffix | Trigger and detail |
|---|---|
| `GapShiftApplied` | A shifted boolean attempt produces the accepted outcome. `Change` names shifted retry and bounds the actual accepted displacement. |
| `EmbeddingReachApplied` | Held-embedding repair accepts a position from its positive-reach stage. `Change` excludes ordinary float-box rounding. |
| `StitchMergeApplied` | Tolerant Stitch actually merges distinct vertex positions. `Change` describes the accepted merge, using its existing charged bound. |
| `ChainAllowanceApplied` | A local held bound exceeds the ordinary pair chord gate and passes using a configured chain allowance. `Chain` records all three gate inputs. |

The warning constants also use the `OpDiag` prefix. Each mechanism contributes at most one summary per
public call and public argument location. Summaries conservatively cover all affected pieces or internal
steps at that location. They do not expose private component IDs or retain per-facet data.

Record only the accepted retry's changes. Discard warnings from rejected trial positions and abandoned
routes. If a shifted attempt ends in `BooleanEmpty`, return that existing error and attach its accepted
shift warning beside `ResultEmpty`; no body commits. If every shifted attempt refuses, return the original
refusal and no applied-shift warning. A discarded transient change is not an applied result change.

For multi-input `Union` or a composite feature, collect successful internal-step summaries locally.
Publish them only with the outer successful body. If a later step fails, discard those earlier summaries
and report the failing step's original caller input locations. Do not expose a partial result or claim
that earlier inputs were consumed. A terminal failure may include its own accepted-outcome warning,
as the empty-result case above specifies.

A copied or transformed body starts a new creation record. Do not copy its source body's warning list.
Its geometry still carries all inherited bounds, and any new repair produces its own warning.
Warnings on retired bodies remain readable because they describe the immutable call result.
`Remove` does not edit that record or allocate a warning record.

Warnings never decide whether a body is valid or whether a verification request passes.
There is no warning-as-error mode in this proposal: an operation has already committed when its caller
reads the successful body. A caller needing strict geometric behavior chooses existing construction and
tolerance options before the call, including `ExactOnly` where appropriate.

## Atomicity, order, and storage

Keep collection local to a single public operation. Build final warning records before commit, then attach
them to the immutable result that commits. Do not append warnings to an already published body.
Failure or cancellation preserves live-body membership and the producer identity under Evaluator design.
Level and curve counters retain that design's existing failed-build semantics.

Never allocate a producer identity for diagnostics. Never update an operand to record a failed attempt.
Do not add a mutable collector to `Document`, a context value, or a public callback.
Private evaluator result fields and typed error detail transport records across internal boundaries.
Each parallel worker returns its own records; an owning operation merges them in deterministic input order.
This adds no permission for concurrent document mutation.

Order diagnostics by public argument order, stage order, then stable code token. Preserve the existing
selection residual order. Aggregate repeated mechanism summaries at the same public location using the
owning proof's bound composition. Keep one primary failure reason unless the existing evaluator actually
establishes independent reasons. Never continue geometry work solely to collect more failures.

Keep records bounded by their catalog and input locations, rather than the number of facets, candidate
pairs, trial positions, or failed intermediate routes. Do not retain meshes, ASTs, or arbitrary error
graphs in the diagnostic detail. Error messages and debug formatters must summarize bounded scalar data.

## Verification and other result types

Keep operation diagnostics separate from `Diagnostic` and `Report.Diagnostics`.
Verification diagnostic codes explain a requested proof verdict; operation warnings explain how one
modeling call obtained its accepted outcome. Do not add `Status`, `Passed`, or an error field to an
operation record. Do not add a creation warning to the verification inventory merely because it exists.

The existing equivalence between Sound and an empty verification diagnostic inventory remains unchanged.
Verification judges a repaired body's published bounds, so a body with an operation warning can be Sound,
and a body without one can be Suspect. Preserve the cause-specific pair diagnostic mapping in
Interference design; shared private reason records must not force the two public code sets to coincide.

The first delivery covers failures of modeling operations and their built-in selectors, plus the successful
body warnings cataloged here. It does not attach a modeling warning list to `Mesh`, export writers,
`Report`, motion results, or dynamics results. A read-only boolean inside Verify must not mutate either
body's creation warnings. A later proposal must give query-only tolerance changes an explicit result
location without weakening the verification inventory contract.

## Failure cases that must stay explicit

- A nil context fails at the existing entry gate. It must not inspect an operand or allocate provenance.
- A selector failing inside a modifier still returns the same `SelectionError`, not an outer wrapper.
- A held-bound refusal identifies the touched operand and comparison without suggesting a boolean chord option.
- A canceled bounded worker must finish under its existing cancellation contract; diagnostics add no worker lifetime.
- A tolerance cannot turn an uncertified sketch fragment into an accepted input or an exact reading.
- A message-format change cannot alter any branch, diagnostic code, operand role, or bound field.
- A warning cannot hide an invariant failure or replace the operation's returned error.
- A caller retaining many errors retains their referenced input bodies. Success warning storage must not do that.

## Focused repository proof fixtures

Start with one real path: extend the small face-touching box construction in
`apitest/boolean_gap_tolerance_test.go`. The real boolean evaluator must produce a charged shift that
appears through the real public body accessor. Prove its volume bound against 2000 mm³ and preserve its
single-lump assertion. Stop expansion if private evaluation cannot deliver this record through commit.
Do not replace that boundary with a handcrafted warning fixture.

After that boundary passes, add these focused assertions in the implementation task:

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

Use exported-API fixtures in `apitest/` and private transport tests beside the transport implementation.
Use the existing geometry constructors and real producer/consumer paths. Compare fields, sentinels,
counts, and bounded numerical invariants; never deep-dump bodies, errors with body subjects, or topology.
Check bounded failure formatting with a small deliberate mismatch. Follow [Test guidance](testing.md)
for new test names and CI shards. Keep full consumer models in their consumer repositories.

## Settled scope and API decisions

Use the body-local warning accessor defined in Delivery API proposal. Preserve current operation signatures;
this scope introduces no parallel detailed-result entry points. Warnings become inspectable after commit.

Use the initial catalog defined in Successful-call warnings. Its `WithMatchingFaces` exclusion is settled
scope; adding that event requires the producer evidence and typed detail specified there.

Use the dedicated chain record defined in Record proposal. Expose the touched bound, ordinary pair chord
limit, and configured chain allowance together rather than reducing them to one effective threshold.

Use the error wrapping rules in Delivery API proposal. Preserve existing `BooleanError`, `SelectionError`,
and cancellation delivery. Wrap only otherwise unstructured Decad errors and preserve their error chains.

## Ordered implementation tasks

Implement these phases in order. This section is an execution checklist, not a second API definition;
the linked contract sections above win if wording differs. Proposed new filenames below are implementation
destinations, not claims that those files or APIs already exist. Keep each phase reviewable; no phase
requires opening a PR before the first real producer-to-accessor fixture passes.

### Phase 0: Resolve prerequisites and map entry points

- Read Public API, Evaluator, both Boolean, Verification, Interference, Gap tolerance, Faceted vertex bounds,
  Surface, and Operation history designs before changing their code. Follow repository pre-read and worktree rules.
- Inspect `agent-docs/layout/`, root public modeling functions, and built-in selector resolution. Record an
  entry-point checklist in implementation scratch under `.tmp/`: operation kind, argument names and ordinals,
  gate order, commit owner, error sources, and whether the call can reach a chosen warning producer.
- Record the dependency on the shared vocabulary from
  [Operation history design: Operation kinds](operation-history-design.md#operation-kinds).
  History Phase 1 owns `operation_kind.go` and `OperationKind`; do not add either independently here.
  Diagnostics does not require history lookup or recorded-call coverage beyond that shared vocabulary.
- Plan diagnostic-specific coverage for using each shared kind in operation records, including selector kinds,
  zero, and unknown values. Leave the shared enum's definition and token tests with its history owner.
- Acceptance: every in-scope entry point has an explicit kind and public argument mapping; every terminal error
  has an owning source. The shared vocabulary has one implementation owner and an explicit dependency.
- Stop if the shared vocabulary conflicts with another active implementation, or an entry point requires a
  contract change. Resolve that prerequisite before inventing a diagnostics-only vocabulary or changing behavior.

### Phase 1: Add records, error transport, and immutable accessors

- Depends on Phase 0 and the shared vocabulary landing from
  [History Phase 1](operation-history-design.md#phase-1-add-unconditional-storage-and-the-atomic-publication-seam).
  Add root `operation_diagnostics.go` and `operation_diagnostics_error.go` for the surface in
  [Delivery API proposal](#delivery-api-proposal) and [Record proposal](#record-proposal).
  Keep algorithmic aggregation in a new leaf package, `internal/opdiagnostic`, with plain-data records only.
- Implement diagnostic enums and total `String()` methods, typed subject/change/chain/work-limit details,
  `OperationDiagnosticError`, and `OperationError`. Preserve its original error object through `Unwrap` and its
  text through `Error`; do not reconstruct causes from messages.
- Add body-local immutable storage through `internal/topo/body.go` and root's `Body` adapter in `topology.go`.
  Store private summaries without input body references. Root adapters alone turn error locations into original
  caller body handles and convert private lengths to public `units.Value` fields.
- Implement newly owned slices and deep copies of optional detail records, including `ChainAllowance` and any
  optional ordinal. Keep copied body identities shared only where the error-subject contract permits them.
- Add `operation_diagnostics_test.go` and `apitest/operation_diagnostics_test.go` for enum tokens, unknowns,
  `errors.Is`, caller `%w` wrapping plus `errors.As`, original error text, and independent accessor copies.
  Test diagnostic records using each shared operation kind, including selector kinds, zero, and unknown values;
  test diagnostic-owned enum tokens here and shared enum tokens only at their history owner.
  Keep synthetic records limited to these value/ownership tests; geometry transport needs Phase 2's real fixture.
- Acceptance: supported error branches survive `OperationError`; zero records have unknown codes; body accessors
  can return only warnings. Mutating one returned optional value cannot change the next read or another body.
- Stop on a package cycle, a need to import root from `internal/`, or a need for a document collector, context
  collector, public callback, or parallel detailed-result entry point. Use private return records instead.

### Phase 2: Prove one real shifted Boolean end to end

- Depends on Phase 1. Modify `internal/booleaneval/shift.go`, `internal/booleaneval/eval.go`,
  `internal/booleandispatch/dispatch.go`, `boolean.go`, and the body publication adapter only as needed to carry
  the accepted retry's typed shift summary to `(*Body).OperationDiagnostics()` before `document.go` commits it.
- Derive displacement from `shiftOperandMesh`'s actual accepted per-piece charges. Summarize the moved pieces
  with a certified upper bound; do not use configured reach as the displacement or charge untouched pieces.
  Keep rejected-attempt records local and discard them when retrying or returning the original refusal.
- Extend only the face-touching Union subtest in `apitest/boolean_gap_tolerance_test.go` first. Use its real
  operands, dispatch, mesh evaluator, result builder, commit, and public accessor. Assert the expected warning,
  public operation/location, positive displacement within allowed reach, one lump, and volume bound covering
  2000 mm³. Run this subtest before adding the remaining warning mechanisms or expanding the test matrix.
- Acceptance: the real producer's charged change reaches the immutable public body with no fabricated record,
  changed route admission, extra producer identity, or extra committed body.
- Stop expansion if this fixture fails. Report the boundary that failed, required repair scope, and measured
  cost. A defect in a prerequisite needs a scope decision before repair; a handcrafted transport fixture does
  not replace this gate.

### Phase 3: Preserve failure classifications and add typed reasons

- Depends on Phase 2. Extend `errors.go`, `selection_error.go`, `boolean.go`'s `asBooleanError` and
  `newBooleanError`, private expected outcomes in `internal/meshbool/boolean.go`, and root entry points from
  Phase 0's checklist. Keep root adapters thin; place reusable plain-data mapping in `internal/opdiagnostic`.
- Add accessors directly to existing `BooleanError` and `SelectionError`. Preserve their concrete identity,
  existing fields, sentinels, `BooleanError.Code`, selection rendering, and residual order. Selector diagnostics
  must name selector resolution even when an enclosing modifier, stop, or axis propagates the error.
- Wrap only otherwise unstructured, in-scope Decad operation errors in `OperationError`. Preserve original
  validation order and error precedence. Return the exact `ctx.Err()` on cancellation/deadline; preserve
  caller-created typed-nil panics. Leave arbitrary writer and third-party errors under their current contracts.
- Implement the failure catalog at the source that knows the reason: input/ownership gates, sketch recording,
  operand proof/chording, contact classification, feature reach, result emptiness, and invariant checks.
  Do not derive subtype distinctions from `ErrUnsupported`, `ErrDegenerate`, or error text alone.
- Preserve typed counts at existing resource gates in `internal/prismcells/work_cap.go`,
  `internal/proofbound/budget.go`, and their modeling callers when those gates actually know observed and
  maximum counts. Emit `WorkLimitReached` with the existing sentinel mapping; do not add limits, continue
  evaluation for extra evidence, or invent observed counts for an opaque refusal.
- Extend `apitest/boolean_test.go`, `apitest/selection_error_test.go`, and small private error tests for every
  catalog mapping. Include `TestImplicitExactlyOneStopFailedAssertionPassesThrough`, malformed-input versus
  coarse-chording `ErrDegenerate`, proof-unavailable versus contact `ErrUnsupported`, nil context, and existing
  cancellation fixtures. Compare error pointers and bounded fields instead of deep-dumping error graphs.
- Acceptance: caller wrapping exposes the new interface while all existing supported branches and gate
  precedence survive. Analytic admission misses stay silent; admitted-route refusals keep their current mapping.
- Stop if a source cannot establish a proposed distinction. Use the catalog's broader known reason or document
  the missing producer detail; never change classification or parse a message to fill it.

### Phase 4: Carry the local chain gate's three inputs

- Depends on Phase 3. Change `internal/booleaneval/eval.go`, `internal/meshbool/boolean.go`,
  `internal/meshbool/staging_error.go`, `internal/meshbool/boolean_mesh.go`, and the near-contact gate callers
  to preserve typed touched bound, ordinary pair chord limit, and optional configured allowance.
- Keep `CoarseHeldContactError` details discoverable after `StagingError` formats its text. The current
  formatting path loses the typed cause; transport it without changing the existing public sentinel mapping.
- Add `ChainAllowanceApplied` only when a real touched gate needs the configured allowance. Preserve the
  ordinary pair chord separately from `max`'s effective threshold. Keep retry shift credit separate privately;
  credit already charged by a shifted retry must not create a chain-allowance warning by itself.
- Aggregate repeated local gates without combining unrelated threshold values into a fictitious comparison.
  Preserve an actual gate's tuple and a conservative summary under [Atomicity, order, and storage](#atomicity-order-and-storage).
  Establish that aggregation rule in private transport tests before using it across multiple internal steps.
- Extend `TestBooleanChainDepthGateReadsOnlyTheTouchedRims` in `apitest/boolean_test.go` and
  `TestChainToleranceAdmitsAContactOnACoarserRim` in `apitest/chain_tolerance_test.go`. Replace the latter's
  regexp/`ParseFloat` extraction with structured fields. Cover absent allowance, insufficient allowance,
  sufficient allowance, an allowance configured but unused, untouched coarse regions, and retry credit alone.
- Acceptance: refusal and warning expose real local values with length kinds and correct original input
  locations. Geometry bounds and pair chording are unchanged; configuring an allowance alone emits nothing.
- Stop if only the effective gate survives a boundary, or repeated gates cannot produce a truthful bounded
  summary. Preserve the missing inputs at their producer before publishing a partial or misleading record.

### Phase 5: Report accepted positive-reach embedding repairs

- Depends on Phase 4. Add typed actual-use output at `RepairHeldEmbedding`/`EnforceHeldEmbedding` in
  `internal/meshbool/held_embedding.go`. Propagate it through `internal/meshbool/mesh_stitch.go`,
  `internal/facetproof/faceted.go`, `internal/facetproof/placement.go`, `internal/facetproof/similarity.go`,
  `internal/facetedrecord/placed.go`, and their public placement/scale adapters in `topology_handles.go`,
  `document.go`, and `point_plane_scale.go`.
- Track whether the final accepted geometry uses a positive-reach candidate. Distinguish that event from
  ordinary nearest rounding, float-box corner repair, inherited vertex bounds, and failed reach trials.
  Reuse the existing final-position proof to bound displacement; keep geometry and its existing charges unchanged.
- Add `EmbeddingReachApplied` at the outer public operation/location. Cover mesh-Boolean result repair,
  rigid placement, and similarity placement. A copy or transform gets only its newly applied repair summary;
  inherited bounds do not copy the source body's warnings.
- Extend `internal/facetproof/placement_reach_test.go`, corresponding `internal/meshbool` reach and clean-mesh
  fixtures, and the similarity tests. Add the smallest real public placement fixture reaching the repaired
  payload; retain the existing embedding audit and displacement assertions. Cover positive reach, float-box-only
  success, no movement, refused repair, and cancellation before publication.
- Acceptance: every reached repair producer supplies actual-use evidence; only final accepted reach changes
  warn. Public warning bounds cover the real displacement and do not substitute total accumulated body bounds.
- Stop if a producer exposes only move counts or total rounding bounds. Add the typed stage detail there;
  do not infer reach use from `Approximate`, a positive bound, or a nonzero configured tolerance.

### Phase 6: Report actual tolerant Stitch merges

- Depends on Phase 5. Extend `internal/stitchrecord/plan.go`, `internal/stitchrecord/build.go`, and `stitch.go`
  to transport actual accepted merge summaries. Reuse `Plan.Merged` and its charged separation bounds.
- Map touched source vertices/faces to the caller's `bodies` ordinals while inputs are available. Publish only
  argument locations and conservative merge summaries; keep private vertex classes out of public records.
- Emit `StitchMergeApplied` only for distinct positions merged by the accepted plan. Discard a tolerant plan's
  records when `loopsKeepTheirShape` selects the exact plan. Replaying an old plan during placement must not
  misreport that inherited merge as a new tolerant Stitch event.
- Extend small `internal/stitchrecord` plan tests and `apitest/gap_tolerance_test.go` with a minimal planar
  Stitch fixture. Assert the real merge warning and existing geometric/volume bound, plus exact positions,
  `ExactOnly`, ineligible curved/bounded vertices, and rejected collapsing/repeated-loop merges.
- Acceptance: warnings describe only accepted distinct-position merges; unchanged exact plans have no merge
  warning. The existing weld eligibility, closure classification, and charged geometry remain unchanged.
- Stop if a warning needs geometry to be remeasured or a weld admission rule to change. Use the accepted plan's
  existing proof data; a new geometry capability is outside this task.

### Phase 7: Finish composite publication, order, and storage bounds

- Depends on Phase 6. Audit `boolean_many.go`, `pattern.go`, `tool_combine.go`, `brep_modify.go`, their callers,
  `document.go`'s `commitOperation*` helpers, and the private builders used by those operations.
  Carry warnings locally until the outer public operation has its final result and final cancellation check.
- Normalize operation kind and argument locations at the public boundary. Map a failing step back only to
  original caller inputs; omit a body subject when a transient composite prevents a sound attribution.
  Discard earlier successful-step summaries after a later failure or abandoned route.
- Complete shifted retry error transport: an accepted empty Intersect keeps `BooleanEmpty` and
  `ErrBooleanFailed`, with its own applied-shift warning beside `ResultEmpty`. Exhausted retries keep the
  original refusal and no applied-shift warning. A failure never publishes a partial body.
- Add deterministic merge/aggregation in `internal/opdiagnostic`: public argument order, stage order, then code
  token. Bound storage by catalog and public locations, with one summary per mechanism/location. Worker results
  merge in input order; facet count, trial count, or private route count must not grow the public inventory.
- Keep final warning attachment before commit and never mutate a published body. Audit body-copy/placement
  code for accidental warning inheritance. `Remove` must allocate no warning record and retired-body reads must
  still work. Add no producer allocation, operand mutation, or new concurrent-document-mutation permission.
- Add real three-input Union final-step refusal and composite-success fixtures in `apitest/`; preserve all
  original live inputs on refusal. Add private aggregation/order tests and public accessor tests for retired
  bodies, fresh copies, `%w` errors, concurrent reads, optional-pointer copies, and unaffected later calls.
- Check bounded formatting with a tiny deliberate mismatch under hard memory/time limits. Compare scalar
  fields, tokens, counts, and geometric invariants; never pass bodies or body-owning errors to a recursive diff.
- Acceptance: outer success publishes all and only its accepted summaries once. Failure/cancellation preserves
  the live set and producer identity; the existing level/curve failed-build semantics do not change.
- Stop if aggregation needs per-facet storage, references to consumed inputs on a successful body, or mutation
  after commit. Return bounded private summaries from producers instead.

### Phase 8: Prove verification separation and complete API documentation

- Depends on Phase 7. Inspect `verify.go`, `verify_result.go`, `report/verify.go`,
  `internal/verifyrun/interference.go`, and `internal/verifypublish/`. Shared private reasons may be read there;
  public operation records must not enter verification inventories or change cause-specific pair mapping.
- Extend `TestVerifyGapToleranceDecidesAFlushPair` in `apitest/verify_gap_tolerance_test.go`. Verify
  a real shifted result with a loose request and a stricter request. Assert Sound plus operation warnings when
  bounds pass, existing bound diagnostics when they fail, and unchanged creation warnings after read-only Verify.
  Preserve existing invariant-error and cancellation behavior; warnings never decide the verdict.
- Add one ordinary exact rectangular operation with exact readings and an empty warning slice. Cover
  `WithMatchingFaces` with no new warning code, consistent with the settled catalog exclusion.
- Update `docs/api.md`, `docs/data-structures.md`, public Go doc comments, and an executable example under
  `examples/` for body warnings, `errors.As` through caller wrapping, and stable code branching. Explain the
  unchanged typed-error/cancellation exceptions and empty-inventory meaning by linking their owning contracts.
- Update this design's implementation status only for delivered behavior. Update `agent-docs/api-design.md`,
  affected companion contracts, and owning rows in `agent-docs/layout/` for every added root Go file and internal
  package. Follow [Splitting oversized agent files](layout.md#splitting-oversized-agent-files) if a budget fails.
- Acceptance: operation diagnostics introduce no warning-as-error mode, report verdict field, or warnings on
  Mesh/export/motion/dynamics results. Executable examples show current API behavior and bounded field output.
- Stop if a query-only repair needs a new result location, or verification's Sound/empty-inventory contract
  would change. Those changes require a separate proposal.

### Phase 9: Run focused checks and prepare review

- Depends on Phase 8. Add every new/renamed root, `apitest`, fuzz, and example name to its owning shard manifest.
  Put unmeasured tests in shard 10; do not rebalance without measured `_shardgen` input.
- At each preceding phase's checkpoint, run only the touched package or named fixtures. Before handoff, run
  the selected public diagnostics fixtures, private transport/producer tests, example tests, and the focused
  concurrent-accessor tests with `-race`. Store reusable or large output under `.tmp/`.
- Run `go test . ./apitest/ -run '^TestCI'` for changed test names and
  `go test . -run '^TestCLAUDEMDLayoutStaysCompact$'` for layout changes. Run the repository's applicable API/doc
  guard tests discovered beside the touched public surface. Fix failures without weakening assertions.
- Before any Go-code PR push or creation, follow `git-operations.md`'s Go Lint Before PR requirement. Current
  `.github/workflows/ci.yml` runs golangci-lint v2.12.2 and `go vet ./...`; run
  `golangci-lint run --timeout=5m` and `go vet ./...` after the last Go edit and resolve change-caused failures.
- Let remote CI run the full suite. Follow Push Guard, message lint, and PR CI Watch rules for any authorized
  PR delivery; watch checks to completion and report the actual CI result. Do not call focused local checks
  the full suite. If CI is unavailable, follow the repository's full-suite fallback rule.
- Acceptance: the Phase 0 checklist has no unhandled in-scope entry points, every chosen warning has a real
  producer/consumer proof, supported error branches pass focused tests, documentation/layout/shards agree,
  required lint passes, and any opened PR has completed checks.
- Stop on unresolved behavior failures, unavailable required lint, out-of-scope prerequisite defects, or CI
  failures requiring a design decision. Report the exact failed check, bounded evidence, and remaining scope.

## Behavioral reference boundary

Read-only reference inspection used the sibling OCCT checkout's
`src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Options.hxx` and
`src/FoundationClasses/TKernel/Message/Message_Report.hxx`.
The relevant observed behavior was per-operation inspection of typed failures and warnings, including
results from nested work. This proposal uses Decad's existing Go errors, immutable body results, and proof
contracts to specify that behavior independently. No OCCT implementation, declaration, or text is copied
or adapted. OCCT is not a dependency, a runtime oracle, or an interoperability test requirement.
