# Queries after a gap-shifted Boolean

Status: Proposal; unimplemented.

## Scope and owners

Certify point classifications that a moved second operand cannot change. Preserve the existing mesh proof for all
other points. Keep general exact `Body.Section` refused until a producer certifies the requested planar set.

`gap-tolerance-design.md` owns retry geometry and its charges. `point-containment-design.md` owns the public
classification and mesh-transfer proof. `section-design.md` owns feature-ready section admission.
`operation-history-design.md` owns computational certificates. Those documents win over this proposal until an
implementation updates their applicable sections.

This proposal changes no public type, tolerance, geometry, diagnostic or measurement bound. A certified
`PointInside` does not make the body's volume `Exact`.

## Verified current behavior

The following observations apply to base `5ed2045c6`.

| Site | Observed behavior |
|---|---|
| `internal/booleaneval/eval.go`, `Evaluate` | The ordinary evaluation runs first. Contact refusal and positive gap reach can retry with the second operand changed. |
| `internal/booleaneval/shift.go`, `pieceShift` | Each connected piece can translate, grow or shrink independently. A piece without a requested change stays put. |
| `internal/booleaneval/shift.go`, `shiftOperandMesh` | The decision copy moves coordinates. The charged copy approximates the original operand and widens displacement and volume bounds. |
| `internal/booleaneval/eval.go`, `ShiftSummary` | The accepted evaluation publishes the largest displacement and allowed reach. It does not preserve the per-piece map for later queries. |
| `internal/booleandispatch/dispatch.go`, `Body` | The faceted output receives the accepted evaluation's diagnostics after its mesh is built and audited. |
| `internal/boolchain/chain.go`, `Expandable` | A geometry-change warning prevents original-operand Boolean replay. This also protects regrouping and history sections. |
| `internal/pointquery/history_location.go`, `LocateBoolean` | The query uses that eligibility gate, so a shifted result falls through to the verified mesh. |
| `internal/pointquery/location.go`, `locate` | Exact held parity transfers to the body's true region only after boundary and occupied-volume bounds prove the answer. |
| `section_outline.go`, `SectionOutline` | A shifted result already has mesh outlines with `OffSurface`; they are not profiles and do not bound true section area. |

`apitest/point_containment_history_test.go`, `TestPointLocationHistoryRefusals/gap-shifted_union`, builds a
30 × 30 × 4 plate and its full-width 45-degree copy about the x axis. The public Union takes the shifted retry.
Its point at `(1, 1, 3.999)` is inside the unchanged plate, but currently reads `PointUndecided`.
The test explicitly identifies this as the conservative eligibility choice.

The focused baseline passed:

```text
go test ./apitest -run '^TestPointLocationHistoryRefusals$' -count=1 -v
```

The saved output is `.tmp/gap-shifted-query-baseline.log`. No new query implementation was tested.

## Denotation and proof boundary

Keep three objects distinct:

- `T0` is the exact requested Boolean over the recorded input solids.
- `Mδ` is the occupied held mesh selected after moving pieces of the second operand and stitching the result.
- `T` is the body's true region under the existing public containment contract.

The charged mesh bounds relate the approximation to original geometry. They do not assert that the tolerant
contact decision preserved `T0` exactly. The retry can open a wall or join a contact, as the gap-tolerance owner
states. A diagnostic that says which operand moved is not a certificate that `T = T0`.

Do not redefine `T` as `Mδ`, and do not add a public choice between literal and tolerant query meanings.
The existing public contract still decides which region the result denotes. Use a stronger sufficient proof:
accept only an input-derived conclusion that holds for the existing region and is independent of every permitted
change to the second operand. This cannot return different answers by choosing an interpretation of the retry.

The unchanged-target relation is an implementation proof obligation; no existing evaluator certificate states it.
The producer must certify that its first input's exact solid is unchanged in this relation. Merely observing
that the first tessellation's vertex array was not translated is insufficient: additional embedding repair,
specialized trim substitution or later modification can invalidate that relation.

## First increment: one-sided point implications

Let `A` be the unchanged first operand and `C` any admissible second-operand solid after the retry.
For regularized solid operations, the following implications require no information about `C`:

| Operation | Exact classification in `A` | Certified classification in the output |
|---|---|---|
| `Union(A, C)` | Strictly inside | `PointInside` |
| `Cut(A, C)` | Strictly outside | `PointOutside` |
| `Intersect(A, C)` | Strictly outside | `PointOutside` |

An interior point of `A` has an open neighborhood in `A`; that neighborhood is in every Union above.
An exterior point of regular closed `A` has an open neighborhood disjoint from `A`; that neighborhood is disjoint
from every Cut and Intersect above. These statements also hold for the requested second operand, so the proof
does not replay original inputs as a set-equality claim about the changed result.

Return no boundary answer through this increment. A point on `A`'s boundary can become internal to a Union,
and a Cut or Intersect can remove it. All other source classifications hand the point to the existing mesh proof.

### Producer certificate

Add a private immutable query certificate for an original two-input generic mesh Boolean. It records:

- The public producer identity, operation kind and ordered input producer identities.
- The accepted output identity and its original coordinate frame.
- The evaluator's proof that only second-operand retry geometry changed, under this increment's relation.

Mint it only after the successful output audit and before operation commit. The constructor must validate the
actual accepted evaluation path, not reconstruct its proof from `OperationDiagnostics` or a caller-set marker.
Keep quantity and boundary charges exactly as they are.

Issue no certificate for a result with embedding-reach movement, a point-cone trim, a specialized source
substitution or unknown change mechanism. Limit the first increment to an exact record-class first operand
that `pointquery.LocateRecord` already classifies and whose actual first-operand tessellation has verified boundary
and occupied volume with `Bound == 0` and `VolSymDiff == 0`. This admits the straight-sided plate fixture and
excludes a chorded circle even when its source record is exact. Check those facts on `ma` before `PrepBoolMesh`.
The proposed certificate must establish that the exact mesh Boolean combines `A` itself with the accepted second
operand before final rounding, and that assembly approximates that operation without redefining its source `A`.
Read the retained original input through operation history;
retirement does not invalidate it. Do not retain a mutable mesh, document cache or public diagnostic slice.

`StitchFacetsContext`, `RefuseRimPastPair` and `AuditFacetedMesh` currently establish rounded output construction,
finite trim amplification and the held shell audit. `VolumeReading` composes occupied-volume error.
None of those post-stitch gates alone proves the unchanged-target relation. Producer certification is therefore
a prerequisite, not an inference from the existing gates or a positive diagnostic.

Validate that prerequisite on the real crossed-plate fixture first. In a focused evaluator prototype, capture the
accepted `ma`, selected exact `kept` facets and stitched output from the same call. Check `ma`'s zero bounds and
source identity, the exact kept-set classification at the acceptance point, its stitched classification and
published charges, and the absence of every additional change mechanism. Then audit the exact-facet-to-output
provenance rules for the entire admitted path. The fixture settles the component boundary; the general certificate
still needs the set implication and provenance proof. If assembly provides only a charged approximation with no
such relation, stop this capability slice and retain current bounded mesh queries.

Attach the certificate to the original output only. Drop it on `Copy`, `Transform`, `ScaledAt`, retained blends,
subsequent Boolean outputs and every other rewrite unless that producer issues its own new certificate.
An identity placement does not restore the original producer identity.

### Reader

After ordinary exact-record/history routes miss, try this certificate before mesh preparation. Verify its output,
operation, ordered inputs and frame against the retained public producer. Ask `LocateRecord` about the unchanged
first input at the world point. Apply only the three implications above.

Do not widen `boolchain.Expandable`: regrouping and `Body.Section` need complete set equality.
Use a separate point-only query entry point. The proposed BRep point route in
`brep-point-containment-design.md` must not turn this limited certificate into a general Boolean identity.

Use `proofbound.WorkBudget` for certificate/input checks and existing exact predicates. Preserve the entry point's
point, chord-tolerance and solid checks. Cancellation returns `ctx.Err()`; a missing proof returns an undecided
route and preserves ordinary mesh fallback. No tolerance changes an inside/outside predicate.

## Distance exclusion and a possible second increment

The direct bounded exclusion available today is the existing mesh proof in
`point-containment-design.md`, "Proof". It uses held distance, the published boundary bound and occupied-volume
bound together. The retry already charges the shift into those bounds. Keep that proof for every first-increment
miss; a recorded displacement alone is not a replacement.

To decide points near an unchanged source face outside the three implications, a later producer can preserve:

- An exact source-to-piece association for every changed component.
- A certified correspondence or continuous motion from each source piece to its accepted held piece.
- Conservative world-space support boxes enclosing original and moved pieces, their rounding and source bounds.
- Evidence that stitching and output assembly introduce no additional unrecorded local change.

A point outside these closed support boxes has positive separation from every changed piece. Only a producer
certificate of local set equality permits replay there. A sharper boundary-distance test requires a certified
source boundary-distance lower bound strictly above its full motion and approximation charge, plus an occupancy
witness for that component. A boundary tube alone proves no global occupancy relation.

Use `r3` for coordinate/frame operations and exact or directed-bound helpers for separation. A placement must
transport the correspondence and its bounds before a certificate can survive it. Equality at a box face or at a
distance threshold is a miss. No sampling or small residual can admit a source relation.

Do not implement this larger increment until the first real producer/consumer case succeeds. Its tighter point
coverage and additional per-piece state must be measured separately; no cost improvement is established here.

## Section behavior

General exact history sections remain refused. A shifted second operand can change intersections with the
requested plane, and its new junctions need not be representable by an exact Sketch profile.
`SectionOutline` remains the bounded mesh-reading alternative under its existing contract.

A separate future slice may admit a plane with strict separation from every certified change-support box,
provided the producer also proves local output-set equality around that entire plane. It must then obtain exact
source sections and pass the existing Sketch arrangement and reconstruction gates. Whole entities record from
their source; every partial fragment requires upstream `TExact`. Current history sections also reject curved cuts
without an exact junction and any nonzero cut displacement. Keep those stronger reconstruction conditions.

The first acceptance fixture for that future slice is the existing flush-top cube Cut in
`TestSectionMeshBooleanHistoryRefusals`, queried at `z = 2`, below the tool's `z = 5` lower cap with strict certified
separation. The producer must prove the whole cut there equals the target's 10 × 10 square. Its reconstructed
profile must feed a 1 mm `Extrude` and read volume 100 mm³ with zero bound. Querying at `z = 7`, through the changed
tool, must still refuse. Running the real fixture must precede extending the method to curved sections.

This slice is blocked on a local-set-equality producer certificate. A finite gap diagnostic, global volume bound,
or entity count does not settle that prerequisite. No such certificate or acceptance result is claimed here.

## Files and verification

| File or area | Proposed responsibility |
|---|---|
| `internal/booleaneval/eval.go` | Identify the accepted generic retry path and expose point-certificate evidence after its audit. |
| `internal/booleandispatch/dispatch.go` | Bind evidence to the actual built output and ordered inputs. |
| `internal/facetedrecord/payload.go` and private record file | Store the immutable original-output certificate; audit every copying/rewrite producer. |
| Root operation commit and `historyProducers` adapter | Bind producer identities atomically and provide retained inputs without public mutation. |
| `internal/pointquery/history_location.go` and `point_containment.go` | Add the limited point-only route, leaving complete Boolean expansion unchanged. |
| `apitest/point_containment_history_test.go` | Exercise real shifted producer outputs and their public point consumer. |

Implement one small capability PR for the first increment after its producer prerequisite is proven.
The current conservative refusal is sound. Add focused certificate lifecycle tests and preserve
the existing mesh route tests. Update the owning containment and history docs and the matching missing-features
qualification when behavior ships. Register any new test names under `agent-docs/testing.md`.

Required fixtures and invariants:

- Change only `(1, 1, 3.999)` in the existing shifted Union fixture to `PointInside`; its top-face point remains on
  the ordinary fallback. Assert a real `OpDiagGapShiftApplied` and a positive volume bound before querying.
- Use real shifted flush Cut and Intersect fixtures for a point strictly outside their unchanged target.
- Preserve undecided/mesh behavior for target boundary points and input answers that are not in the table.
- Reject forged/missing/mismatched certificates, reversed Cut roles, embedding reach and unknown mechanisms.
- Prove a retired source remains readable and all placement/rewrite descendants lose the certificate.
- Check cancellation and invalid tolerances before any new route result. Report only scalar answers and bounds.

Run focused `internal/pointquery` and touched certificate-package tests, then the relevant `apitest` history cases.
Let CI run the full suite. The first baseline command above is the only test result established by this design task.

## Implementation task plan

Follow the [ordered implementation tasks](plans/gap-shifted-query-implementation.md).
This design owns the proposed contract; the plan supplies execution steps.
