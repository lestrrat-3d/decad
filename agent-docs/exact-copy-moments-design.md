# Exact rotation copy moment design

## Status and owners

This slice is implemented for one direct public exact-rotation Copy of an
admitted original contained Cut. It retains the existing source and mesh
checks and transfers moments through the exact rotation record.
`contained-cut-moments-design.md` owns the implemented source-moment capture
for a rectangular-prism target minus a whole circular-prism tool.
This proposal transfers that evidence through one direct public
`Copy(..., WithCopyRotation(...))` and consumes it in `MassProperties`.

Use `dynamic-mass-design.md` §2–§3 for V/P/Q, physical inertia signs,
density, and positivity. Use `evaluator-design.md` §8 and
`mirror-pattern-design.md` "Exact rotation copies" for the copy's denoted
solid and its existing placement proof. Use `operation-history-design.md`
"Atomic publication and storage" for commit and public history.

## Selected calculation and alternatives

Compute tighter mass and inertia readings for an exact rotation copy of an
already admitted contained Cut. Preserve the retained difference moments;
rotate them by the exact rational rotation the real copy denotes. Compare
the complete candidate with that copy's independently audited mesh result.
This adds one producer-to-consumer boundary without adding a source class,
containment algorithm, Boolean route, or public API.

- More contained-Cut source profiles improve initial parts but need new
  integration and admission proofs for every profile.
- A second contained Cut improves machining histories but needs a new
  whole-material containment proof for its faceted target.
- General Copy and Transform cover more placements but must transfer the
  actual float delta, including determinant and stretch.
- Direct exact rotation Copy reuses the existing exact image map, admitted
  source and query gates. This is the selected slice.

The result must demonstrate smaller certified inertia bounds after actual
placement. Recording more ancestry without improving a computed reading
does not satisfy this slice. Runtime reduction is not an acceptance claim.

## Admission and scope

Attempt capture only in `Body.copyUnder` when all these facts hold:

- The public operation kind is `OperationCopy` and its final selected option
  supplied a non-nil `exactrotation.Rotation` to `facetedrecord.PlacedWith`.
- The source is an original solid faceted contained-Cut result with present,
  valid `VolumeLineage.Moments` matching its own producer.
- The source payload has identity `Xform`, nil `CopyLineage`, nil
  `ExactCopyMoments`, and `ExactPlaced == false`.
- The real copy rebuild succeeded. Its direct `CopyLineage` identifies that
  exact source, source producer and placements, and the same immutable exact
  rotation passed to the rebuild.
- The original moment record validates against its original source-volume
  lineage, including its strict containment facts and source identities.

Source admission is inherited from the validated contained-Cut record.
Do not infer a cylinder, box, cavity or containment from a copied mesh,
bounding box, public operation kind, ancestry, or a small residual.

Default Copy, `WithCopyPlacement`, Transform, MirroredCopy, ScaledAt,
patterns, modified bodies, a copy of a copy, and another Cut do not transfer
moments. A zero-angle `WithCopyRotation` is admitted through the same exact
record path. Repeated Copy options keep their existing last-option rule.
An unsupported source or absent evidence preserves existing behavior.
Present malformed source evidence in this admitted route fails before commit.

## Immutable evidence and layering

Add `ExactCopyMoments *ExactCopyMomentRecord` to `facetedrecord.Payload`.
Put the new record in `internal/facetedrecord/exact_copy_moments.go`.
Keep `topo.Body`, `prooflineage.VolumeLineage`, and the topology dependency
floor unchanged. In particular, do not import `exactrotation`, `massmoment`
or `facetedrecord` into `prooflineage`.

The record freezes these values behind unexported fields:

- The original Cut producer, receiving copy producer, and their held
  placements.
- A copied original `VolumeLineage` with only its scalar source/result
  readings, source producer identities, containment facts and a freshly
  frozen `ContainedCutMoments`. Omit its unused centroid lineage.
- The same immutable `exactrotation.Rotation` used by the real rebuild.

The constructor validates the original moment record, reads independent
source snapshots, and rebuilds its private contained-Cut moment record with
`prooflineage.NewContainedCutMoments`. Every rational endpoint is thereby
copied. Sharing the exact rotation is permitted: its fields are private and
its accessors return fresh rational copies.

`Read` validates the receiving producer and placement, then validates the
frozen original moment record again and returns its difference snapshot
and independent rotation coefficients. Check non-nil rational coefficients,
exact orthogonality and determinant +1. Never use a float tolerance to admit
the map. Store no density, selected tuple, query cache, source body,
topology node, document, context, or caller option in this record.

The existing `FacetedCopyLineage` has separate live-source consumers. Use it
only to check this call's direct rebuild during capture. Mass queries use
the frozen record after the original Cut body is removed or retired.
This does not change `CertifyCopies` or its live-source requirements.

## Moment transport

Let the validated source difference be V/P/Q about held world anchor O.
Let the copy denote `x' = Q x + t`, using the exact record, not the float
placement or its accumulated basis. Q is exactly orthogonal with determinant
+1. About the exact image anchor `O* = Q O + t`, compute:

```text
V*    = V
P*    = Q P
Q*    = Q Q_source Q^T
```

Use `massmoment.Transform(source, rotation.Linear())`. Its determinant
factor is exactly one. Source uncertainty already belongs to each input
interval; do not cancel it or replace it with a point estimate.

Choose a finite held anchor A near O* with the existing
`facetedrecord.PlacedCapCentroidBy` routine. Supply an Exact zero-radius
reading whose value is O, and the exact rotation and translation. The
returned bound e encloses only rounding from O* to A; O is an arbitrary
integration anchor, so the source body's centroid uncertainty is irrelevant.
This reuses the existing exact-map arithmetic and introduces no new
coordinate transform implementation.

Enclose `D = O* - A` by `D_i = [-e,e]`. Reuse the interval-translation
arithmetic currently owned by `massmoment.widenTranslation`, exposed as
`WidenTranslation`, on the rotated moments:

```text
V'    = V*
P'_i  = P*_i + V* D_i
Q'_ij = Q*_ij + D_i P*_j + P*_i D_j + V* D_i D_j
```

Read every right-hand term from the unshifted rotated snapshot, fill the
second matrix symmetrically, and return without widening when e is zero.
The returned snapshot uses A. This independent interval enclosure may be
wider than an exact correlated shift; the ordinary comparison decides
whether it is useful. Do not charge the held mesh's vertex rounding,
embedding correction or `VolSymDiff` again: those describe its distance to
the exact image, not a change of that image's source moments.

Do not substitute a float rotation, a nominal sine/cosine rotation by the
requested angle, a bounding-box tensor, or the source's published inertia.
The exact rotation selected by `WithCopyRotation` is the copy's denotation.

## Capture and query flow

`massdispatch.CaptureExactCopyMoments` runs after the real copy rebuild and
direct copy-lineage binding, before the final context check and
`commitOperationIn`. It validates and freezes the record, transports the
moments, checks its volume against the rebuilt body's cached Volume and its
computed center ball against the rebuilt body's cached centroid, then
returns the optional record. Disjoint certified evidence fails the Copy.
An unrepresentable finite anchor or center omits this optional evidence.
Do not call public `MassProperties`, rerun containment, or create an extra
public operation during capture.

Root attaches the returned record to the pending copy's payload by value.
It leaves the rebuilt body's Volume, centroid, topology, bounds and mesh
proofs unchanged. The source's lineage also stays unchanged.

For faceted `MassProperties`, obtain `facetedMassProperties` first, including
its matching VerifyAll certificate and complete existing audits. Then read
either the original contained-Cut moments or this exact-copy record. Both
present at once is contradictory evidence. Transport the exact-copy record
as above and pass its snapshot to the existing source publication code.
Compute center, central tensor and density scaling from that one V/P/Q
enclosure. Keep its positivity and finite-reading gates.

Apply `contained-cut-moments-design.md` "Selection" to the resulting complete
tuple, using the receiving copy's cached centroid in its consistency check.
The ordinary candidate remains available. A mesh failure is not rescued by
source evidence. Ties, wider consistent source bounds and optional publication
limits return the ordinary tuple unchanged. No component mixing or
density-dependent caching is permitted.

## Dropping evidence on descendants

Clear `ExactCopyMoments` unconditionally in `Payload.PlacedWith` before any
rebuild and in `ScaleImage` before returning its new payload. Only the
authorized direct public Copy capture can attach a new record afterward.
Every Boolean or modify result must start without this record. Check the
payload-copy rewrite in `loftcylinder/chamfer_build.go`; it must clear the
record too. Fresh payload literals already omit it.

Keep identity/placement validation even though those builders drop the
record. An accidentally retained record must fail as contradictory evidence
instead of publishing the previous body's moments. Preserve `ExactPlaced`
and the existing motion-query refusals independently of this new field.

## Errors, context and atomicity

| Condition | Outcome |
|---|---|
| Operation, source or evidence is outside Admission and scope | Keep ordinary behavior and omit the new record. |
| Any required evidence or identity is malformed | Return an evidence error wrapping `ErrBooleanFailed`. |
| Certified capture volume intervals or center balls are disjoint | Fail before Copy commit with `ErrBooleanFailed`. |
| Capture cannot publish a finite anchor or center | Omit optional evidence and keep the rebuilt copy. |
| Ordinary mass mesh admission fails | Return its existing error and zero properties. |
| Query finds contradictory or disjoint evidence | Return `ErrBooleanFailed` and zero properties. |
| Valid source publication cannot prove positivity or finite readings | Return the ordinary successful tuple. |
| Cancellation after existing input validation | Return `ctx.Err()` unchanged and no partial output. |

Copy keeps its normal operation error wrapper. Do not add a BooleanError
classification to Copy. Contradictory evidence never selects fallback.

Poll context before reading/integrating evidence, after transport and before
publication. Keep the existing public input-validation order. Fixed-size
moment transport adds no mesh refinement or separate work-budget policy.

All capture and consistency checks precede commit. Failure leaves source
liveness, ordered live bodies, producer allocation, container membership
and operation history unchanged. Successful Copy references its source and
adds exactly one Copy operation. Queries add no operations and do not change
geometric measurements. Concurrent queries read immutable evidence only.

## First real acceptance fixture

Build the sources with `containedCutCentroidSources` and run the real public
Cut from `TestContainedCutSourceMassProperties`. Copy that real result with
`WithCopyRotation`, center `(10,10,10)`, axis `(1,2,3)`, angle 37 degrees.
Use the real rebuild, original moment producer, frozen copy evidence,
verified mesh and public `MassProperties`; no fixture may invent a proof.

Require one material lump, one closed void shell, a direct exact copy,
and a new Copy history entry with its source still live. Compare against
the same copy's ordinary mass result with only this new consumer disabled
in a sequential test-local payload copy. Both results must enclose the
independent box-minus-cylinder reference after the exact rational rotation.
Check mass, center in 3D and all six inertia entries. All eight selected
bounds must be no larger, and at least one diagonal inertia bound must be
strictly smaller. Do not require Exact for pi.

Run this producer-to-consumer instance before adding the test matrix. A
failure stops expansion. Report the failing component and both bounds,
smallest reproduction and proposed prerequisite repair; do not weaken the
comparison or expand source admission to obtain a passing fixture.

## Further acceptance and boundaries

Cover density rescaling, a nonzero anchor-rounding enclosure, non-cardinal
rotation, repeated and concurrent read-only queries, source removal, copied
endpoint isolation, stale identities/placements, malformed source lineage,
disjoint evidence, optional fallback and cancellation. Test unsupported
placement paths without changing their existing geometry behavior.

No unresolved behavior choice remains in this proposal. Whether the real
rotated source candidate wins is unmeasured until the first implementation
fixture runs. No general placement transfer, chained moment record,
reconstruction, face-history expansion, new dependency, OCCT code, or source
code implementation belongs to this design-only task.
