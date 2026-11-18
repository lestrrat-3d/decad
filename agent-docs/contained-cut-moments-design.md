# Contained Cut and exact copy moment design

## Status and owners

The contained-Cut source-moment consumer is implemented for a rectangular
prism target and whole circular-prism tool. The implementation follows this
design's admission, interval, selection, and fallback rules. "Exact rotation
copy" transfers the captured evidence through one direct public
`Copy(..., WithCopyRotation(...))` and is implemented too.

`operation-history-design.md` "Contained Cut centroid" owns the existing
source-volume and source-centroid calculations. This document owns the
contained-Cut second-moment composition and its `MassProperties` consumer for
one source class.

Use `agent-docs/multibody-dynamics-design.md` Part M §2–§3 for moment meanings, density, physical
inertia signs, and positivity. Use `evaluator-design.md` §9 for the real mesh
Boolean and `interference-design.md` §4 for strict whole-solid containment.
`prism-boolean-design.md` and `general-boolean-design.md` own Boolean route
admission. This slice adds no route or geometric admission.

## Evaluator result

For a certified contained `Cut` of a rectangular-prism target by a tilted
whole-disk-prism tool, compute density-independent source volume, first moments,
and second moments before retiring either operand. Let the resulting faceted
body's `MassProperties` compare a source-derived result with its ordinary
audited-mesh result. Select the entire source result only under "Selection".

The concrete benefit is smaller certified mass and inertia bounds for an
already-buildable hollow solid. The source consumer does not inherit the
mesh's occupied-volume approximation. This slice claims no runtime reduction
and no additional `MassProperties` admission.

The first acceptance fixture is the real geometry in
`operation_history_centroid_test.go`, `TestContainedCutUsesCertifiedSourceCentroid`:
a 20 mm cube minus a radius-1 mm, length-4 mm disk extrusion, rotated 20°
about X and placed fully inside the cube. Assert improvement against the
ordinary mesh calculation, not against a fabricated moment record.

## Admitted sources

Capture optional moment evidence only after the existing mesh Cut succeeded
and its existing `VolumeLineage` contains a matching strict containment proof.
Require all of the following:

| Source | Required record |
|---|---|
| Both | `prismrecord.Payload`, solid, `SurfaceResult == false`, no holes, finite valid frame and placement, finite levels `Z0 < Z1`, and zero `SectionDelta`, `Z0Delta`, and `Z1Delta`. |
| Target | `pair/box.RectangularProfile` succeeds; frame and placement bases are cardinal; placement is not a reflection. |
| Tool | Exactly one whole CCW `sectionrecord.CircleSeg`, `TStart == 0`, `TEnd == 1`, finite center, positive finite length radius; placement is not a reflection. |

Permit any valid proper rigid frame and placement for the tool. Read the
record's denoted linear map; a float rotation is not exactly orthogonal.
The existing containment certificate must match tool-as-inner and target-as-outer.

Other profiles, displaced inputs, Revolves, spheres, multiple tools, modified
results, analytic Boolean outputs, and another faceted Cut as a source do not
gain moment evidence. They use their existing path. A box inferred from
`Body.Bounds`, a cylinder inferred from a mesh, and history ancestry never
admit a source.

## Moment construction

### Common anchor

Choose `O = target.n.Centroid.Value` as an exact held point in world coordinates.
It is an arbitrary integration anchor; its use does not assert that it is the
true target centroid. Do not add centroid uncertainty to the anchor.

Add `massmoment.PrismWorldMoments(ctx, record, O)` as a topology-free helper.
Build each source's intervals through the following existing algorithms:

1. Call `PrismProfileMoments` with the admitted record and its zero displacements.
   Its moments use local coordinates `(u, v, z - zm)`.
2. Read `zm` with `PrismMidLevel`. Use `Shift` with `(0, 0, zm)` to move the
   local anchor to the frame origin. Do not round the midpoint to `float64`.
3. Read `L = PrismRotation(record.Frame, record.Transform)`.
   Require `MapChargeOf(L)` to succeed. Use `Transform(local, L)`, including
   its exact `abs(det(L))` factor on every volume moment.
4. Form held world origin `t = Transform.Apply(Frame.ToWorldUV(0, 0))` with
   `r3`. Read its certified rounding radius with
   `proofbound.ExactFrameLiftRound(frame, transform, 0, 0, 0, t)`.
5. Read `t - O` through `proofbound.HeldDelta`; convert its exact dyadic
   components to rationals. Apply `Shift` with that exact displacement.
6. Enclose the residual translation by an independent interval `D_i = [-e,e]`
   per coordinate, where `e` is the origin's rounding radius. Apply the
   interval shift below to the moments from step 5.

### Interval translation shift

For an uncertain translation `D`, read every right-hand term from the same
unshifted snapshot:

```text
V'    = V
P'_i  = P_i + V D_i
Q'_ij = Q_ij + D_i P_j + P_i D_j + V D_i D_j
```

Use `proofbound.RatInterval` arithmetic throughout. Interval multiplication
can lose correlations but must enclose them. Fill the second-moment matrix
symmetrically. With `e == 0`, return the exact-shift result without widening.
`massmoment.WidenTranslation` implements this shift; it is proof arithmetic,
not a new coordinate transform. All held coordinate lifts use `r3`.

The determinant in `Transform` accounts for the denoted map's volume scaling.
Do not add a second determinant factor or substitute an ideal orthogonal
rotation for the stored basis. The origin-rounding enclosure accounts for
the difference between held `t` and that map's exact translation.

### Subtraction and publication

At the common anchor, form `R = massmoment.Sub(target, tool)`. Each source
interval contributes its own uncertainty before subtraction. Reuse the
formulas owned by `agent-docs/multibody-dynamics-design.md` Part M §2 for the center and central
second moment. Call `AffineInertia` with the identity matrix because `R`
already uses world axes. Apply the requested density only at query time.

Publish the center from the same `R.Volume` and `R.First` used for inertia.
Round coordinate interval midpoints outward and convert their maximum error
to a Euclidean radius with `Radius3D`, as `HeldMeshMassProperties` does.
Call `massmoment.Publish` to round scalars and prove positive definiteness.
Do not substitute `Body.Centroid` for this calculation, or reconstruct second
moments from the selected Volume and centroid alone.

## Immutable evidence and layering

Add `Moments *ContainedCutMoments` to `prooflineage.VolumeLineage`.
Keep `topo.Body`'s field list and dependency floor unchanged.
`prooflineage` must not import `massmoment`, `prismrecord`, or `topo`.

Define a floor-safe `prooflineage.MomentSnapshot` using only `r3.Vec` and
`proofbound.RatInterval`: the common anchor, Volume, three First entries, and
the symmetric Second matrix. Explicit adapters in `massdispatch` convert
between this record and `massmoment.Moments`.

`ContainedCutMoments` stores the result producer, target and tool producers,
the copied containment facts, both source snapshots, and their difference.
Use unexported fields, a validating constructor, and a snapshot method that
returns deep copies. Clone both endpoints of every interval, including all
matrix entries. `big.Rat` pointers are mutable despite their mathematical use
as values. Do not retain bodies, topology nodes, profiles, documents,
contexts, caches, or density-dependent results.

`massdispatch.CaptureContainedCutMoments` owns source admission, integration,
and adaptation. `boolean.go`'s existing `containedCutQuantities` calls it
after the centroid calculation and before assigning lineage or committing.
Do not call public `MassProperties` during capture.

Validate source producer identities against the parent VolumeLineage and the
result identity against the receiving body. Validate the copied containment
facts, finite anchor, every non-nil ordered interval, and exact matrix
symmetry. Recompute subtraction and compare rational endpoints, not rounded
floats. Require each source Volume interval to overlap its parent source
reading, and the difference to overlap both parent result readings.

Transform, MirroredCopy, and ScaledAt descendants do not transfer this record.
One direct exact-rotation Copy transfers a frozen source record under
"Exact rotation copy"; default Copy and all later descendants keep
their existing mesh readings. A stale record with a different result producer
is an error, not permission to consume its source moments.

## Selection

Validate present moment evidence before using it. Obtain the ordinary result
through the existing `facetedMassProperties` path, with its matching VerifyAll
certificate and all its existing audits. If that path fails, return its
existing error; this slice does not rescue its admission.

If source publication succeeds, compare both candidates:

1. Require overlap of the mass intervals and all six inertia intervals.
   Compute endpoints exactly from the held readings and their bounds.
2. Require overlap of the two center balls using exact held-center distance,
   as `ContainedCutCentroid` does. Also check the source center against the
   selected cached `Body.Centroid` ball.
3. Select the complete source `MassReadings` only when its mass bound, center
   radius, and each of its six inertia bounds are no larger than their
   ordinary counterparts, and at least one is strictly smaller.
4. Otherwise return the ordinary result unchanged. Never mix tensor entries,
   mass, or center from different candidates. A full tie keeps ordinary output.

Do not cache density-dependent candidates or the selection decision on a body.
Concurrent queries must only read the committed evidence.

## Failure and rollback

| Condition | Required outcome |
|---|---|
| Source class or containment proof is absent | Omit moment evidence; keep current Cut and mass behavior. |
| Supported source integrator returns `ErrUnsupported` | Omit optional moment evidence; do not broaden its integration reach. |
| Integration reports cancellation | Return `ctx.Err()` unchanged; publish no Cut result or history entry. |
| Supported source integration returns another error | Abort Cut through its existing Boolean evaluator-failure mapping. |
| Difference Volume straddles zero | Omit moment evidence; retain current volume and centroid behavior. |
| Difference Volume is wholly nonpositive | Report contradictory evidence before commit. |
| Constructor finds malformed evidence or disjoint certified volume intervals | Return a dedicated internal moment-evidence error; map Cut to `BooleanEvaluatorFailure` wrapping `ErrBooleanFailed`. |
| Query finds malformed evidence or disjoint valid candidates | Return zero `MassReadings` with an error wrapping `ErrBooleanFailed`; do not retry through mesh. |
| Valid source publication cannot prove positivity or represent finite readings | Return the successful ordinary result. |
| Query is canceled | Return `ctx.Err()` unchanged and zero public properties. |

Malformed endpoints or source identities are never an optional-optimization
miss. Keep validation errors separate from `ErrUnsupported` publication
failures. Do not flatten every source error into mesh fallback.

All capture work precedes `commitOperationIn`. Failure leaves live inputs,
the live-body order, producer allocation, container membership, and operation
history unchanged. Mass queries never change those records.

## Acceptance and boundaries

The real fixture must produce a faceted result with one material lump and a
closed void shell, commit one public Cut, retain real captured source moments,
and return properties enclosing an independent box-minus-cylinder reference.
All six inertia components must be checked. The source candidate must win
under "Selection", and at least one diagonal inertia bound must shrink.

Test source placement with a non-cardinal basis, origin rounding at large
coordinates, source uncertainty, density rescaling, absence of evidence,
tie and worse candidates, contradictory evidence, cancellation, and deep-copy
isolation. Use bounded scalar diagnostics; never deep-diff a topology graph.

Preserve `MeshBound`, vertex bounds, `VolSymDiff`, `AreaSlack`, topology,
provenance, public history, and existing selected Volume and centroid readings.
The direct exact-rotation Copy exception is owned by "Exact rotation copy"
and does not extend this Cut's source admission.
No general analytic reconstruction, chained-Cut proof, external kernel,
OCCT source, new module, or public API belongs to this slice.

## Exact rotation copy

A direct public exact-rotation Copy of an admitted original contained Cut
transfers the Cut's retained moments through the exact rotation record. It
retains the existing source and mesh checks and consumes the transferred
moments in `MassProperties`. It adds no source class, containment algorithm,
Boolean route or public API.

Use `evaluator-design.md` §8 and `mirror-pattern-design.md` "Exact rotation
copies" for the copy's denoted solid and its existing placement proof. Use
`operation-history-design.md` "Atomic publication and storage" for commit and
public history. Moment meanings, density and positivity come from
`agent-docs/multibody-dynamics-design.md` Part M §2–§3, as above.

### Selected calculation and alternatives

The copy's certified mass and inertia bounds are tighter than its
independently audited mesh result. The retained difference moments are rotated
by the exact rational rotation the real copy denotes, and the complete
candidate is compared with that mesh result under "Selection". Recording more
ancestry without improving a computed reading does not satisfy this section.
Runtime reduction is not an acceptance claim.

- More contained-Cut source profiles improve initial parts but need new
  integration and admission proofs for every profile.
- A second contained Cut improves machining histories but needs a new
  whole-material containment proof for its faceted target.
- General Copy and Transform cover more placements but must transfer the
  actual float delta, including determinant and stretch.
- Direct exact rotation Copy reuses the existing exact image map, admitted
  source and query gates. This is the selected route.

### Copy admission and scope

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

### Copy evidence and layering

Add `ExactCopyMoments *ExactCopyMomentRecord` to `facetedrecord.Payload`.
The record lives in `internal/facetedrecord/exact_copy_moments.go`.
`topo.Body`, `prooflineage.VolumeLineage` and the topology dependency floor
stay unchanged. In particular, `prooflineage` must not import
`exactrotation`, `massmoment` or `facetedrecord`.

The record freezes these values behind unexported fields:

- The original Cut producer, receiving copy producer, and their held
  placements.
- A copied original `VolumeLineage` with only its scalar source/result
  readings, source producer identities, containment facts and a freshly
  frozen `ContainedCutMoments`. Its unused centroid lineage is omitted.
- The same immutable `exactrotation.Rotation` used by the real rebuild.

The constructor validates the original moment record, reads independent
source snapshots, and rebuilds its private contained-Cut moment record with
`prooflineage.NewContainedCutMoments`, so every rational endpoint is copied
as "Immutable evidence and layering" requires. The exact rotation is shared:
its fields are private and its accessors return fresh rational copies.

`Read` validates the receiving producer and placement, then validates the
frozen original moment record again and returns its difference snapshot
and independent rotation coefficients. Check non-nil rational coefficients,
exact orthogonality and determinant +1. Never use a float tolerance to admit
the map. The record stores no density, selected tuple, query cache, source
body, topology node, document, context, or caller option.

The existing `FacetedCopyLineage` has separate live-source consumers. Use it
only to check this call's direct rebuild during capture. Mass queries use
the frozen record after the original Cut body is removed or retired.
`CertifyCopies` and its live-source requirements are unchanged.

### Moment transport

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
This reuses the existing exact-map arithmetic and adds no coordinate
transform implementation.

Enclose `D = O* - A` by `D_i = [-e,e]` and apply "Interval translation shift"
(`massmoment.WidenTranslation`) to the rotated moments. The returned snapshot
uses A. The independent interval enclosure may be wider than an exact
correlated shift; the comparison under "Selection" decides whether it is
useful. The held mesh's vertex rounding, embedding correction and `VolSymDiff`
are not charged again: they describe its distance to the exact image, not a
change of that image's source moments.

Do not substitute a float rotation, a nominal sine/cosine rotation by the
requested angle, a bounding-box tensor, or the source's published inertia.
The exact rotation selected by `WithCopyRotation` is the copy's denotation.

### Capture and query flow

`massdispatch.CaptureExactCopyMoments` runs after the real copy rebuild and
direct copy-lineage binding, before the final context check and
`commitOperationIn`. It validates and freezes the record, transports the
moments, checks its volume against the rebuilt body's cached Volume and its
computed center ball against the rebuilt body's cached centroid, then
returns the optional record. Disjoint certified evidence fails the Copy.
An unrepresentable finite anchor or center omits this optional evidence.
Capture does not call public `MassProperties`, rerun containment, or create an
extra public operation.

Root attaches the returned record to the pending copy's payload by value.
The rebuilt body's Volume, centroid, topology, bounds and mesh proofs and the
source's lineage stay unchanged.

For faceted `MassProperties`, obtain `facetedMassProperties` first, including
its matching VerifyAll certificate and complete existing audits. Then read
either the original contained-Cut moments or this exact-copy record. Both
present at once is contradictory evidence. Transport the exact-copy record
as above and pass its snapshot to the existing source publication code.
Center, central tensor and density scaling come from that one V/P/Q
enclosure, under the same positivity and finite-reading gates.

Apply "Selection" to the resulting complete tuple, using the receiving copy's
cached centroid in its consistency check. The ordinary candidate remains
available. A mesh failure is not rescued by source evidence. Ties, wider
consistent source bounds and optional publication limits return the ordinary
tuple unchanged. No component mixing or density-dependent caching is
permitted.

### Dropping evidence on descendants

`Payload.PlacedWith` clears `ExactCopyMoments` unconditionally before any
rebuild, and `ScaleImage` clears it before returning its new payload. Only the
authorized direct public Copy capture can attach a new record afterward.
Every Boolean or modify result starts without this record. The payload-copy
rewrite in `loftcylinder/chamfer_build.go` clears the record too, and fresh
payload literals omit it.

Identity and placement validation stay even though those builders drop the
record. An accidentally retained record fails as contradictory evidence
instead of publishing the previous body's moments. `ExactPlaced` and the
existing motion-query refusals are preserved independently of this field.

### Copy errors, context and atomicity

| Condition | Outcome |
|---|---|
| Operation, source or evidence is outside "Copy admission and scope" | Keep ordinary behavior and omit the new record. |
| Any required evidence or identity is malformed | Return an evidence error wrapping `ErrBooleanFailed`. |
| Certified capture volume intervals or center balls are disjoint | Fail before Copy commit with `ErrBooleanFailed`. |
| Capture cannot publish a finite anchor or center | Omit optional evidence and keep the rebuilt copy. |
| Ordinary mass mesh admission fails | Return its existing error and zero properties. |
| Query finds contradictory or disjoint evidence | Return `ErrBooleanFailed` and zero properties. |
| Valid source publication cannot prove positivity or finite readings | Return the ordinary successful tuple. |
| Cancellation after existing input validation | Return `ctx.Err()` unchanged and no partial output. |

Copy keeps its normal operation error wrapper and has no BooleanError
classification. Contradictory evidence never selects fallback.

Poll context before reading/integrating evidence, after transport and before
publication. The public input-validation order is unchanged. Fixed-size
moment transport adds no mesh refinement or separate work-budget policy.

All capture and consistency checks precede commit. Failure leaves source
liveness, ordered live bodies, producer allocation, container membership
and operation history unchanged. Successful Copy references its source and
adds exactly one Copy operation. Queries add no operations and do not change
geometric measurements. Concurrent queries read immutable evidence only.

### Copy acceptance and boundaries

The first fixture builds the sources with `containedCutCentroidSources` and
runs the real public Cut from `TestContainedCutSourceMassProperties`. It
copies that result with `WithCopyRotation`, center `(10,10,10)`, axis
`(1,2,3)`, angle 37 degrees, and uses the real rebuild, original moment
producer, frozen copy evidence, verified mesh and public `MassProperties`.
No fixture invents a proof.

The fixture requires one material lump, one closed void shell, a direct exact
copy, and a new Copy history entry with its source still live. It compares
against the same copy's ordinary mass result with only this consumer disabled
in a sequential test-local payload copy. Both results must enclose the
independent box-minus-cylinder reference after the exact rational rotation.
Mass, center in 3D and all six inertia entries are checked. All eight selected
bounds must be no larger, and at least one diagonal inertia bound must be
strictly smaller. Exact pi is not required. A failing fixture is reported with
its failing component and both bounds; the comparison is not weakened and
source admission is not widened to make it pass.

Further tests cover density rescaling, a nonzero anchor-rounding enclosure,
non-cardinal rotation, repeated and concurrent read-only queries, source
removal, copied endpoint isolation, stale identities and placements,
malformed source lineage, disjoint evidence, optional fallback and
cancellation. Unsupported placement paths are tested without changing their
existing geometry behavior.

General placement transfer, chained moment records, reconstruction,
face-history expansion, new dependencies and OCCT code are outside this
section.
