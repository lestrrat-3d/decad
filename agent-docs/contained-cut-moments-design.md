# Contained Cut moment composition design

## Status and owners

The contained-Cut source-moment consumer is implemented for a rectangular
prism target and whole circular-prism tool. The implementation follows this
design's admission, interval, selection, and fallback rules.

`operation-history-design.md` "Contained Cut centroid" owns the existing
source-volume and source-centroid calculations. This document owns the
contained-Cut second-moment composition and its `MassProperties` consumer for
one source class.

Use `dynamic-mass-design.md` §2–§3 for moment meanings, density, physical
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
Keep this small interval-shift helper in `massmoment`; it is proof arithmetic,
not a new coordinate transform. All held coordinate lifts use `r3`.

The determinant in `Transform` accounts for the denoted map's volume scaling.
Do not add a second determinant factor or substitute an ideal orthogonal
rotation for the stored basis. The origin-rounding enclosure accounts for
the difference between held `t` and that map's exact translation.

### Subtraction and publication

At the common anchor, form `R = massmoment.Sub(target, tool)`. Each source
interval contributes its own uncertainty before subtraction. Reuse the
formulas owned by `dynamic-mass-design.md` §2 for the center and central
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

Copy, Transform, MirroredCopy, and ScaledAt descendants do not transfer this
record in this slice. Their builders must omit it and retain their existing
mesh readings. A stale record with a different result producer is an error,
not permission to consume its source moments.

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
No general analytic reconstruction, chained-Cut proof, external kernel,
OCCT source, new module, or public API belongs to this slice.

## Design decisions and implementation stop

The source class, common anchor, translation enclosure, immutable record,
whole-result selection, and unchanged mesh admission are decided here.
No caller policy choice is required to implement them.

The unmeasured question is whether the complete source candidate passes the
comparison for the real fixture. Run that producer-to-consumer instance
before expanding tests or source classes. If it fails, report the failing
component and its two bounds. Do not weaken acceptance or enlarge scope to
make the result win.
