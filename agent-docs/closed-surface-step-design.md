# Analytic STEP for closed sphere and ring faces

## Status and ownership

Proposal; unimplemented. This document specifies two additions to the analytic STEP writer.
`step-export-design.md` owns the shipped export contract; it wins until these additions ship.
Do not change the public export API or Decad's source-body topology.

## Verified gap

- `internal/revolveplan/build.go`, `buildRevolveLoop`, leaves a full closed generator's wall without loops.
  `fullRevLoops` also emits no latitude loop where both ends are on the axis.
- `apitest/revolve_test.go`, `TestRevolveSphere`, builds one spherical face without edges or loops.
  `TestRevolveTorus` builds the corresponding one-face closed ring from a whole circle.
- `export/step_analytic.go`, `supportsAnalyticFaces`, admits a sphere only with one supported patch loop.
  `export/step_analytic_torus.go`, `supportsAnalyticTorus`, admits only one or two loops.
- `export/step_analytic_sphere.go`, `addSphereFace`, and `addTorusFace` read the first loop directly.
  Admitting zero loops without separate builders would panic.
- `internal/facedeparture/facedeparture.go`, `TagDeparts`, already checks the surface's recorded denotation.
  The sphere arm additionally checks the recorded radius. Preserve that gate.
- `export/step.go`, `NewSTEPFile`, requires a boundary-verified tessellation before choosing either output path.
  It selects analytic output only for one shell without voids. Preserve those conditions.

A scratch probe at base `5ed2045c6` used real Sketch profiles, `Revolve(FullRevolution{})`, `NewSTEPFile`
and `step.File.Marshal`, with a 0.2 mm chord tolerance. Both source faces had `facedeparture.Stated == false`.

| Source | Source faces / edges / loops | Current STEP output |
|---|---|---|
| Half-disk revolved about its diameter, radius 5 mm, centre (5,0,0) | 1 / 0 / 0 | 224 planar faces; no sphere |
| Circle revolved about sketch u, major radius 10 mm, minor radius 3 mm | 1 / 0 / 0 | 624 planar faces; no torus |

The probe establishes the existing producer-to-writer boundary, not acceptance of the proposed encoding.
Its command was `timeout 60s go run .tmp/closed_step_probe.go`; output is
`.tmp/closed-step-producer-export.log`. These scratch files are investigation evidence, not permanent tests.

## Encoding decision

Use a north-pole `VERTEX_LOOP` for a whole sphere. Use two explicit periodic seam edges for a whole ring torus.
Do not apply one encoding to both surface kinds.

The STEP implementors' [Issue 023, Sphere Topology](https://www.steptools.com/stds/impforum/isslog.pdf#page=65)
accepts a north-pole vertex loop as minimum sphere topology. Issue 005 in the same document describes the torus's
two intersecting circle edges and one vertex. These are topology guidance, not evidence that a specific reader
accepted Decad output.

OCCT V7_9_0's
[StepToTopoDS_TranslateFace.cxx](https://github.com/Open-Cascade-SAS/OCCT/blob/V7_9_0/src/StepToTopoDS/StepToTopoDS_TranslateFace.cxx)
constructs natural spherical bounds for a sole vertex loop and explicitly skips torus vertex loops.
That source behavior supports choosing different encodings. Runtime acceptance has not been measured.

The STEP Tools [face schema](https://www.steptools.com/stds/stp_aim/html/t_face.html) requires at least one bound.
Its [advanced-face schema](https://www.steptools.com/docs/stp_aim/html/t_advanced_face.html) accepts vertex and edge
loops. Those pages describe the merged AP schema and flag AP214 differences; they do not replace an AP214
conformance check. Do not emit an empty bound set merely because `ap214.AdvancedFace` can serialize one.

## Initial admission

Admit only a solid with one lump, one non-void shell, one face and no body edges or face loops.
Require its face to be an undeparted `Sphere` or a regular ring `Torus`, with finite positive radii.
Require `Major > Minor` for the ring. Whole horn and spindle tori are outside this proposal.
Preserve all current bounded-face arms and the body-wide faceted fallback.

Require a complete-carrier record from the body's current `revolveplan.Payload`, not its tags or tessellation.
The existing producer is `revolveplan.Build`: `buildRevolveLoop` resolves the meridian with
`revolveaxis.ResolveLoopWithFreeform`, skips axis walls and emits the off-axis walks; `fullRevolveShellsContext`
assembles their complete full-turn shells. Authenticate these specific source classes:

- For a sphere, require `contactround.SphereRecord` on the current body. That existing proof reads one whole
  semicircle and its exact on-axis diameter, no profile holes, a cardinal source frame and zero axis bounds.
  It proves the occupied solid is a complete ball. Require its returned face to be the proposed export face,
  and compare its exact centre and radius with the surface tag in addition to `TagDeparts`.
- For a ring, add a narrow reader in `revolveplan` for one outer `sectionrecord.CircleSeg`, whole range `[0,1]`,
  positive radius and no holes. Require the same resolver used by `buildRevolveLoop` to return `SingleClosed`
  and exactly one `WallTorus` walk. Compare its centre, radius and positive axis distance with the original
  circle exactly; require axis distance greater than radius. The full angular fibre of this entire circle
  covers the torus once, by the construction argument in `revolveaxis.ProvesSimple`.
- For both classes, require `Full`, `!SurfaceResult`, `SectionDelta == 0`, `!SectionWhole`, zero axis anchor and
  direction bounds, zero `RadialAdmitAllow`, zero `Snap`, and successful `Payload.ProvesSimple(ctx)`.
  Require the angular denotation to state exactly one turn: finite rad/turn endpoints, no derived spans,
  equal radian parts and turn difference exactly one, as `pointquery.fullTurn` currently checks.
  A held angle close to `2*pi`, `Full` alone or a successful tessellation audit proves no such coverage.

Initially require `Payload.Xform == r3.Identity()`. A copy rebuilding an identity-placement Revolve qualifies
only after these checks authenticate its own current payload again. Exclude nonidentity translations, rotations
and reflections, and every faceted retained-tag copy, even if its carrier tag is exact.
A later placement slice must transfer the complete source set through an exact isometry and independently prove
the resulting tag equals that image. A copied denotation alone transfers the carrier, not complete-face coverage.

Expose the authenticated plain-data record through a new internal export bridge, installed by a thin root
adapter that can read `Body.n.Payload` and topology identity, following `face_departure.go`'s access pattern.
Keep this coverage reader separate from `facedeparture.Stated`; the latter checks carrier departure only.
No source payload is mutated, and no synthetic seam or vertex enters a selector or public topology accessor.

Synthetic points must be exactly representable coordinates on the admitted carrier.
Compute candidate sums and surface identities over the stored inputs as exact rationals, using existing proof
helpers. Map frames and vectors through `r3`. Compare the exact candidate with its stored float coordinates.
If it cannot be represented, decline the analytic addition and write the verified faceted result.
Never snap a candidate onto the carrier or use a residual tolerance to accept it.

Initially use world +Z and +X as the sphere placement axis and reference.
For a ring, admit a coordinate-axis torus and choose the first perpendicular positive coordinate axis as reference,
in deterministic X/Y/Z order. Other axes continue through the faceted writer until an exact basis is proved.
This keeps the first seam construction free of rounded normalization and trigonometric endpoints.

## Sphere construction

Let `C` and `r` be the admitted sphere's centre and radius in millimetres.

1. Form north pole `P = C + r*Z` through the exact-point gate above.
2. Allocate a file-local `CARTESIAN_POINT(P)` and `VERTEX_POINT` referring to it.
3. Emit `VERTEX_LOOP('', vertex)` and one `FACE_OUTER_BOUND('', loop, .T.)`.
4. Emit the existing `SPHERICAL_SURFACE` placement at `C`, axis +Z, reference +X, radius `r`.
5. Emit one `ADVANCED_FACE` with that bound and one-face `CLOSED_SHELL` through the existing assembler.

Read face sense with `Face.NormalAt(P)` against +Z. The geometric sphere normal exists at this pole;
the longitude coordinate is singular there. Do not evaluate a longitude or a tangent frame at the pole.
Require an available, zero-bound normal whose direction agrees with the admitted solid's outward surface.
A failed sense check declines the analytic route; it does not invent an orientation.

There are no edge curves or oriented edge uses in this encoding. Existing tests that require every emitted edge
once in each direction apply vacuously. Add separate assertions that the vertex loop is the sole face bound and
that its point is the placement's north pole exactly.

The pinned `step` module has no `ap214.VertexLoop` constructor. It can already serialize a generic `step.Entity`.
Emit this one entity locally with its two schema parameters; do not make another repository a prerequisite merely
for a convenience constructor. A schema-aware validation must check its parameter types.

## Ring construction

Let `C`, axis `A`, reference `E`, major radius `R` and minor radius `r` be admitted exact data.
Let `F = A cross E`; use `r3` to construct those directions.

The periodic chart is
`S(theta,phi) = C + (R + r*cos(phi))*(E*cos(theta) + F*sin(theta)) + r*sin(phi)*A`.
This equation specifies orientation; the writer evaluates only the cardinal seam data below.

1. Form tube centre `T = C + R*E` and shared seam vertex `P = C + (R+r)*E` through exact-point gates.
2. Emit `TOROIDAL_SURFACE` at `C` with axis `A`, reference `E`, and radii `R`, `r`.
3. Emit the outer-equator `CIRCLE` centred at `C`, axis `A`, reference `E`, radius `R+r`.
4. Emit the tube-meridian `CIRCLE` centred at `T`, axis `E cross A`, reference `E`, radius `r`.
5. Emit one `EDGE_CURVE(P,P,parallel,.T.)` and one `EDGE_CURVE(P,P,meridian,.T.)`.
6. Bound one face with the oriented edge sequence `parallel+`, `meridian+`, `parallel-`, `meridian-`.
7. Use one `EDGE_LOOP`, one `FACE_OUTER_BOUND`, one `ADVANCED_FACE`, and the existing solid assembler.

The loop is the boundary of the full periodic parameter rectangle: theta increases along its bottom,
phi increases along its right side, theta decreases along its top, and phi decreases along its left side.
Both copies of each seam share one `EDGE_CURVE`, and all four corners share one `VERTEX_POINT`.
The paired uses identify opposite sides; they do not trim out material or create two separate coincident circles.
Require `V-E+F = 0` for this one-face torus topology, in addition to opposite directed edge uses.

At `P`, the chart's positive normal is `E`. Read the material-outward sense against it, with the same exact-normal
gate as the sphere. Reverse the loop consistently if the surface sense requires it; do not reverse only one use.
The initial one-face outer solid should have outward sense; test that expectation on the real source.

The candidate uses ordinary 3D circle geometry, as existing cylindrical and toroidal seams do.
A reader might require paired parameter curves to identify the two periodic branches. Do not assume the 3D circle
sequence is sufficient. If the first consumer gate fails for that reason, stop this slice and scope explicit
`SEAM_CURVE`/`PCURVE` output separately before broadening the implementation.

## Integration and failure behavior

- Add a closed-surface admission result containing validated synthetic data before writing entities.
  Do not add independent permissive zero-loop cases to existing patch builders.
- Dispatch the sphere and ring result to dedicated builders before existing code indexes `Loops()[0]`.
- Keep zero-loop faces outside `isWalkedPatch`; these one-face solids have no neighbour to choose a patch walk.
- Keep entity allocation deterministic and local to one export call. Do not add a global cache.
- Check cancellation at admission, before synthetic construction, and before publishing the complete file.
- Keep existing error propagation for `Tessellate`, invalid bodies, cancellation and writer failures.
  Unsupported axes, inexact synthetic points, departed tags or unavailable normals select the faceted fallback.
- Preserve the complete-file-before-write behavior. The caller never receives half an analytic candidate.

## First implementation gate

Run one real sphere end to end before enabling the sphere admission or adding a fixture matrix:
Sketch half-disk -> `Revolve` -> candidate `NewSTEPFile` -> file marshal -> local independent STEP reader.
Then repeat once for the ring. Do not generate the consumer file by hand.

Use a pinned local OCCT reader or an existing equivalent CAD reader. Check transfer success, one solid, closure,
validity, analytic surface type and signed volume. Compare reconstructed geometry with the source's bounded
volume and centroid. Record reader version, options, warnings and whether healing altered topology or geometry.
Merely parsing Part 21 or finding `SPHERICAL_SURFACE`/`TOROIDAL_SURFACE` strings is insufficient.

No OCCT/OCP/FreeCAD runtime was available in this investigation. Reader-source inspection is the only compatibility
evidence. Provisioning a local consumer is a prerequisite for implementation acceptance, not a completed check.

`ReadSTEP` cannot be substituted for that first independent check without extending its supported classes:
`step-import-design.md` I4 excludes tori, and I15 excludes whole spheres with seams or non-cap bounds.
The permanent Decad reader may keep refusing the new analytic output; preserve a test of that documented refusal.

## Focused fixtures and assertions

| Fixture | Required result |
|---|---|
| Radius-5 sphere centred at (5,0,0) | One spherical face; one north-pole vertex loop; volume 500*pi/3 |
| Ring about +X, R=10, r=3 | One toroidal face; two circle edges used in opposite pairs; volume 180*pi^2 |
| Same shape and explicit timestamp exported twice | Identical bytes and unchanged source face/edge/loop counts |
| Copy rebuilt with identity placement and the same complete Revolve record | Reauthenticate coverage; preserve outward sense |
| Nonidentity placement, reflection, retained-tag copy, or empty-loop tag without a complete source record | Faceted fallback |
| Carrier with a stated departure or unrepresentable synthetic point | Entire body takes the existing faceted writer |
| Ring axis outside admitted cardinal class, void shells, or several lumps | Existing writer selection is preserved |
| Canceled context before construction | Existing cancellation error; supplied writer remains untouched |

Decode every synthetic placement, point, bound and edge reference from the emitted file in focused export tests.
Compare scalars, signs, coordinates and reference identities; never recursively dump topology graphs.
Keep radius-5 and R=10/r=3 consumer checks separate so a failing sphere prevents expansion to ring experiments.

## Files and implementation slices

| Slice | Files and acceptance |
|---|---|
| 1. Validate sphere encoding | New scratch/export prototype and local reader runner; require the real first gate |
| 2. Ship sphere arm | Export admission/builder, new internal coverage bridge and root adapter, focused tests |
| 3. Validate and ship ring arm | Narrow `revolveplan` coverage reader, same bridge/builder, separate consumer gate |
| 4. Update current-state owners | `step-export-design.md`, its missing-feature row and export API documentation |

No public Go type changes or dependency-module additions are required by the candidate.
The sphere slice is small after consumer access exists. The ring slice is medium if 3D seams transfer directly.
If the reader requires parameter curves, the implementation cost is unknown until the failed transfer identifies
the missing representation. Keep the verified faceted output until that additional design is complete.

## Implementation task plan

Follow the [ordered implementation tasks](plans/closed-surface-step-implementation.md).
This design owns the proposed contract; the plan supplies execution steps.
