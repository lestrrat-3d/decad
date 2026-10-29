# Wall survey of split spherical meridians

## Status and scope

Proposal; unimplemented. This document proposes sharing wall-survey carrier
identities across recorded arcs of one on-axis circle. The current survey
contract remains in `verification-design.md`, under its wall-survey section.

Admit only full solid Revolves with an undisplaced meridian and circular
centres already proven on the axis. Preserve the existing bounded wall
reading; exact geometry does not imply a zero arithmetic bound.

Do not merge topology faces, replace several arcs by a full circle, or widen
the admitted axis class. Partial sweeps and uncertain or off-axis centres keep
their current survey path.

## Observed behavior

Source observations below refer to base `5ed2045c6`.

- `internal/wallsurvey/record.go`, `RevolveWall`, refuses a nonzero
  `SectionDelta` before resolving the loops.
- `RevolveLoopsPlane` reads each recorded segment through
  `boundarywalk.WalkOf`, expresses its walk through `revolveaxis.Frame.Walk`,
  and keeps recorded segment indices in `SideWalk.Segs` after coalescing.
- `internal/revolvesurvey/survey.go`, `WallReading`, assigns a fresh nonzero
  `Carrier` to each on-axis circular walk and its mirror.
- `OnAxisCentre` requires a circular walk, `CV == 0`, and `CVBound == 0`.
  A centre snapped toward the axis or expressed through a tilted uncertain
  frame does not pass this proof.
- `internal/survey2d/wall_candidates.go`, `ArcArcCands`, skips a concentric
  pair only when both elements have the same nonzero carrier and material
  side. Distinct IDs leave a zero-radius family for pieces of the same circle.
- `internal/survey2d/wall_kernel.go`, `Generate`, separately emits whole-arc,
  vertex, pair and triple candidates. The carrier rule changes only the
  concentric pair family; containment continues to read all original arcs.

Two public producer-to-consumer probes reproduce the gap. Both use a fixed
Sketch profile, public `Revolve`, and public `Verify` with a 1.5 mm minimum:

| Meridian | Current wall outcome | Current report |
|---|---|---|
| Radius-10 half disk, semicircle split at `(0,10)` | `ScalarUndecided` | `Suspect` |
| Radii-8-and-10 half annulus, each semicircle split at its equator | `ScalarUndecided` | `Suspect` |

Scratch evidence is in `.tmp/sphere-wall-probe_test.go`,
`.tmp/sphere-wall-overlay.json`, and `.tmp/sphere-wall-probe.log`. The focused
command passed while asserting the current undecided result:

```sh
go test -overlay .tmp/sphere-wall-overlay.json ./apitest \
  -run '^TestDesignProbeSplitSphereWall$' -count=1 -v
```

A separate scratch overlay shared IDs for these exact quadrant fixtures.
The resulting solid and shell reports were `Sound`, with intervals containing
20 mm and 2 mm. The values and bounds were approximately
`19.99999999999999 ± 1.35e-14` and `1.999999999999993 ± 7.11e-15` mm.
This verifies that carrier assignment reaches the real consumer; the scratch
float-key prototype is not the proposed general equality proof.

## Exact carrier certificate

Build a per-survey identity from original recorded circle data, before axis
re-expression. Equal held walk radii or centres are insufficient: a hypot or
axis subtraction can round distinct exact values to one float.

For each eligible normalized `ArcSeg`, read the finite stored coordinates as
exact dyadics. Compute both squared radii from `Start` and `End` to `Center`
with the existing `internal/proof` exact arithmetic. Require both squared
radii to be positive and exactly equal. The key is the exact centre plus that
squared radius. Require a whole, non-`Partial` source arc in the first slice.

An endpoint inconsistent with that circle receives no shared identity. Do not
project it onto the circle or accept a small residual. Existing single-walk
mirror handling still applies independently.

Keep this comparison private to the survey record adapter. It compares stored
geometry for identity; it neither computes a new 2D intersection nor admits a
Sketch fragment. Reuse exact numeric and canonical-key helpers from
`internal/proof`. Do not reconstruct a Sketch scene just to compare recorded
centres and radii. `internal/prismcells/coincident.go`, `circleCarrier`, is the
existing semantic precedent for comparing both endpoint radii; its entity
adapter need not be imported into the survey package.

Do not add CircleSeg mixed-carrier support in this slice. A CircleSeg already
has a self/mirror ID; converting its `units.Value` radius into an exact
denotation must follow its owning conversion contract before it can share an
identity with an arc. The split-arc defect needs no such conversion.

## Assigning identities

Extend the wall-only adapter in `internal/wallsurvey/record.go` to build
carrier IDs alongside its resolved loops. Do not add persistent fields to
public types or mutate the recorded profile or shared walks.

1. Resolve loops exactly as today. Keep the profile and `SideWalk.Segs`
   available while building the wall-only carrier table.
2. For each circular side walk, require `revolvesurvey.OnAxisCentre`.
   Also require an exact cardinal axis direction: `(DU,DV)` is a signed
   coordinate basis direction and both direction bounds are zero. Do not
   infer cardinality from an angular tolerance.
3. For every source segment named by that side walk, require an eligible arc
   with the same exact carrier key. An unsupported or mixed source leaves
   that walk without a shared ID.
4. Allocate IDs from one table for the complete survey, including hole loops.
   Compare centres before frame mapping, so rounded transformed centres
   cannot create an identity. Equal source centres undergo the same map.
5. Pass the table, indexed by loop and side walk, to
   `revolvesurvey.WallReading`. Its contract requires matching dimensions;
   malformed adapter data is an internal error, never an exactness claim.
6. Use one allocator for the shared IDs and the existing per-walk fallback
   IDs. A missing entry must receive its own fresh ID; it must never collide
   with a shared group.

Apply shared IDs only in the `full` arm. Assign each walk and its mirror the
same group ID. Leave all element coordinates, angles, radius bounds, material
sides, vertices, containment elements and pinch checks unchanged.

The restriction to exact cardinal directions keeps uncertain tilted-axis
cases outside this increment. Axis anchor subtraction may round, but equal
source centres map together; `OnAxisCentre` still must prove their radial
coordinate exactly zero. No new centre approximation is accepted.

## Why the skipped family is absent

Let two source arcs have one exact centre and positive squared radius. They
are subsets of the same supporting circle. Their mirrors remain subsets of
that circle because its centre lies exactly on the revolve axis.

When both arcs have the same material side, no positive-radius disk lies
between two concentric skins of that circle. The current concentric pair
formula generates only zero-radius candidates there. Sharing a carrier
therefore removes a nonexistent wall, as the kernel's existing carrier
contract already permits.

No continuity or complete-circle coverage is assumed. The original angular
ranges remain in the kernel. Disjoint same-carrier arcs may share identity
without filling the gap between them; containment still checks the actual
arcs and every intervening boundary segment. Opposite material sides retain
their candidates because `ArcArcCands` compares `MatInside` independently.

Outer and inner sphere skins have different squared radii and therefore
different IDs. Their pair still supplies the shell-thickness candidate.
For a solid ball, whole-arc and other existing candidates still provide its
diameter. This increment adds no special formula for either shape.

## Bounds, refusals and work

- Keep every original radius bound and the existing minimum aggregate. Never
  force the split ball or shell reading to `Exact` merely because carrier
  comparison is exact.
- Keep displaced meridians undecided at `RevolveWall`'s existing gate.
- Keep off-axis, snapped-centre, tilted-frame, unsupported source and unequal
  carrier cases on their current path. A missing shared ID is not a failure
  of the entire survey.
- Keep partial sweeps unchanged, including wedge and axis-contact rules.
- Keep original boundary and junction checks for missing arc intervals,
  opposite material sides, holes, pinches and degenerate boundaries.
- Charge each source visit and key comparison to the same
  `proofbound.WorkBudget`. Use exact fixed-width-input expressions; do not
  build an unmetered all-pairs carrier search.
- Keep all grouping state local to one call. Cancellation propagates the
  context error through `Verify`; it must not become a `Suspect` report.

## Verification

Add public regressions to `apitest/survey_sphere_test.go` using its existing
`sphereMeridianSketch` and `sphereWallReport` helpers:

| Fixture | Required result |
|---|---|
| Split radius-10 half disk, tool 1.5 and 25 mm | Interval contains 20 mm; `Sound` then `Violating`. |
| Split radius-8/10 half annulus, tool 1.5 and 2.5 mm | Interval contains 2 mm; `Sound` then `Violating`. |
| Same profiles with reversed recorded entity order | Same interval enclosure and report outcomes. |
| Split spherical zone with exact flats at axial ±6 | Interval contains 12 mm; actual flat boundary remains effective. |
| Existing snapped-centre and tilted-axis fixtures | `ScalarUndecided` and `DiagUndecidedWall` persist. |

Use bounded scalar assertions and independent exact values. Keep tests of
carrier equality in `internal/wallsurvey/record_test.go`: unequal centres or
squared radii, equal rounded hypots with unequal exact squared radii,
inconsistent arc endpoints, unsupported fragments, IDs across hole loops,
and cancellation. Assert IDs or small booleans; never dump whole bodies.

Extend `internal/revolvesurvey/survey_test.go` for shared/fallback ID
allocation. Preserve `internal/survey2d/wall_candidates_test.go`'s opposite
material-side and concentric-different-radius tests. Do not change the kernel
candidate equations to make the new fixtures pass.

Before implementation acceptance, run focused package tests and the public
sphere tests in plain and race modes. Register new `apitest` test names in
`.github/test-shards-apitest.txt` under `agent-docs/testing.md`; full suites
belong to CI. Remove the carrier assignment in a mutation check and confirm
that the split-sphere public regressions fail.

## Implementation slices

1. Add exact source-arc keys and metered wall-only grouping. Test all equality
   and conservative refusal branches before wiring the consumer.
2. Pass carrier IDs into `WallReading` and add the real split-ball and
   split-shell producer-to-consumer tests. Preserve all old survey outcomes.
3. Update `verification-design.md` and the split-circle row in
   `missing-features.md` to state the implemented class. Update public survey
   documentation only if it states the previous restriction.

The scratch producer-to-consumer probe settled whether grouping alone can
remove this refusal for exact quadrant fixtures. General non-cardinal axes,
CircleSeg/ArcSeg mixed groups and certified partial arcs are separate later
increments; this proposal makes no reach claim for them.

## Implementation task plan

Follow the [ordered implementation tasks](plans/split-circle-wall-implementation.md).
This design owns the proposed contract; the plan supplies execution steps.
