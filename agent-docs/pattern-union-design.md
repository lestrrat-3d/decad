# Pattern union design

One part per pattern-then-join route. Each part keeps its own decision IDs and `§` numbers; cite a `§` number as "Part name §N".

## Prism pattern union

### Admission

`Union(blank, section, copies...)` takes one centered circular XY prism and
two to 32 axial extrusions from one recorded section. All bodies share the
same positive interval starting at Z = 0. The blank has one circle and no
holes. Each section has no holes, no section or axial displacement, a final
centered inward root arc, and rigid radial `Copy` placement without
translation or reflection. Copies retain the first section's exact source
record identity. A separately drawn similar section does not enter this
path.

The root arc's denoted radius comes from its recorded start. Its recorded
end can differ slightly, so both endpoints must have squared radius
strictly below the blank's squared radius after multiplying by the copy's
exact dyadic XY scale squared. A tangent or outward root refuses this route
before a joined shell is built. A set with a free-form section that this route
refuses, or that has more than 32 copies, is then tried as a union of the
sections (`agent-docs/axial-section-boolean-design.md`), which joins a root arc
lying on the blank's rim and any number of copies. Every other boundary curve stays outside the blank:
lines and fitted-spline Bezier spans use a strict support line for all their
controls; centered arcs use their denoted radius. The two lines next to the
root start inside and end outside the circle, with a positive radial
derivative at the inner end. These tests prove each connector has one
circle crossing and the root arc enters the blank.

Each section's `Tessellate(VerifyAll)` mesh supplies its source-to-held
boundary and occupied-volume bounds. A candidate separating line through
the difference of two held root endpoint sums must leave every held vertex
of one section strictly beyond every held vertex of the other, after
charging both mesh bounds times the line's exact length. The boundary and
therefore each filled section lie in their separate halfplanes. Every pair
must pass. The root endpoint angle intervals must leave an ordered gap
between adjacent copies.

### Joined shell

The build keeps each section's exterior wall facets and omits its inward
root wall. It connects consecutive root endpoints with a wall sampled on
the blank circle at steps no wider than 0.02 radians. Endpoint angles use
`Atan2Interval`, and interior points charge both their angle interpolation
and `RadSinCosInterval` rounding. The two caps triangulate the same ordered
exterior and circle-gap outline used by the walls.

`tessellation.OrientJoinedMesh` proves every edge pairs and the result has one orientable
component. The bounded exact triangle crossing audit checks the complete
shell. Each cap audit proves a simple counterclockwise outline containing
the axis and an exact tiling by its cap triangles. A separate exact
point-in-polygon and edge-contact check keeps every omitted held root chord
strictly inside both cap outlines, except for its two shared endpoints.
Any failed check returns `ErrUnsupported` and leaves all inputs live.

### Boundary and occupied volume

For a connector from inner point `q` along step `d`, squared radius grows
at first derivative `2 q·d > 0`. Its unique circle crossing parameter is
at most `(R²−|q|²)/(2 q·d)`. The copy's exact XY scale enters both terms;
the crossing distance multiplies that parameter bound by the scaled step
length. Nonconnector held wall triangles must remain outside
`R + mesh.Bound()` by their exact minimum projected radius.

The seam band adds the source root-radius gap, both connector crossing
reaches, any held root-vertex excess, circle-chord sagitta, and ring-point
rounding. The joined boundary bound adds that band to the largest section
mesh bound and must be at most 0.1 mm. The complete ring and connector
changes lie in the symmetric annulus from `R−band` to `R+band`; admission
requires `band < R`. Its occupied volume is at most `4πR·band·height`.
The output's symmetric-difference bound adds that annulus volume to each
source section mesh's already verified occupied-volume bound. Its area
slack includes a perturbed area upper bound for the joined shell and upper
areas of the input faces. The two cap groups retain the blank's planar
surfaces and receive the result's own cap roles; other faces are faceted.

### Axial edge selection

The authenticated inputs and their joined shell are one XY section swept
through a common Z interval. The result records this source proof. A faceted
edge chain receives a Z direction for `ParallelTo` only when every held
chord has exactly fixed X/Y and moves strictly in one Z direction. Bent,
stationary, reversed, and unproved faceted chains have no selector direction.
The edge remains a bounded `FacetedCurve`; this certificate does not make its
held XY position exact. A rigid placement drops the source-specific axial
selector certificate until that placed result has its own proof.

### Local root Fillet

The joined body retains its authenticated circular blank, copied section
records, source meshes and audited shell. `Fillet` admits selected
source-root axial chains on that body. Other faceted chains and placed joins
refuse. The radius must have zero input displacement.

For each selected corner, exact dyadic connector and placement coordinates
define an outward line. Its material-side offset by the requested radius
meets the circle of radius `R+r` at the outward quadratic root. Rational
interval arithmetic brackets the center, the line foot and the circle foot.
The line foot must lie beyond the blank and before the next source vertex;
the circle foot must lie in a strictly bracketed shared-ring step. The bracket
may follow several consumed ring stations. Held station order and source
circle station order must both be certified, including ring-point rounding.
The entire tangent arc fits within `3r` of the source root. The source control-hull
certificate keeps every nonconnector wall beyond `R+2r`, the farthest radial
reach of the whole fillet circle. Pinned source arcs charge their radial
residual. The exact contact center, line foot and source root define a narrow
polar sector for the connector cutback, circular-wall cutback and short
fillet arc. The circle's tangent cone bounds the complete arc. Sectors of
selected contacts and all other copied connectors must be strictly separate
modulo a full turn. The connector belonging to each selected root is the only
exception. Sector overlap or an ambiguous angle refuses.

The shell moves each selected connector endpoint to its line foot, removes
the ring facet pairs before the circle foot, and moves the near endpoint of
the retained bracket pair to that foot. It removes the consumed stations
from both cap outlines, then joins the feet with a sampled circular wall.
Overlapping cuts and missing facet pairs refuse. The edge retains its bounded
faceted type. Endpoint angle intervals,
trigonometric intervals and chord sagitta bound every new arc station and
facet. The rebuilt shell must be connected and orientable with paired edges,
pass the full exact triangle-crossing audit, tile both caps exactly, and
keep the omitted source-root chords inside its caps.

Unchanged source and ring walls keep the Union boundary bound. The new
boundary bound adds the arc station and sagitta allowance and must stay
within 0.1 mm. The occupied-volume certificate sums two complete `3r`
cylindrical disks per selected root over the old Union certificate, enlarging
the held disk by the old boundary and arc bounds. These disks may overlap;
the sum remains an upper bound. The area allowance charges both old and new
walls and caps in those disks. `Tessellate(VerifyAll)` checks the detached
result before Fillet commits and retires its input.

## Axial pattern join

`Union` of axial extrusions (`agent-docs/axial-section-boolean-design.md`) takes operands that carry a rigid
placement, such as the copies `PatternCopies` and `Copy` make. One tooth, one rib or one lobe is modeled once,
patterned about the sweep axis, and joined to its base in one `Union`.

The rule names no shape. It applies to any axial operands whose placement keeps the sweep planes.

### 1. What a placed operand is

An operand is a prism or a matching axial loft, as in axial-section-boolean §1, with any placement `Xform`. It denotes
its recorded section on its frame, mapped by `Xform` read as an exact rational affine map of the held floats. A copy of
a free-form section therefore denotes the rigid image the placement states, and its record is untouched.

A prism that carries a section displacement, and a loft that carries a bore, a blend, a construction delta or a surface
result, are still not operands.

### 2. Placement onto the reference frame

`axialsection.Place` brings every operand that is not the reference onto the frame of the reference operand, the first
loft that carries no placement. A frame is orthonormal only to rounding, so `patternrecord.NewPlaneMap` reads the
reference's coordinates through the exact inverse of its Gram matrix, which is exact for a point in its plane, and keeps
the defects it cannot remove:

- The image of a source point can sit off the reference plane by `|S0 + S1·u + S2·v| / |n|`, with `n` the reference's
  plane normal. `MovePlaneRegion` bounds it over the section's extent and charges it.
- The map `A` need not be a similarity. A line and a fit spline map exactly under `A`. A circle or arc maps to an
  ellipse, which the charge bounds against the circle of the nearest similarity `S* = [[a, −b], [b, a]]`, whose scale
  lies in `[ScaleLo, ScaleHi]` and whose distance from `A` in the operator norm is `Defect`.
- Each end plane sits at a height from the reference's (`PlaneHeight`, `HeightGap`) that is charged in full.
- An end frame made by adding a float multiple of the normal to an origin sits a little off the sweep axis, so a loft's
  shift along its axis is the exact projection (`SweepShift`) and its lateral offset is charged.

A placement is refused (a miss) when the off-plane distance or a height gap or a lateral offset exceeds
`2⁻⁴⁴·(1 + extent + |translation|)` mm. A placement matrix and a frame are exact to a few float steps, far below that,
so a larger value names a tilted, lifted or sheared sweep. The limit only refuses: a value it admits is charged in
full.

### 3. Re-recording and the charge

`patternrecord.MovePlaneRegion` maps every point of the section through the exact map and rounds it once to a float.
It returns the moved record and an upper bound `δ` on how far any point of that record sits from the exact image of the
source curve:

| Segment | Charge |
|---|---|
| Line | The larger endpoint rounding. |
| Circle | The centre rounding, `r·|λ − 1|` and twice `Defect·(|u|max + |v|max)`. |
| Arc | Three times the largest of its centre, start and end roundings (the argument of `offset2d.OffsetSectionDelta`), `2·ρ·|λ − 1|` for its largest radius `ρ`, and four times `Defect·(|u|max + |v|max)`. |
| Fit spline | The distance between the refit cubics and the image of the source cubics, below. |

A fit spline is a natural cubic spline over the sketch's float chord-length parameters. `geom.FitInterpolant.ExactSpans`
publishes each span's cubic with exact rational coefficients in the local parameter `u ∈ [0, 1]`, from the same code the
arrangement certificates use. The source spline's spans are mapped by `A` and `T` exactly. The moved spline is the
interpolant through the rounded image points, with its own exact spans. The two have the same span count, since the
moved points are the images of the source points to within rounding. For each matched span the difference of the two
cubics is a cubic in `u` with rational coefficients. Its Bernstein coefficients enclose it on `[0, 1]`, so the larger of
`√(Σ max|b|²)` over the two coordinates bounds the distance from each point of one curve to the point of the other at
the same `u`. That is an upper bound on the Hausdorff distance between the curves. It is a charge: it admits nothing,
and it widens every reading of the result.

The reference operand contributes `δ = 0`, and so does an operand on the reference's own frame. An operand's `Delta` is
its section's `δ` plus the off-plane distance, its two end-plane gaps and any `JoinDelta` it already carries. The result
adds each operand's lateral offset and the reference's, since the result keeps the reference's end frame. The scale term of an arc is conservative: the three
pinned points of the image arc are the exact images, so the three-times argument already covers the arc.

### 4. The result

`UnionMany` runs `Place` first, then the route of axial-section-boolean §2 over the moved sections. The loft it builds
carries `loftrecord.Payload.JoinDelta`, the largest `Delta` of its operands (an operand that is itself a join
contributes its own).

`loftrecord.Build` adds `JoinDelta` to the held-vertex displacement, the chord displacement and the matched
displacement of the join body, so every reading of that body encloses the ideal union of the placed operands. A body
rebuilt from a copy of the payload (`Fillet`, a through-bore `Cut`, a placement) keeps the term as extra slack and
denotes its own record: its readings make no claim about the operation applied to the ideal join. `ConstructionDelta`
is not used, because it records a fitted-spline fillet's departure from its ideal rewrite and the fillet refuses a
second one.

### 5. Limits

- The placement must keep the sweep planes to the rounding of the placement (§2): a rotation about the sweep axis, a
  translation in the plane, or any composition, on any frame. A tilt, a lift off the plane or a differently sized
  sweep is a miss, and the Union takes its next route.
- `Fillet`, a through-bore `Cut` and a cap `Chamfer` of the join read the frame's exactness themselves: the loft fillet
  needs an end frame exactly on the sweep axis (`loftrecord.FilletStraight`), so a join on an oblique frame refuses a
  fillet.
- A set whose only loft is itself placed has no reference frame and is a miss.
- Segments of kind line, arc, circle and fit spline are placed. Other kinds are a miss.
- A tooth or lobe that only touches the base (a bottom arc resting on the circle with the flank ends on it) or misses it by
  a micron is a miss: it makes no region `sketch` can certify as one union. `apitest.TestPatternJoinRefusesLobesThatOnlyTouchTheBase`
  pins both.
- Copies whose free-form edges cross one another (two arched tops that overlap), or whose sides nearly coincide without
  sharing a whole edge, are a miss: `sketch` certifies a spline against a circle but not a spline against another spline
  or an arc. `apitest.TestPatternJoinRefusesCopiesItCannotCertify` pins both refusals against the 4 mm step that joins.
- The route is `Union` only.

### 6. Tests

- `internal/patternrecord`: `MovePlaneRegion` refuses a tilt and a lift and accepts the identity, charges a 17th-turn
  turn about the normal of an oblique frame within 1e-12 and refuses a hundredth-radian tilt of it; the fit charge is below
  1e-13 for a 17th-turn rotation and is at least a deliberate 1e-3 displacement of one knot, so it is not vacuous; the
  circle charge is the radius scale deviation.
- `internal/axialsection`: a lobe placed by a 17th turn is re-recorded within a positive `δ` below 1e-12 and each fit
  point is the rotated source point; misses for a tilt, a lift and a placed reference; each height and plane guard fires
  alone (shown to fail by disabling it); a section of quarter-millimetre coordinates slid by whole millimetres is charged
  exactly zero, spline included (`TestPlaceChargesNothingWhenTheMapIsExact`), so the charge is nonzero only where a
  placement rounds.
- `apitest.TestPatternJoinManyCopies` joins 64 and 128 lobes to a disc and checks the sum of the parts.
- `apitest.TestPatternedLobesJoinADisc`: one lobe patterned 8, 12, 17 and 24 times and joined to a disc has the volume
  of the same lobes drawn at their angles within both bounds, with a larger bound, and a `VerifyAll` mesh whose boundary
  and volume are verified. `TestPatternedLobesAboutATiltedAxisAreRefused`: `ErrUnsupported`.
  `TestPatternJoinOnAnObliqueFrame`: the same part on a frame no axis aligns with has the world-frame part's volume
  within both bounds, with a verified boundary and volume.

### 7. PatternJoin

```go
func PatternJoin(ctx context.Context, target, feature *Body, spec PatternSpec) (*Body, error)
```

`PatternJoin` repeats `feature` by `spec` and joins every instance, `feature` included, to `target` in one `Union`, and
retires `target` and `feature`. It is `PatternCopies` followed by `Union(target, feature, copies...)` with the copies
never registered: they are built under consecutive producer identities, handed to the many-operand Union route chain
(`unionManyBodies`, which `Union` shares) and dropped. A refusal leaves the document unchanged and both bodies live.

When `feature` is a loft, the copies are first held as placed payloads (`loftrecord.Payload.WithPlacement`) and handed to
the axial-section route alone (`axialsection.UnionManyPayloads`), which reads nothing of a body but its payload; no copy is
built. When that route declines, the copies are built and the other routes run as for `Union`. Both give the same
volume and bound to the bit (`apitest.TestPatternJoinMatchesBuiltInstancesExactly`).

The gates are `PatternCopies`' for `feature` and `spec`, and `Union`'s for `target`: a nil, foreign or retired target is
refused, `target` and `feature` must be distinct, and a sheet in either role is `ErrUnsupported`. The join takes the
routes `Union` takes over the same operands, so every refusal of §5 and of `agent-docs/axial-section-boolean-design.md`
§3 is returned unchanged.

`Patterned` keeps its own rule (`agent-docs/mirror-pattern-design.md` §4.3, §6.1): it returns the instances of one body
as one body, and refuses disjoint lumps. `PatternJoin` is the call for a feature that joins a base.

### 8. Features whose flanks cross the base

A feature whose fit-spline flanks start inside the base's circle and leave it joins the base the same way: the private
scene cuts each spline against the circle, and `sketch` certifies the crossings (`TExact` on every `Partial` fragment of
its pair certificate, `sketch/docs/pair-certificate-design.md`). The union records the circle's remainders as trimmed
`CircleSeg` fragments. A union `sketch` leaves uncertified is a miss, as in axial-section-boolean §3. The joined body
then takes a concave `Fillet` where each flank leaves the circle, a through-bore `Cut` and a cap `Chamfer`.

Tests: `apitest.TestPatternJoinFlanksCrossTheBase` joins 8, 12 and 24 patterned lobes whose flanks cross the disc and
compares their volume with the same lobes drawn at their own angles.
`TestPatternJoinFlanksCrossTheBaseThenFilletBoreChamfer` takes 12 of them through the fillet, bore and chamfer.
