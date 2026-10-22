# Axial Union of placed operands

`Union` of axial extrusions (`agent-docs/axial-section-boolean-design.md`) takes operands that carry a rigid
placement, such as the copies `PatternCopies` and `Copy` make. One tooth, one rib or one lobe is modeled once,
patterned about the sweep axis, and joined to its base in one `Union`.

The rule names no shape. It applies to any axial operands whose placement keeps the sweep planes.

## 1. What a placed operand is

An operand is a prism or a matching axial loft, as in axial-section-boolean §1, with any placement `Xform`. It denotes
its recorded section on its frame, mapped by `Xform` read as an exact rational affine map of the held floats. A copy of
a free-form section therefore denotes the rigid image the placement states, and its record is untouched.

A prism that carries a section displacement, and a loft that carries a bore, a blend, a construction delta or a surface
result, are still not operands.

## 2. Placement onto the reference frame

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

## 3. Re-recording and the charge

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

## 4. The result

`UnionMany` runs `Place` first, then the route of axial-section-boolean §2 over the moved sections. The loft it builds
carries `loftrecord.Payload.JoinDelta`, the largest `Delta` of its operands (an operand that is itself a join
contributes its own).

`loftrecord.Build` adds `JoinDelta` to the held-vertex displacement, the chord displacement and the matched
displacement of the join body, so every reading of that body encloses the ideal union of the placed operands. A body
rebuilt from a copy of the payload (`Fillet`, a through-bore `Cut`, a placement) keeps the term as extra slack and
denotes its own record: its readings make no claim about the operation applied to the ideal join. `ConstructionDelta`
is not used, because it records a fitted-spline fillet's departure from its ideal rewrite and the fillet refuses a
second one.

## 5. Limits

- The placement must keep the sweep planes to the rounding of the placement (§2): a rotation about the sweep axis, a
  translation in the plane, or any composition, on any frame. A tilt, a lift off the plane or a differently sized
  sweep is a miss, and the Union takes its next route.
- `Fillet`, a through-bore `Cut` and a cap `Chamfer` of the join read the frame's exactness themselves: the loft fillet
  needs an end frame exactly on the sweep axis (`loftrecord.FilletStraight`), so a join on an oblique frame refuses a
  fillet.
- A set whose only loft is itself placed has no reference frame and is a miss.
- Segments of kind line, arc, circle and fit spline are placed. Other kinds are a miss.
- Copies whose free-form edges cross one another (two arched tops that overlap), or whose sides nearly coincide without
  sharing a whole edge, are a miss: `sketch` certifies a spline against a circle but not a spline against another spline
  or an arc. `apitest.TestPatternJoinRefusesCopiesItCannotCertify` pins both refusals against the 4 mm step that joins.
- The route is `Union` only.

## 6. Tests

- `internal/patternrecord`: `MovePlaneRegion` refuses a tilt and a lift and accepts the identity, charges a 17th-turn
  turn about the normal of an oblique frame within 1e-12 and refuses a hundredth-radian tilt of it; the fit charge is below
  1e-13 for a 17th-turn rotation and is at least a deliberate 1e-3 displacement of one knot, so it is not vacuous; the
  circle charge is the radius scale deviation.
- `internal/axialsection`: a lobe placed by a 17th turn is re-recorded within a positive `δ` below 1e-12 and each fit
  point is the rotated source point; misses for a tilt, a lift and a placed reference; each height and plane guard fires
  alone (shown to fail by disabling it).
- `apitest.TestPatternedLobesJoinADisc`: one lobe patterned 8, 12, 17 and 24 times and joined to a disc has the volume
  of the same lobes drawn at their angles within both bounds, with a larger bound, and a `VerifyAll` mesh whose boundary
  and volume are verified. `TestPatternedLobesAboutATiltedAxisAreRefused`: `ErrUnsupported`.
  `TestPatternJoinOnAnObliqueFrame`: the same part on a frame no axis aligns with has the world-frame part's volume
  within both bounds, with a verified boundary and volume.

## 7. PatternJoin

```go
func PatternJoin(ctx context.Context, target, feature *Body, spec PatternSpec) (*Body, error)
```

`PatternJoin` repeats `feature` by `spec` and joins every instance, `feature` included, to `target` in one `Union`, and
retires `target` and `feature`. It is `PatternCopies` followed by `Union(target, feature, copies...)` with the copies
never registered: they are built under consecutive producer identities, handed to the many-operand Union route chain
(`unionManyBodies`, which `Union` shares) and dropped. A refusal leaves the document unchanged and both bodies live.

The gates are `PatternCopies`' for `feature` and `spec`, and `Union`'s for `target`: a nil, foreign or retired target is
refused, `target` and `feature` must be distinct, and a sheet in either role is `ErrUnsupported`. The join takes the
routes `Union` takes over the same operands, so every refusal of §5 and of `agent-docs/axial-section-boolean-design.md`
§3 is returned unchanged.

`Patterned` keeps its own rule (`agent-docs/mirror-pattern-design.md` §4.3, §6.1): it returns the instances of one body
as one body, and refuses disjoint lumps. `PatternJoin` is the call for a feature that joins a base.

## 8. Features whose flanks cross the base

A feature whose fit-spline flanks start inside the base's circle and leave it joins the base the same way: the private
scene cuts each spline against the circle, and `sketch` certifies the crossings (`TExact` on every `Partial` fragment of
its pair certificate, `sketch/docs/pair-certificate-design.md`). The union records the circle's remainders as trimmed
`CircleSeg` fragments. A union `sketch` leaves uncertified is a miss, as in axial-section-boolean §3. The joined body
then takes a concave `Fillet` where each flank leaves the circle, a through-bore `Cut` and a cap `Chamfer`.

Tests: `apitest.TestPatternJoinFlanksCrossTheBase` joins 8, 12 and 24 patterned lobes whose flanks cross the disc and
compares their volume with the same lobes drawn at their own angles.
`TestPatternJoinFlanksCrossTheBaseThenFilletBoreChamfer` takes 12 of them through the fillet, bore and chamfer.
