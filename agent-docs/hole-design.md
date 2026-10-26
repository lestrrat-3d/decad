# Hole design

`Hole` cuts a round hole into a planar face of a solid. It is a feature-level entry point over tools the library
already builds and cuts: it adds placement, validation and an atomic commit, and no new geometry kernel. The public
contract is in `agent-docs/api-design.md` §8; this document owns the decisions behind it.

## 1. Contract

```go
func Hole(ctx context.Context, b *Body, spec HoleSpec) (*Body, error)
```

`HoleSpec` holds `Face` (a `FaceSelector`), `At` (a world `r3.Vec`), `Diameter`, `Depth` (`ThroughHole` or
`BlindHole`), an optional `Head` (`Counterbore` or `Countersink`) and an optional `Shape` (`RectangularHole`, §5, `SlotHole`, §6, or `PolygonHole`, §10). `Hole` retires `b` and returns the drilled body.

| Decision | Rule |
|---|---|
| H1 | `Face` resolves under the exactly-one rule of `FaceSelector.SelectFaces`. The face MUST have an analytic `Plane` or `Cylinder` surface (§7); any other surface is `ErrUnsupported`. |
| H2 | `At` is a world point. `Frame.ToLocal` gives its plane coordinates. A point more than `holeplan.OnPlaneTol` (1e-6 mm) off the plane is `ErrDegenerate`; the point is never moved onto the plane. |
| H3 | The hole runs along the face's inward normal. The outward normal is `Face.NormalAt` at the plane origin. The tool sweeps against the frame normal exactly when that normal points outward. |
| H4 | `Diameter`, a blind `Depth`, a counterbore's diameter and depth, and a countersink's diameter are positive finite lengths. A countersink angle is the included angle, strictly between 0 and π. A violation is `ErrDegenerate`, or `ErrUnitKind` for a wrong unit kind. |
| H5 | A head must be wider than the hole. A counterbore must be shallower than the hole. A countersink's cone depth, `(Dhead/2 − D/2)/tan(angle/2)`, must be shallower than the hole. A violation is `ErrDegenerate`. |
| H6 | A through hole runs `holeplan.Reach` deep: the extent of the body's bounds behind the face, plus the bounds' own bound, plus `holeplan.ThroughMargin`. No tool face lies in a body face at the far side. |
| H7 | Every tool starts `holeplan.MouthMargin` outside the face. No tool face lies in the face plane, because a coplanar contact is `BooleanUnsupportedContact`. |
| H8 | `Body.LocatePoint` classifies the point a quarter of the bore depth inside the hole axis. A certified `PointOutside` is `ErrDegenerate`. Any other answer, including an error, lets the hole proceed: the check can refuse and never admits. |

## 2. Tools and the cut

- The bore is a circle sketched on the face frame moved `MouthMargin` outward, extruded inward by the bore depth plus
  `MouthMargin`.
- A counterbore is a second, wider circle extruded the same way by its depth plus `MouthMargin`.
- A countersink is a quadrilateral revolved a full turn about the hole axis. It is sketched on a plane holding the
  axis, with radial and outward-height coordinates. Its corners are the axis at the cone bottom, the bore radius at
  the cone bottom, the cone at an overshoot height of half the head diameter above the face, and the axis at that
  height. The cone's base never lies in the face plane (H7).
- The bore is cut first and the head second, so the head tool's inner part lies in empty space.
- Each cut runs through `booleanBody` with no commit. The final body replaces `b` in one operation commit, and private tools never enter
  the live body set. A failure in any step removes the tools and leaves `b` live and the document unchanged.
- A tool is live in the document between its build and the removal. A concurrent `Document.Bodies` can see it.

## 3. Exactness

`Hole` returns what `Cut` returns for the same tools. A round hole is `Approximate` because no `CircleSeg` build is
exact (`agent-docs/prism-boolean-design.md` §7). The bore wall is one cylinder face and a countersink wall is one cone
face. A countersink cut gives a faceted result whose planar faces are split into facets.

## 4. Not covered

- A hole in a face that is not an analytic plane or cylinder, or in a sheet.
- A noncircular hole other than `RectangularHole`, `SlotHole` and a regular `PolygonHole`, a threaded shaped hole, a threaded hole on a cylindrical face, and a thread on a counterbore or countersink.
- Several holes in one call. Call `Hole` per hole, or build one hole and use `Patterned`.

## 5. Rectangular holes

A non-nil `HoleSpec.Shape` replaces the round bore. `RectangularHole` holds `Width` and `Height` (positive lengths) and
`Angle`. The rectangle is centred on `At`, with `Width` along the face frame's U axis before `Angle` turns it
counter-clockwise about the hole axis.

| Decision | Rule |
|---|---|
| R1 | `Diameter` MUST be the zero `units.Value` and `Head` MUST be nil when `Shape` is set. Either one set is `ErrDegenerate`. |
| R2 | `Width` and `Height` are positive finite lengths and `Angle` is a finite angle. A violation is `ErrDegenerate`, or `ErrUnitKind` for a wrong unit kind. |
| R3 | Placement (H2, H3), depth (H6, H7) and the axis check (H8) are the round hole's. |
| R4 | The bore tool is the rectangle sketched on the mouth plane and extruded inward by the bore depth plus `holeplan.MouthMargin`. Its four corners are fixed sketch points joined by lines. |

The cut is a prism cut, so a rectangle turned by a multiple of a quarter turn about the face frame keeps the planar
walls of the prism classes. Every wall is a plane, and a blind hole adds one planar floor.

## 6. Slot holes

`SlotHole` holds `Length`, `Width` and `Angle`. The bore is a stadium: a strip `Width` wide whose two ends are half
circles, `Length` long end to end, centred on `At`. Its long axis lies along the face frame's U axis before `Angle`
turns it counter-clockwise about the hole axis.

| Decision | Rule |
|---|---|
| L1 | R1 and R3 apply. |
| L2 | `Length` and `Width` are positive finite lengths with `Length` greater than `Width`, and `Angle` is a finite angle. A violation is `ErrDegenerate`, or `ErrUnitKind` for a wrong unit kind. A `Length` equal to `Width` is a round hole and is refused. |
| L3 | The bore tool is the stadium sketched on the mouth plane and extruded inward as in R4. Two fixed lines join two counter-clockwise arcs about the end centres, which are `(Length − Width)/2` from the axis along the slot. |

The walls are two planes and two cylinders, so a slot hole is `Approximate` as a round hole is (§3).

## 7. Holes on a cylindrical face

A face that is an analytic `Cylinder` takes a radial hole: the axis runs along the radial line through `At`, into the
material, whether the face is a rod's outer wall or a bore's inner wall.

| Decision | Rule |
|---|---|
| Y1 | `At` lies within `holeplan.OnPlaneTol` of the cylinder's surface, a distance from its axis within 1e-6 mm of the radius, and not on the axis. A violation is `ErrDegenerate`. |
| Y2 | The site is the plane tangent to the cylinder at `At`, with the face's outward normal there as its normal. Placement, depth, reach and the axis check (H2 to H8) then read it as they read a plane. A blind `Depth` is measured from the surface at `At`. |
| Y3 | The tools start `holeplan.CurvedMargin` outside the tangent plane. A rod's wall falls away from the plane, so the margin is `holeplan.MouthMargin`. A bore's wall rises toward it by the sagitta `R − sqrt(R² − r²)` at distance `r`, so the margin is `MouthMargin` plus the sagitta of the largest radius the bore's section reaches, and no sliver of wall stands above the tool's start. |
| Y4 | The largest radius is half the diameter of a round hole, half the diagonal of a `RectangularHole` and half the length of a `SlotHole`. In a bore it MUST be less than the bore's radius, or the hole is `ErrUnsupported`. |
| Y5 | A `Head` on a cylindrical face is `ErrUnsupported`. |

The result is a mesh Boolean's, faceted, with the Cut's own bounds. The tests compare volumes against integrals
computed independently: the chord of the rod weighted by the hole's section, and the wall's thickness weighted by the
hole's section.

## 8. Tapped holes

`HoleSpec.Thread` taps a round hole in the same commit that drills it.

```go
type HoleThread struct {
    Pitch    units.Value
    LeftHand bool
    Length   units.Value
}
```

| Decision | Rule |
|---|---|
| X1 | The hole's `Diameter` is the thread's minor (tap drill) diameter, and the groove is the internal ISO profile of `agent-docs/thread-design.md` T5 about that bore. |
| X2 | The coil is built as `Thread` builds one, with its frame on the hole axis where the bore proper starts: the face for a plain hole, the floor of a counterbore, the bottom of a countersink's cone (`holeplan.Plan.HeadDepth`). Its V axis runs into the body, and the thread is right-handed unless `LeftHand` is set. |
| X3 | The coil is one more tool after the bore and its head. `combineTools` cuts the tools in order, so the thread is cut from the holed body and the whole is one `OperationHole` commit. A failure leaves the plate live and the document unchanged. |
| X4 | `Length` is measured from where the bore proper starts. The zero `Value` threads the whole bore: a through hole's groove runs out of its far end, and a blind hole's stops at the bottom with the coil's flat end. A positive `Length` no longer than the bore stops the groove there likewise. A longer one is `ErrDegenerate`. |
| X5 | A `Pitch` that is zero is `ErrDegenerate`, a negative one `ErrNegativeMagnitude` and a wrong unit kind `ErrUnitKind`. A pitch too coarse for the bore, so that the groove reaches the axis, is `ErrDegenerate`. |
| X6 | A thread needs a round hole: a `Shape` with a thread is `ErrDegenerate`. A hole on a cylindrical face with a thread is `ErrUnsupported`. |
| X7 | The groove under a counterbore's floor lies inside the counterbore's radius, so the head void holds the groove's start: a counterbore narrower than the thread's major diameter would be cut into by the groove and is the caller's to avoid. |

## 9. Drafted holes

`HoleSpec.Draft` leans the round bore's wall. It passes `WithTaper` to the `Extrude` that builds the bore tool, as
`agent-docs/draft-design.md` §2 states it.

| Decision | Rule |
|---|---|
| Z1 | `Diameter` is the bore's diameter at the face. A positive `Draft` makes the bore narrower toward its bottom, with the radius `Diameter/2 − t·tan(Draft)` at the depth `t`, and a negative one wider. The zero `Value` is a straight bore. |
| Z2 | The tool starts `Margin` outside the face (H7) and the taper runs from there, so its circle is wider than the bore by `Margin·tan(Draft)`: the wall is then the stated diameter at the face (`holeplan.Plan.BoreDraft`, `holeCylinder`). |
| Z3 | A counterbore's or countersink's own tool is not drafted. The bore proper below a counterbore leans from the face, so its radius at the counterbore's floor is `Diameter/2 − HeadDepth·tan(Draft)`. |
| Z4 | `Draft` is read as a signed angle. A wrong unit kind is `ErrUnitKind`, a magnitude at or past a right angle is `ErrDegenerate`, and a draft that closes the bore before its bottom is the drafted `Extrude`'s own refusal, `ErrUnsupported`. |
| Z5 | A shaped hole with a draft, a thread with a draft (a tapped hole is straight) and a draft in a hole on a cylindrical face are `ErrDegenerate`, `ErrDegenerate` and `ErrUnsupported`. |
| Z6 | `BossBore.Draft` is the same for a boss's pilot (`agent-docs/boss-design.md` §5). |

## 10. Polygonal holes

`PolygonHole` holds `Sides`, `AcrossFlats` and `Angle`. The bore is a regular polygon centred on `At`, built as `RectangularHole`'s is:
a prism over the polygon from `holeplan.Plan.Corners`.

| Decision | Rule |
|---|---|
| P1 | `Sides` is from 3 to 64: any other is `ErrDegenerate`. `AcrossFlats` is a positive length, the distance between two opposite flats of an even polygon and twice the inradius of any. `Angle` is an angle, so a zero value of no kind is `ErrUnitKind`: write `units.Degrees(0)`. |
| P2 | The corners lie at the circumradius `AcrossFlats / (2·cos(π/Sides))` from the axis, at the angles `Angle + π/Sides + 2π·i/Sides`, counter-clockwise from the face frame's U axis. With a zero `Angle` a flat faces +U, between the corners at ±π/Sides, whatever the number of sides. |
| P3 | The polygon replaces the round bore as `RectangularHole` does: `Diameter` stays zero, a head is `ErrDegenerate`, and a through or blind depth reads as for a round hole. The mouth margin and the reach gates take the circumradius as the largest radius of the bore (Y4). |
| P4 | The removed volume is `Sides · (AcrossFlats/2)² · tan(π/Sides) · depth`, which the tests compute independently, and the polygon's walls are planes. |
