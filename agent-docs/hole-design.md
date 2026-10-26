# Hole design

`Hole` cuts a round hole into a planar face of a solid. It is a feature-level entry point over tools the library
already builds and cuts: it adds placement, validation and an atomic commit, and no new geometry kernel. The public
contract is in `agent-docs/api-design.md` §8; this document owns the decisions behind it.

## 1. Contract

```go
func Hole(ctx context.Context, b *Body, spec HoleSpec) (*Body, error)
```

`HoleSpec` holds `Face` (a `FaceSelector`), `At` (a world `r3.Vec`), `Diameter`, `Depth` (`ThroughHole` or
`BlindHole`), an optional `Head` (`Counterbore` or `Countersink`) and an optional `Shape` (`RectangularHole`, §5, or `SlotHole`, §6). `Hole` retires `b` and returns the drilled body.

| Decision | Rule |
|---|---|
| H1 | `Face` resolves under the exactly-one rule of `FaceSelector.SelectFaces`. The face MUST have an analytic `Plane` surface; any other surface is `ErrUnsupported`. |
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

- A hole in a face that is not an analytic plane, or in a sheet.
- A noncircular hole other than `RectangularHole` and `SlotHole`, a threaded hole, and a tapped-hole annotation. `Thread` (`agent-docs/thread-design.md`) threads a bored hole afterward.
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
