# Hole design

`Hole` cuts a round hole into a planar face of a solid. It is a feature-level entry point over tools the library
already builds and cuts: it adds placement, validation and an atomic commit, and no new geometry kernel. The public
contract is in `agent-docs/api-design.md` §8; this document owns the decisions behind it.

## 1. Contract

```go
func Hole(ctx context.Context, b *Body, spec HoleSpec) (*Body, error)
```

`HoleSpec` holds `Face` (a `FaceSelector`), `At` (a world `r3.Vec`), `Diameter`, `Depth` (`ThroughHole` or
`BlindHole`) and an optional `Head` (`Counterbore` or `Countersink`). `Hole` retires `b` and returns the drilled body.

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
- A noncircular hole, a threaded hole, and a tapped-hole annotation.
- Several holes in one call. Call `Hole` per hole, or build one hole and use `Patterned`.
