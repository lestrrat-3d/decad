# Section outline design

`Body.SectionOutline` cuts a solid's verified tessellation with a plane and returns the closed outlines of the cut. It
reaches every solid that tessellates with a verified boundary, a mesh Boolean result among them, where `Body.Section`
(`agent-docs/section-design.md`) refuses all but certified classes. The result is a reading of the mesh. It is not a
sketch and enters no feature, because a sketch profile is exact input and this outline is not.

## 1. Contract

```go
type SectionOutline struct {
    Plane      r3.Frame
    Loops      [][]r3.Vec
    OffSurface units.Value
}

func (s SectionOutline) PolygonArea() units.Value
func (b *Body) SectionOutline(ctx context.Context, plane r3.Frame, tol units.Value) (SectionOutline, error)
```

| Decision | Rule |
|---|---|
| O1 | The body is tessellated at `VerifyBoundary` and `tol`, as `DistanceToPoint` does. A mesh without a verified finite boundary bound is `ErrUnsupported`. A sheet or unsound solid is `ErrNotSolid`. A nil body or context, or an invalid plane, is `ErrDegenerate`. |
| O2 | A mesh vertex whose distance along the plane's normal is zero or positive counts as on the normal's side, as if the plane were moved a hair against the normal. The cut then never passes through a vertex, and every outline closes. A plane lying in a face reads the cut just behind it along the normal. |
| O3 | A triangle with vertices on both sides contributes one segment between two points on its crossing edges. Walked counter-clockwise seen from outside, a triangle has one edge crossing toward the normal's side (up) and one crossing back (down), and its segment runs from the down crossing to the up crossing. No geometry decides the direction. |
| O4 | Segments chain by the identity of the mesh edge they cross, not by coordinates. A point on an edge is computed from the edge's vertices in index order, so the two triangles that share the edge compute the same point. A segment that starts no loop or ends at no segment makes the cut open, which is `ErrUnsupported`. |
| O5 | An outer loop runs counter-clockwise seen from the tip of the plane's normal and a hole clockwise, when the mesh winds counter-clockwise seen from outside. |
| O6 | `Loops` are in world millimetres and each loop's last point joins its first. A plane that misses the body gives no loop and no error. |
| O7 | `OffSurface` is the mesh's proven displacement bound plus eight units of float64 rounding at the largest coordinate magnitude of the vertices, the loop points and the plane origin. Every loop point lies within `OffSurface` of the body's true boundary and of the plane. |
| O8 | `PolygonArea` sums the shoelace area of each loop seen from the plane's normal, an outer loop positive and a hole negative. It is the polygons' area and no more. |

## 2. What the bound does not say

`OffSurface` bounds how far each loop point lies from the true surface and the plane. It does not bound how far the
true section's outline lies from the loops. Where the plane meets the surface at a shallow angle, the true section can
differ in outline and in area by far more than `OffSurface`, and a plane that touches the surface may cut the mesh
where it misses the true body or the reverse. `PolygonArea` inherits that: it is the polygons' area, not an enclosure
of the true section's area. A caller that needs a certified section uses `Body.Section`.

## 3. Verification

The slicer tests cut a cube at mid-height, through a coplanar face, obliquely and past it, with two shells and with a
cavity wound inward, and cut a mesh missing a triangle. The API tests cut a block, a drilled plate whose hole is
faceted, a cube by a plane square to its diagonal and a plane lying in a face. Reversing the segment direction or
counting an on-plane vertex as below turns them red.

## 4. Not covered

- A sketch or profile result that enters a feature. Use `Body.Section` where it admits the body.
- An enclosure of the true section's outline or area.
- Sheets, and meshes whose cut does not close.
- Polyline simplification: a loop holds every cut point, including collinear ones.
