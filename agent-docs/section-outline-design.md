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

## 5. Clearance

```go
type SectionClearance struct {
    Distance Measurement
    A, B     r3.Vec
    Overlaps bool
}

func (c SectionClearance) Lower() units.Value
func (s SectionOutline) Clearance(other SectionOutline) (SectionClearance, error)
```

`Clearance` reads how closely two outlines cut by the same plane come, such as the two ribbons of a screw gear pair.
It measures the material regions the outlines enclose, in the plane. It is a pure reading of the polylines: it runs no
tessellation and takes no context.

| Decision | Rule |
|---|---|
| C1 | The planes must be one frame: origin, `U` and `V` agree within 1e-9 (millimetres in the origin, and in each component of the unit axes). Otherwise, or for an outline without a loop, a loop of fewer than three points, a point that is not finite or an `OffSurface` that is negative or not finite, the result is `ErrDegenerate`. |
| C2 | Points are projected into the first outline's plane. Material is where the winding numbers of an outline's loops sum to a positive number, so an outer loop adds material and a hole removes it, and a loop inside a hole inside an outer loop is material again. |
| C3 | The regions overlap when a segment of one outline properly crosses a segment of the other (each end strictly on both sides of the other's line), or when a loop of one outline lies, away from the other's boundary, inside the other's material. Then `Distance` is zero, `Overlaps` is true and `A` and `B` are the crossing point or a vertex of the contained loop. |
| C4 | Otherwise `Distance` is the least distance between any segment of one outline and any segment of the other, and `A` and `B` are the points of the loops that attain it. Outlines that touch or coincide without a proper crossing read zero with `Overlaps` false. |
| C5 | Both outlines' segments sit in bounding-box trees. A crossing search descends only box pairs that meet. The distance search descends the nearer pair first and prunes every pair no nearer than the best found, so outlines that stay apart cost near `(n+m)log(n+m)`. Two outlines of 5,000 segments take milliseconds. |
| C6 | `Distance` is `Approximate`. Its `Bound` is the sum of both `OffSurface` values plus thirty-two units of float64 rounding at the largest coordinate magnitude and the effect of the plane tolerance on a point at that magnitude, rounded up. |
| C7 | `Lower` is `Distance` less `Bound`, not below zero, and zero when `Overlaps` is true. A gap is certified only when `Lower` is positive: a reading whose `Bound` reaches `Distance` certifies nothing. |

The bound inherits the limit of §2. `OffSurface` bounds how far each loop point lies from the true boundary and the
plane, so the bound covers the points' displacement. It does not cover a true section whose outline differs from the
polylines by more, which happens where the plane meets the surface at a shallow angle. A reading whose points lie
near such a place states a gap no tighter than its mesh can support.

The tests check two squares at a known gap, a corner gap, touching squares, overlapping squares, a square inside a
square, a square in a hole and a plate with a hole and an island, the bound arithmetic, the refusals, and two boxes
built in decad and cut by one plane. A property test compares the result with an O(n*m) brute force on random
polygons, and a timing test runs two circles of 5,000 segments.
