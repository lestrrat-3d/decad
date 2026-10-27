# Mesh to solid design

`SolidFromMesh` turns a closed `MeshBody`, such as an imported STL or STEP part, into a solid `Body` of a document, so it
takes part in Booleans, measurement, selection and `Verify`. It builds the same Faceted body a mesh Boolean returns,
with the proofs the builder demands, and adds no geometry kernel.

## 1. Contract

```go
func SolidFromMesh(ctx context.Context, d *Document, m *MeshBody) (*Body, error)
```

| Decision | Rule |
|---|---|
| M1 | `m` MUST be closed with every edge in two triangles, and wound outward: an open or non-manifold mesh and a mesh wound inward are `ErrNotSolid`, as is one whose exact signed volume is not positive. A nil context, document or mesh is `ErrDegenerate`. |
| M2 | The mesh must not cross itself. `loftmesh.LoftCrossingAuditBounded`, the exact contact audit of the lofted bodies, proves it for at most `loftmesh.MaxLoftAuditTriangles` (65,528) triangles within a 16 million pair-scan budget. A crossing it finds is `ErrDegenerate`; a larger mesh, or one whose audit exceeds its work ceiling, is `ErrUnsupported`. The audit can only refuse: no body is built that it has not cleared. |
| M3 | The held vertices are the mesh's own floats, so the vertex bound is zero everywhere and so are the mesh bound, the volume's symmetric difference and the area slack. The volume is read from the exact signed volume of the triangles by `facetproof.AuditFacetedMesh` and `booleaneval.VolumeReading`, with whatever bound turning that rational to a float needs; the diameter bound is the bounding box's diagonal, rounded up. |
| M4 | Triangles share a source group, and so a face, when their planes are the same exactly: the normal `(b − a) × (c − a)` and the offset `n · a`, worked exactly from the floats as dyadics and divided by the first non-zero magnitude of the normal, are equal rationals, so a plane and its orientation have one group (`internal/meshplane`). The comparison cross-multiplies by the two magnitudes; a float key, the downward rounding of each quotient, only buckets the planes, since equal planes round alike. The group is `Planar`. A triangle of no area is a group of its own, not `Planar`. Edge-connectivity then splits a group into its patches, as for any faceted body. |
| M5 | The faces are Faceted: they carry no analytic surface tag, so selection by `Planar` or a normal does not find them, and a Boolean takes them through the generic mesh path with its pair tolerance. The result is registered in the document as one `OperationSolidFromMesh` entry with no input. A refusal leaves the document unchanged. |
| M6 | A group whose exact normal has one non-zero part, so the plane is square to a world axis, keeps that plane as a retained tag: its frame has the group's first vertex as origin, which lies on the plane exactly, and the unit axes ordered so the frame's normal is the outward one. The tag departs from the exact plane by nothing, so its normal bound is zero. Every other group stays Faceted, because no float unit normal names its plane. |

## 2. Blends on a mesh solid

A retained plane is what the retained edge blend reads (`agent-docs/retained-edge-blend-design.md`, `agent-docs/retained-modify-reach-design.md`),
so `Fillet` and `Chamfer` build on an edge of a mesh solid between two axis-square faces whose ends lie on axis-square
faces, exactly as on a mesh Boolean result with retained planes: the call is that route's, its gates and bounds
unchanged. An edge that meets a slanted face has no plane to blend against and is that route's `ErrUnsupported`. The tests
state a block's filleted and chamfered volumes from the section removed, `(1 − π/4)·r²·L` and `r²·L/2`, and require them inside
the result's bound, and check that the tetrahedron's slanted face is Faceted and its blend refused.

## 3. Stretching a mesh solid

`Stretch(ctx, b, anchor, x, y, z)` scales a faceted solid by a separate factor along each world axis about `anchor`
and returns a new solid, retiring `b`. `b` MUST be faceted with all-zero vertex bounds, otherwise `ErrUnsupported`;
a body with a vertex bound would need the bound charged through the stretch, which this route does not do. The
mesh is stretched by `ScaleMesh`, which reorders corners under an odd number of negative factors, and read back by
`solidFromMeshBody`, so the stretched planes are exact over the stretched coordinates. Each factor is dimensionless,
finite and not zero. The tests state the volume as the product of the factors, the bounds and a point inside the
widened part, and a swapped pair of factors fails them.

## 4. Features on a mesh solid

A mesh solid's faces are exact planes (§2), so `Hole`, `Boss` and the other tool features that cut or raise a prism
take it as they take a prism-faced body, through `Union` and `Cut` of the prism tool with the faceted body. A tool
with straight sides and an axis-square placement (a rectangular or polygonal pocket, an `OutlineHole`, a rectangular
boss) is read by the mixed route of `agent-docs/orthogonal-boolean-design.md` B7, so the result is exact; a round
bore or boss is a tool the grid does not hold, so it takes the general mesh Boolean route and its volume is read
inside the bound that route states. The tests cut a rectangular pocket, an L-shaped through hole and a round bore
and raise a rectangular boss on a plate, and read volumes, face counts and points. `Emboss` of a rectangular
relief reads the same way, through the tool-body combine of B13.

## 6. Extents against a mesh solid

`SolidFromMesh` gives each face square to a world axis its plane and a record of that plane: the plane's own frame at
level zero, held and denoted alike, with no displacement. `ToFace` reads that record as it reads a prism's cap, so a
sketch swept up to such a face stops at its coordinate exactly, and a face square to another axis is refused as before.
A placement or a similarity drops the record, as for every record. A face that is not axis-square has no record and is
no stop.

A held mesh whose vertices carry no bound states its extent along any direction as the least and greatest projection of
a vertex, beside the rounding of that projection, so `ThroughAll` meets a mesh solid: the sweep ends at its far side
and the stop reads Approximate with the rounding as its bound. A mesh with a vertex bound states no interval and is
refused. The tests sweep a sketch to the top of a mesh plate with `ToFace` (exact bounds and an exact volume) and cut
the pocket, sweep it through all with `ThroughAll`, and send a sweep to a side face. Removing the record fails the
first, and removing the extent fails the second.

## 7. Face centroids

`SolidFromMesh` gives each planar face its area centroid. Coplanar triangles of one orientation have areas in the ratio
of the dot products of their normals with the first one's, so the centroid is the exact rational
`Σ wᵢ(aᵢ+bᵢ+cᵢ)/3 / Σ wᵢ` over the vertices' exact float values, `(a+b+c)/3` for a face of one triangle, rounded to
the nearest float per coordinate by dyadic quotient rounding, with no rational formed. Its bound is
the largest rounding error, Exact when that is zero. A group that falls into several disconnected patches gives none of
them a centroid, as for every group. A Boolean result, a placement or a mesh Boolean does not carry the reading. The tests
read the five faces of a triangular prism, and a square top face split into unequal triangles, where an unweighted mean
of the triangles' centroids fails.

`Body.ExtremeFacesByCentroid` ranks a mesh solid's faces by these readings. The least face is the one whose centroid
interval, along the axis, lies strictly below every other face's, and the greatest the one strictly above; an overlap or
a tie is `ErrUnsupported`, because the readings do not decide the order. The tests rank the faces of a prism along a
diagonal and along z, and refuse a tie along x; comparing intervals with `<=` instead of `<` fails the tie test.

## 5. Not covered

- Welding, orienting and hole filling: repair the mesh first with `RepairMesh`.
- Recovering planes, cylinders or cones from a curved mesh, and so a retained analytic tag. A cylinder read from STL is a
  prism of many faces.
- A mesh of more than 65,528 triangles.
- Merging coplanar groups that touch only along a vertex, and reading planes through an approximation: the grouping is
  exact or none.
