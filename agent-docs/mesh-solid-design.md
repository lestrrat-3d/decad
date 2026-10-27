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
| M4 | Triangles share a source group, and so a face, when their planes are the same exactly: the normal `(b − a) × (c − a)` and the offset `n · a` are worked in rational arithmetic from the floats and divided by the first non-zero magnitude of the normal, so a plane and its orientation have one key. The group is `Planar`. A triangle of no area is a group of its own, not `Planar`. Edge-connectivity then splits a group into its patches, as for any faceted body. |
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

## 5. Not covered

- Welding, orienting and hole filling: repair the mesh first with `RepairMesh`.
- Recovering planes, cylinders or cones from a curved mesh, and so a retained analytic tag. A cylinder read from STL is a
  prism of many faces.
- A mesh of more than 65,528 triangles.
- Merging coplanar groups that touch only along a vertex, and reading planes through an approximation: the grouping is
  exact or none.
