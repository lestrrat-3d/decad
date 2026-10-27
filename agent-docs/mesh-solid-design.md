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

## 2. Not covered

- Welding, orienting and hole filling: repair the mesh first with `RepairMesh`.
- Recovering planes, cylinders or cones from a curved mesh, and so a retained analytic tag. A cylinder read from STL is a
  prism of many faces.
- A mesh of more than 65,528 triangles.
- Merging coplanar groups that touch only along a vertex, and reading planes through an approximation: the grouping is
  exact or none.
