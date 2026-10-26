# Mesh import design

`ReadSTL` and `ReadOBJ` read an STL or OBJ file into a `MeshBody`: triangles and nothing else. A `MeshBody` is not a `Body`. It
has no faces, edges or analytic surfaces, and no feature, Boolean or `Verify` takes it, and only the mesh writers of §6 export it. It reads out what its
triangles say, so a mesh from another tool can be checked (is it closed, how much does it hold, how large is it)
before a model is built around it. The separate type is `agent-docs/api-design.md` invariant 5.

## 1. Contract

```go
// package decad
func NewMeshBody(ctx context.Context, vertices []r3.Vec, triangles [][3]int) (*MeshBody, error)

func (m *MeshBody) Vertices() []r3.Vec
func (m *MeshBody) Triangles() [][3]int
func (m *MeshBody) Report() MeshReport
func (m *MeshBody) Bounds() Box
func (m *MeshBody) Area() (Measurement, error)
func (m *MeshBody) Volume() (Measurement, error)
func (m *MeshBody) LocatePoint(ctx context.Context, p r3.Vec) (PointLocation, error)
func (m *MeshBody) DistanceToPoint(ctx context.Context, p r3.Vec) (Measurement, error)
func (m *MeshBody) ClosestPoint(ctx context.Context, p r3.Vec) (ClosestPoint, error)

func ReadSTL(ctx context.Context, r io.Reader, opts ...MeshImportOption) (*MeshBody, error)
func ReadOBJ(ctx context.Context, r io.Reader, opts ...MeshImportOption) (*MeshBody, error)
func WithMaxMeshTriangles(n int) MeshImportOption
func WithMaxMeshBytes(n int64) MeshImportOption

var ErrMalformedMesh, ErrMeshTooLarge error
```

The readers live in the root package because a separate public package would need its own race shard in
`.github/workflows/ci.yml`. The parsing is plain-data code in `internal/meshparse/`.

| Decision | Rule |
|---|---|
| M1 | A `MeshBody` holds the vertices and triangles it was built from, copied, and nothing derived from a tolerance. Coordinates are millimetres. |
| M2 | `NewMeshBody` repairs nothing and welds nothing. A nil context or a mesh with no triangle is `ErrDegenerate`. A vertex that is not finite or an index outside the vertices is `ErrUnsupported`. |
| M3 | `MeshReport` counts vertices, triangles, zero-area triangles, boundary edges (used by one triangle), non-manifold edges (used by three or more), misoriented edges (used by two triangles in the same direction) and shells (groups of triangles joined across shared edges). `Closed` means every edge is used by exactly two triangles in opposite directions. `Inverted` means closed with a negative signed volume. A zero-area triangle joins no edge and no shell. |
| M4 | `Bounds` is the box of the vertices and is `Exact`. A vertex no triangle uses still counts. |
| M5 | `Area` is the sum of triangle areas. Each is half a square root of an exact rational, so the reading encloses the sum between a directed-rounded lower and upper sum and is `Approximate` unless every root is exact. |
| M6 | `Volume` is `ErrNotSolid` unless the mesh is `Closed` and not `Inverted`. It is the exact sum of signed tetrahedra about the origin, so it is `Exact` when that rational is a float and otherwise carries only the rounding of the nearest float. |
| M7 | `ReadSTL` reads a file as binary when its length is 84 bytes plus 50 per triangle its count names, and as ASCII when it starts with `solid`. Facet normals and attribute bytes are ignored. A binary coordinate is a 32-bit float taken exactly as a 64-bit one. |
| M8 | `ReadOBJ` reads `v` and `f` lines only. A face index is positive (from 1) or negative (back from the latest vertex), and any `/` part after the vertex is ignored. A face with more than three vertices is a fan from its first vertex. |
| M9 | `ReadSTL` and `ReadOBJ` weld vertices with identical coordinates, and only identical ones. A repeated corner joins the triangles that share it, and two corners that differ in any bit stay separate. A stored `-0` is `+0`. |
| M10 | Input past `WithMaxMeshTriangles` (default 5,000,000) or `WithMaxMeshBytes` (default 512 MiB) is `ErrMeshTooLarge`. Malformed input is `ErrMalformedMesh`. A non-positive limit and a nil context or reader are `ErrDegenerate`. |

## 2. Why the reading is exact

Binary STL and OBJ coordinates are finite floats, so every vertex is an exact dyadic rational. The signed volume, the
edge counts and the zero-area test are computed over those rationals with no tolerance: a sliver triangle is
degenerate only when its area is exactly zero. A mesh that is closed in the file is closed here, and one with a
one-ulp gap between two corners reports the gap as two boundary edges, as M9 states.

## 3. Point queries

The mesh is its own boundary, so the three queries of `agent-docs/point-containment-design.md` run on it with a
mesh bound of zero and an occupied-volume bound of zero. No tolerance argument exists.

| Decision | Rule |
|---|---|
| Q1 | Each query is `ErrNotSolid` unless `Report().Closed`. A nil context or mesh is `ErrDegenerate` and a point that is not finite is `ErrNotFinite`. |
| Q2 | `LocatePoint` is the exact ray-crossing parity of the point over the triangles, and a point on a triangle is `PointOnBoundary`. It is never `PointUndecided` for a closed mesh. Where the mesh crosses itself the parity decides, so a region covered twice reads as outside. An inverted mesh encloses the same region as its flip. |
| Q3 | `DistanceToPoint` is zero for a point inside or on the mesh, and otherwise the exact distance to the nearest triangle, published as the nearest float with a bound that is the rounding of that float alone. |
| Q4 | `ClosestPoint` returns the point itself when it is inside or on the mesh, and otherwise the nearest point of the nearest triangle, rounded to floats. `OffSurface` is that rounding alone. |

## 4. Verification

The tests take the output of the real writers, `export.STL` and `export.OBJ` for a block, through the real readers and
compare the volume, bounds and area against the block's. A cube checks the point queries at inside, outside,
face, edge, corner and one-ulp-off points, in both windings. A tetrahedron checks an inexact volume and its bound. Mutated
meshes check each report count: a dropped face, a flipped triangle, a third triangle on one edge, degenerate
triangles, two shells and an inverted mesh.

## 5. Not covered

- STEP, IGES and 3MF import.
- Turning a `MeshBody` into a `Body`, or taking one in a feature, a Boolean or `Verify`.
- Repair: filling holes, flipping triangles, merging near-equal vertices or removing slivers.
- Writing a `MeshBody` as STEP or 3MF.
- Concave or non-planar OBJ faces, which a fan cuts wrongly.
- Units other than millimetres.

## 6. Export

`export.MeshSTL`, `MeshOBJ`, `MeshGLTF`, `MeshPNG` and `MeshDXF` write a `*decad.MeshBody` as the matching body writer does.

```go
func MeshSTL(ctx context.Context, w io.Writer, m *decad.MeshBody) error
func MeshOBJ(ctx context.Context, w io.Writer, m *decad.MeshBody) error
func MeshGLTF(ctx context.Context, w io.Writer, m *decad.MeshBody) error
func MeshPNG(ctx context.Context, w io.Writer, m *decad.MeshBody, view View) error
func MeshDXF(ctx context.Context, w io.Writer, m *decad.MeshBody, view DrawingView) error
```

| Decision | Rule |
|---|---|
| E1 | Each function and its body writer share one core over the vertices and triangles (`export/mesh_body.go`), so the layout, number format and determinism are the body writer's. |
| E2 | The mesh is written as it is. There is no tolerance, no tessellation and no proof, so a mesh read from an STL file and written back holds the same triangles. |
| E3 | A nil context, writer or mesh body is `ErrDegenerate`. A cancelled context writes nothing. |
| E4 | A zero-area triangle has no STL normal and is `ErrDegenerate` for `MeshSTL`. `MeshPNG` shows an open or inside-out mesh with its orange back faces, and `MeshDXF` draws its boundary edges. |
| E5 | STEP and 3MF are not offered: both need a sound solid, which an imported mesh is not. |

