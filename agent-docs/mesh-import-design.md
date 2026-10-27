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
func ReadThreeMF(ctx context.Context, r io.Reader, opts ...MeshImportOption) (*MeshBody, error)
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

- STEP and IGES import, and 3MF materials, colours, slices and beam lattices.
- Turning a `MeshBody` into a `Body`, or taking one in a feature, a Boolean or `Verify`.
- Reduction that keeps the surface within a stated distance of the source, and remeshing.
- Repair beyond §7: self-intersections, edges three triangles use, bent holes closed to their true surface, and slivers.
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

## 7. Repair

`RepairMesh` mends a `MeshBody` into a new one. The work is plain-data code in `internal/meshrepair/`.

```go
func RepairMesh(ctx context.Context, m *MeshBody, opts ...MeshRepairOption) (*MeshBody, MeshRepairReport, error)
func WithMeshWeld(distance units.Value) MeshRepairOption
func WithMeshOrient() MeshRepairOption
func WithMeshFillHoles() MeshRepairOption
```

| Decision | Rule |
|---|---|
| M1 | `m` is unchanged. The result is built with `NewMeshBody`, so its `Report` says what the repair achieved and whether the mesh closed. A repair never claims a mesh sound. |
| M2 | Triangles that name a vertex twice, have zero area or repeat an earlier triangle's vertex set are always dropped. Vertices no triangle uses are dropped with them. |
| M3 | The steps run in this order: weld, drop, orient, fill. |
| M4 | `WithMeshWeld` merges each vertex into the first earlier vertex within the distance, found by a grid of that cell size. Zero merges only equal coordinates. The merge is greedy and by input order, so it is deterministic and not transitive. |
| M5 | `WithMeshOrient` flips triangles so that every edge two triangles share runs both ways, shell by shell over edges that exactly two triangles use, and flips a closed shell whose volume is negative. A shell holds the triangles joined by such edges, and an edge that three or more triangles use joins nothing. |
| M6 | `WithMeshFillHoles` closes each boundary loop in which every vertex has one way onward: one triangle for a loop of three, otherwise a fan from a new vertex at the loop's centroid. The fan is flat only when the loop is, so a bent loop closes to a surface that need not match the source. A boundary that branches stays open and counts once in `SkippedLoops`. |
| M7 | A weld distance that is negative is `ErrNegativeMagnitude`, not a length `ErrUnitKind`. An option given twice, a nil option, a nil context and a nil mesh are `ErrDegenerate`, as is a result with no triangle. |
| M8 | The float arithmetic here decides only which triangles to flip and drop. It claims nothing about the result: every reading of the new mesh body is exact over its own triangles, as §2 says. |

## 8. 3MF

`ReadThreeMF` reads a 3MF Core package into a `MeshBody`. `internal/meshparse/threemf.go` unpacks and parses it.

| Decision | Rule |
|---|---|
| T1 | The model part is the target of the package's `.../3dmodel` relationship in `_rels/.rels`, or `3D/3dmodel.model` when there is none. A package that is not a zip, or has no model part, is `ErrMalformedMesh`. |
| T2 | Every `<item>` of the `<build>` is read, in order, and the meshes are welded together as the other readers weld. A build with no item or no triangle is `ErrMalformedMesh`. Items of type `support` and `solidsupport` are skipped; every other type is read. |
| T3 | An object holds a `<mesh>`, `<components>`, or both. A component places another object by `objectid` with its own optional `transform`. Components nest to 16 deep, and a deeper or looping set is `ErrMalformedMesh`. |
| T4 | The model's `unit` scales every coordinate to millimetres: `micron`, `millimeter` (the default), `centimeter`, `meter`, `inch` and `foot`. Another unit is `ErrMalformedMesh`. |
| T5 | A `transform` is twelve numbers: the rows of a 3 x 3 matrix then the translation, for a point as a row vector. A component's transform is applied first, then each enclosing component's and then the item's. A transform that is not rigid is `ErrUnsupported`, because `r3.Transform` holds rotations, translations and reflections only. |
| T6 | A reflection reverses the winding of the triangles it moves, so a mirrored body stays outward. |
| T7 | A triangle index outside its object's vertices, a vertex that is not finite, a malformed transform and XML that does not parse are `ErrMalformedMesh`. A part that expands past the byte limit, as well as input past either limit, is `ErrMeshTooLarge`. |
| T8 | Materials, colours, textures, slices and beam lattices are ignored. |

## 9. Reduction

`ReduceMesh` thins a `MeshBody` by vertex clustering into a new one. The work is plain-data code in `internal/meshreduce/`.

```go
func ReduceMesh(ctx context.Context, m *MeshBody, cell units.Value) (*MeshBody, MeshReduceReport, error)
```

| Decision | Rule |
|---|---|
| R1 | Space is cut into cubic cells of side `cell`, aligned to the origin. The vertices of a cell become one vertex at their mean, and each triangle is rebuilt over the replacements. |
| R2 | A triangle whose corners share a cell, or that repeats another triangle's three cells in any order, is dropped. The first of a repeated set is kept. Vertices no kept triangle uses are dropped, and the rest are numbered in the order the kept triangles first use them. |
| R3 | `MaxVertexMove` is the greatest distance from an original vertex to its replacement. It bounds how far the vertices moved, never how far the surface moved: nothing here claims the result is close to the source, closed, manifold or free of self-intersection. `MeshBody.Report` on the result says what it is. |
| R4 | The mean lies inside its cell, so a convex corner is pulled in. A cell smaller than the vertex spacing changes nothing, and one larger than the mesh drops every triangle. |
| R5 | A cell that is not positive is `ErrDegenerate` for zero, `ErrNegativeMagnitude` for a negative and `ErrUnitKind` for a wrong kind. A reduction that leaves no triangle, a nil context and a nil mesh are `ErrDegenerate`. |

## 10. Scaling

`ScaleMesh` stretches a `MeshBody` along the world axes into a new one.

```go
func ScaleMesh(ctx context.Context, m *MeshBody, anchor r3.Vec, x, y, z units.Value) (*MeshBody, error)
```

| Decision | Rule |
|---|---|
| Z1 | Each vertex `p` moves to `anchor + (p − anchor)` with its components multiplied by `x`, `y` and `z`. The triangles keep their vertex indices, so the result has the source's triangle and vertex counts. |
| Z2 | The volume of a closed mesh is multiplied by `\|x·y·z\|`. Areas follow no one factor, and a uniform stretch keeps every angle while any other does not. |
| Z3 | A negative factor mirrors across the plane through the anchor square to its axis. When the product of the factors is negative the triangles' second and third corners are swapped, so a mesh wound outward stays wound outward. |
| Z4 | A factor must be dimensionless (`ErrUnitKind`), finite (`ErrNotFinite`) and not zero (`ErrDegenerate`), as must the anchor be finite. A nil context or mesh is `ErrDegenerate`, and the result is read by `NewMeshBody`, so overflowing vertices are `ErrNotFinite`. |
| Z5 | The solid kernel has no per-axis scale: `ScaledAt` is uniform and `Transform` is rigid. Only a mesh stretches. `ScaleBodyMesh(ctx, b, anchor, x, y, z, tol, opts...)` tessellates a solid body at chord tolerance `tol` and stretches that mesh as Z1 to Z4 say, returning a `MeshBody`: the stretched solid as a mesh, not a `Body`. A flat-faced body keeps its volume times `\|x·y·z\|` exactly, and a curved one to its tessellation's own error. |

## 11. Refinement

`RefineMesh` splits the long edges of a `MeshBody` into a new one. The work is plain-data code in `internal/meshrefine/`.

```go
func RefineMesh(ctx context.Context, m *MeshBody, maxEdge units.Value) (*MeshBody, error)
```

| Decision | Rule |
|---|---|
| N1 | In each round every edge longer than `maxEdge` is split at its midpoint, and the rounds repeat until no edge is longer. The midpoint of an edge is one vertex, shared by every triangle that uses the edge, so a closed mesh stays closed. |
| N2 | A triangle with one split edge becomes two triangles, with two split edges three, with three four. The quad left by two split edges is cut along its shorter diagonal. Every piece is wound as its triangle was, so the orientation readings of the mesh stand. |
| N3 | No vertex moves, and every new vertex lies on an edge of the mesh, so the refined mesh covers the same surface. Its area and volume are the mesh's, to the exactness of the arithmetic: midpoints of dyadic coordinates are dyadic, so a mesh of exact floats keeps its exact volume. |
| N4 | `maxEdge` is a positive length (`ErrDegenerate` for zero, `ErrNegativeMagnitude` for a negative, `ErrUnitKind` for a wrong kind). A nil context or mesh is `ErrDegenerate`. A refinement that passes five million triangles, or whose edges are still long after 40 rounds, is `ErrUnsupported`. A mesh with no long edge comes back as a copy. |
| N5 | A diagonal cut in a quad can be longer than `maxEdge` and is split in the next round, so the result has no edge longer than `maxEdge` whatever the diagonals. |

## 12. Reduction within a distance

`ReduceMeshWithin(ctx, m, distance)` is `ReduceMesh` with the cell chosen for a stated surface distance, and it proves the distance.

| Decision | Rule |
|---|---|
| W1 | Clustering moves each vertex to its cell's mean, so the result is the image of the input under the map `f` that is affine on each input triangle and takes each corner to its replacement. `f` is continuous, because neighbouring triangles share corners, and moves every input point by at most the greatest corner move `M`: a point is a convex combination of corners and its image the same combination of their replacements. So every input point is within `M` of the result wherever its triangle survived. |
| W2 | A triangle that collapsed to a segment or a point survives only as its image, which is covered when that segment is an edge of a kept triangle, or that point one of its corners. The reduction checks that for every collapsed triangle and records it as covered; an uncovered one leaves part of the input without a counterpart and the cell is not used. A triangle dropped as a repeat of another has the other's image and needs no check. |
| W3 | Every point of the result is the image of an input point, because the result is `f` of the input, so the distance holds in the other direction too, from the result to the input. Both directions give `SurfaceBound`: `M` raised by a relative `1e-12` and eight ulps of the largest coordinate, for the rounding of the distances it was measured with. |
| W4 | The cell is chosen by trying `0.9·distance/√3`, whose every vertex moves less than `distance`, and 24 cells coarser in steps of 1.25, and keeping the proven reduction with the fewest triangles. Clustering is not monotone in the cell, so a coarser distance can keep more triangles. A distance no cell proves, or one that leaves no triangle, is `ErrUnsupported`. |
| W5 | The bound says nothing of the result's topology: it may be open, non-manifold or self-intersecting even when the input is closed, as `ReduceMesh` says. A mesh that was a thin sheet can thicken or pinch within the bound. |
| W6 | The tests measure both directions by brute force from outside: the distance from a sample of each mesh (vertices, edge midpoints and triangle centroids) to the other's triangles stays within `SurfaceBound`, for a UV sphere and a finely cut block; a bound understated to `0.4·M` fails them. The internal test builds a collapsed piece no kept triangle covers and requires the refusal. |

## 13. Reduction by edge collapse

`CollapseMesh(ctx, m, triangles)` thins a closed mesh by merging edges until it has at most `triangles` triangles.

| Decision | Rule |
|---|---|
| C1 | The mesh must be closed, every directed edge occurring once and its reverse once; otherwise `ErrUnsupported`. A target under 4 is `ErrDegenerate`. |
| C2 | The shortest edge is merged first, at its midpoint, then the next, from a queue that drops an entry when either end has moved or gone since it was queued. Ties go to the lower vertex indices, so the result is deterministic. |
| C3 | An edge is merged only when the vertices its ends share are exactly the two opposite it (the link condition), so no vertex pinches and no triangle doubles and the surface stays a closed manifold. |
| C4 | A merge is refused when a triangle around the edge would turn so that the cosine of its normals falls to 0.2 or below, about 78 degrees, so a collapse never folds a neighbour over. |
| C5 | At least four triangles remain. Each merge removes two, so the result has `triangles` or one fewer, or more when no remaining edge passes C3 and C4. |
| C6 | The report's `MergedVertices` counts the merges, `DroppedTriangles` is twice that and `MaxVertexMove` is the greatest distance from an input vertex to the vertex its merges ended it at. `SurfaceBound` is the zero Value: no distance to the input surface is proven, and the result can pass through itself where a coarse mesh pinches a thin feature. |
| C7 | The tests thin a sphere to three targets and read the closed manifold, the volume loss and the vertices staying within the sphere, read a mesh already coarse enough unchanged, thin bipyramids (a regular octahedron, a tall and a flat one and two stars with concave corners) to every target and require a closed, consistently wound, positively enclosing result, and check the refusals. Dropping the link condition fails the bipyramids, and loosening C4 to any turn short of a reversal fails them too. |

## 14. Remeshing to even triangles

`RemeshMesh(ctx, m, edge)` rebuilds a closed mesh from near-equilateral triangles of about `edge`.

| Decision | Rule |
|---|---|
| R1 | The mesh must be closed, consistently wound and manifold (C1), otherwise `ErrUnsupported`; `edge` is a positive length, and an `edge` that would make over a million triangles from the mesh's area is `ErrUnsupported`. |
| R2 | The algorithm is Botsch and Kobbelt's incremental isotropic remeshing, eight passes of: split the edges over 4/3 of `edge` at their midpoints, merge the edges under 4/5 of it unless the midpoint would sit over 4/3 of `edge` from a neighbour, flip an edge when that lowers the sum of the squares of the four vertices' distance from six neighbours, and move each vertex half way to its neighbours' centroid within the plane of its area-weighted normal. |
| R3 | A merge keeps C3 and C4. A flip needs the two opposite vertices not already joined and neither new triangle to turn past C4's cosine from the one it replaces. A smoothing move needs every triangle around the vertex to keep C4. So the result stays closed, manifold and wound as it was. |
| R4 | The vertices are not projected back onto the input, so a convex surface shrinks a little and a vertex can leave the input surface by a hair on either side; no distance to the input is proven, and a feature thinner than `edge` can pinch. |
| R5 | The tests remesh the sphere of the reduction tests (2,208 triangles of uneven shape) at 2 mm and require a closed manifold, a mean edge within a quarter of 2 mm, the spread of edge lengths under a quarter of the mean and under half the input's, over 60 per cent of vertices with six neighbours, a mean square valence distance under 0.3, the volume within 6 per cent and no vertex more than 0.05 mm outside the sphere; coarsen at 6 mm and refine at 0.8 mm; flip edges on a 24-sided bipyramid to a greatest valence of 9; and check the refusals. Dropping the smoothing, the splits, the merges or the flips fails a test each (the flips by the valence distance, 0.17 with them against 0.55 without). |

