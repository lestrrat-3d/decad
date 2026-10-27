# Subdivision surface design

`SubDMesh` refines a polygon cage into its Catmull-Clark surface and returns it as a closed or open `MeshBody`. The
subdivision is plain-data code in `internal/subd/`. It is a smooth shape drawn by moving cage vertices, as the Form
workspace of a CAD system sculpts one, in the one representation `decad` can read: a mesh.

## 1. Contract

```go
type SubDCage struct {
    Vertices []r3.Vec
    Faces    [][]int
}

func SubDMesh(ctx context.Context, cage SubDCage, levels int) (*MeshBody, error)
```

| Decision | Rule |
|---|---|
| S1 | `Faces` hold indices into `Vertices`, each face three or more distinct vertices wound counter-clockwise seen from outside. A face of fewer, a repeated or missing vertex, an edge in more than two faces, and two faces that walk an edge the same way (they disagree on which side is outside) are `ErrDegenerate`. A vertex that is not finite is `ErrNotFinite`. |
| S2 | One round makes, for each face, a face point (the mean of its vertices) and for each edge an edge point: the mean of its two ends and the two face points beside it, or its midpoint on the boundary. An n-gon becomes n quads, `[v, edge(v, next), face, edge(prev, v)]` for each vertex `v`, so the winding is kept. |
| S3 | An interior vertex of valence `n` moves to `(Q + 2R + (n − 3)P)/n`, `Q` the mean of the face points around it and `R` the mean of the midpoints of the edges around it. A boundary vertex touching two boundary edges moves to `(6P + A + B)/8`, `A` and `B` its boundary neighbours. An interior vertex in fewer than three faces, and a boundary vertex that touches other than two boundary edges, are `ErrDegenerate`. |
| S4 | `levels` is from 1 to 8, otherwise `ErrDegenerate`, and no round may make more than 1,000,000 faces. The refined quads are cut into two triangles each and read into a `MeshBody` by `NewMeshBody`, so a closed cage gives a closed mesh wound outward with a volume and an open cage an open one. |

## 2. Exactness

The mesh is the cage refined `levels` times and not the limit surface: the vertices are the Catmull-Clark points of that
round. A convex cage's mesh lies inside it, and the first round leaves the face points on the cage's faces. The tests
work the first round on a cube by hand, the 8 corners at `(±5/9)³`, the 6 face points at the face centres and the 12
edge points at `(±3/4, ±3/4, 0)` and their permutations, and compare the volume (`10/3` for the 8 mm³ cube) and the
vertex and triangle counts of three rounds with a separate calculation: the volume falls with each round.

## 3. Not covered

- Creases, a vertex or edge weight, and corner sharpness: the cage is smooth everywhere but at its boundary.
- The limit surface itself, an exact surface and a NURBS or T-spline conversion. The result is a mesh body that no
  feature or Boolean takes.
- Cage editing: move the vertices in the caller and call again.
- A boundary of vertices that touch other than two boundary edges.
