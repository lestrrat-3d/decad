# Lattice design

`GyroidLattice` fills a box with a gyroid, the triply periodic wall used for lightweight print infill, and returns it as a
closed `MeshBody`. The sampling and the cutting are plain-data code in `internal/lattice/`.

## 1. Contract

```go
type LatticeSpec struct {
    Min, Max   r3.Vec
    Cell       units.Value
    Level      units.Value
    Resolution int
}

func GyroidLattice(ctx context.Context, spec LatticeSpec) (*MeshBody, error)
```

| Decision | Rule |
|---|---|
| G1 | The solid is `{ p in box : |g(p)| ≤ Level }` where `g = sin kx·cos ky + sin ky·cos kz + sin kz·cos kx` and `k = 2π / Cell`, with coordinates measured from the world origin. `g` runs from −3/2 to 3/2, so `Level` lies strictly between 0 and 3/2: 0.3 is a thin wall, 1 a heavy one, and values near 3/2 almost fill the box. |
| G2 | The lattice is capped by the box. The field cut is `F = min(Level − |g|, d)` where `d` is the distance to the nearest face of the box, so the surface runs through the box's faces and the mesh is closed. The box's bounds are the mesh's bounds. |
| G3 | `F` is sampled at `Resolution` nodes to a cell along each axis (zero takes 16), on a grid offset half a step from the box, so no node lies on a face and the first and last nodes lie outside it. |
| G4 | Each grid cube is cut into six tetrahedra about its diagonal from the corner of least coordinates to the opposite corner, the same in every cube, so neighbouring cubes agree on the faces they share. A node is inside when `F > 0`. A tetrahedron with both kinds of node adds one triangle, or two when it has two of each, on the edges between them, each vertex found by linear interpolation of `F` and shared by every triangle on that edge. |
| G5 | Each triangle winds so that its normal points from the inside nodes of its tetrahedron toward the outside ones. The mesh is then closed with consistent winding, wound outward, and `MeshBody.Volume` reads. |
| G6 | `Cell` is a positive length (`ErrDegenerate` for zero, `ErrNegativeMagnitude` for a negative, `ErrUnitKind` for a wrong kind). `Level` is dimensionless (`ErrUnitKind`) and outside 0 to 3/2 `ErrDegenerate`. `Resolution` below 4 is `ErrDegenerate`. A box with no volume is `ErrDegenerate` and a non-finite corner `ErrNotFinite`. A lattice that needs over 8 million samples or 2 million triangles is `ErrDegenerate`. A canceled context returns its error. |

## 2. Exactness

The result is a triangulation of the field's zero set and not the lattice itself, so its volume is a reading of the
mesh, as every `MeshBody` reading is. The linear interpolation inside each tetrahedron pulls a thin wall in: the error
falls as the grid refines, from about 6 percent at 16 samples to a cell on a wall a tenth of a cell thick to a third of a percent at 32. The tests compute the
solid fraction by sampling `g` directly on a grid of its own and compare the volumes, and require the error to fall as
`Resolution` grows.

## 3. Not covered

- Strut lattices, a lattice of a kind other than the three of §4, and a lattice that varies its thickness through space.
- Filling a body. The lattice fills a box; no feature or Boolean takes a `MeshBody`, so a lattice cannot be intersected
  with a part.
- A mesh that is smooth or merged to a stated surface distance: the mesh is the grid's.

## 4. Other surfaces

`SchwarzPLattice` and `DiamondLattice` take the same `LatticeSpec` and cut the same way: only the field `g` changes, so G1
to G6 hold with that field and its range.

| Decision | Rule |
|---|---|
| K1 | Schwarz P is `g = cos kx + cos ky + cos kz`, from −3 to 3. `Level` lies strictly between 0 and 3. |
| K2 | The diamond is `g = sin kx·sin ky·sin kz + sin kx·cos ky·cos kz + cos kx·sin ky·cos kz + cos kx·cos ky·sin kz`, from −√2 to √2 (the largest of `|g|`, at `kx = ky = kz = π/4`). `Level` lies strictly between 0 and √2. |
| K3 | A level at or above the kind's largest `|g|`, or at or below zero, is `ErrDegenerate`; the other gates are G6's. `internal/lattice.Surface` takes the kind and `Gyroid` is its gyroid case. |
| K4 | The tests write each field out from the definition, read the solid fraction by sampling it on a grid of their own as §2 does, and require the closed mesh, the capped bounds, the convergence with `Resolution` and the growth with `Level` of every kind, and that the three kinds' volumes differ at one level. |
