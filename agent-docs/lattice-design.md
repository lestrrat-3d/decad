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
- A lattice that follows a body's surface, and an infill whose lattice mesh is not one `SolidFromMesh` takes (a closed mesh of
  at most 65,528 triangles).
- A mesh that is smooth or merged to a stated surface distance: the mesh is the grid's.

## 4. Other surfaces

`SchwarzPLattice`, `DiamondLattice`, `NeoviusLattice` and `IWPLattice` take the same `LatticeSpec` and cut the same way: only the field `g` changes, so G1
to G6 hold with that field and its range.

| Decision | Rule |
|---|---|
| K1 | Schwarz P is `g = cos kx + cos ky + cos kz`, from −3 to 3. `Level` lies strictly between 0 and 3. |
| K2 | The diamond is `g = sin kx·sin ky·sin kz + sin kx·cos ky·cos kz + cos kx·sin ky·cos kz + cos kx·cos ky·sin kz`, from −√2 to √2 (the largest of `|g|`, at `kx = ky = kz = π/4`). `Level` lies strictly between 0 and √2. |
| K5 | The Neovius surface is `g = 3(cos kx + cos ky + cos kz) + 4 cos kx·cos ky·cos kz`, from −13 to 13 (at `k = 0` and `k = π`). `Level` lies strictly between 0 and 13. |
| K6 | The IWP surface is `g = 2(cos kx·cos ky + cos ky·cos kz + cos kz·cos kx) − (cos 2kx + cos 2ky + cos 2kz)`, from −5 (at `kx = π`, `ky = kz = 0`) to 3. Its largest `|g|` is 5, so `Level` lies strictly between 0 and 5. |
| K3 | A level at or above the kind's largest `|g|`, or at or below zero, is `ErrDegenerate`; the other gates are G6's. `internal/lattice.Surface` takes the kind and `Gyroid` is its gyroid case. |
| K4 | The tests write each field out from the definition, read the solid fraction by sampling it on a grid of their own as §2 does, and require the closed mesh, the capped bounds, the convergence with `Resolution` and the growth with `Level` of every kind, and that the kinds' volumes differ at one level. A thin wall (a level of 0.15 of the range, one cell sampled 48 times to a cell) follows the Neovius and IWP fields within 2.5 per cent, so a wrong coefficient fails it. `Infill` takes every kind (`NeoviusInfill`, `IWPInfill`). |

## 5. Infill

`Infill(ctx, b, InfillSpec{Kind, Cell, Level, Resolution})` fills a solid with a lattice.

| Decision | Rule |
|---|---|
| I1 | The lattice is built over `b`'s bounding box grown by a quarter cell on every side, so the box's faces are clear of the body's and no face of the lattice lies in one of the body's (the mesh Boolean refuses coplanar overlap). The field is that of §1 and §4 measured from the world origin, so the same spec fills two bodies in step. |
| I2 | The lattice mesh is made a solid by `SolidFromMesh` and intersected with `b` by `Intersect`; both are consumed and the result replaces them in `b`'s document. The result is the lattice material inside the body: the body's skin is not kept, and a body that needs one is shelled and the infill united into its cavity. |
| I3 | The result is the Boolean's: a faceted body whose volume reads inside the bound the Boolean proves, and whose own refusals (an ambiguous parity ray at some resolutions, a lattice past 65,528 triangles) surface unchanged, the latter as `ErrUnsupported`. An unknown kind, a non-positive cell, a non-length cell or a level outside the kind's range are the lattice builders' errors. |
| I4 | The tests fill a 10 x 10 x 12 mm block with the gyroid and read the result against the field written out from its definition: points well inside the wall are inside and points well clear are outside, to 99% (the lattice mesh at 8 samples to a cell misplaces a few points at its saddles even unfilled), the volume agrees with the field's sampled fraction to a tenth, the bounds stay inside the block, and the block and the lattice are consumed. Returning the unfilled lattice fails them. |

