# Orthogonal offset design

`Offset` of a faceted solid every face of which squares to a world axis moves every face along its outward normal by a
stated distance, neighbouring faces extended or trimmed to meet, and returns the result as a second solid. It is the
grid code of the orthogonal shell (`agent-docs/orthogonal-shell-design.md`) in `internal/orthoshell/offset.go`; the
root entry is `offsetSolid` in `offset.go`.

## 1. Contract

| Decision | Rule |
|---|---|
| F1 | The receiver is a live faceted solid whose held vertices carry a zero bound and every triangle of which squares to an axis exactly (a mesh solid, a result of the orthogonal Boolean, `MoveFace`, `Stretch` or an orthogonal `Shell`). Any other solid keeps `ErrUnsupported` ("Offset requires a sheet body ..."); a sheet takes the route of `agent-docs/surface-design.md` §17. |
| F2 | `distance` is a positive length, as for a sheet. `OffsetPositive` (the default) moves each face outward, growing the solid, and `OffsetNegative` inward, shrinking it. A zero distance is `ErrDegenerate`, a wrong kind `ErrUnitKind` and a negative value `ErrNegativeMagnitude`. |
| F3 | The result is the solid dilated (or eroded) by the cube of half-side `distance`, computed on the grid of the solid's coordinates and those coordinates `± distance`. For faces that all square to an axis that is exactly the mitred face offset: a convex edge stays sharp and a reflex edge fills, a slot narrower than twice the distance closes, and a part thinner than twice it is gone. The new coordinates are the float sums `x ± distance`, to the rounding of the sum, as for `Shell`. |
| F4 | An erosion that leaves no cell is `ErrDegenerate`. A result the solid proof refuses (kept cells that meet only along an edge) is `ErrUnsupported`. A refusal registers nothing. |
| F5 | The receiver stays live: the result is committed as one `OperationOffset` with the receiver referenced and not consumed, as for a sheet. |
| F6 | The tests grow and shrink a block and read bounds and volumes, offset an L-shaped union both ways (the sharp reflex corner checked by point), close a 1 mm slot in a plate by growing it, refuse to shrink a 2 mm sheet by 1 mm, and refuse a non-faceted solid, a zero distance, a wrong kind and a negative value. Dropping the solid's own cells from a growth fails them. |

## 2. Not covered

- A solid with a slanted or curved face, a rounded offset (a radius at convex edges), and a vertex bound.
- An offset of selected faces, which `MoveFace` does one face at a time.
