# Orthogonal shell design

`Shell` of a faceted solid whose every face squares to a world axis, such as a part read from STL or STEP and made a solid by
`SolidFromMesh` (`agent-docs/mesh-solid-design.md`). The construction is plain-data grid code in `internal/orthoshell/`; the
root hook is `shell_orthogonal.go`, called from `Shell` before the receiver routes of `agent-docs/modify-design.md` §8.

## 1. What it builds

The wall of a shell of thickness `t` is the solid less its cavity. For a polyhedron whose faces all square to an axis the
cavity is the solid eroded by a cube of side `2t`, which is its inward offset with the offset planes meeting in a sharp corner at
every edge, the mitered offset a CAD shell makes. The walls are then a union of grid cells, and the result is read back as a mesh
solid. No Boolean runs.

| Decision | Rule |
|---|---|
| O1 | The route applies to a faceted receiver whose held vertices all carry a zero bound, every triangle of which has its three corners on one world-axis plane exactly, in floats. Any other receiver, a receiver with a vertex bound, and an `Outward` sense take the receiver's own route and its own refusal. |
| O2 | The coordinates of the grid are the solid's vertex coordinates on each axis, plus, for each removed face, its plane moved `2t` outward. Cells of that grid are occupied by the exact signed coverage of the x-facing triangles along each column, which must be 0 or 1 everywhere, and the grid's volume must equal the mesh's own; otherwise the solid is `ErrUnsupported`. |
| O3 | The solid joined to a slab `2t` thick over each removed face (the cells between its plane and `2t` out, over its footprint, the footprint summed over the face's triangles and taken where it exceeds half a cell) gives the occupancy `S⁺`. |
| O4 | The grid is refined by every coordinate `± t`. A refined cell is in the cavity when every cell of `S⁺` that the cube of half-side `t` about the cell's middle meets is occupied, which is constant over a refined cell because of the refinement. The cavity reaches `t` past a removed face, so it opens it. |
| O5 | The walls are the cells of the solid itself that are not in the cavity. Walls with no cells are `ErrDegenerate` (the cavity takes the solid), and a cavity with no cells is `ErrDegenerate` (the walls meet: `t` is not under half the thinnest part). |
| O6 | The wall mesh is the boundary of those cells, wound outward from the material, each cell face a quad fanned in four triangles from a point inside it that varies from quad to quad. The fan keeps a probe off the cell diagonals, where the evaluator's axis rays of later Booleans cannot classify it. A boundary that touches itself along an edge is `ErrUnsupported`. |
| O7 | `solidFromMeshBody` proves the walls a solid as `SolidFromMesh` does, and the result is committed as one `OperationShell` entry with the receiver consumed. Coplanar triangles group into faces by exact plane, and every face keeps its axis plane as a retained tag, so `Fillet` and `Chamfer` build on its edges. |
| O8 | `WithNoOpenings` builds the closed hollow body: an outer shell around the cavity's inner shell, with no slab. A selector removes faces as for any shell. |
| O9 | The offsets are float sums `x ± t`, so a wall is `t` thick to the rounding of that sum, and the volume the body reads is exact for the mesh built. Coordinates of the grid within `1e-9` of the body's size are one coordinate. |

## 2. Not covered

- A receiver that is not orthogonal, one with a vertex bound (a Boolean result, whose held vertices carry the evaluator's
  bound), and an `Outward` shell.
- A grid of more than four million cells.
- Walls whose boundary touches itself along an edge.
- Rounded inside corners: the offset is mitered, as the planes of a CAD shell are.

## 3. Tests

The internal tests work boxes and an L-shaped prism by hand: the walls' volume is the solid's less the cavity's, with the cavity
eroded from the floor to the open top, and the L's cavity has its sharp corner and area `5`. The public tests shell a box
and an L-shaped mesh solid, open and closed, and read points in the wall, the miter and the cavity.
