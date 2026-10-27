# Orthogonal Boolean design

`Union`, `Cut` and `Intersect` of two faceted solids every face of which squares to a world axis are computed on a
grid of the operands' coordinates, so they are exact and take any contact: a shared face, a coplanar overlap, a shared
edge. The grid code is that of the orthogonal shell (`agent-docs/orthogonal-shell-design.md`) in
`internal/orthoshell/combine.go`; the root hook is `boolean_orthogonal.go`, called by `performBoolean` before the
general dispatch.

## 1. Contract

| Decision | Rule |
|---|---|
| B1 | The route applies when both operands are faceted bodies, every held vertex of a zero bound and every triangle exactly square to an axis, such as mesh solids (`SolidFromMesh`, `agent-docs/mesh-solid-design.md`), the results of `MoveFace`, `Stretch` and an orthogonal `Shell`, and the results of this route. Any other pair takes the general route. |
| B2 | Both solids are read onto one grid: the lines of each axis are the coordinates of the vertices of either operand that lie on two planes, an edge or a corner. A vertex on one plane is a point a mesh adds to fan a face and gives no line. The grid tolerance is a billionth of the larger of 1 mm and the pair's extent, and two distinct vertex coordinates within it, in one operand or between the two, are refused rather than merged, so no face of the result stands anywhere but on a coordinate it was given. |
| B3 | A cell is kept by the operation: `Union` where either operand occupies it, `Cut` where the first does and the second does not, `Intersect` where both do. The boundary of the kept cells is meshed as the shell meshes its walls, and `solidFromMeshBody` proves it a solid. The corners of every face of the result are coordinates of the operands, so the result is the exact common solid and carries no mesh bound. |
| B4 | The route only ever declines: a result with no cell, an unequal grid, a grid past `MaxCells`, kept cells that meet only along an edge (a non-manifold mesh) or a result the solid proof refuses leave the pair to the general route and its own error, so a disjoint `Intersect` is still `ErrBooleanFailed`. |
| B5 | The result replaces both operands as one `Union`, `Cut` or `Intersect` operation with the same input roles. |
| B6 | The tests join two blocks that touch at a face into one bar of six faces, take the union, common part and difference of a cube and a slab sharing a plane, cut a pocket whose floor and walls lie in the plate's planes, chain unions into a cut, and read volumes written out by hand with a zero bound, bounds, face counts and points. A union that keeps `||` as `&&` fails them. |
| B7 | Beside a faceted operand that meets B1, the other may be any solid whose tessellation (at 0.1 mm, verified) carries no displacement bound and has every triangle square to an axis, such as an extruded rectilinear profile: its mesh is the body exactly and is read as an operand. Two operands that are not faceted never take this route and keep their certified ones. |
| B8 | A result of this route is a mesh solid whose axis faces keep exact planes, so `Shell` (`agent-docs/orthogonal-shell-design.md`) and `Fillet` and `Chamfer` of its straight edges (`agent-docs/retained-edge-blend-design.md`) take it, a convex and a reflex edge alike. The blend of a straight edge accepts the fan points the grid meshing leaves in a face, since they lie exactly on the face's plane and the face is built again from its loops. The tests fillet and chamfer the outer corner and fillet the reflex corner of an L-shaped union and read volumes from the section added or removed, and shell it open at the top. |
| B9 | A placed operand qualifies when its placement left every held vertex at a zero bound: a vertex whose exact image under the stored basis and translation is itself a float pays no rounding (`facetproof.PlaceMesh`), as a whole-number translation of whole-number coordinates does, and the held vertices are world coordinates, so the route reads them as they are. A rotation by an angle that leaves float images pays its rounding and so leaves the route. |
| B10 | `QuarterTurn(ctx, b, axis, about, turns)` turns a faceted solid by whole quarter turns about a world axis through `about` by exchanging and negating coordinates, which rotates exactly where a 90 degree `r3.Rotation` leaves vertex images that are not floats. Every relative coordinate `p − about` and every sum `about + turned` must be held exactly, proved by the error-free sum (a nonzero rounding error is `ErrUnsupported`), so the turned solid keeps a zero vertex bound and stays on the routes above. The tests turn a block about each axis, both ways, five turns, and about a point, read bounds and volumes, join a turned bar to another exactly, and refuse whole turns, a skew axis, a centre the sums round on, a non-finite centre, a non-faceted solid and nil arguments. A sum taken as exact when it rounds fails them. |

## 2. Not covered

- A mixed pair whose second operand tessellates with a bound or a slanted triangle.
- A pair with a slanted or curved face, or an operand with a vertex bound (the result of a general mesh Boolean, or a body placed by a motion whose vertex images are not floats).
- Many-operand `Union` as one step: it folds two at a time through this route when its pairs qualify.
