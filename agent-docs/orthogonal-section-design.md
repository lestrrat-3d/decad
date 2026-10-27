# Orthogonal section design

`Body.Section` of a faceted solid every face of which squares to a world axis is read from the grid of the
solid's coordinates (`internal/orthoshell/section.go`, hook `section_orthogonal.go`), so a mesh solid, a result of
the orthogonal Boolean (`agent-docs/orthogonal-boolean-design.md`) or of `MoveFace`, `Stretch` and the orthogonal
`Shell` sections to a sketch that enters a feature, with exact corners.

## 1. Contract

| Decision | Rule |
|---|---|
| X1 | The route applies to a faceted body with an identity placement, no vertex bound and every triangle exactly square to an axis, cut by a plane whose `U`, `V` and normal are exactly world axes (either sign, any pairing). Any other body or plane takes the other routes, and the route declines to them on every refusal below. |
| X2 | The plane lies strictly inside one layer of the grid along its normal, farther than the grid tolerance from every grid line, so the cut reads the same just either side of it. A plane on a grid line, outside the solid, or through a layer with no cell declines. |
| X3 | The cut is the boundary of the union of that layer's occupied cells, traced with collinear corners merged: every corner is a coordinate of the solid, an outer loop runs counter-clockwise round material and a hole clockwise. Cells that meet only at a corner decline. |
| X4 | The loops are stated in the plane's frame as `u = ±(p − origin)` along `U` and likewise `v`; a corner whose frame coordinate does not give back the world coordinate exactly (`origin + u = p`) declines, and a frame that flips handedness reverses its loops to keep their sense. |
| X5 | The cut is a single region: one outer loop and its holes. Several outer loops decline, to the history route of `agent-docs/section-design.md` and its refusal. The sketch is rebuilt by `sectionclip.ReconstructExactSection`, so its profile measures exactly. |
| X6 | The tests section an L-shaped union in plan, with the plane axes swapped and with one reversed, and in two vertical cuts, read the exact areas, extrude a section to the matching volume, section a plate with a bored hole to a profile with a hole, and refuse a plane on a face or inner line, one past the solid, one oblique, and a cut that leaves two regions. A reversed outer-loop test or a skipped handedness flip fails them. |

## 2. Not covered

- A placed body, a body with a vertex bound, and a plane that is not square to an axis.
- A cut of several regions, which a single returned profile cannot hold.
