# Orthogonal section design

`Body.Section` of a faceted solid every face of which squares to a world axis is read from the grid of the
solid's coordinates (`internal/orthoshell/section.go`, hook `section_orthogonal.go`), so a mesh solid, a result of
the orthogonal Boolean (`agent-docs/orthogonal-boolean-design.md`) or of `MoveFace`, `Stretch` and the orthogonal
`Shell` sections to a sketch that enters a feature, with exact corners.

## 1. Contract

| Decision | Rule |
|---|---|
| X1 | The route applies to a faceted body with no vertex bound and every triangle exactly square to an axis, cut by a plane whose `U`, `V` and normal are exactly world axes (either sign, any pairing). Any other body or plane takes the other routes, and the route declines to them on every refusal below. |
| X2 | The plane lies strictly inside one layer of the grid along its normal, farther than the grid tolerance from every grid line, so the cut reads the same just either side of it. A plane on a grid line, outside the solid, or through a layer with no cell declines. |
| X3 | The cut is the boundary of the union of that layer's occupied cells, traced with collinear corners merged: every corner is a coordinate of the solid, an outer loop runs counter-clockwise round material and a hole clockwise. Cells that meet only at a corner decline. |
| X4 | The loops are stated in the plane's frame as `u = ±(p − origin)` along `U` and likewise `v`; a corner whose frame coordinate does not give back the world coordinate exactly (`origin + u = p`) declines, and a frame that flips handedness reverses its loops to keep their sense. |
| X5 | Every region of the cut is a profile of the one returned sketch, an outer loop and the holes inside it, in the order the outer loops are traced: a hole belongs to the smallest outer loop that contains its first corner, so a post standing in a frame's hollow is a profile of its own beside the frame's wall. The sketch is rebuilt by `sectionclip.ReconstructExactSections`, which checks that Sketch arranges every record as exactly its region (the empty space a hole encloses, which Sketch also arranges, is left out), so each profile measures exactly. |
| X6 | The tests section an L-shaped union in plan, with the plane axes swapped and with one reversed, and in two vertical cuts, read the exact areas, extrude a section to the matching volume, section a plate with a bored hole to a profile with a hole, list the two blocks a slot leaves and a post inside a frame beside the frame, and refuse a plane on a face or inner line, one past the solid and one oblique. A reversed outer-loop test, a skipped handedness flip or a hole given to the wrong outer loop fails them. |

## 2. Not covered

- A body with a vertex bound, such as one placed by a motion whose vertex images are not floats, and a plane that is not square to an axis.
