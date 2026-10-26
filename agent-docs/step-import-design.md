# STEP import design

`ReadSTEP` reads the planar-, cylinder- and cone-faced solids of a STEP file into a `MeshBody`. It is the import counterpart of the faceted and
planar arms of `export.STEP` (`agent-docs/step-export-design.md`) and adds no geometry kernel. Three plain-data packages
do the work: `internal/stepread` parses the file's syntax, `internal/stepbrep` reads its B-rep and `internal/polytri` cuts
a planar polygon with holes into triangles.

## 1. Contract

```go
func ReadSTEP(ctx context.Context, r io.Reader, opts ...MeshImportOption) (*MeshBody, error)
```

| Decision | Rule |
|---|---|
| I1 | The file is ISO 10303-21 (Part 21): the first `DATA;` section is read, the header and comments are skipped, and an instance may be complex, a run of `NAME(...)` groups in parentheses. A file that is not well-formed is `ErrMalformedMesh`. |
| I2 | Every `MANIFOLD_SOLID_BREP` and `BREP_WITH_VOIDS` is read. Each closed shell, and each void shell inside an `ORIENTED_CLOSED_SHELL`, gives its faces. A file with neither is `ErrMalformedMesh`. |
| I3 | A face is an `ADVANCED_FACE` or `FACE_SURFACE` on a `PLANE`, bounded by `FACE_OUTER_BOUND` and `FACE_BOUND` loops of `EDGE_LOOP` of `ORIENTED_EDGE` or of `POLY_LOOP`. A loop's vertices are those of its oriented edges' starts, taken backwards where the edge is `.F.`. |
| I4 | A face on a surface other than a plane, a cylinder or a cone, or an edge that is neither a `LINE` nor a `CIRCLE`, is `ErrUnsupported`, naming the entity type. Spheres, tori, free-form surfaces, ellipses and curves are not read. |
| I5 | The face's outward normal is its plane's axis, reversed where `same_sense` is `.F.`. The outer loop is wound counter-clockwise about it, the holes clockwise, whatever order the file lists them in, and the triangles are wound counter-clockwise about it. A void shell's flag `.F.` on its `ORIENTED_CLOSED_SHELL` turns every face of that shell over. |
| I6 | A polygon with holes is cut by ear clipping after the holes are bridged into the outline, the rightmost hole first. A face with no area, a hole outside its outline or one that crosses it, and a boundary with no ear are `ErrMalformedMesh`. |
| I7 | Lengths are in millimetres. The file's unit is the length unit its `GLOBAL_UNIT_ASSIGNED_CONTEXT` lists: an `SI_UNIT` with a prefix of metre, or a `CONVERSION_BASED_UNIT` named `INCH` or `FOOT`. A file that names none is read as millimetres. |
| I10 | A `CIRCLE` edge is cut into chords of at most `2·acos(1 − 0.01/r)` radians, a function of its radius alone: 0.01 mm is how far the polygon may lie inside its arc. The points are made once for each `EDGE_CURVE`, in its own direction, and reused by every face that walks it, so the chords of two faces on one edge are the same and the mesh closes. An arc runs counter-clockwise about its axis when the edge's `same_sense` is `.T.` and clockwise when not, and is a whole turn when its ends coincide. |
| I11 | A plane's loops may hold arcs: the polygon of their chords is cut as in I6, so a drilled plate's faces read. |
| I12 | A `CYLINDRICAL_SURFACE` or `CONICAL_SURFACE` wall is read in one of two shapes, the same for both: the ladder and the strip use the points of the arcs and their angles about the axis, and never the radius. A wall with one bound of arcs at two heights and lines along the axis, which is a seamed full wall or a partial wall, is a ladder of triangles between its two arcs, the points of each in order of angle and the two lists merged. A wall with two bounds that each go once round the axis, with no seam, is a band between its two rings, merged by angle the same way. A triangle spans neighbouring points only, so it lies on the cylinder within the chord tolerance, and on a cone within it for the two arcs' own radii. A cone whose boundary reaches the axis, at its apex, is `ErrUnsupported`. Any other bound, a wall with holes, or a boundary point between the two heights is `ErrUnsupported`. |
| I13 | A wall's triangles face along the cylinder's outward radius, or against it when the face's `same_sense` is `.F.`, as a bore's is, and are turned over with a void shell whose flag is `.F.` (I5). |
| I14 | A file that places its shapes as an assembly is `ErrUnsupported`: any `NEXT_ASSEMBLY_USAGE_OCCURRENCE`, `ITEM_DEFINED_TRANSFORMATION` or `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION`, as a simple instance or as a member of a complex one. The reader applies no placement, so reading the solids would lay every part at its own origin over the others. A file without them is read as before. |
| I8 | Vertices at identical coordinates are welded, as the other readers weld. Nothing else is repaired, so `MeshBody.Report` says what the faces made. |
| I9 | The limits of `MeshImportOption` apply: bytes before the parse, and triangles as the faces are cut. A canceled context returns its error. |

## 2. What the read does and does not claim

A solid read from a file that decad wrote reads as it was written: the analytic arm's plane faces and the faceted arm's
triangles each give their polygon, so a block, a prism, a cavity, and a cone's tessellation round-trip to their own
volumes. A curved wall reads as the polygon its arcs are cut into, inscribed in the circle: a rod's volume is the
inscribed prism's, which the tests compute from the chord rule (I10) and compare to 1e-6, and it is a little under the
round part's. The mesh is the file's polygons and nothing else: its volume is exact over those triangles (`MeshBody`'s own
reading), not a claim about the part the file's author meant.

## 3. Not covered

- Spherical, toroidal and free-form faces, a cone at its apex, cylinder and cone walls with holes or oblique boundaries, and ellipses and splines as edges.
- IGES, reading STEP assemblies (they are refused, I14) and product structure, placements of shapes, colours, layers, PMI and units other than length.
- Turning the mesh into a `Body`, which no feature or Boolean takes.
