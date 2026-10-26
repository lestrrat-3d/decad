# STEP import design

`ReadSTEP` reads the planar-faced solids of a STEP file into a `MeshBody`. It is the import counterpart of the faceted and
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
| I4 | A face on any other surface, or an edge that is not a `LINE`, is `ErrUnsupported`, naming the entity type. decad reads no curved boundary representation. |
| I5 | The face's outward normal is its plane's axis, reversed where `same_sense` is `.F.`. The outer loop is wound counter-clockwise about it, the holes clockwise, whatever order the file lists them in, and the triangles are wound counter-clockwise about it. A void shell's flag `.F.` on its `ORIENTED_CLOSED_SHELL` turns every face of that shell over. |
| I6 | A polygon with holes is cut by ear clipping after the holes are bridged into the outline, the rightmost hole first. A face with no area, a hole outside its outline or one that crosses it, and a boundary with no ear are `ErrMalformedMesh`. |
| I7 | Lengths are in millimetres. The file's unit is the length unit its `GLOBAL_UNIT_ASSIGNED_CONTEXT` lists: an `SI_UNIT` with a prefix of metre, or a `CONVERSION_BASED_UNIT` named `INCH` or `FOOT`. A file that names none is read as millimetres. |
| I8 | Vertices at identical coordinates are welded, as the other readers weld. Nothing else is repaired, so `MeshBody.Report` says what the faces made. |
| I9 | The limits of `MeshImportOption` apply: bytes before the parse, and triangles as the faces are cut. A canceled context returns its error. |

## 2. What the read does and does not claim

A solid read from a file that decad wrote reads as it was written: the analytic arm's plane faces and the faceted arm's
triangles each give their polygon, so a block, a prism, a cavity, and a cone's tessellation round-trip to their own
volumes. The mesh is the file's polygons and nothing else: its volume is exact over those triangles (`MeshBody`'s own
reading), not a claim about the part the file's author meant.

## 3. Not covered

- Cylindrical, conical, spherical, toroidal and free-form faces, and arcs, ellipses and splines as edges.
- IGES, STEP assemblies and product structure, placements of shapes, colours, layers, PMI and units other than length.
- Turning the mesh into a `Body`, which no feature or Boolean takes.
