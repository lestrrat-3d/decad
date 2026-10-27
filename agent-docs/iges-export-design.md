# IGES export design

`export.IGES` writes a body as the planar-faced manifold solid B-reps of an IGES 5.3 file, the form `ReadIGES` reads
(`agent-docs/iges-import-design.md`). It is the faceted arm of the STEP writer (`agent-docs/step-export-design.md` §9)
in IGES records: one planar face for each triangle of the body's verified tessellation.

## 1. Contract

```go
func IGES(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value) error
```

| Decision | Rule |
|---|---|
| E1 | The body is a valid solid, and its tessellation at `tol` is boundary verified, with every edge used by two triangles with opposite senses and one connected facet set for each of the body's shells; the checks and the grouping of shells into lumps are STEP's (`checkMesh`, `meshSolids`), reworded for IGES. A nil context, writer or body is `ErrDegenerate`, an invalid solid `ErrNotSolid`, and a mesh the checks refuse `ErrUnsupported`. A failure writes nothing. |
| E2 | The file is the five sections of IGES 5.3 in records of 80 columns: a start record, the global section (the delimiters `,` and `;`, millimetres by unit flag 2, a model space scale of 1, a resolution of a micron, version 11), the directory entries of two records each, the parameter data of 64 columns with the entity's directory number in columns 65 to 72, and the terminate record that counts each section. A parameter is never split between two records. |
| E3 | A real is the shortest text that reads back as the same float, with a decimal point and an upper case `E` exponent, so a vertex reads back to the last digit. |
| E4 | The entities are one vertex list (502) of every vertex, one line (110) for each distinct edge and one edge list (504) of them, each from the lower vertex to the higher; for each triangle a plane (108, shared by triangles of one exact plane), a loop (508) of its three edges, each with the sense in which the triangle runs it, and a face (510) on the plane with that one outer loop; for each connected facet set a shell (514) of its faces; and for each lump a manifold solid B-rep (186) of its outer shell and its void shells. Every entity below the solids is marked physically dependent. |
| E5 | The triangles are the tessellation's own, wound outward from the material, so a face's loop agrees with the plane's normal for an outer shell and runs into the cavity for a void. `ReadIGES` winds each shell afresh from its edges and does not read the flags, so a reader that does gets the senses the body states. |
| E6 | The tests read each file back through `ReadIGES`, which was written against the format and not against this writer: a block (the same vertices to the last digit, the same triangle count, volume 1200 and the bounds), a cylinder (a closed mesh of the tessellation's volume), a cube with a cavity (a void shell, volume 7000), a body of two lumps (a solid for each, volume 2400); the records (80 columns, numbered sections in order, the terminate counts and 52 entities for a block); and the refusals. Reversed edge senses, a rounded real and a dropped void each fail them. |

## 2. Not covered

- Analytic surfaces and curves: a cylinder is its facets, as in STEP's faceted arm, and no NURBS surface is written.
- Assemblies, colours, layers, names and the other entity types, and a transformation matrix.
- A reader other than `ReadIGES`: the file follows the specification's layout and has not been opened by another CAD system.
