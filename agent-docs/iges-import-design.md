# IGES import design

`ReadIGES` reads the planar-faced solids of an IGES 5.3 file into a `MeshBody`, as `ReadSTEP` does for STEP
(`agent-docs/step-import-design.md`). The record parser is `internal/igesread/`, the B-rep reader
`internal/igesbrep/`, and the triangulation of a planar face with holes is `internal/polytri/`, shared with STEP.

## 1. Contract

```go
func ReadIGES(ctx context.Context, r io.Reader, opts ...MeshImportOption) (*MeshBody, error)
```

| Decision | Rule |
|---|---|
| I1 | The file is the fixed-format records of IGES 5.3: 80 columns, the section letter in column 73 (`S`, `G`, `D`, `P`, `T`), a short line padded, CRLF read as a line end. The Global section fixes the parameter and record delimiters (`1H,,1H;`, or any other pair, set off by the default comma or by the new delimiter), the model space scale (parameter 13) and the unit (parameters 14), whose flag gives the length of the unit in millimetres for inch, millimetre, foot, mile, metre, kilometre, mil, micron, centimetre and microinch. Any other flag, or a missing terminate record, is `ErrMalformedMesh`. |
| I2 | An entity is two directory records, the entity type and form number from them, and its parameter records, gathered by the directory sequence number in columns 65 to 72 and split at the delimiters with Hollerith strings (`nHtext`) taken whole and the Fortran `D` exponent read as `E`. A parameter list whose first number is not the directory's entity type is `ErrMalformedMesh`. |
| I3 | The solids are the manifold solid B-rep entities (186). Each names a shell (514), its orientation flag and any void shells; a shell lists faces (510), a face its plane (108 or 190) and its loops (508), a loop its edges as entries of an edge list (504) with a sense, and an edge list its straight lines (110) and the end vertices as entries of a vertex list (502). A file with no such entity is `ErrMalformedMesh`. |
| I4 | Only planes with straight edges are read. A face on any other surface, an edge that is another curve, a loop entry that is a vertex, a face whose first loop is not its outer boundary, an entity placed by a transformation matrix (directory field 7) and a model space scale other than 1 are `ErrUnsupported`, so a part is never read at a place or size the file did not state. |
| I5 | A face is cut into triangles in the plane of its outer loop (the Newell normal of the loop as written), with the holes turned clockwise and the loops required to lie in that plane within a billionth of the model's size, by `polytri`. Vertices are welded by exact coordinate after scaling to millimetres. |
| I6 | A file need not agree with itself on which way a loop runs, so each shell's triangles are wound by crossing shared edges, every edge being used by exactly two triangles (otherwise `ErrMalformedMesh`), and the faces that cannot be wound consistently are not orientable (`ErrMalformedMesh`). A piece's signed volume then decides its sense: the first shell is wound outward from the material and a void's shell into its cavity. The number of closed pieces must equal the number of shells the solid names. |
| I7 | The reader claims no more than the file's faces: `SolidFromMesh` decides whether the mesh is a solid, and `RepairMesh` is the way to mend one that is not. A mesh past the triangle limit is `ErrMeshTooLarge`. |
| I8 | The tests write IGES files record by record from the format's layout, so the reader is tested against the format and not against a writer built from it: a block, a block whose faces are written both ways round, a block in each of several units, a block with a cubic void, a frame whose top and bottom faces have a hole, and refusals for another surface, an arc edge, a scaled model space, a transformation pointer, malformed text, a file with no solid, nil arguments and a triangle limit. The parser's own tests cover the delimiters, the units, Hollerith strings and the D exponent. A winding that is not propagated fails them. |

## 2. Not covered

- Surfaces and curves with no B-rep (trimmed NURBS surfaces, entities 128, 144, 126), the surface-of-revolution and cylinder entities, arcs, and any curved edge.
- Transformation matrices (124), assemblies and subfigures, and the other entity types of the file, which are ignored.
- A file this reader has not met in practice: the tests are written from the specification, and a real exporter's file that departs from it is refused (`ErrMalformedMesh` or `ErrUnsupported`) and never repaired.
- IGES export.
