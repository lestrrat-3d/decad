# Interchange format design

One part per file-format reader or writer that is small enough to share a document. Each part keeps its own `§` numbers; cite a `§` number as "Part name §N". STEP export has its own document, `agent-docs/step-export-design.md`.

## STEP import

`ReadSTEP` reads the planar-, cylinder- and cone-faced solids of a STEP file into a `MeshBody`. It is the import counterpart of the faceted and
planar arms of `export.STEP` (`agent-docs/step-export-design.md`) and adds no geometry kernel. Three plain-data packages
do the work: `internal/stepread` parses the file's syntax, `internal/stepbrep` reads its B-rep and `internal/polytri` cuts
a planar polygon with holes into triangles.

### 1. Contract

```go
func ReadSTEP(ctx context.Context, r io.Reader, opts ...MeshImportOption) (*MeshBody, error)
```

| Decision | Rule |
|---|---|
| I1 | The file is ISO 10303-21 (Part 21): the first `DATA;` section is read, the header and comments are skipped, and an instance may be complex, a run of `NAME(...)` groups in parentheses. A file that is not well-formed is `ErrMalformedMesh`. |
| I2 | Every `MANIFOLD_SOLID_BREP` and `BREP_WITH_VOIDS` is read. Each closed shell, and each void shell inside an `ORIENTED_CLOSED_SHELL`, gives its faces. A file with neither is `ErrMalformedMesh`. |
| I3 | A face is an `ADVANCED_FACE` or `FACE_SURFACE` on a `PLANE`, bounded by `FACE_OUTER_BOUND` and `FACE_BOUND` loops of `EDGE_LOOP` of `ORIENTED_EDGE` or of `POLY_LOOP`. A loop's vertices are those of its oriented edges' starts, taken backwards where the edge is `.F.`. |
| I4 | A face on a surface other than a plane, a cylinder, a cone or a sphere cap (I15), or an edge that is neither a `LINE` nor a `CIRCLE`, is `ErrUnsupported`, naming the entity type. Tori, free-form surfaces, ellipses and curves are not read. |
| I5 | The face's outward normal is its plane's axis, reversed where `same_sense` is `.F.`. The outer loop is wound counter-clockwise about it, the holes clockwise, whatever order the file lists them in, and the triangles are wound counter-clockwise about it. A void shell's flag `.F.` on its `ORIENTED_CLOSED_SHELL` turns every face of that shell over. |
| I6 | A polygon with holes is cut by ear clipping after the holes are bridged into the outline, the rightmost hole first. A face with no area, a hole outside its outline or one that crosses it, and a boundary with no ear are `ErrMalformedMesh`. |
| I7 | Lengths are in millimetres. The file's unit is the length unit its `GLOBAL_UNIT_ASSIGNED_CONTEXT` lists: an `SI_UNIT` with a prefix of metre, or a `CONVERSION_BASED_UNIT` named `INCH` or `FOOT`. A file that names none is read as millimetres. |
| I10 | A `CIRCLE` edge is cut into chords of at most `2·acos(1 − 0.01/r)` radians, a function of its radius alone: 0.01 mm is how far the polygon may lie inside its arc. The points are made once for each `EDGE_CURVE`, in its own direction, and reused by every face that walks it, so the chords of two faces on one edge are the same and the mesh closes. An arc runs counter-clockwise about its axis when the edge's `same_sense` is `.T.` and clockwise when not, and is a whole turn when its ends coincide. |
| I11 | A plane's loops may hold arcs: the polygon of their chords is cut as in I6, so a drilled plate's faces read. |
| I12 | A `CYLINDRICAL_SURFACE` or `CONICAL_SURFACE` wall is read in one of two shapes, the same for both: the ladder and the strip use the points of the arcs and their angles about the axis, and never the radius. A wall with one bound of arcs at two heights and lines along the axis, which is a seamed full wall or a partial wall, is a ladder of triangles between its two arcs, the points of each in order of angle and the two lists merged. A wall with two bounds that each go once round the axis, with no seam, is a band between its two rings, merged by angle the same way. A triangle spans neighbouring points only, so it lies on the cylinder within the chord tolerance, and on a cone within it for the two arcs' own radii. A cone whose boundary reaches the axis, at its apex, is `ErrUnsupported`. Any other bound, a wall with holes, or a boundary point between the two heights is `ErrUnsupported`. |
| I13 | A wall's triangles face along the cylinder's outward radius, or against it when the face's `same_sense` is `.F.`, as a bore's is, and are turned over with a void shell whose flag is `.F.` (I5). |
| I14 | A file that places its shapes as an assembly is `ErrUnsupported`: any `NEXT_ASSEMBLY_USAGE_OCCURRENCE`, `ITEM_DEFINED_TRANSFORMATION` or `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION`, as a simple instance or as a member of a complex one. The reader applies no placement, so reading the solids would lay every part at its own origin over the others. A file without them is read as before. |
| I15 | A `SPHERICAL_SURFACE` face is read when it is a cap: one bound whose points lie on the sphere and on one circle of latitude about the axis of the ring's walk (Newell's normal). The cap is the part of the sphere to the left of the walk seen from outside, round that axis, and round its opposite where `same_sense` is `.F.`, whose normal points to the centre. It is cut into `ceil(θ·m/2π)` rings of latitude, `θ` the cap's polar angle and `m` the boundary's points, each ring carrying the boundary's own azimuths, and a fan to the pole; the outermost ring is the boundary's own points, so the cap meets its neighbours along the same chords (I10). A bound with a point off the sphere is `ErrMalformedMesh`; any other bound, a zone with two bounds, a whole sphere with its seam and a boundary that is not a circle of latitude are `ErrUnsupported`. A cap's triangles face along the radius, or against it where `same_sense` is `.F.`, and turn with a void shell (I5, I13). |
| I8 | Vertices at identical coordinates are welded, as the other readers weld. Nothing else is repaired, so `MeshBody.Report` says what the faces made. |
| I9 | The limits of `MeshImportOption` apply: bytes before the parse, and triangles as the faces are cut. A canceled context returns its error. |

### 2. What the read does and does not claim

A solid read from a file that decad wrote reads as it was written: the analytic arm's plane faces and the faceted arm's
triangles each give their polygon, so a block, a prism, a cavity, and a cone's tessellation round-trip to their own
volumes. A curved wall reads as the polygon its arcs are cut into, inscribed in the circle: a rod's volume is the
inscribed prism's, which the tests compute from the chord rule (I10) and compare to 1e-6, and it is a little under the
round part's. The mesh is the file's polygons and nothing else: its volume is exact over those triangles (`MeshBody`'s own
reading), not a claim about the part the file's author meant.

### 3. Not covered

- Spherical faces other than a cap with one circular bound, toroidal and free-form faces, a cone at its apex, cylinder and cone walls with holes or oblique boundaries, and ellipses and splines as edges.
- IGES, reading STEP assemblies (they are refused, I14) and product structure, placements of shapes, colours, layers, PMI and units other than length.
- Turning the mesh into a `Body`, which no feature or Boolean takes.

## IGES import

`ReadIGES` reads the planar-faced solids of an IGES 5.3 file into a `MeshBody`, as `ReadSTEP` does for STEP
(the STEP import part). The record parser is `internal/igesread/`, the B-rep reader
`internal/igesbrep/`, and the triangulation of a planar face with holes is `internal/polytri/`, shared with STEP.

### 1. Contract

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

### 2. Not covered

- Surfaces and curves with no B-rep (trimmed NURBS surfaces, entities 128, 144, 126), the surface-of-revolution and cylinder entities, arcs, and any curved edge.
- Transformation matrices (124), assemblies and subfigures, and the other entity types of the file, which are ignored.
- A file this reader has not met in practice: the tests are written from the specification, and a real exporter's file that departs from it is refused (`ErrMalformedMesh` or `ErrUnsupported`) and never repaired.
- IGES export is `export.IGES`.

## IGES export

`export.IGES` writes a body as the planar-faced manifold solid B-reps of an IGES 5.3 file, the form `ReadIGES` reads
(the IGES import part). It is the faceted arm of the STEP writer (`agent-docs/step-export-design.md` §9)
in IGES records: one planar face for each triangle of the body's verified tessellation.

### 1. Contract

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

### 2. Not covered

- Analytic surfaces and curves: a cylinder is its facets, as in STEP's faceted arm, and no NURBS surface is written.
- Assemblies, colours, layers, names and the other entity types, and a transformation matrix.
- A reader other than `ReadIGES`: the file follows the specification's layout and has not been opened by another CAD system.

## glTF export

`export.GLTF(ctx, w, body, tol, opts...)` writes one body's tessellation as a glTF 2.0 file. It takes the chord
tolerance and `decad.TessellateOption` values of STL, OBJ and 3MF, and defaults to `VerifyNone`. A successful
`Body.Tessellate` result supplies every vertex and triangle. The writer changes no index or winding.

### File

| Decision | Rule |
|---|---|
| F1 | The file is one JSON document ending in a newline. Its fields follow the order of the Go structs in `export/gltf.go`, so equal body, tolerance and options give equal bytes. |
| F2 | `asset` is version `2.0` with generator `decad`. One scene holds one node, which names one mesh with one triangle primitive (`mode` 4). |
| F3 | The primitive has a `POSITION` attribute and 32-bit indices. It has no normals, tangents, colours, materials or textures, so a viewer shades flat from the winding. |
| F4 | One buffer holds the positions, then the indices. It is embedded as a `data:application/octet-stream;base64,` URI, so the file stands alone. |
| F5 | The two buffer views and accessors use the targets and component types glTF names: `ARRAY_BUFFER` and `FLOAT` for `VEC3` positions, `ELEMENT_ARRAY_BUFFER` and `UNSIGNED_INT` for `SCALAR` indices. The position accessor carries `min` and `max`, which glTF requires. |
| F6 | Positions are in metres, as glTF requires. Each millimetre coordinate is multiplied by `0.001` and stored as a 32-bit float, so a stored coordinate differs from the mesh by the float32 rounding of that product. Negative zero is stored as zero. |
| F7 | No axis is exchanged. decad's +Z stays +Z although glTF viewers conventionally draw +Y up. |
| F8 | Triangles keep the mesh's counter-clockwise winding seen from outside, which is glTF's front face. A sheet writes an open mesh. |

### Errors

- A nil context, writer or body is `ErrDegenerate`. A mesh with no vertex or triangle is `ErrDegenerate`.
- Tessellation errors, including a wrong unit kind for `tol` and cancellation, propagate unchanged.
- The document is built before the first write, so a failure in tessellation, encoding or cancellation leaves the
  writer untouched. An I/O error during the single write may leave a partial file.

### Not covered

- Binary `.glb` containers and external `.bin` buffers.
- Normals, materials, colours and per-face groups.
- Scaling to other units or exchanging axes. Transform the body first.
- Reading glTF.

The [glTF 2.0 specification](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html) defines the JSON schema, the
accessor and buffer view fields, and the metre unit.

## 3MF export

`export.ThreeMF(ctx, w, body, tol, opts...)` writes one body's tessellation as
a 3MF Core package. It accepts the same chord tolerance and
`decad.TessellateOption` values as STL and OBJ. It defaults to `VerifyNone`;
`WithVerification` can request the stronger tessellation audits. A successful
`Body.Tessellate` result supplies every vertex and triangle. The writer does
not change coordinates, indices, or winding.

### Package

- Write a ZIP archive with `[Content_Types].xml`, `_rels/.rels`, and
  `3D/3dmodel.model` in that order. Use Deflate, fixed metadata, and no
  timestamps from the clock. Equal body, tolerance, and options produce equal
  bytes.
- Map the model part to
  `application/vnd.ms-package.3dmanufacturing-3dmodel+xml`. The package root's
  StartPart relationship points to `/3D/3dmodel.model`.
- Write one `<model unit="millimeter">` in the 3MF Core namespace. Its
  `<resources>` holds one mesh `<object id="1">`; its `<build>` references that
  object with one `<item objectid="1"/>`.
- A `BodySolid` uses `type="model"`; a `BodySheet` uses `type="surface"`.
  The latter records a sheet without claiming enclosed material. The mesh's
  existing solid or sheet manifold audit is the admission gate.
- Write `Mesh.Vertices()` as zero-based `<vertex x y z>` entries and
  `Mesh.Triangles()` as `<triangle v1 v2 v3>` entries. Decimal coordinates use
  the shortest round-trip float form and normalize negative zero.

### Errors

- Reject nil context, writer, or body with `ErrDegenerate`. Propagate
  tessellation errors, including cancellation and invalid tolerance.
- Build the ZIP archive before writing to the caller's writer. A failure in
  tessellation, ZIP construction, or cancellation leaves it untouched. An I/O
  error while copying the completed archive may leave a partial file.

### Several bodies

`export.ThreeMFItems(ctx, w, items, tol, opts...)` writes several bodies as one package. It knows nothing of parts or
of any process; a caller that explodes an assembly passes each part's placement.

- An `export.ThreeMFItem` is a name, a body and an `r3.Transform`. Item `i` becomes object `i+1` and build item
  `i+1`, in input order, with the name as the object's `name` attribute, XML-escaped. An empty name writes none.
- Vertices are moved by the placement before they are written, not through the 3MF `transform` attribute, so every
  reader places them the same way. Triangles keep their indices and winding.
- A body named by several items is tessellated once, at the one tolerance and options.
- A placement that reflects or is not a valid transform, a nil body, an empty item list, a nil context or writer
  are `ErrDegenerate`; a reflection would turn the mesh inside out. Equal input writes equal bytes.
- `ThreeMF` and `ThreeMFItems` share the package writer, so the archive layout, determinism and the
  build-before-write rule above hold for both.

The [3MF Core specification](https://github.com/3MFConsortium/spec_core/blob/master/3MF%20Core%20Specification.md)
defines the ZIP package, StartPart relationship, mesh indices, object types,
and millimetre units.

## G-code perimeter export

`export.GCodePerimeters` writes the first stage of a slicer: the toolpath that prints a body's outline layer by layer, with rectilinear infill when asked.
It reads the same section outline as `export.DXFSection` (`agent-docs/section-outline-design.md`).

### 1. Contract

```go
func GCodePerimeters(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, spec GCodeSpec) error
```

| Decision | Rule |
|---|---|
| G1 | The file is millimetres, absolute positioning and absolute extrusion: `G21`, `G90`, `M82`, `G92 E0`, then each layer under a `; layer i z=` comment. |
| G2 | With `zmin` the body's lowest point and `h` the layer height, there are `floor((zmax − zmin)/h)` layers; layer `i` is traced along the section at `zmin + (i + 1/2)·h` and printed at `zmin + (i + 1)·h`. A body lower than one layer, or needing over 100000 layers, is `ErrDegenerate`. The part above the last whole layer is not printed. |
| G3 | The section is `Body.SectionOutline` at `tol`: every loop, an outer loop and a hole alike, is a travel move (`G0`) to its first point at the layer's height and then extruding moves (`G1`) along it and back to the start. The outline is the mesh's, so a curved wall is chords. |
| G4 | The extrusion of a move is its length times `LayerHeight·LineWidth` over the filament's cross-section `π(d/2)²`, accumulated into the absolute `E`. The feed rate, a velocity, is written in millimetres a minute on each extruding move. |
| G5 | A length of the spec is positive (`ErrDegenerate` for zero, `ErrNegativeMagnitude`, `ErrUnitKind`), as is the feed rate; a nil context, writer or body is `ErrDegenerate`. Every failure writes nothing. |
| G7 | With `InfillSpacing` set (a positive length, else `ErrDegenerate`, `ErrNegativeMagnitude` or `ErrUnitKind`; the zero value writes none) each layer follows its perimeters with rectilinear infill: lines parallel to X on even layers and to Y on odd ones, one every spacing across the outline's extent from half a spacing in. A line is cut at every crossing with a loop, an outer loop and a hole alike, by the even-odd rule, so it runs through material between crossings in pairs, and each piece is shortened by `LineWidth` at both ends so that it does not pile onto a perimeter; a piece no longer than that is dropped. Each piece is a `G0` to its first point and one `G1`, and alternate lines run opposite ways. |
| G9 | `SolidLayers` (a count, zero for none, negative is `ErrDegenerate`) fills the first and last that many layers solid: their infill is G7's with the lines `LineWidth` apart whatever `InfillSpacing` is, and it is written even when `InfillSpacing` is zero, so the solid layers can stand alone. The layers between keep G7's sparse infill. The count is of the print's layers and no skin is found from what lies above or below a layer. |
| G10 | `PerimeterInset` moves each perimeter loop half a `LineWidth` into the material (`insetLoops`): the outer loops inward and the loops of holes outward, so that the printed line's edge lies on the outline. A loop is wound with the material on its left, outer loops counter-clockwise and holes clockwise, by how many other loops enclose it. Each corner is the meeting of its two moved sides, and a corner whose sides turn back by more than 170 degrees is cut off at the inset from the corner instead. The infill's lines are unchanged: they end a full line width from the outline, which is the inset perimeter's edge. |
| G12 | `Retraction` (a positive length, zero for none) pulls the filament back before each travel that follows an extruding move and pushes it forward after: a `G1 E` to the extrusion so far less the retraction, the `G0`, and a `G1 E` back to the extrusion so far, at the feed rate. The first travel of the print has none, the retractions net to nothing so the final `E` is as without them, and a negative or wrongly typed length is `ErrNegativeMagnitude` or `ErrUnitKind`. |
| G13 | `NozzleTemperature` and `BedTemperature` (a `units.AffineValue`, converted to Celsius, above 0 and below 1000, else `ErrDegenerate`; the zero value leaves the heater alone) write, after the header, `M140` and `M104` to set the bed and nozzle and `M190` and `M109` to wait for them, and after the last layer `M104 S0` and `M140 S0`. `StartCode` follows the heating and `EndCode` precedes the heaters going off, each as given with its line breaks. Without them the file is as before. |
| G14 | The tests read each travel after the first of an infilled two layer print for its retraction and push forward at the stated length, and the same final `E` as without; and the order of the heating lines, the start and end code, the Fahrenheit conversion (410 degrees is 210 Celsius) and the refusals. A retraction of twice the length fails the first. |
| G15 | `FanSpeed` (a dimensionless fraction in (0, 1], else `ErrDegenerate`, `ErrUnitKind`; the zero value leaves the fan alone) writes `M106 S<round(255·speed)>` just before the `; layer` comment of layer `FanFromLayer` (from 0 to one less than the layers, else `ErrDegenerate`), and `M107` after the last layer, before the end code. The test reads a half speed from the third layer of a six layer print as `M106 S128` before `; layer 2`, one `M106` only, `M107` before the end code, and the refusals; a fan that starts at the first layer fails it. |
| G6 | The tests print a 20 x 10 x 6 mm block as twelve layers, require each layer top, each coordinate inside the block's outline, the feed rate and an `E` that never backs up, and read the final `E` against the 720 mm of line and the filament's area. A wrong filament area fails them. |
| G11 | The inset tests print the 20 x 10 block, whose perimeter reaches 0.2 and 19.8 mm in x and 0.2 and 9.8 mm in y and runs 58.4 mm a layer, and the 20 x 20 plate with its 6 x 6 hole, whose outer loop reaches 0.2 and 19.8 and whose hole loop 6.8 and 13.2, 104 mm a layer. A loop wound the wrong way, or an inset the wrong way, fails them. |
| G8 | The infill tests print the block with a 5 mm spacing (two lines of 19.2 mm on an even layer and four of 9.2 mm on an odd one, at the stated coordinates, 451.2 mm of infill in all) and a 20 x 20 plate with a 6 x 6 hole (the lines through the hole break into two pieces of 6.2 mm, 63.2 mm of infill a layer), read the final `E` against the lengths, and refuse a negative or wrongly typed spacing. An infill without the inset, or one that ignores the hole, or one that never turns, fails them. A 12 layer print with two solid layers at each end reads 25 and 50 lines on the solid even and odd layers and the sparse ones between, with the solid layers alone as well, and fails with solid layers ignored. |
| G16 | `Skirt` (0 to 20 loops, else `ErrDegenerate`) traces that many loops round the part in the first layer, before its perimeters, from the one nearest the part outward: the outer loops of the layer's outline (`outerLoops`, those no other loop encloses an odd number of times, so a hole gets none) moved out by `SkirtDistance + n·LineWidth` for loop `n` from zero, by `insetLoops` with a negative distance. `SkirtDistance` is a positive length that a skirt requires, and a distance without a skirt is `ErrDegenerate`; the zero `Skirt` writes nothing. The tests print the 20 x 10 block with two loops 3 mm out (the first reaching −3 to 23 in x and −3 to 13 in y, the second 0.4 mm farther, in layer 0 only, before the perimeter, adding 171.2 mm of line), a holed plate (a skirt round the plate and none round the hole), and the refusals. A loop step of twice the line width fails the first, and a skirt round the holes fails the second. |

### 2. Not covered

- Infill other than rectilinear lines, a skin found from what lies above or below a layer, support, acceleration and a bed levelling or purge sequence.
- A toolpath for a mill or a lathe, and a body whose section the outline refuses.
