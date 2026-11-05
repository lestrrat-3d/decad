# 3MF export design

`export.ThreeMF(ctx, w, body, tol, opts...)` writes one body's tessellation as
a 3MF Core package. It accepts the same chord tolerance and
`decad.TessellateOption` values as STL and OBJ. It defaults to `VerifyNone`;
`WithVerification` can request the stronger tessellation audits. A successful
`Body.Tessellate` result supplies every vertex and triangle. The writer does
not change coordinates, indices, or winding.

## Package

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

## Errors

- Reject nil context, writer, or body with `ErrDegenerate`. Propagate
  tessellation errors, including cancellation and invalid tolerance.
- Build the ZIP archive before writing to the caller's writer. A failure in
  tessellation, ZIP construction, or cancellation leaves it untouched. An I/O
  error while copying the completed archive may leave a partial file.

## Several bodies

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
