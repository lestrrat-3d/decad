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

The [3MF Core specification](https://github.com/3MFConsortium/spec_core/blob/master/3MF%20Core%20Specification.md)
defines the ZIP package, StartPart relationship, mesh indices, object types,
and millimetre units.
