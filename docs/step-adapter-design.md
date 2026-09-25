# STEP adapter design

`stepadapter` is an optional package in this module. It imports decad and
`github.com/lestrrat-3d/step/ap214`; decad's root package does not import STEP.
`NewFile(ctx, body, header, tol)` returns a `step.File`, and `Write` writes it.
The caller supplies complete Part 21 header metadata and a positive length
chord tolerance. `ap214.NewFile` sets `AUTOMOTIVE_DESIGN` in `FILE_SCHEMA`.

## Geometry contract

- Request `Body.Tessellate` at `VerifyBoundary`. The resulting mesh is an
  approximation of decad's analytic boundary, with `Mesh.Bound()` retaining
  the proven two-sided displacement. The STEP file represents the held facets
  exactly; it does not carry that bound or claim analytic faces survived.
- Admit one valid `BodySolid` with one non-void shell. Refuse sheets, multiple
  shells, and disconnected facet sets. Multiple shells need separate solids or
  void relationships, which this adapter cannot safely infer.
- Preserve mesh vertex indices as distinct `VERTEX_POINT`s and share one
  `EDGE_CURVE` per undirected mesh edge. Emit one oriented edge per facet side,
  one `EDGE_LOOP`, `FACE_OUTER_BOUND`, and planar `ADVANCED_FACE` per facet.
  The facet winding fixes the plane axis and face sense.
- Require each mesh edge twice with opposite directions and one connected
  facet set. These checks only reject an inconsistent mesh; they never admit
  a body that `Tessellate` refused.

## File contract

- Use millimetres for coordinates, radians for plane angles, and steradians
  for solid angles. Link one `MANIFOLD_SOLID_BREP` through an
  `ADVANCED_BREP_SHAPE_REPRESENTATION` to one AP214 product definition.
- Allocate entity IDs in deterministic traversal order. The same body,
  tolerance, and header produce identical bytes. Never read the clock.
- Honor context cancellation while building records. `Write` validates and
  builds the complete file before touching its writer; `step.File.Write`
  validates its model before output. I/O errors may leave a partial file.
