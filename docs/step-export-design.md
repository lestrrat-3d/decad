# STEP export design

`export` groups STL, OBJ, and STEP writers in one package. Every writer takes
`ctx`, `w`, `body`, and a positive length chord tolerance in that order.
`STL` and `OBJ` accept `decad.TessellateOption` values and default to
`decad.VerifyNone`. They write the mesh returned by `Body.Tessellate` at the
requested verification level. `export` imports decad and
`github.com/lestrrat-3d/step/ap214`; decad's root package does not import
STEP. `NewSTEPFile(ctx, body, tol, header)` returns a `step.File`. The
`STEP(ctx, w, body, tol, opts...)` writer requires options for a nonempty file
name, author, and organization. It defaults the timestamp to the current UTC
time unless `WithSTEPTimestamp` supplies one. It sets
`Description` to `decad solid`, `PreprocessorVersion` to `decad export`,
and `OriginatingSystem` to `decad`. Callers needing different or multiple
header values pass `WithSTEPHeader(step.Header)` to `STEP`. This uses the
entire supplied header, including zero values, without filling fields from
defaults or individual options. Mixing `WithSTEPHeader` with a field option
returns `ErrDegenerate`. If several header options are given, the last wins.
`ap214.NewFile` sets `AUTOMOTIVE_DESIGN` in `FILE_SCHEMA`.
The product description says `analytic decad solid` or `faceted decad solid`
according to the writer path.

## Geometry contract

- Request `Body.Tessellate` at `VerifyBoundary` before either writer path.
  Its closed and embedded boundary is the shared admission check.
- Admit one valid `BodySolid` with one non-void shell. Refuse sheets, multiple
  shells, and disconnected facet sets. Multiple shells need separate solids or
  void relationships, which this adapter cannot safely infer.
- Use analytic AP214 faces when every body face is a `Plane` or a `Cylinder`,
  every edge is a `Line3` or full `Circle3`, and each cylindrical wall has
  exactly two one-circle boundary loops whose start vertices align along its
  axis. Emit one `ADVANCED_FACE` per body face. Share `VERTEX_POINT`s and
  `EDGE_CURVE`s by body topology identity.
  A cylindrical wall receives one synthetic straight seam edge, used twice
  in opposite directions in its STEP loop. This seam changes STEP topology,
  not the body's geometry. Plane faces use their source outer and inner loops.
  A face's plane sense follows its outward normal; reverse its loop walks when
  the source walk opposes that normal.
- Otherwise preserve mesh vertex indices as distinct `VERTEX_POINT`s and
  share one `EDGE_CURVE` per undirected mesh edge. Emit one oriented edge per
  facet side, one `EDGE_LOOP`, `FACE_OUTER_BOUND`, and planar `ADVANCED_FACE`
  per facet. The resulting file represents the held facets exactly, but does
  not carry `Mesh.Bound()` or claim analytic faces survived.
- Require each mesh edge twice with opposite directions and one connected
  facet set. These checks only reject an inconsistent mesh; they never admit
  a body that `Tessellate` refused.

## File contract

- Use millimetres for coordinates, radians for plane angles, and steradians
  for solid angles. Link one `MANIFOLD_SOLID_BREP` through an
  `ADVANCED_BREP_SHAPE_REPRESENTATION` to one AP214 product definition.
- Allocate entity IDs in deterministic traversal order. The same body,
  tolerance, and explicit timestamp or header produce identical bytes.
- Honor context cancellation while building records. `STEP` validates and
  builds the complete file before touching its writer; `step.File.Write`
  validates its model before output. I/O errors may leave a partial file.
