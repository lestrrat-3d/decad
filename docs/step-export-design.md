# STEP export design

`export` groups STL, OBJ, 3MF, and STEP writers in one package. Every writer takes
`ctx`, `w`, `body`, and a positive length chord tolerance in that order.
`STL`, `OBJ`, and `ThreeMF` accept `decad.TessellateOption` values and default to
`decad.VerifyNone`. They write the mesh returned by `Body.Tessellate` at the
requested verification level. The 3MF package contract is in
`docs/3mf-export-design.md`. `export` imports decad and
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
- Use analytic AP214 faces when every body face is a `Plane`, a `Cylinder` or a
  `Torus` and every edge is a `Line3`, an `Arc3`, an `Ellipse3` or a full `Circle3`. A plane's loops
  are one full circle each, or chains of lines and arcs (at least three
  edges, or two when one is an arc). A cylindrical wall is either full —
  exactly two one-circle boundary loops whose start vertices align along its
  axis — or partial: one loop of at least four edges, each an `Arc3` about
  the cylinder's axis (its `Axis` that axis or its negation, within 1e-12 of
  parallel, the rounding a frame lift leaves) or an `Ellipse3`, or a `Line3`
  along it (an exactly zero cross product), with lines and arcs or ellipses
  both present, since a side line split by a neighbouring face's vertex is
  several `Line3`s. A loop fillet's straight-walk patch is such a wall, closed
  by quarter-meridian arcs or by mitre ellipses. Emit one `ADVANCED_FACE`
  per body face. Share `VERTEX_POINT`s and `EDGE_CURVE`s by body topology
  identity. An `Arc3` edge is a `CIRCLE` placed about its own `Axis`, with the
  reference direction to its start vertex, trimmed by its two vertices; it
  sweeps counter-clockwise about that axis from start to end, so its
  `EDGE_CURVE` keeps the circle's sense.
  An `Ellipse3` edge is an `ELLIPSE` placed at its `Center` about its `Axis`
  with `Major` as the reference direction and `SemiMajor`, `SemiMinor` as the
  semi-axes; it sweeps counter-clockwise from start to end, as the `EDGE_CURVE`
  states. A torus is a `TOROIDAL_SURFACE` at its `Center` about its `Axis`
  with its major and minor radii (a minor above the major, a spindle torus,
  is written as is). A whole-turn torus has exactly two one-circle loops whose
  start vertices lie on one azimuth (an exactly zero cross product); it
  receives one synthetic seam, the tube circle through both start vertices,
  used twice in opposite directions. A torus patch has one loop of at least
  four `Arc3` edges, each about the torus axis (a parallel) or in a plane
  holding it (a meridian), both kinds present. Its sense follows its outward
  normal against the torus's own, which points away from the tube's centre
  circle, and its loop is reversed when its signed area in the (azimuth, tube
  angle) plane disagrees with that sense. A horn torus patch is a three-edge
  loop and keeps the faceted writer.
  A full cylindrical wall receives one synthetic straight seam edge, used
  twice in opposite directions in its STEP loop. This seam changes STEP
  topology, not the body's geometry. Plane faces use their source outer and
  inner loops. A face's plane sense follows its outward normal; reverse its
  loop walks when the source walk opposes that normal. A plane whose outer
  loop is a chain of lines and arcs takes its own plane normal as the
  placement axis, turned so the loop's signed area about it (each arc adding
  its circular segment) is positive; the sense is never read off the turn at
  the loop's first vertex, which a reflex corner reverses. A partial wall's sense follows its outward normal
  against the radial direction, and its loop is reversed when its signed area
  in the cylinder's (θ, z) parameter plane disagrees with that sense.
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
