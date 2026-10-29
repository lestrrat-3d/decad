# STEP export design

`export` groups STL, OBJ, 3MF, and STEP writers in one package. Every writer takes
`ctx`, `w`, `body`, and a positive length chord tolerance in that order.
`STL`, `OBJ`, and `ThreeMF` accept `decad.TessellateOption` values and default to
`decad.VerifyNone`. They write the mesh returned by `Body.Tessellate` at the
requested verification level. The 3MF package contract is in
`agent-docs/3mf-export-design.md`. `export` imports decad and
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
- Admit one valid `BodySolid` of one or more lumps, each with one outer shell
  and any number of void shells (§9). Refuse sheets and a mesh whose connected
  facet sets do not match the body's shells one to one.
- Use analytic AP214 faces when every body face is a `Plane`, a `Cylinder`, a `Cone`, a `Sphere` or a
  `Torus` whose tag states no departure from the face's surface (below), and every edge is a `Line3`, an
  `Arc3`, an `Ellipse3` or a full `Circle3`. A plane's loops
  are one full circle each, or chains of lines and arcs (at least three
  edges, or two when one is an arc). A cylindrical wall is either full —
  exactly two one-circle boundary loops whose start vertices align along its
  axis — or partial: one loop of at least four edges, each an `Arc3` about
  the cylinder's axis (its `Axis` that axis or its negation, exactly) or an `Ellipse3`, or a `Line3`
  along it (an exactly zero cross product), with lines and arcs or ellipses
  both present, since a side line split by a neighbouring face's vertex is
  several `Line3`s. A loop fillet's straight-walk patch is such a wall, closed
  by quarter-meridian arcs or by mitre ellipses.
  A conical wall is full or a patch. A full wall has exactly two one-circle boundary loops whose
  start vertices lie on one meridian. It is written as a `CONICAL_SURFACE` placed at the cone's `Origin`,
  with the cone's `Radius` there and its `HalfAngle`, and a seam along the generator between the starts.
  A cone patch has one loop of at least three edges on a cone whose `Radius` is exactly zero, so its
  `Origin` is the apex. Each edge is an `Arc3` about the cone's axis (its `Axis` the cone's or its
  negation and its `Center` on the axis, exactly) or a `Line3` along a generator (an exactly zero cross
  product of its two ends taken from the apex), with both kinds present. A loop chamfer's convex corner
  gives two arcs and two generators; a reflex corner gives one arc and two generators meeting at the apex
  vertex. Every such corner cone is a cap-blend band patch whose tag states a departure, so a chamfered
  body takes the faceted writer, and no producer builds a cone patch the analytic writer receives. The
  arm's own tests write the chamfer corners through the writer directly. Its face sense is its outward
  normal against the radial direction at its first vertex off the axis, since the `CONICAL_SURFACE`
  normal leans away from the axis at every point off it.
  A sphere patch has one loop of at least three `Arc3` edges, each a great circle: its `Center` and
  `Radius` the sphere's, exactly. A box corner's fillet ball is three quarter arcs. It is written as a
  `SPHERICAL_SURFACE` at the sphere's `Center`, placed about the world z axis, with the sphere's
  `Radius`; its sense is its outward normal against the direction from the centre.
  Every arc of a cone, sphere or torus patch has both vertices in its own plane (an exactly zero dot
  product of each with its `Axis` from its `Center`), and every vertex of a cone, sphere or horn torus
  patch is `Exact`. Both checks only refuse: an arc record whose axis does not hold its vertices, or a vertex
  whose position carries a bound, is not proven to lie on the curves and surface the file states
  through it, and the body takes the faceted writer. A loop fillet's meridians pass the plane check
  where the record denotes their feet exactly: the fillet holds those feet at the exact point
  (`agent-docs/loop-fillet-design.md` §4 step 1), so the rounded plate's top-loop fillet (loop-fillet
  P8 with its top loop filleted r = 2) holds every vertex `Exact` and writes analytic. The vertex
  check covers cone, sphere and horn torus patches only. The same reason
  holds for the other arms, which do not apply it: a plane, cylinder or other torus face writes its held
  vertex coordinates through its curves and surface whatever bound they carry, so such a file states
  points the body does not prove lie on them. The bounds those faces carry today are rounding, measured on amd64 at
  most 5.3e-15 mm on a shell with an oblique opening, and 7.7e-14 mm and 1.5e-13 mm on the round bosses
  crossing a plate's outline. Applying the
  check to every arm would send each of those bodies to the faceted writer
  (`agent-docs/missing-features.md`).
  A cone, sphere or horn torus patch's loop is walked by the edge-use rule, never by a parameter-plane
  area, which a cone's or horn torus's apex vertex leaves undefined. Every other face is walked as its own rule below decides. A
  closed shell uses each edge once in each direction, so each patch is walked so that every edge it
  shares with a face already decided runs opposite to that face's use, and the decided set grows until
  no patch is left. A patch that no decided face reaches, or whose shared edges ask for both walks,
  sends the body to the faceted writer.
  A cone with one circle loop, which reaches its apex without a vertex there, takes the faceted writer.
  A complete sphere or regular ring from an authenticated full Revolve with identity or pure translation
  placement writes analytically when the translated carrier matches the tag exactly and the synthetic point
  and outward normal checks pass. The placement must be finite and valid with an exactly identity basis. A sphere uses a north-pole vertex loop.
  A ring uses two shared circle edges, each used in opposite directions. Unsupported or inexact cases fall back
  to the faceted writer. `ReadSTEP` still refuses whole spheres and tori (I15 and I4).
  Emit one `ADVANCED_FACE`
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
  holding it (a meridian), both kinds present, every meridian's `Radius` the
  torus's `Minor`, exactly, so the tag holds the tube the meridians run round.
  Its sense follows its outward
  normal against the torus's own, which points away from the tube's centre
  circle, and its loop is reversed when its signed area in the (azimuth, tube
  angle) plane disagrees with that sense. A horn torus patch is a torus whose
  `Minor` equals its `Major`, exactly, so its tube touches its axis at its
  `Center`, bounded by one loop of three `Arc3` edges, each a parallel or a
  meridian holding both its vertices in its own plane: exactly one vertex at
  the `Center` (the apex), every edge at the apex a meridian, and exactly one
  parallel. A pocket mouth fillet's reflex corner (loop-fillet LF6) is one:
  its connector arc and the two meridians that run down to the apex. It is a
  `TOROIDAL_SURFACE` with both radii equal, walked by the edge-use rule, since
  its apex has no azimuth, and its sense is its outward normal against the
  torus's own at its first vertex off the axis, where the surface has a
  normal. Its meridians' `Radius` is the torus's `Minor`, exactly, as on a
  four-edge patch. Every leg only refuses; P2's pocket mouth fillet, r = 1.5,
  writes its four corners so. The horn patch is a plain `TOROIDAL_SURFACE`
  because that entity has no rule comparing its radii, while
  `DEGENERATE_TOROIDAL_SURFACE` requires its major radius below its minor, and
  no outside STEP reader in this repository checks the result.
  The analytic writer copies each face's tag as it is held: the STEP surface
  is the float image of the `Plane`, `Cylinder`, `Cone`, `Sphere` or `Torus` tag. A face whose tag
  is not proven to be the surface the face carries sends the whole body to the faceted writer before
  any face is read. Such a face states a nonzero `topo.Face.NormalBound` or
  `topo.Face.HeldNormalBound`, states a nonzero `topo.Face.AxialDelta` (a planar face displaced along
  its own normal from the level its tag holds), or carries a denoted surface (`topo.Face.Denoted`, a
  revolve face's record, which a mesh Boolean's retained face keeps) that its tag does not equal
  exactly. A `Plane`, `Cylinder` or `Torus` tag equals its record when `stitchflux.TagIsDenoted` says
  so, by exact rational equality on the record's own points; a `Sphere` tag also needs its radius to
  equal the recorded radius, which `TagIsDenoted` does not read. A `Cone` tag never does:
  `TagIsDenoted` checks that the meridian lines meet at one apex and that the axis is the record's, but
  not the tag's own apex or half-angle, and the revolve builds both from a float apex and an `atan2`
  half-angle. `TagIsDenoted` refuses every cap record, so both caps of a partial revolution depart,
  the start cap even where it is placed exactly; the body's output is the same either way, since its
  end cap's angular displacement already refuses it. The check only refuses: such a face is not proven
  to lie on its tag, so a file stating that surface would claim more than the body proves. `export`
  reads the face through `internal/facedeparture`, which the root package installs
  (`face_departure.go`), since a public `Face` exposes none of these fields.
  The bodies the check sends to the faceted writer include every Loft with a cap not proven to lie
  exactly on its section plane, which states the loft's vertex displacement (`agent-docs/loft-design.md`
  §18; a cap on a tilted plane whose held points round off it), a Loft bore cylinder
  with a rounded center or an inexact frame map (`agent-docs/loft-design.md` §18), an extrusion up to
  a face at an offset whose level no float holds (a `ToFace` stop 0.1 mm above a cap at z = 10), every
  loop chamfer's corner cone, which is a cap-blend band patch (`agent-docs/modify-reach-design.md`
  §8.3), every drafted wall, plane or cone, whose tag is a rounding of the tilted surface
  (`agent-docs/draft-design.md` DD4), every partial revolution, and every revolved cone. Their
  departures are rounding-sized, measured on amd64 at 5.0e-14 mm on the end cap of a loft with a
  radius-1 hole on the frame tilted to (1, 0, 4), 3.6e-16 mm on the offset `ToFace` cap, 4.3e-17 to 2.2e-16 on the chamfer cones, 1.5e-16 to
  2.9e-16 on the drafted walls, and 2.3e-15 mm on the apex of a frustum revolved from radius 3 to 7.1.
  A revolved tube's cylinders and annular caps equal their records and keep the analytic writer, and
  so does a world-frame Loft of a square with a centred circle hole, whose bore and caps are exact. A
  revolved ball's sphere and a revolved ring's torus may use the complete-surface arm described above. No producer builds a full conical wall or a
  cone patch that the analytic writer receives; the arm's own tests write a revolved frustum and the
  loop chamfer corners through the writer directly.
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
- Require each mesh edge twice with opposite directions, and as many
  connected facet sets as the body has shells. These checks only reject an
  inconsistent mesh; they never admit a body that `Tessellate` refused.

## File contract

- Use millimetres for coordinates, radians for plane angles, and steradians
  for solid angles. Link one `MANIFOLD_SOLID_BREP` through an
  `ADVANCED_BREP_SHAPE_REPRESENTATION` to one AP214 product definition.
- Allocate entity IDs in deterministic traversal order. The same body,
  tolerance, and explicit timestamp or header produce identical bytes.
- Honor context cancellation while building records. `STEP` validates and
  builds the complete file before touching its writer; `step.File.Write`
  validates its model before output. I/O errors may leave a partial file.

## 9. A solid with a cavity, and a body of several lumps

Each lump (`Body.Lumps`) is one solid: its outer shell and its void shells. A lump with a cavity writes a
`BREP_WITH_VOIDS` in place of the `MANIFOLD_SOLID_BREP`. A body with a cavity or with several lumps is the faceted arm's
alone: the analytic arm builds the faces of one outer shell.

| Decision | Rule |
|---|---|
| V1 | Every lump has exactly one non-void shell (`FaceShell.IsVoid` is false). Any number of void shells may follow. A lump without one outer shell is `ErrUnsupported`. |
| V2 | The verified mesh has one connected facet set for each shell, and `Mesh.SourceFaces` places every facet in one shell of the body. A count that differs, a facet set that spans two shells, or a shell without facets is `ErrUnsupported`. A cavity that touches its outer shell along an edge or at a vertex, as `Cut` returns it, is one shell of the body and one facet set of the mesh, so STEP writes it as that one shell (`TestSTEPWritesACavityThatTouchesTheOuterShell`). |
| V3 | The body, not the mesh, names the outer shell of a lump. The mesh only locates each shell's facets. |
| V4 | The outer shell is a `CLOSED_SHELL` of its triangles' planar `ADVANCED_FACE`s. Each void is a `CLOSED_SHELL` of its own faces, taken as the mesh winds them, which look into the cavity and away from the material, inside an `ORIENTED_CLOSED_SHELL` with the orientation `.T.`. |
| V5 | `BREP_WITH_VOIDS(name, outer, (voids))` names the product's solid, the name being the header's. Entity IDs follow the same deterministic order as every file. |
| V6 | A lump with no cavity is a `MANIFOLD_SOLID_BREP` named by the header. A single-lump solid with no cavity is analytic where the faces allow it. |
| V7 | One `ADVANCED_BREP_SHAPE_REPRESENTATION` holds one item per lump, in `Body.Lumps` order. |
| V8 | The signed volume of each facet set only refuses: an outer set whose volume is not positive, or a void set whose volume is not negative, is `ErrUnsupported`, so a mesh wound against its shell's kind is never written as an `ORIENTED_CLOSED_SHELL`. |

The tests build a cube with a cubic cavity, a disjoint union of two blocks, and a cavity cube beside a block, check the entity counts, and check that every outer face's normal points away
from the cube's middle and every void face's toward the cavity's. A body of a closed shell (`WithNoOpenings`) is such a
solid.

