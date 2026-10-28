# decad API reference

This is the user-facing reference for the `decad` module. It names what the library offers and how the pieces fit.
Exact signatures and full behaviour live in the Go doc comments:
[pkg.go.dev/github.com/lestrrat-3d/decad](https://pkg.go.dev/github.com/lestrrat-3d/decad).
Runnable, output-checked usage lives in [`examples/`](../examples).
The values these functions take and return are described in [Data structures](data-structures.md).

Root-package identifiers carry the `decad.` prefix. `apitest/api_reference_test.go` checks that each one exists.

## Contents

- [Packages](#packages)
- [The modeling loop](#the-modeling-loop)
- [Conventions](#conventions)
- [Documents and bodies](#documents-and-bodies)
- [Operation history and diagnostics](#operation-history-and-diagnostics)
- [Building bodies](#building-bodies)
- [Modifying bodies](#modifying-bodies)
- [Booleans](#booleans)
- [Copies, patterns, mirrors and placement](#copies-patterns-mirrors-and-placement)
- [Sheets and surfaces](#sheets-and-surfaces)
- [Selecting topology](#selecting-topology)
- [Reading bodies](#reading-bodies)
- [Verification](#verification)
- [Motion, linkages and contact](#motion-linkages-and-contact)
- [Import](#import)
- [Export](#export)
- [Errors](#errors)
- [Test helpers](#test-helpers)

## Packages

| Package | Purpose |
|---|---|
| `decad` | Documents, bodies, features, selectors, measurement and verification. |
| `decad/export` | Writes a body as STEP, STL, OBJ, 3MF or glTF, draws it as a PNG image, or writes a 2D DXF drawing of it. |
| `decad/dynamics` | Rigid-body worlds, stepping and replayable timelines. |
| `decad/decadtest` | Assertions for tests that build and verify bodies. |

decad builds on three sibling modules. `sketch` solves 2D profiles, `r3` supplies vectors, frames and transforms,
and `units` supplies typed quantities.

## The modeling loop

1. Solve a 2D profile in a `sketch.Sketch`.
2. Create a `decad.Document` with `decad.New`.
3. Turn the profile into a body with a feature function such as `decad.Extrude`.
4. Modify or combine bodies with further feature functions.
5. Call `Document.Verify` and gate on `Report.Passed`.
6. Read measurements, tessellate, or export.

The full loop is the executable example in [`example_quickstart_test.go`](../example_quickstart_test.go).

## Conventions

### Immutable bodies

A `*decad.Body` never changes. A feature returns a new body and retires its inputs from the document.
`decad.Offset`, `decad.Copy`, `decad.MirroredCopy` and `decad.PatternCopies` leave their input live.

### Features build and methods ask

Features are package functions that take a context and return `(*Body, error)`. Methods on bodies, faces and edges
only read.

### Typed quantities

Lengths, angles and tolerances are `units.Value`, for example `units.Millimeters(10)`. A bare `float64` is never a
measurement. Coordinates are `r3.Vec` in millimetres.

### Measurements carry their error

A `decad.Measurement` holds a value, a `decad.Exactness` (`decad.Exact` or `decad.Approximate`) and an absolute
error bound.

### Selectors, not indices

Features take queries such as `decad.Edges(decad.Convex())`, never an edge index.

### Sealed option sets

`decad.Extent`, `decad.AngularExtent`, `decad.Axis`, `decad.PatternSpec` and `decad.MirrorPlane` list every legal
variant, so an illegal combination does not compile.

### Loud failure

An unsupported or unprovable request returns an error wrapping a sentinel. It never returns a guessed body.
See [Errors](#errors).

### Concurrency

A `decad.Document` is not safe for concurrent mutation. Bodies are immutable and safe to read from any goroutine.
`Document.Bodies`, `decad.Remove` and `Document.Verify` may run concurrently with each other.

## Documents and bodies

| Identifier | Meaning |
|---|---|
| `decad.New` | Creates an empty `decad.Document`. |
| `Document.Bodies` | Lists the live bodies. |
| `Document.OperationHistory` | Returns an immutable snapshot of recorded successful modeling calls. |
| `Document.Root`, `Container.NewChild` | Read the document's root container and add named children to a container. A container groups sketches, planes and bodies and changes no geometry. |
| `Container.Name`, `Container.Parent`, `Container.Children` | Read a container's name, parent (nil for the root) and children in creation order. |
| `Container.AddSketch`, `Container.AddPlane` | Place a sketch or plane in a container. Unplaced ones belong to the root. |
| `Container.Sketches`, `Container.Planes`, `Container.Bodies` | List a container's members in creation order. |
| `Document.ContainerOf`, `Document.ContainerOfSketch` | Read the container of a body or a sketch. |
| `Document.MoveBody` | Puts a live body in another container and changes nothing else. |
| `decad.Remove` | Retires a body from its document. |
| `decad.Body` | An immutable solid or sheet. |
| `decad.BodyKind` | `decad.BodySolid` or `decad.BodySheet`. `Body.Kind` and `Body.IsSolid` read it. |

## Operation history and diagnostics

`Document.OperationHistory` snapshots recorded calls in commit order. Use
`Operations`, `Producer` and `Consumers` to inspect body dependencies, including
retired bodies. Each operation reports its public kind, ordered body inputs and
outputs. `FaceSuccessors` reports `HistoryUnknown` when the operation has no
complete face map.

Every document records successful public body-producing calls and successful `Remove` calls. Temporary tools used
inside `Hole`, `Rib`, `Web` and `Emboss` do not create separate history entries. A failed or canceled call creates
no entry. History collection has no option.

`Body.OperationDiagnostics` returns warnings attached to the call that created
the body. An accepted shifted Boolean retry reports `OpDiagGapShiftApplied`
with its charged displacement bound. A Boolean that uses a configured chain
allowance reports `OpDiagChainAllowanceApplied` with the touched bound, pair
chord limit and allowance. A tolerant Stitch merge reports
`OpDiagStitchMergeApplied` with its charged displacement bound. Calls that make
no reported adjustment return an empty warning slice. A held-mesh repair that
uses its positive-reach stage reports `OpDiagEmbeddingReachApplied` with the
repair's displacement bound; ordinary coordinate rounding does not add it.
An empty Boolean result after an accepted shift includes both the empty-result
failure and the shift warning on its error.

Use `errors.As` with `OperationDiagnosticError` to inspect structured reasons,
including through caller `%w` wrapping. `BooleanError` and `SelectionError`
keep their concrete types. `OperationError` wraps other mapped Decad errors and
preserves `errors.Is` behavior. Cancellation and deadline errors pass through
unchanged. Existing bounded work-limit refusals report their resource and
observed and maximum counts when the producer knows them. See
`Example_operation_diagnostics` for code-based branching.

## Building bodies

Section-fed features accept a closed `*sketch.Profile`, which builds a solid, or an open `*sketch.Chain`, which builds
a sheet. The `decad.Section` type constraint covers both.

| Function | Builds |
|---|---|
| `decad.Extrude` | A prism along the sketch normal, sized by a `decad.Extent`. |
| `decad.Revolve` | A solid of revolution about a `decad.Axis`, sized by a `decad.AngularExtent`. |
| `decad.SheetMetal` | A bent strip of flat legs and circular bends as a solid, with its flat pattern length from the K-factor. |
| `decad.Sweep` | A section moved along a `*decad.Path`. |
| `decad.Loft` | A ruled solid between two profiles on different planes. |
| `decad.LoftChain` | A ruled sheet between two open chains. |
| `decad.LoftSections` | A smooth solid through three or more profiles, including up to 1024 sections of one fitted edge and whole lines that turn about the plane origin. |
| `decad.LoftFaces` | A solid through ordered cap faces. |
| `decad.LoftFacesToConeTangent` | A face loft that ends tangent to a cone. |
| `decad.LoftFromPoint` | A solid from an apex point to a profile. |
| `decad.LoftBetween` | A solid between two ends, each a profile (`decad.ProfileEnd`) or a point (`decad.PointEnd`). |
| `decad.Coil` | A profile moved along a screw motion, such as a thread or spring. |
| `decad.ConeFromFace` | The cone that extends a face's straight meridian. |

### Extents

`decad.Extent` is what `decad.Extrude` accepts: `decad.Distance`, `decad.Symmetric`, `decad.TwoSided`,
`decad.ThroughAll` and `decad.ToFace`. `Extrude` to a planar face of a mesh solid that is tilted from the sweep cuts the extrusion at the face's plane, if the face is a convex outline holding the whole profile seen along the sweep. A `decad.Direction` is `decad.Along` or `decad.Against` the sketch normal.
`decad.Between` joins two placements along a screw path for motion checks.

`decad.AngularExtent` is what `decad.Revolve` accepts: `decad.FullRevolution`, `decad.AngleExtent`,
`decad.SymmetricAngle`, `decad.TwoSidedAngle` and `decad.ToFaceAngular`.

### Axes and paths

A revolve or coil axis is a `decad.SketchLine`, a `decad.ConstructionAxis` or a `decad.EdgeAxis`.
Build a sweep path with `decad.NewPath`. Its segments are `decad.LineTo`, `decad.ArcThrough` and the other
`decad.PathSegment` values.

### Options

| Function | Applies to |
|---|---|
| `decad.WithTaper` | `decad.Extrude` |
| `decad.WithLoftAlignment`, `decad.WithLoftSplits` | `decad.Loft` |
| `decad.WithMitredJoins`, `decad.WithSectionScale`, `decad.WithSweepTwist` | `decad.Sweep` |
| `decad.WithLeftHand` | `decad.Coil` |
| `decad.WithSurfaceResult` | Section features: build a sheet instead of a solid. |

## Modifying bodies

Each of these returns a new body and retires its input.

| Function | Effect |
|---|---|
| `decad.Fillet` | Rounds selected edges with a tangent arc of a given radius. |
| `decad.Chamfer` | Bevels selected edges. `decad.WithAsymmetricChamfer` sets a second distance. Selecting the whole outer cap loop of a prism whose section is a smooth chain of fit splines bevels it within a measured ratio of the ideal chamfer, as a cut. Several convex edges of an axis-square mesh solid that share corners are bevelled as the solid less each edge's wedge, and several concave ones whose ends lie in material or open to air as the solid plus each edge's wedge. |
| `decad.FilletChord` | Rounds convex or concave edges between two planar faces, or circular ones between a plane and a surface of revolution, with an arc stated by its chord length. |
| `decad.ChamferAngle` | Bevels convex or concave edges between two planar faces, or circular ones between a plane and a surface of revolution, by a distance along a reference face and an angle from that face. |
| `decad.Stretch` | Scales a faceted solid, such as a mesh solid, by a separate factor along each world axis about an anchor, and returns a solid. |
| `decad.MoveFace` | Press-pulls one face of a mesh solid whose faces all square to an axis along its normal; a positive distance grows the body and a negative one cuts it, and the walls beside it follow. |
| `decad.Shell` | Removes the selected faces and lines the rest with a wall. `decad.WithShellFaceThickness` gives the walls facing one world axis direction their own thickness on a mesh solid whose faces all square to an axis. `decad.WithShellSense` picks `decad.Inward`, `decad.Outward` or, for a mesh solid whose faces all square to an axis, `decad.Centered`, a wall half the thickness each side of the surface. `decad.WithNoOpenings` keeps every face and hollows the body into one outer shell around a cavity: for a straight prism, a revolve, or a drilled bar whose hole runs straight through, where the cavity is a ring round a tube lining the hole. Outward, the cavity is the original body and each hole's lining grows into the hole, so a hole whose radius is at most the thickness is `decad.ErrUnsupported`. A mesh solid whose faces all square to an axis shells inward with mitered corners. |
| `decad.Draft` | Leans selected walls about a `decad.NeutralPlane` (`decad.NeutralFace`). Every wall of a mesh solid with exact vertices that is a straight prism along a world axis, about one of its two caps, is drafted as the tapered extrude of its section. |
| `decad.Hole` | Cuts a round hole (or a rectangular, slot or regular polygonal one from `HoleSpec.Shape`), optionally drafted with `HoleSpec.Draft` or tapped with a `decad.HoleThread`, at a point on a planar face (a planar face of a mesh solid tilted from every axis too, along its normal), or radially into a cylindrical one, through or blind, with an optional `decad.Counterbore` or `decad.Countersink` head. A `decad.RectangularHole` or `decad.SlotHole` shape cuts that cross-section instead, without a head. |
| `decad.LookupThread` | Reads a thread designation such as `M10`, `M10x1.25`, `1/4-20`, `#10-24` or `G1/2` into its major diameter and pitch. |
| `decad.Thread` | Cuts an ISO metric thread, or a trapezoidal one from a `decad.ThreadForm`, with an optional lead-in chamfer, right- or left-handed, into a cylindrical face, a rod or a bore, from its pitch, along the whole face or a stated length of it. |
| `decad.Boss` | Raises a round boss, optionally braced with gusset ribs from `decad.BossSpec.Ribs` or leaning by `decad.BossSpec.Tilt`, or a rectangular, slot or polygonal one from `decad.BossSpec.Shape`, on a planar (a planar face of a mesh solid tilted from every axis too) or cylindrical face, with an optional blind pilot hole that may carry a counterbore, a countersink or a thread, and a fillet round its root. |
| `decad.Rib` | Thickens one straight sketch line, extrudes it by an `decad.Extent` and joins it to the body. `decad.WithRibOverrun` buries its ends in the walls, and `decad.WithRibDraft` leans its walls. |
| `decad.RibArc` | Thickens one sketch arc and joins it to the body as a rib, like `decad.Rib` for a line. |
| `decad.Web` | Adds one such rib for each of several lines in one step. |
| `decad.WebPath` | Adds one rib for each line and arc of a run, in one step, so a polyline or a slot-shaped run becomes one rib. |
| `decad.TextProfiles` | Draws lines of text from a TrueType font (Go Regular by default) as closed sketch profiles for `decad.Emboss`. |
| `decad.Emboss` | Raises or engraves closed profiles on a planar face (a planar face of a mesh solid tilted from every axis too), or on a cylindrical face from a sketch plane tangent to it. `decad.EmbossRaised` or `decad.EmbossEngraved` picks the mode, `decad.WithEmbossDraft` leans the walls and `decad.WithEmbossRimFillet` rounds the rim. |
| `decad.Thicken` | Turns a sheet into a solid. `decad.WithThickenSide` picks the side. |
| `decad.QuarterTurn` | Turns a mesh solid by whole quarter turns about a world axis, exchanging coordinates so the result stays exact. |
| `decad.Offset` | Builds a second sheet at a normal distance and leaves the source live; for a mesh solid whose faces all square to an axis it moves every face and builds a second solid. |

## Booleans

| Function | Result |
|---|---|
| `decad.Union` | The volume enclosed by two or more bodies. |
| `decad.Cut` | The target minus the tool. A straight-profile twisted `Sweep` is an admitted tool: a boolean restates its closer stationed boundary, and refuses a crossing or gap narrower than the two operands' held bounds together. |
| `decad.Intersect` | The volume common to both bodies. |
| `decad.Split` | One solid per arranged target cell. |

Two full `decad.Revolve` bodies about one axis combine exactly, with no `Faceted` face, when their meridians
are lines, arcs or circles: a sphere cut by a coaxial disk or shaft, or two coaxial spheres, keep their
sphere, torus, cone, cylinder and plane faces (an off-axis circle meridian gives torus faces). Meridians may share an
arc span when both record the same circle exactly, so a dome intersected with a ball of its radius is the dome and a
ball unioned onto an equal ball is that ball. Arcs whose recorded centres or radii differ, two equal whole-circle
meridians, and a contact at a pole or an edge are not joined this way.

A plain turned shaft, a full `decad.Revolve` of a rectangle with one side on an axis that runs along the sketch's
U or V, is cut exactly as the circular `decad.Extrude` it equals. A box tool at right angles to the shaft, drawn
on a datum plane and running past both shaft ends, gives a D-flat or a keyway with no `Faceted` face, and a
`decad.Thread` on its cut wall builds. A stepped shaft, a full `decad.Revolve` of a staircase of such rectangles
with one side on the axis, is cut the same way: the flat or keyway may cross any run of consecutive steps, a step
it misses keeps its whole wall, and each shoulder keeps the part the tool leaves. Both operands must keep the same
placement: a shaft or a tool placed by `decad.Transform` takes the faceted path, as do a tool that ends inside the
shaft's length, a tool that meets two steps but not a narrower step between them in one of its planes, a slot
that splits a step holding a narrower step's shoulder, a tool passing beside a narrower step within its
shoulder's bounding square, and a tube. `Fillet` and `Chamfer` refuse the exact flat's
edges, as they do for an extruded shaft. To blend those edges, place the tool by `decad.Transform`: the faceted
result keeps the shaft's plane and cylinder faces, and the blend builds on them.

A pin drawn at right angles to a plate, on a datum plane, whose end cap lies flat inside one of the plate's faces
combines exactly, with no `Faceted` face. A `Union` of a pin standing on the face from outside adds it, and a `Cut`
of a pin ending in the face from inside leaves a blind hole that opens there; an `Intersect` of that pin is the pin.
The cap and the face must hold the same level exactly. A pin that stops short by any gap, a length that only rounds
onto the face (such as an inch distance), and a section touching the face's edge take the faceted path.

A pin whose two end caps lie flat in two opposite faces combines exactly in the same way. A `Union` of a pin bridging
the gap of a U-shaped plate, from one prong's inner face to the other's, adds the pin with no end cap. A `Cut` of a
pin running from one wall of a plate exactly to the opposite wall drills a through hole, and an `Intersect` of it is
the pin. A `Cut` or `Intersect` of the bridging pin, which touches the plate only in its two end disks, returns a
`BooleanError` with code `BooleanUnsupportedContact`.

A pin that passes through several faces of a plate combines exactly too, such as a pin through both prongs of a
U-shaped plate, a pin across a slotted plate, or a pin that bridges the U's gap and runs on into the far prong. Each
end may lie strictly inside the plate, strictly outside it, or flat in a face. A `Union` adds the pin's parts outside
the plate, and a `Cut` removes its parts inside the plate. An `Intersect` is exact when the pin lies inside the plate
over one stretch only; through two prongs it takes the faceted path. Every face the pin meets must lie across it with
the pin's section strictly inside the face. A section that crosses a face's edge, and a pin or face level that
carries rounding (such as an inch distance), take the faceted path.

A blind pocket `Cut` from either cap of an extruded plate that partly overlaps the plate's one through hole builds
with no `Faceted` face when the plate, the hole and the pocket are drawn on parallel sketch planes with the same
placement and every outline is made of lines along the sketch axes, circles and arcs: a rectangular pocket across a
round hole or a slot, or a round pocket across a square hole. Any circle or arc in the result makes the volume read
`Approximate` with a small proven bound. A round pocket across a round hole, a pocket wall at an angle to the sketch
axes, a pocket edge tangent to the hole, a pocket drawn with lines overshooting its corners, and a pocket that also
crosses the plate's outline take the faceted path.

Blind pockets `Cut` from the two caps of an extruded plate that meet at one level and overlap there in plan build with
no `Faceted` face under the same drawing conditions: every outline is made of lines along the sketch axes, circles and
arcs, such as a round pocket across a rectangular one, a slot across a rectangle, or a counterbore drilled from both
faces. The level must hold no rounding. Any circle or arc in the result makes the volume read `Approximate` with a
small proven bound; rectangles alone read `Exact`. A second pocket on one face that meets nothing of the other
face's pockets at that level takes the faceted path. Pockets that only touch, along an edge, at a corner or where a
circle is tangent to the other pocket's edge, two circles of opposite pockets that meet, and a pocket wall at an angle
to the sketch axes take the faceted path, which refuses the coplanar contact under `ExactOnly` with a `BooleanError`
of code `BooleanUnsupportedContact`.

A `Cut` whose round tool touches a wall of the target at one point without crossing it, such as a round pocket
tangent to a square hole or to the plate's side, takes the faceted path. A blind pocket gives an `Approximate`
faceted result that meshes. A tool through the whole target takes the faceted path too, which may refuse the
contact with a `BooleanError` of code `BooleanUnsupportedContact`, as for a round tool tangent to a box's side, or
build a faceted result, as for one tangent inside a cylinder's wall.

A tapered `decad.Extrude` (`decad.WithTaper`) is an operand when every corner where a circular wall meets its
neighbour is tangent. A line and an arc, or two arcs, that meet tangent up to rounding, turning by an angle whose sine
is at most 2⁻³², count as tangent, and the sliver between them is charged into the result's volume bound. A larger
turn is refused with `decad.ErrUnsupported`, as is a trimmed segment; the body still meshes for export.

Two faceted solids every face of which squares to an axis, such as mesh solids, combine exactly on a grid of their coordinates under `Union`, `Cut` and `Intersect`, any contact between them included. A two-body `Union`, `Cut` or `Intersect` that would be refused because an operand is the mesh result of earlier Booleans is retried from
the source bodies of that chain, when all but one pair combine by exact routes. The result is the same solid. Its
history records the one call, and faces made by the exact steps carry fresh roles, so select them by geometry.

`Cut` and `Intersect` take options. `decad.WithMatchingFaces()` says faces of the two bodies are meant to match
where they meet, such as a cap drawn to touch a cone. A miss of a few last-place steps is then taken as the
match you intended and charged into the result's error bounds; a larger miss is still refused. See
`Example_intersect_matching_faces`.

A boolean never takes a result parameter and never changes an operand. It retires its operands and returns the
result. A geometric failure is a `*decad.BooleanError`; read its `Code` with `errors.As`. Booleans that cannot be
proven exact may return a faceted body whose measurements are `decad.Approximate` with a proven bound.

## Copies, patterns, mirrors and placement

| Function | Effect |
|---|---|
| `decad.Copy` | Returns a new live body and leaves the source live. `decad.WithCopyPlacement` and `decad.WithCopyRotation` place it. A rotated copy of a mesh Boolean result keeps the retained plane, cylinder, sphere and torus surfaces of its faces; its other faces read `Faceted`. On a faceted copy rotated about z, the retained +z and -z cap planes stay exact, so `ToFace` and `MirrorFace` accept them; a retained plane the rotation turns refuses both. |
| `decad.WithChainTolerance` | A `decad.New` option. Sets the coarsest bound, as a length, that a facet of an earlier boolean's result may carry where a later boolean touches it. The default is the pair's own chord tolerance, and a contact on a coarser facet is refused. A boolean that cuts across a facet an earlier cut rimmed, such as a prism window over a bored tube wall, meets it; set the tolerance to a tenth of a millimetre for that, or cut the window first. A value admits contacts up to it and the result's bound grows to match. |
| `decad.WithGapTolerance` | A `decad.New` option. Sets how far, as a length, the evaluator may move a vertex of a faceted result, or the second operand of a boolean whose faces lie in one plane, off its exact place to keep the operation from refusing. The default is 0.01 mm, and the move is added to the result's bound. |
| `decad.WithParams` | A `decad.New` option. Makes the document read parameters from a `sketch/param` table, such as `sketch.World.Params()`. |
| `Document.Param`, `Document.Expr`, `Document.ParamInt` | Read a named parameter, an expression or a whole-number parameter as the value a feature takes. Editing the table and running the builder again rebuilds the model. |
| `Document.Bindings` | Lists every parameter read in call order: name, source text, value and direct dependencies. |
| `decad.ExactOnly` | A `decad.New` option. Refuses any operation that only such a move would allow. |
| `decad.Transform` | Moves a body by an `r3.Transform` and retires the input. |
| `decad.ScaledAt` | Scales a supported body uniformly about an anchor point. A scaled mesh Boolean result keeps the retained plane, cylinder, sphere and torus surfaces of its faces with their radii scaled; its other faces read `Faceted`. Other faceted solids, such as `LoftSections` and `LoftFromPoint` results or a fillet of a mesh Boolean result, scale once their held mesh passes a closure and self-crossing audit; they keep their plane faces and retained cylinder, sphere and torus faces, and their other curved faces read `Faceted`. Sheets are refused. |
| `decad.Patterned` | Returns one body holding every instance of a `decad.PatternSpec`. |
| `decad.PathPatternJoin` | Repeats a body along a `decad.Path` and unites every instance with a target through one `decad.Union`; `decad.PathPatternJoinSpaced` places them by distance. |
| `decad.PathPatternSpaced` | Returns copies of a body at a stated arc-length distance along a `decad.Path`, as `decad.PathPatternCopies` does for a count. |
| `decad.PathPatternCopies` | Returns copies of a body spaced at equal arc length along a `decad.Path`, keeping its orientation, or turned to follow the path with `decad.WithPathFollow`, about its start or a `decad.WithPathPivot` point. |
| `decad.PatternCopies` | Returns the extra instances as separate bodies. |
| `decad.PatternJoin` | Repeats a feature by a `decad.PatternSpec` and joins every instance to a target body in one Union, or two at a time when no route takes the set. |
| `decad.Arrange` | Moves bodies to lie side by side on a flat bed, each resting on its surface and a gap apart; `decad.WithArrangeTurns` lets it turn them a quarter turn to fit and `decad.WithArrangeAngle` turns each to the angle of least footprint. |
| `decad.Mirrored` | Returns the mirror image across a `decad.MirrorPlane` and retires the input. |
| `decad.MirroredCopy` | Returns the mirror image and leaves the source live. |

Pattern layouts are `decad.LinearPattern` and `decad.CircularPattern`. Mirror planes are `decad.MirrorFrame` and
`decad.MirrorFace`. `decad.WithJoin` joins a mirror to its source.

To repeat one feature and join the copies to a base, call `decad.PatternJoin`, or call `decad.PatternCopies` and pass the
feature and its copies to one `decad.Union` with the base. When the feature and the base are extrusions between the same two planes and the
pattern keeps those planes (a turn about the sweep axis, or a slide in the plane), the result is one extrusion of the
joined section, and its bound includes the rounding of the placed copies. A pattern that tilts the planes is refused.

A `Union` or `Patterned` whose extrusions touch at one point without crossing, such as a disc tangent to a plate's side
or to another disc, a boss on a plate's top tangent inside its edge, or a triangle's apex on a square's side, takes the
faceted path and gives an `Approximate` result that meshes. That path can still refuse: three or more tangent
`Patterned` copies in a row return `decad.ErrUnsupported`, because the joined mesh is held more coarsely than the next
pair's chord tolerance. A disc tangent inside the plate at the same height adds nothing and stays exact.

## Sheets and surfaces

A sheet encloses no material, so `Body.Volume` and `Body.Centroid` return `decad.ErrNotSolid` for it.

| Function | Effect |
|---|---|
| `decad.PatchProfile` | Builds a one-face sheet from a profile. |
| `decad.PatchEdges` | Fills closed chains of free edges. |
| `decad.Stitch` | Welds matching free edges across bodies. Stitching a closed sheet yields a solid. |
| `decad.Unstitch` | Splits a body into one single-face sheet per face. |
| `decad.Trim` | Cuts a sheet where it meets a tool and keeps `decad.KeepInside` or `decad.KeepOutside`. |
| `decad.Extend` | Lengthens a sheet's chosen edges until they meet a tool. |

## Selecting topology

Features choose edges and faces with queries.

- `decad.Edges` returns a `*decad.EdgeQuery`. Edge predicates are `decad.Circular`, `decad.Concave`, `decad.Convex`,
  `decad.Free`, `decad.LongerThan`, `decad.ParallelTo`, `decad.EndpointAt`, `decad.CreatedBy` and
  `decad.OuterLoopOf`.
- `decad.Faces` returns a `*decad.FaceQuery`. Face predicates are `decad.Planar`, `decad.Cylindrical`,
  `decad.Facing`, `decad.NormalTo`, `decad.FaceCreatedBy` and `decad.Walls`.
- `EdgeQuery` and `FaceQuery` chain `Or`, `Exactly` and `AtLeast`.
- `decad.CapStart` and `decad.CapEnd` name the two caps of a body as a `decad.FeatureRef`.
- A query that matches nothing, or the wrong count, returns a `*decad.SelectionError` that lists how many
  elements each clause matched.

## Reading bodies

| Reading | Methods |
|---|---|
| Size | `Body.Volume`, `Body.Area`, `Body.Centroid`, `Body.Bounds`, `Body.MassProperties` |
| Position | `Body.LocatePoint`, `Body.DistanceToPoint`, `Face.DistanceToPoint`, `Body.ClosestPoint`, `Face.ClosestPoint` |
| Topology | `Body.Lumps`, `Body.Shells`, `Body.Faces`, `Body.Edges`, `Body.Vertices`, `Face.Loops`, `Edge.Faces` |
| Geometry | `Face.Surface`, `Edge.Curve`, `Face.NormalAt`, `Face.Centroid` (point-loft caps and mesh-solid planar faces), `Edge.Length`, `Vertex.Position` |
| Cross-section | `Body.Section` returns a sketch and its profiles on a plane. A full `Revolve` about a world-axis line sections exactly by a world-axis plane that holds the axis, as the meridian and its mirror, or that is normal to it, as one disk or ring per material interval. The cut must pass no meridian vertex, every cut radius and corner must be a binary64 value, and a plane holding the axis refuses a meridian that meets the axis outside one run of axis lines. A faceted solid every face of which squares to an axis, such as a mesh solid, sections exactly by a plane square to an axis, each region of the cut a profile of the returned sketch. A mesh `Union`, `Cut` or `Intersect` result of two prism, stacked-prism, slab-stack BRep or full `Revolve` bodies sections exactly from its operation history when both operands have exact sections there made of whole lines, arcs and circles. The result is one profile per region with its holes. Every corner where the two sections cross must be a line crossing whose point binary64 holds exactly, and Sketch must prove which operand each piece of the plane lies in; a section that cuts an arc or circle, or two regions that share a corner, refuse. Operands that touch tangentially are refused earlier, by the mesh Boolean itself. `Body.SectionOutline` returns the closed outlines of the verified mesh cut by a plane, for bodies `Body.Section` refuses. `SectionOutline.Clearance` returns the least distance between the material two outlines of one plane enclose, with the closest pair of points, whether the regions overlap, and a bound from both outlines' `OffSurface`. Its `Lower` reading is positive only when the gap is certified. |
| Mesh | `Body.Tessellate` returns a `*decad.Mesh` with its chord error bound. It refuses a body whose cavity or outer shell winds the wrong way, and `Document.Verify` reports such a body as an invalid solid. |

`Body.MassProperties` uses certified source moments for some strictly
contained prism Cuts when they produce tighter readings that agree with the
verified mesh. Other faceted results continue to use their verified mesh.

`Body.LocatePoint` classifies a point in exact arithmetic against an
undisplaced prism, for example an extrude to a stated millimetre distance,
whose profile is made of lines, whole circles with a millimetre radius and
whole arcs whose end lies exactly on the start's radius. A blind pocket, a
boss united onto a plate and other stacked prisms with undisplaced levels and
such profiles answer the same way, and so does the BRep a `Union` builds when
a boss stands flush with the plate's outline or crosses it. A full solid `Revolve` answers
exactly when its meridian is made of whole lines, whole circles and such
whole arcs, its axis is resolved exactly, and its sweep is `FullRevolution`
or a whole turn in degrees; points on the axis are included. Such a body
answers `PointOnBoundary` for a point exactly on its boundary and
`PointInside` or `PointOutside` for a point off it. An extrude to a computed
level (`ToFace`, `ThroughAll`, a distance in another unit that does not
convert exactly, a chamfered end), a prism Boolean result whose section moved,
a pocket crossing a through hole, a partial or radian-stated revolve, every
other body except the mesh Boolean results described next, and a point on an
arc's chord or on two loops at once are classified through the verified mesh
at the given tolerance and can read `PointUndecided` near the boundary.

A mesh `Union`, `Cut` or `Intersect` result first combines its operands'
exact answers through the operation history. A point inside either operand
of a `Union` is inside it; a point inside the target of a `Cut` and on the
tool's boundary is on its boundary; a point outside either operand of an
`Intersect` is outside it. Each operand answers from its own exact record as
described above, and an operand that is itself such a result is read the same
way. A point on the boundary of two operands at once, or where an
operand has no exact answer, falls back to the verified mesh. A result with
any operation diagnostic other than `OpDiagChainAllowanceApplied` (a shifted
retry or an embedding reach, for example), a Boolean that trims a `LoftFromPoint` cone, and a moved or copied
result always read the verified mesh.

Surfaces are `decad.Plane`, `decad.Cylinder`, `decad.Cone`, `decad.Sphere`, `decad.Torus`, `decad.Faceted`,
`decad.NURBSSurface` and others, told apart by `decad.SurfaceKind`. A `decad.Mesh` is an output only. It is never
the body representation.

## Verification

`Document.Verify` checks every live body and returns a `*decad.Report`. Pass options to add checks:

| Option | Check |
|---|---|
| `decad.WithTolerance` | Sets the relative tolerance. |
| `decad.WithMinWallThickness` | Finds walls thinner than a minimum. |
| `decad.WithPullDirection` | Finds undercuts along a mold pull direction. |
| `decad.WithConcaveRadius` | Reports concave corner radii. |
| `decad.WithClearances` | Reports the gaps between bodies. |

`decad.Status` is the verdict, from best to worst: `decad.Sound`, `decad.Suspect`, `decad.Violating`,
`decad.Interfering`, `decad.Unsound`. `decad.Unverified` means nothing was decided.
`Report.Passed` is true only for `decad.Sound`. `Report.ForBody` returns one body's `*decad.BodyReport`.
A `decad.Diagnostic` states each reason a report is not sound, with its `decad.DiagnosticCode`.

## Motion, linkages and contact

- `Document.VerifyMotion` sweeps moving bodies along a `decad.Motion` and reports collisions. Tune it with
  `decad.WithResolution`, `decad.WithMotionTolerance` and `decad.WithMinClearance`.
- A `decad.Linkage` (from `decad.NewLinkage`) joins links with `decad.Revolute` and `decad.Prismatic` joints.
  `Document.VerifyLinkage` checks a `decad.Drive`, and `Document.VerifyJointBox` checks a whole box of joint values.
- `Document.ContactPair` classifies how two posed bodies touch, and `Document.SweepPair` finds first contact along
  two paths.
- The `dynamics` package builds a `dynamics.World` from a document, steps it, and records a `dynamics.Timeline`.

## Import

`decad.ReadSTL`, `decad.ReadOBJ`, `decad.ReadThreeMF`, `decad.ReadSTEP` and `decad.ReadIGES` read a triangle mesh file, a STEP solid or a planar-faced IGES solid B-rep into a `decad.MeshBody`. A `decad.MeshBody` is not a
`decad.Body`: no feature or Boolean takes it. `decad.SolidFromMesh` turns a closed one into a solid body that does take part, after proving the mesh does not cross itself. It reports whether its triangles close into a surface, and its bounds,
area and enclosed volume.

| Reading | Method |
|---|---|
| Connectivity | `MeshBody.Report` counts boundary, non-manifold and misoriented edges, shells and zero-area triangles. |
| Size | `MeshBody.Bounds`, `MeshBody.Area`, `MeshBody.Volume` |
| Triangles | `MeshBody.Vertices`, `MeshBody.Triangles` |
| Position | `MeshBody.LocatePoint`, `MeshBody.DistanceToPoint`, `MeshBody.ClosestPoint` answer exactly for a closed mesh. |

`MeshBody.Volume` is `decad.ErrNotSolid` unless the mesh is closed and wound counter-clockwise seen from outside.

`decad.RepairMesh` returns a mended copy of a mesh body and a `decad.MeshRepairReport` of what changed. It always drops
triangles that carry no surface, and `decad.WithMeshWeld`, `decad.WithMeshOrient` and `decad.WithMeshFillHoles` merge
close vertices, wind shells consistently and close simple holes. Read `MeshBody.Report` on the result to see whether
the mesh closed. `decad.ReduceMesh` thins a mesh body by clustering its vertices into cells of a stated size, and `decad.ReduceMeshWithin` picks the cell for a stated surface distance and proves it. `decad.CollapseMesh` thins a closed mesh by merging its shortest edges to a stated triangle count, keeping it closed and without a proven surface distance, and `decad.RemeshMesh` rebuilds one from near-equilateral triangles of a stated edge length. `decad.RefineMesh` splits the long edges of a mesh body at their
midpoints, so it covers the same surface with shorter edges. `decad.SubDMesh` refines a polygon cage by Catmull-Clark subdivision into a mesh body, a smooth shape drawn by moving the cage's vertices. `decad.GyroidLattice`,
`decad.SchwarzPLattice`, `decad.DiamondLattice`, `decad.NeoviusLattice`, `decad.IWPLattice`, `decad.FischerKochSLattice`, `decad.LidinoidLattice` and `decad.SplitPLattice` fill a box with a gyroid, Schwarz P, diamond, Neovius, IWP, Fischer-Koch S, Lidinoid or Split P lattice, the walls used for print infill, as a closed mesh body, and `decad.Infill` fills a solid body with one, keeping the lattice material inside it and, when `InfillSpec.Skin` is set, a solid skin of that thickness round the part. `decad.ScaleMesh`
stretches a mesh body by a separate factor along each axis about a point; a negative factor mirrors it.
`decad.ScaleBodyMesh` does the same to a solid body, as a mesh, after tessellating it.

## Export

`export.STEP`, `export.IGES`, `export.STL`, `export.OBJ`, `export.ThreeMF` and `export.GLTF` write a body to an `io.Writer` at a chord tolerance. `export.IGES` writes the body's tessellation as planar-faced solid B-reps of an IGES 5.3 file, a solid for each lump with a void shell for each cavity, which `decad.ReadIGES` reads back to the same vertices.
`export.GLTF` writes a glTF 2.0 JSON file with its mesh embedded, in metres.
`export.PNG` draws a body as an orthographic, flat-shaded PNG image from a `export.View`, such as `export.IsometricView`. `export.PNGBodies` draws several bodies together in one image, each in its own colour when `export.View.Colors` gives them. Set `export.View.Outlines` to draw the visible feature edges over the shading, `export.View.HiddenEdges` for the hidden ones as dashes, `export.View.Shadows` for the key light's shadows, `export.View.ShadowSoftness` for soft ones, `export.View.Gloss` for a specular highlight, `export.View.Opacity` for translucent bodies, `export.View.BackColor` for the colour of faces looking away, or `export.View.FieldOfView` for a perspective camera, which draws outlines, hidden edges and shadows too.
`export.GCodePerimeters` writes the outline of each print layer of a body, and rectilinear infill inside it when the spec sets an infill spacing, as G-code from an `export.GCodeSpec` (layer height, line width, filament diameter, feed rate, an optional infill spacing, solid layers, a perimeter inset, a retraction, nozzle and bed temperatures, a fan, a skirt and start and end code). `export.DXF` writes the feature edges of a body as an ASCII DXF drawing along an `export.DrawingView`, with hidden edges on their own dashed layer. `export.DXFViews` lays several views of a body in a row on one sheet. `export.DXF` and its siblings draw a whole circle that faces the view as a `CIRCLE` and an arc as an `ARC`. `export.DXFSheet` draws them at a stated scale on paper of a stated size, inside a border with a title block of a title, up to six labelled fields and the scale, with the overall size of each view, the diameter of each whole circle and the radius of each whole arc as dimensions, and a centre cross on each whole circle and arc when `CentreLines` is set. `export.DXFSection` draws the cut of a body by a plane with its outline and hatching, as a section view.
`export.MeshSTL`, `export.MeshOBJ`, `export.MeshGLTF`, `export.MeshPNG` and `export.MeshDXF` write a `decad.MeshBody` in the same formats, as it is, with no tolerance.
`export.STEP` writes planes, cylinders, cones, spheres and tori as analytic surfaces when every face of the body allows it, and otherwise writes the body's mesh facets. A face whose surface is not proven to be exactly its tagged surface, such as a drafted wall, a loop chamfer's corner cone, a revolved cone or a loft's cap, sends the body to facets.
A fillet's horn torus corner, which closes on its axis at a vertex, such as a pocket mouth's, is written as a torus. A
whole sphere or ring with no edge on it, such as a revolved ball, is written as facets.
`export.STEP` writes a solid with a cavity as a `BREP_WITH_VOIDS` of its outer shell and the cavity shells, and a body of several lumps as one solid per lump.
`export.NewSTEPFile` builds a STEP file in memory. STEP header fields use `export.WithSTEPAuthor`,
`export.WithSTEPName`, `export.WithSTEPOrganization` and `export.WithSTEPTimestamp`.

## Errors

Branch on a sentinel with `errors.Is`.

| Sentinel | Meaning |
|---|---|
| `decad.ErrUnsupported` | The evaluator does not build that combination. |
| `decad.ErrDegenerate` | The input is nil, empty or geometrically degenerate. |
| `decad.ErrNegativeMagnitude` | A length or angle magnitude was negative. |
| `decad.ErrNotFinite` | A number was NaN or infinite. |
| `decad.ErrUnitKind` | A quantity had the wrong unit kind. |
| `decad.ErrNoMatch` | A selector matched nothing. |
| `decad.ErrCardinality` | A selector matched the wrong count. |
| `decad.ErrBooleanFailed` | A boolean could not produce a result. |
| `decad.ErrNotSolid` | A solid-only reading was asked of a sheet. |
| `decad.ErrForeignBody` | A body belongs to another document. |
| `decad.ErrCrossContainer` | One operation read bodies or sketches from different containers. |
| `decad.ErrRetiredBody` | A body was already consumed by a feature. |
| `decad.ErrMalformedMesh`, `decad.ErrMeshTooLarge` | An STL or OBJ file was malformed, or held more than the import limits allow. |
| `decad.ErrForeignProfile`, `decad.ErrStaleProfile`, `decad.ErrInvalidProfile`, `decad.ErrUnrecordableProfile` | A sketch profile was rejected. |

## Test helpers

The `decadtest` package offers assertions such as `decadtest.IsManifold`, `decadtest.HasStatus` and
`decadtest.Agree` for tests that build and verify bodies.
