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
- [Parts, exploded views and diagrams](#parts-exploded-views-and-diagrams)
- [Import](#import)
- [Export](#export)
- [Errors](#errors)
- [Test helpers](#test-helpers)

## Packages

| Package | Purpose |
|---|---|
| `decad` | Documents, bodies, features, selectors, measurement and verification. |
| `decad/export` | Writes a body as STEP, IGES, STL, OBJ, 3MF or glTF, draws it as a PNG image, writes a 2D DXF drawing of it, or writes its print layers as G-code. |
| `decad/dynamics` | Rigid-body worlds, stepping and replayable timelines. |
| `decad/parts` | The parts and joints of a build: an exploded view and a connection diagram. |
| `decad/decadtest` | Assertions for tests that build and verify bodies. |

decad builds on three sibling modules. `sketch` solves 2D profiles, `r3` supplies vectors, frames and transforms,
and `units` supplies typed quantities.

## The modeling loop

1. Solve a 2D profile in a `sketch.Sketch`.
2. Create a `decad.Document` with `decad.New`.
3. Turn the profile into a body with a feature command such as `decad.Extrude`, and run it with `Do(ctx)`.
4. Modify or combine bodies with further feature commands.
5. Call `Document.Verify` and gate on `Report.Passed`.
6. Read measurements, tessellate, or export.

The full loop is the executable example in [`example_quickstart_test.go`](../example_quickstart_test.go).

## Conventions

### Immutable bodies

A `*decad.Body` never changes. A feature returns a new body and retires its inputs from the document.
`decad.Offset`, `decad.Copy`, `decad.MirroredCopy` and `decad.PatternCopies` leave their input live.

### Features build and methods ask

A feature is a command. A package function such as `decad.Fillet` takes the bodies and the mandatory inputs and
returns the command. Each optional input is a method that sets it and returns the same command. `Do(ctx)` runs the
command once and returns what the feature builds, usually `(*Body, error)`.

`Do` refuses a nil context before it reads any input, and a canceled context returns `ctx.Err()` unchanged with
nothing built. A command runs once: a second `Do` returns `decad.ErrConsumed`, so build a new command to retry.
Calling an option method twice keeps the second value.

Methods on bodies, faces and edges only read. A read that takes a context, such as `Body.Tessellate` or
`Document.Verify`, takes it first and its options last, as functional options.

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
A feature command is not safe for concurrent use. When two goroutines call `Do` on one command, the feature runs
once and the other call returns `decad.ErrConsumed`.

## Documents and bodies

| Identifier | Meaning |
|---|---|
| `decad.New` | Creates an empty `decad.Document`. |
| `Document.Bodies` | Lists the live bodies. |
| `Document.OperationHistory` | Returns an immutable snapshot of recorded successful modeling calls. |
| `Document.Root`, `Container.NewChild` | Read the document's root container and add named children to a container. A container groups sketches, planes and bodies and changes no geometry. `NewChild` refuses a name a sibling has. |
| `Container.ChildNamed`, `Container.NewUniqueChild` | `ChildNamed` returns the child with that name, adding it when missing. `NewUniqueChild` adds a child and, when a sibling has the name, appends " 2", " 3", ... |
| `Container.Name`, `Container.Parent`, `Container.Children`, `Container.Path` | Read a container's name, parent (nil for the root), children in creation order and its "/"-joined path. |
| `Container.AddSketch`, `Container.AddPlane` | Place a sketch or plane in a container, taking it out of the container it was in. Unplaced ones belong to the root. |
| `Container.Sketches`, `Container.Planes`, `Container.Bodies` | List a container's members in creation order. |
| `Document.ContainerOf`, `Document.ContainerOfSketch` | Read the container of a body or a sketch. |
| `Container.AddBody` | Places a live body in the container, taking it out of the container it was in, and changes nothing else. |
| `Document.VerifyContainers`, `decad.ContainerReport`, `decad.ErrContainerLayout` | Report live bodies left directly in the root and non-root containers holding no bodies beneath them. `Passed` and `Err` summarise the report. |
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
inside `Hole`, `WebPath` and `Emboss` do not create separate history entries. A failed or canceled call creates
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
| `decad.Extrude(d, s, sec, e)` | A prism along the sketch normal, sized by a `decad.Extent`. `ExtrudeCommand.Taper` leans the walls and `ExtrudeCommand.SurfaceResult` builds a sheet. |
| `decad.Revolve(d, s, sec, axis, a)` | A solid of revolution about a `decad.Axis`, sized by a `decad.AngularExtent`. `RevolveCommand.SurfaceResult` builds a sheet. |
| `decad.SheetMetal(d, thickness, innerRadius, width, legs, bends)` | A bent strip of flat legs and circular bends as a solid. `Do` returns the body and a `decad.SheetMetalReport` with its flat pattern length, from the K-factor 0.44 or the one `SheetMetalCommand.KFactor` sets. Results record `OperationSheetMetal`. |
| `decad.Sweep(d, s, sec, path)` | A section moved along a `*decad.Path`. `SweepCommand.Twist` turns the section as it travels, `SweepCommand.MitredJoins` admits corners, `SweepCommand.SectionScale` states a factor per span and `SweepCommand.SurfaceResult` builds a sheet. |
| `decad.Loft(d, s0, p0, s1, p1)` | A ruled solid between two profiles on different planes. `LoftCommand.Alignment` and `LoftCommand.Splits` state the correspondence, and `LoftCommand.SurfaceResult` builds a sheet. |
| `decad.LoftChain(d, s0, c0, s1, c1)` | A ruled sheet between two open chains. |
| `decad.LoftSections(d, sections...)` | A smooth solid through three or more profiles, including fitted-edge sections that turn about the plane origin and a four-arc circular section transitioning to fitted-edge sections. `LoftSectionsCommand.CircularCorrespondence` gives a circular-to-spline loft caller-supplied rational quarter correspondence maps. |
| `decad.LoftFaces(sections)` | A solid through ordered cap faces. `decad.LoftFacesToCone(sections, cone, radial)` ends it with a section tangent to a cone. |
| `decad.LoftFromPoint(d, apex, s, p)` | A solid from an apex point to a profile. |
| `decad.PatchProfile(d, s, p)` | A one-face sheet on the sketch plane from a profile. |
| `decad.Coil(d, s, p, axis, pitch, turns)` | A profile moved along a screw motion, such as a thread or spring. `CoilCommand.LeftHand` turns it left-handed. |
| `decad.ConeFromFace(body, face, far)` | The cone that extends a face's straight meridian. |

### Extents

`decad.Extent` is what `decad.Extrude` accepts: `decad.Distance`, `decad.Symmetric`, `decad.TwoSided`,
`decad.ThroughAll` and `decad.ToFace`. `Extrude` to a planar face of a mesh solid that is tilted from the sweep cuts the extrusion at the face's plane, if the face is a convex outline holding the whole profile seen along the sweep. A `decad.Direction` is `decad.Along` or `decad.Against` the sketch normal.
`decad.Between` joins two placements along a screw path for motion checks.

`decad.AngularExtent` is what `decad.Revolve` accepts: `decad.FullRevolution`, `decad.AngleExtent`,
`decad.SymmetricAngle`, `decad.TwoSidedAngle` and `decad.ToFaceAngular`.

### Axes and paths

A revolve or coil axis is a `decad.SketchLine`, a `decad.ConstructionAxis` or a `decad.EdgeAxis`.
Build a sweep path with `decad.NewPath` and `decad.LineTo`, `decad.ArcThrough`, or `decad.ArcAround` segments.
`ArcAround` takes a center, an oriented axis, and an endpoint. The start and end must have exactly equal nonzero
distances from the center and lie perpendicular to the axis. The positive turn about the axis is less than one full turn.

### Feature methods

Each command declares its own methods. A method called twice keeps the second call.

| Method | Command |
|---|---|
| `Taper` | `ExtrudeCommand`, `CutExtrudeCommand` |
| `Alignment`, `Splits` | `LoftCommand` |
| `CircularCorrespondence` | `LoftSectionsCommand` |
| `MitredJoins`, `SectionScale`, `Twist` | `SweepCommand` |
| `LeftHand` | `CoilCommand` |
| `RimFillet`, `Draft` | `EmbossCommand` |
| `Draft`, `Overrun` | `WebPathCommand` |
| `SurfaceResult` | `ExtrudeCommand`, `RevolveCommand`, `SweepCommand`, `LoftCommand`: build a sheet instead of a solid. |

`LoftChainCommand`, `LoftFromPointCommand`, `LoftFacesCommand`, `PatchProfileCommand` and `CutRevolveCommand`
have no method.

## Modifying bodies

Each of these returns a new body and retires its input.

| Function | Effect |
|---|---|
| `decad.Fillet(b, sel, r)` | Rounds selected edges with a tangent arc of a given radius. On a mesh solid, straight edges that meet end to end on one face at right-angle corners, such as a box's top loop, are rounded together and their arcs meet in a mitre at each corner; three edges meeting at a corner of three perpendicular faces end in a sphere octant. On a loft, a radius that consumes a short line between a circle and a fit spline removes the line: the arc touches the fit spline, or, where the fit spline turns away from the arc, ends at the line's far end and leaves a corner there no sharper than the line's own. A loft's two sections may differ, such as a helical gear outline lofted to the same outline turned about its axis, when they have the same kinds of curves in the same order; each section's selected corner gets its own tangent arc of the radius. `FilletCommand.TangentChain` extends the selection along the edges that continue it tangentially. |
| `decad.Chamfer(b, sel, d)` | Bevels selected edges. `ChamferCommand.Asymmetric` sets a second distance. Selecting the whole outer cap loop of a prism whose section is a smooth chain of fit splines bevels it within a measured ratio of the ideal chamfer, as a cut. Several convex edges of an axis-square mesh solid that share corners are bevelled as the solid less each edge's wedge, and several concave ones whose ends lie in material or open to air as the solid plus each edge's wedge. Selecting both whole outer cap loops of a loft whose two sections differ, such as a helical gear outline lofted to the same outline turned about its axis, bevels its held facets, with or without a centered through-bore: each cap is inset by the distance and joined by a band to the walls cut at that height, and the bore stays sharp. The same holds for such a loft joined to its mirror image, as for a herringbone gear, including after its through-bore. Selecting whole loops of lines and arcs with sharp corners on a flat face of a mesh Boolean result, such as the windows of a cover, bevels them by cutting with a polyhedral cutter, with the Cut's faceted readings. An edge that ends at a tangent junction takes the edges that continue it tangentially, as `ChamferCommand.TangentChain` does. A tangent chain of a mesh Boolean result that runs over several flat faces, such as the rim of a bar rounded about different axes at its two ends or of a slot through its rounded end, is bevelled with one setback as a plane along each line and a cone along each arc, also where it ends on an earlier bevel; a chain whose faces state a displacement is bevelled by cutting with the bevel's section swept along the chain, with the Cut's faceted readings. |
| `decad.FilletChord(b, sel, chord)` | Rounds convex or concave edges between two planar faces, or circular ones between a plane and a surface of revolution, with an arc stated by its chord length. |
| `decad.ChamferAngle(b, sel, d, reference, angle)` | Bevels convex or concave edges between two planar faces, or circular ones between a plane and a surface of revolution, by a distance along a reference face and an angle from that face. |
| `decad.Stretch` | Returns a command that scales a faceted solid, such as a mesh solid, by a separate factor along each world axis about an anchor, and returns a solid. |
| `decad.MoveFace(b, sel, distance)` | Press-pulls one face of a mesh solid whose faces all square to an axis along its normal; a positive distance grows the body and a negative one cuts it, and the walls beside it follow. |
| `decad.Shell(b, sel, thickness)` | Removes the selected faces and lines the rest with a wall. `ShellCommand.FaceThickness` gives the walls facing one world axis direction their own thickness on a mesh solid whose faces all square to an axis, and a second call for the same direction replaces its thickness. `ShellCommand.Sense` picks `decad.Inward`, `decad.Outward` or, for a mesh solid whose faces all square to an axis, `decad.Centered`, a wall half the thickness each side of the surface. `decad.ShellClosed(b, thickness)`, which has the same `Sense` and `FaceThickness` methods, keeps every face and hollows the body into one outer shell around a cavity: for a straight prism, a revolve, or a drilled bar whose hole runs straight through, where the cavity is a ring round a tube lining the hole. Outward, the cavity is the original body and each hole's lining grows into the hole, so a hole whose radius is at most the thickness is `decad.ErrUnsupported`. A mesh solid whose faces all square to an axis shells inward with mitered corners. A convex mesh Boolean result with planar faces (a tilted `decad.Cut` of a plate, say) shells inward with `decad.ShellClosed` into a cavity with one plane face per outer face, unless the thickness drops a face. |
| `decad.Draft(b, sel, neutral, angle)` | Leans selected walls about a `decad.NeutralPlane` (`decad.NeutralFace`). Every wall of a mesh solid with exact vertices that is a straight prism along a world axis, about one of its two caps, is drafted as the tapered extrude of its section. |
| `decad.Hole(b, face, at, diameter, depth)` | Returns a command that cuts a round hole at a point on a planar face (a planar face of a mesh solid tilted from every axis too, along its normal), or radially into a cylindrical one, through or blind (`decad.ThroughHole` or `decad.BlindHole`). `HoleCommand.Head` adds a `decad.Counterbore` or `decad.Countersink`, `HoleCommand.Thread` taps the bore with a `decad.HoleThread` built by `decad.NewHoleThread`, and `HoleCommand.Draft` leans its wall. |
| `decad.HoleShaped(b, face, at, shape, depth)` | Returns a command that cuts a rectangular, slot, regular polygonal or outline hole (`decad.RectangularHole`, `decad.SlotHole`, `decad.PolygonHole` or `decad.OutlineHole`) in place of the round bore. It takes no head, thread or draft. |
| `decad.LookupThread` | Reads a thread designation such as `M10`, `M10x1.25`, `1/4-20`, `#10-24` or `G1/2` into its major diameter and pitch. |
| `decad.Thread(b, face, pitch)` | Returns a command that cuts an ISO metric thread into a cylindrical face, a rod or a bore, from its pitch, along the whole face. `ThreadCommand.Length` stops it after a stated length, `ThreadCommand.LeftHand` makes it left-handed, `ThreadCommand.Form` sets a trapezoidal or buttress groove (`decad.TrapezoidThreadForm` or `decad.ButtressThreadForm`), and `ThreadCommand.LeadIn` or `ThreadCommand.LeadInAngle` chamfers the face's ends first. |
| `decad.Boss(b, face, at, diameter, height)` | Returns a command that raises a round boss on a planar (a planar face of a mesh solid tilted from every axis too) or cylindrical face. `BossCommand.Bore` drills a blind pilot hole built by `decad.NewBossBore`, which may carry a counterbore, a countersink or a thread. `BossCommand.RootFillet` rounds the root, `BossCommand.Draft` leans the wall, `BossCommand.Ribs` braces the root with the gusset ribs of `decad.NewBossRibs`, and `BossCommand.Tilt` leans the boss. |
| `decad.BossShaped(b, face, at, shape, height)` | Returns a command that raises a boss with a rectangular, slot or polygonal section. It takes `BossShapedCommand.Bore`, `BossShapedCommand.Draft` and `BossShapedCommand.Ribs`, and no root fillet or tilt. |
| `decad.CutExtrude(target, s, profiles, e)`, `decad.CutRevolve(target, s, profiles, axis, a)` | Cut a solid by one or more profiles of a sketch swept by an extent or revolved, as one feature with no tool body. Profiles are united and cut once. Results record `OperationCutExtrude` or `OperationCutRevolve`. |
| `decad.WebPath(b, s, path, thickness, e)` | Thickens each line and arc of a run of sketch entities, extrudes it by a `decad.Extent` and joins the ribs to the body in one step. A run of one line or one arc is one rib, and a polyline or a slot-shaped run becomes one rib. `WebPathCommand.Overrun` buries the rib ends in the walls, and `WebPathCommand.Draft` leans the walls. |
| `decad.TextProfiles` | Draws lines of text from a TrueType font (Go Regular by default) as closed sketch profiles for `decad.Emboss`. |
| `decad.Emboss(b, s, profiles, face, mode, height)` | Raises or engraves closed profiles on a planar face (a planar face of a mesh solid tilted from every axis too), or on a cylindrical face from a sketch plane tangent to it. `decad.EmbossRaised` or `decad.EmbossEngraved` picks the mode, `EmbossCommand.Draft` leans the walls and `EmbossCommand.RimFillet` rounds the rim. |
| `decad.Thicken(b, thickness)` | Turns a sheet into a solid. `ThickenCommand.Side` picks the side. |
| `decad.QuarterTurn` | Returns a command that turns a mesh solid by whole quarter turns about a world axis, exchanging coordinates so the result stays exact. |
| `decad.Offset(b, distance)` | Builds a second sheet at a normal distance and leaves the source live; for a mesh solid whose faces all square to an axis it moves every face and builds a second solid. |

## Booleans

A full Revolve with a linear, axially monotone solid-core profile can retain
analytic cone and cylinder walls when cut by a coaxial, full-length prism.
The tool must fit strictly inside the outer walls and meet both end planes
exactly. Tessellation carries bounds for rounding at the two cap rims.

| Function | Result |
|---|---|
| `decad.Union(bodies...)` | The volume enclosed by two or more bodies. `Do` refuses fewer than two with `decad.ErrDegenerate`. |
| `decad.Union(bodies...).RefineTo(maxBound)` | A bevel gear join of three or more bodies (a revolved blank and two or more tooth copies), built once at the bound you ask for and returned as `decad.Union` followed by `decad.Refine` returns it, with the faces of `Union` and one history entry. `Do` returns the joined body or an error with nothing committed. A set that is not such a join (a pair, a union of another kind, a join folded two bodies at a time) is `decad.ErrUnsupported` before anything is built, and a join that cannot meet the bound is `decad.ErrUnsupported` too; the message says to call `decad.Union`, then `decad.Refine`. A bad bound is refused by `Do` and names `RefineTo` as its subject. |
| `decad.Cut(target, tool)` | The target minus the tool. A straight-profile twisted `Sweep` is an admitted tool: a boolean restates its closer stationed boundary, and refuses a crossing or gap narrower than the two operands' held bounds together. A straight through-bore of a loft, including one whose upper section is the lower one turned about the axis, keeps the result a loft, so `CapStart` and `CapEnd` of the result still name its caps. A straight through-bore of a loft joined to its mirror image, such as a herringbone gear, bores both halves. |
| `decad.Intersect(a, b)` | The volume common to both bodies. |
| `decad.Split` | One solid per arranged target cell. |
| `decad.Refine(source, maxBound)` | The same held mesh with a tighter proven surface bound, for mesh Unions of extruded, revolved, swept or coiled circles, then Cuts by extruded circles. A straight-bevel join of a revolved blank and point-Loft teeth is rebuilt with finer cone caps instead. |

A chain of mesh `decad.Union` calls over wire-like solids (extruded cylinders, revolved rings, swept round cords, coils)
publishes a held bound well above its chording where two walls meet nearly tangent, as they do at every bend of a
tube built from straight spans, and `Body.Tessellate` then refuses any tolerance below it. `decad.Cut` by extruded
cylinders, such as a bore, may follow. `decad.Refine(body, maxBound).Do(ctx)` proves how far the held mesh lies from the
true solid those operations denote, facet by facet, and returns a new body, retiring the old one, whose `Tessellate`
publishes that surface bound and succeeds at any tolerance at or above it. No vertex moves; volume, area, vertex and edge readings,
and the bounds a later Boolean reads, stay as they were. A later `decad.Transform` keeps the refined bound, grown by
its rounding, so refine a Union result before placing it. A body made any other way, an already placed Union
result, or a bound that cannot be proven within `maxBound` is `decad.ErrUnsupported`.

A bevel gear join, a `decad.Union` of a revolved blank and copies of one `decad.LoftFromPoint` tooth trimmed by toe and
heel cones, holds up to 0.1 mm because its cone caps stop refining there. `decad.Refine` of that join, before any later
`Cut` or placement, is the one case that moves vertices: it rebuilds the join with finer cone caps until the held bound
is at most `maxBound`, at two to five times the triangles and several seconds more per member. Cuts and unions after it
keep the finer bound. `decad.Union(bodies...).RefineTo(maxBound)` returns the same refined join of a blank and its teeth without
building the 0.1 mm join first.

After ordinary analytic admission misses, a blind prism Cut can use translated
placements with the same stored frame and transform basis when their exact
in-section translation is nonzero. Pure-axial pairs continue through their
preexisting route. The Cut keeps the exact axial offset when deciding which
end of the tool lies inside the target.

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
shoulder's bounding square, and a tube. On a plain shaft, `Fillet` and `Chamfer` of the flat's or keyway's
long edges build from the shaft's exact plane and cylinder when the selection includes every edge where the cut
meets the cylinder: the fillet keeps the exact measurement, and the chamfer's bound carries the crossing's
rounding. A stepped shaft's edges, and other selections, refuse. To blend those, place the tool by `decad.Transform`: the faceted result keeps the
shaft's plane and cylinder faces, and the blend builds on them.

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
to the sketch axes take the faceted path. It classifies the coplanar contact of exact faces, and of two planar faces
held within a bound that lie in one exact plane and whose outlines keep their arrangement under that bound (a flush
pair, such as a tool cap with a true arc far from the other face's edges). Under `ExactOnly` it refuses a coplanar
contact of a curved face, or of a face with a positive bound that is no flush pair, with a `BooleanError` of code
`BooleanUnsupportedContact`.

A `Cut` whose round tool touches a wall of the target at one point without crossing it, such as a round pocket
tangent to a square hole or to the plate's side, takes the faceted path. A blind pocket gives an `Approximate`
faceted result that meshes. A tool through the whole target takes the faceted path too, which may refuse the
contact with a `BooleanError` of code `BooleanUnsupportedContact`, as for a round tool tangent to a box's side, or
build a faceted result, as for one tangent inside a cylinder's wall.

A tapered `decad.Extrude` (`ExtrudeCommand.Taper`) is an operand when every corner where a circular wall meets its
neighbour is tangent. A line and an arc, or two arcs, that meet tangent up to rounding, turning by an angle whose sine
is at most 2⁻³², count as tangent, and the sliver between them is charged into the result's volume bound. A larger
turn is refused with `decad.ErrUnsupported`, as is a trimmed segment; the body still meshes for export.

Two faceted solids every face of which squares to an axis, such as mesh solids, combine exactly on a grid of their
coordinates under `Union`, `Cut` and `Intersect`, any contact between them included. A two-body `Union`, `Cut` or
`Intersect` that would be refused because an operand is the mesh result of earlier Booleans is retried from the
source bodies of that chain. The retry first combines source pairs by analytic routes. A final pair uses the
ordinary Boolean routes; two remaining cut tools may first be united. The result denotes the same solid, and its
history records one call. A face made by the analytic steps names a source body's role only where it lies inside
exactly one source face, so select other faces by geometry.

A two-body `Union` that is still refused because a facet it touches holds an earlier rim's bound is rebuilt: every
`Union` in its operands' history is repeated, in the recorded order and grouping, with rim splitting on
(`decad.WithRimSplit`), and the two rebuilt operands are joined. A cylinder or cone wall holds facets that run its
whole length, so a strut joined to a post leaves its rim's bound along the wall, and the next strut or the plate
the post stands on would touch it; after the rebuild each join touches the wall's own bound wherever the split is
proven. A join that lands on a piece the split cannot prove, such as a sliver beside a rim, is still refused. The
result denotes the same solid, its history records one call, and its faces hold split facets that a later `Fillet`
or `Chamfer` may refuse. A rebuild that needs more than 32 `Union` calls, the final join included, is not tried.

A three-body `Union` over admitted prism sources can also retry its grouping
when its second pair touches a coarse held rim. It joins the last two sources
first, then joins that result with the first source. Both pairs keep their
ordinary proof limits, and a successful call records one operation in the
original input order. The route is specified in
[`prism-boolean §4.7`](../agent-docs/prism-boolean-design.md#47-three-source-prism-pair-retry).

`CutCommand.MatchingFaces` and `IntersectCommand.MatchingFaces` say faces of the two bodies are meant to match
where they meet, such as a cap drawn to touch a cone. A miss of a few last-place steps is then taken as the
match you intended and charged into the result's error bounds; a larger miss is still refused. See
`Example_intersect_matching_faces`.

A boolean never takes a result parameter and never changes an operand. It retires its operands and returns the
result. A geometric failure is a `*decad.BooleanError`; read its `Code` with `errors.As`. Booleans that cannot be
proven exact may return a faceted body whose measurements are `decad.Approximate` with a proven bound.

## Copies, patterns, mirrors and placement

| Function | Effect |
|---|---|
| `decad.Copy` | Returns a command that makes a new live body and leaves the source live. `CopyCommand.Place` takes a `decad.CopyTransform`, a `decad.CopyRotation` or a `decad.CopyLiftTranslation` to place it, and a second call replaces the first. `CopyLiftTranslation{Lift, From, To}` moves a body by the exact world vector `lift·(to − from)`, formed in rationals, and `Verify` reads a faceted copy made this way as its source moved by that vector. A rotated copy of a mesh Boolean result keeps the retained plane, cylinder, sphere and torus surfaces of its faces; its other faces read `Faceted`. On a faceted copy rotated about z, the retained +z and -z cap planes stay exact, so `ToFace` and `MirrorFace` accept them; a retained plane the rotation turns refuses both. |
| `decad.WithChainTolerance` | A `decad.New` option. Sets the coarsest bound, as a length, that a facet of an earlier boolean's result may carry where a later boolean touches it. The default is the pair's own chord tolerance, and a contact on a coarser facet is refused. A boolean that cuts across a facet an earlier cut rimmed, such as a prism window over a bored tube wall, meets it; set the tolerance to a tenth of a millimetre for that, or cut the window first. A value admits contacts up to it and the result's bound grows to match. |
| `decad.WithRimSplit` | A `decad.New` option. Makes the document's mesh booleans split the facets a rim touches near the rim, so a later boolean that crosses them far from the rim reads the face's own bound and not the rim's. Use it with `decad.WithChainTolerance` for a chain of cuts across a curved wall. A later boolean already reads a retained cylinder wall's own departure without a split, so the option matters for a cone, sphere, torus or free-form wall, and for a cylinder whose tag a placement moved off its surface. A facet is split only where an exact distance proves the whole far piece clear of the other operand's boundary by the bounds both operands hold, so a facet beside a contact chain may stay whole and keep the rim's bound. The result holds more facets, and a planar face is never split. `Fillet` and `Chamfer` may refuse a split face. One that succeeds reads the split facets' own bounds, which are proven the same way. Leave the option off for a document that edge-finishes its mesh results. A document without the option still splits when a two-body `Union` is refused because a facet it touches carries an earlier rim's bound: the `Union` rebuilds its recorded `Union` history with splitting on and joins, and only the body it returns holds split facets. |
| `decad.WithGapTolerance` | A `decad.New` option. Sets how far, as a length, the evaluator may move a vertex of a faceted result, or the second operand of a boolean whose faces lie in one plane, off its exact place to keep the operation from refusing. The default is 0.01 mm, and the move is added to the result's bound. |
| `decad.WithParams` | A `decad.New` option. Makes the document read parameters from a `sketch/param` table, such as `sketch.World.Params()`. |
| `Document.Param`, `Document.Expr`, `Document.ParamInt` | Read a named parameter, an expression or a whole-number parameter as the value a feature takes. Editing the table and running the builder again rebuilds the model. |
| `Document.Bindings` | Lists every parameter read in call order: name, source text, value and direct dependencies. |
| `decad.ExactOnly` | A `decad.New` option. Refuses any operation that only such a move would allow. |
| `decad.Transform` | Returns a command that moves a body by an `r3.Transform` and retires the input. |
| `decad.ScaledAt` | Returns a command that scales a supported body uniformly about an anchor point. A scaled mesh Boolean result keeps the retained plane, cylinder, sphere and torus surfaces of its faces with their radii scaled; its other faces read `Faceted`. Other faceted solids, such as `LoftSections` and `LoftFromPoint` results or a fillet of a mesh Boolean result, scale once their held mesh passes a closure and self-crossing audit; they keep their plane faces and retained cylinder, sphere and torus faces, and their other curved faces read `Faceted`. Sheets are refused. |
| `decad.Patterned` | Returns a command that makes one body holding every instance of a `decad.PatternSpec`. |
| `decad.PathPattern` | Returns a command that copies a body along a `decad.Path`. Its `decad.PathStations` argument is a `decad.PathCount`, which spaces a count of instances at equal arc length, or a `decad.PathSpacing`, which places them a stated arc-length distance apart. The copies keep the body's orientation, or turn to follow the path with `PathPatternCommand.Follow`, about its start or the point `PathPatternCommand.FollowAbout` names. |
| `decad.PatternCopies` | Returns a command that makes the extra instances as separate bodies. |
| `decad.Arrange` | Returns a command that moves bodies to lie side by side on a flat bed, each resting on its surface and a gap apart; `ArrangeCommand.Turns` lets it turn them a quarter turn to fit and `ArrangeCommand.Angle` turns each to the angle of least footprint. |
| `decad.Mirrored` | Returns a command that makes the mirror image across a `decad.MirrorPlane` and retires the input. |
| `decad.MirroredJoin` | Returns a command that mirrors a body across one of its own planar walls and joins the image to it, as one body. |
| `decad.MirroredCopy` | Returns a command that makes the mirror image and leaves the source live. |

Each of these is a command: build it, then call `Do(ctx)` once. `PatternCopies`, `PathPattern` and `Arrange` return
`[]*decad.Body`. Pattern layouts are `decad.LinearPattern` and `decad.CircularPattern`. Mirror planes are
`decad.MirrorFrame` and `decad.MirrorFace`.

To repeat one feature and join the copies to a base, call `decad.PatternCopies`, or
`decad.PathPattern` along a path, and pass the base, the feature and its copies to one `decad.Union`. A refused
`decad.Union` leaves every operand live, the copies included; remove them with `decad.Remove`. When the feature and the base are extrusions between the same two planes and the
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
  `decad.Reflex`, `decad.Salient`, `decad.Free`, `decad.LongerThan`, `decad.ParallelTo`, `decad.EndpointAt`,
  `decad.CreatedBy` and `decad.OuterLoopOf`.
- `decad.Convex` and `decad.Concave` read `Edge.IsConvex`, the walked boundary, so a boss root on a `Union` result
  is convex and a hole's rim is concave. `decad.Reflex` and `decad.Salient` read `Edge.Wedge`, the material angle
  across the edge: `Edges(decad.Reflex())` picks boss roots and inside corners. An edge whose wedge
  `Edge.Wedge` reports `decad.WedgeUndecided` matches neither.
- `decad.Faces` returns a `*decad.FaceQuery`. Face predicates are `decad.Planar`, `decad.Cylindrical`,
  `decad.Facing`, `decad.NormalTo`, `decad.FaceCreatedBy` and `decad.Walls`.
- `EdgeQuery` and `FaceQuery` chain `Or`, `Exactly` and `AtLeast`.
- `decad.CapStart` and `decad.CapEnd` name the two caps of a body as a `decad.FeatureRef`.
- After an exact (analytic) `Union`, `Cut` or `Intersect`, `decad.FaceCreatedBy(decad.CapEnd(boss))` still selects a
  face of the result that lies inside the boss's end cap and inside no other operand face on that plane, such as the top of a boss joined
  to a plate. On an exact result, a face two operands share, such as the floor of two flush blocks, names neither operand, and a face on a
  plane parallel to an operand's extrude direction may name none.
- A mesh `Union`, `Cut` or `Intersect` merges adjacent, exactly coplanar, same-facing faces of its two operands into one face, so
  `decad.FaceCreatedBy` with either operand's role selects that one face, and a count such as
  `Faces(decad.Facing(v)).Exactly(2)` sees one face where the operands had two.
- A query that matches nothing, or the wrong count, returns a `*decad.SelectionError` that lists how many
  elements each clause matched.

## Reading bodies

| Reading | Methods |
|---|---|
| Size | `Body.Volume`, `Body.Area`, `Body.Centroid`, `Body.Bounds`, `Body.MassProperties` |
| Position | `Body.LocatePoint`, `Body.DistanceToPoint`, `Face.DistanceToPoint`, `Body.ClosestPoint`, `Face.ClosestPoint` |
| Topology | `Body.Lumps`, `Body.Shells`, `Body.Faces`, `Body.Edges`, `Body.Vertices`, `Face.Loops`, `Edge.Faces` |
| Geometry | `Face.Surface`, `Edge.Curve`, `Face.NormalAt`, `Face.Centroid` (point-loft caps and mesh-solid planar faces), `Edge.Length`, `Edge.Wedge`, `Vertex.Position` |
| Cross-section | `Body.Section` returns a sketch and its profiles on a plane. A matching axial `Loft`, including one with a certified D-shaped or closed line-and-arc-profile through-bore, returns its exact profile at an interior world-XY plane. A full `Revolve` about a world-axis line sections exactly by a world-axis plane that holds the axis, as the meridian and its mirror, or that is normal to it, as one disk or ring per material interval. The cut must pass no meridian vertex, every cut radius and corner must be a binary64 value, and a plane holding the axis refuses a meridian that meets the axis outside one run of axis lines. A faceted solid every face of which squares to an axis, such as a mesh solid, sections exactly by a plane square to an axis, each region of the cut a profile of the returned sketch. A mesh `Union`, `Cut` or `Intersect` result of two prism, stacked-prism, slab-stack BRep or full `Revolve` bodies sections exactly from its operation history when both operands have exact sections there made of whole lines, arcs and circles. The result is one profile per region with its holes. Every corner where the two sections cross must be a line crossing whose point binary64 holds exactly, and Sketch must prove which operand each piece of the plane lies in; a section that cuts an arc or circle, or two regions that share a corner, refuse. Operands that touch tangentially are refused earlier, by the mesh Boolean itself. `Body.SectionOutline` returns the closed outlines of the verified mesh cut by a plane, for bodies `Body.Section` refuses. `SectionOutline.Clearance` returns the least distance between the material two outlines of one plane enclose, with the closest pair of points, whether the regions overlap, and a bound from both outlines' `OffSurface`. For a faceted body, `SectionOutline` charges the largest facet bound near the plane rather than the whole mesh's, so a tolerance below the mesh floor can succeed for a plane away from the coarse facets, and a `MeshBoundError`'s `Floor` from it depends on the plane. Its `Lower` reading is positive only when the gap is certified. |
| Mesh | `Body.Tessellate` returns a `*decad.Mesh` with its chord error bound. It refuses a body whose cavity or outer shell winds the wrong way, and `Document.Verify` reports such a body as an invalid solid. |

`Body.MassProperties` uses certified source moments for some strictly
contained prism Cuts when they produce tighter readings that agree with the
verified mesh. Other faceted results continue to use their verified mesh.

`Body.LocatePoint` classifies a point in exact arithmetic against a prism with
an undisplaced section and exact denoted levels, including distances in other
units. Its profile must be made of lines, whole circles with a millimetre radius and
whole arcs whose end lies exactly on the start's radius. A blind pocket, a
boss united onto a plate and other stacked prisms with undisplaced levels and
such profiles answer the same way, and so does the BRep a `Union` builds when
a boss stands flush with the plate's outline or crosses it. A full solid `Revolve` answers
exactly when its meridian is made of whole lines, whole circles and such
whole arcs, its axis is resolved exactly, and its sweep is `FullRevolution`
or a whole turn in degrees; points on the axis are included. Such a body
answers `PointOnBoundary` for a point exactly on its boundary and
`PointInside` or `PointOutside` for a point off it. An extrude whose computed level lacks an exact denotation, a chamfered end,
a prism Boolean result whose section moved,
a pocket crossing a through hole, a partial or radian-stated revolve, every
other body except the mesh Boolean results described next, and a point on an
arc's chord or on two loops at once are classified through the verified mesh
at the given tolerance and can read `PointUndecided` near the boundary.
`Body.DistanceToPoint` from a point the mesh puts outside, near a bore or slot
of a large body, reads within the mesh bound of the exact distance when a chain
of points clear of the mesh by more than its bound leads out of the body's box.

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

For an unadjusted mesh Cut, Verify can prove non-overlap with an exact same-set
analytic copy of its consumed tool. If a requested gap cannot be measured, the
report has an undecided-clearance diagnostic and no clearance row.

`Document.Verify` checks every live body and returns a `*decad.Report`. Pass options to add checks:

| Option | Check |
|---|---|
| `decad.WithTolerance` | Sets the relative tolerance. |
| `decad.WithMinWallThickness` | Finds walls thinner than a minimum. |
| `decad.WithPullDirection` | Finds undercuts along a mold pull direction. |
| `decad.WithConcaveRadius` | Reports concave corner radii. |
| `decad.WithClearances` | Reports the gaps between bodies. |
| `decad.WithSkippedPair` | Leaves one body pair out of the pair checks; `Report.SkippedPairs` lists it. |

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
  two paths. `Document.ProvePairSeparated` returns whether an exact planar proof certifies a strict gap between two
  posed bodies; false means the caller must use its existing contact check.
- The `dynamics` package builds a `dynamics.World` from a document, steps it, and records a `dynamics.Timeline`.

## Parts, exploded views and diagrams

The `parts` package names the placed bodies of one document and the joints between them. A `parts.Set` is a builder
(`parts.NewSet`, `Set.Add`, `Set.Join`) and edits only itself; every other method is a query that leaves the document
as it found it.

- `Set.Explode` pulls each part away from the part it joins, along the joint's axis or contact normal, and returns
  a `parts.Exploded`: a `Placement` and `Offset` per part, a label `Anchor`, `Leaders` that join the two ends of each
  joint and the part pairs whose exploded boxes still meet. Tune it with `parts.WithExplodeGround` and
  `parts.WithExplodeGap`.
- A `parts.Joint` is a `parts.PinInBore`, `parts.PinInSlot`, `parts.FaceContact`, `parts.Contact`, `parts.Keepout` or
  `parts.Tie`. Its axis is read from the matched cylinder faces; where the faces show none, `Joint.Axis` is used,
  and `ExplodedPart.Pull` says which source gave each direction. `parts.WithFitTolerance`,
  `parts.WithAxisTolerance` and `parts.WithMaxFit` tune the matching.
- `Set.Views` poses each part alone for a part sheet: centred in the assembled orientation, or by `parts.WithPartPose`,
  with its bounds, a label anchor and a `parts.Port` at each joint.
- `Set.BOM` returns the bill of materials: one `parts.BOMRow` per `Spec`, with quantity, material, the volume of one part
  and its mass. `parts.Summary.WriteMarkdown` writes the bill, the joints, the exploded layout and the connection diagram
  as one Markdown page.
- `Set.Explode` also takes stages: `parts.WithExplodeStages` runs ordered `parts.Stage` values, each moving named parts or
  container groups along their joints or a stated direction, by what they must clear (`parts.ClearJoined` or
  `parts.ClearAll`), alone or together. `parts.WithPartGap` sets one part's gap and `parts.WithExplodeScale` scales
  every offset. `Joint.Pull` states the direction a part leaves its neighbour. Each `parts.ExplodedPart` reports its
  stage, direction, travel, clearance and `parts.PathState`, each `parts.Leader` its `Separation`, and
  `Exploded.Hidden` lists the pairs where one part may hide another along a view direction.
- `parts.WithDerivedExplode` derives the removal order and direction of the parts no stage names from their
  tessellations: a part leaves along a joint axis or a world axis once nothing still present blocks it, and parts that
  block each other leave as one rigid group. `parts.WithRemoval` takes a `parts.Removal` (`Direction`, `After`, `Before`,
  `Group`) per part. A pair the tessellation cannot decide counts as free, so the result is a layout, not a proof; a
  part with no free direction is `decad.ErrDegenerate`. A part nothing blocks leaves in the round just before the
  first part it blocks, so it sits next to that part. The layout places the part removed last nearest the assembly
  and each part removed earlier beyond the parts removed after it. Identical parts of one round placed by
  translation end beyond the same parts of their round, so they travel the same distance unless a part removed
  later stands in the way of only some of them; `Exploded.Stages` lists the rounds in that placing order. The
  result sets `Exploded.Derived`
  (layout-grade). Each derived
  `parts.ExplodedPart` reports its `Order` and `Round` and a `Pull` of `AxisFromFaces`, `AxisDeclared`, `PullDeclared`,
  `WorldAxis` or `RemovalDirection`. `Exploded.Blocked` lists each `parts.BlockedPair` that forced the order (mover,
  blocker, direction and the witness point and height), and `Exploded.Undecided` each `parts.UndecidedPair` read as
  free. Identical bodies at different positions share one projection per direction.
- A derived explode of a thin assembly whose layers are not coaxial keeps every layer on its axis. Each explicit stage
  that moves parts along a direction gets a layer of its own, nearest the base. A derived rigid group whose members are
  joined by pins along the spread's axis is stacked along it, the largest footprint nearest the assembly, and
  `ExplodedPart.Stack` gives each member's place. A layer steps along `Spread.Shear` only where a part joined to it,
  not on a shared axis, would be covered. The layers are settled along the axis so no two parts of different layers
  meet. `Exploded.Spread` (a `parts.Spread`) reports whether it ran, the axis, `Shear`, the extents `Width`, `Depth` and
  `Height` and a `View`; each `parts.ExplodedPart` carries its `Layer` and the `Slide` the spread added to its
  `Offset`. `parts.WithoutExplodeSpread` keeps the layers stacked and needs `parts.WithDerivedExplode`,
  else `decad.ErrDegenerate`. Every `Explode` fills `Exploded.Guides`: a `parts.Guide` per moved part, from where it
  assembles on its joint partner (its parent in the joint walk, else the ground) to its `Anchor`, with the `Partner`,
  the `Joint` and the part's `Size` so a caller can skip small parts. Each guide's `Inline` says whether the part sits
  on its partner's line. decad draws nothing; draw the guides as segments
  with a renderer that draws 3D lines, for example solidlens `Scene.Lines`. `Exploded.Hidden` lists a pair only when
  the two boxes' exact projections overlap.
- `Set.Check` judges each declared joint against the faces and bounds of its two parts and lists the pairs of parts whose
  boxes meet with no joint naming them. It never runs `Document.Verify`. Each `parts.JointResult` is `Confirmed`,
  `Contradicted`, `Undecided` or `NotChecked`, and one that is not `Confirmed` carries a typed `parts.Cause`.
  `parts.WithContactTolerance` sets how near parts must come to be listed. A `Contact` or `Keepout` joint is read from
  box gaps and point witnesses; `parts.WithWitnessSearch` also looks for a proven contact in each listed pair, and
  `parts.WithVerifyReport` lets a `Verify` report the caller already has settle what the structural rules leave open.
- `Set.Infer` lists the pin-in-bore and pin-in-slot joints the faces suggest between the parts, with the clearance it
  measured, and `Check` reports those no declared joint covers in `JointReport.Inferred`. Nothing is added to the `Set`.
- `Set.Linkage` builds a `decad.Linkage` from the joints, rooted at a ground part: parts held by fixed joints form a link,
  a turning joint becomes a revolute about its axis and a sliding one a prismatic. It returns the joints it could not place,
  for `Linkage.Close` or for repair, and `Document.VerifyLinkage` checks the result.
- `Set.Graph` returns the connection diagram as nodes and edges, and `Graph.WriteMermaid` writes it as a Mermaid
  flowchart.

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
`decad.SchwarzPLattice`, `decad.DiamondLattice`, `decad.NeoviusLattice`, `decad.IWPLattice`, `decad.FischerKochSLattice`, `decad.LidinoidLattice` and `decad.SplitPLattice` fill a box with a gyroid, Schwarz P, diamond, Neovius, IWP, Fischer-Koch S, Lidinoid or Split P lattice, the walls used for print infill, as a closed mesh body, and `decad.Infill(b, kind, cell, level)` returns a command that fills a solid body with one, keeping the lattice material inside it. `InfillCommand.Resolution` sets the samples to a cell and `InfillCommand.Skin` keeps a solid skin of that thickness round the part. Results record `OperationInfill`. `decad.ScaleMesh`
stretches a mesh body by a separate factor along each axis about a point; a negative factor mirrors it.
`decad.ScaleBodyMesh` does the same to a solid body, as a mesh, after tessellating it.

## Export

`export.STEP`, `export.IGES`, `export.STL`, `export.OBJ`, `export.ThreeMF` and `export.GLTF` write a body to an `io.Writer` at a chord tolerance. `export.IGES` writes the body's tessellation as planar-faced solid B-reps of an IGES 5.3 file, a solid for each lump with a void shell for each cavity, which `decad.ReadIGES` reads back to the same vertices.
`export.GLTF` writes a glTF 2.0 JSON file with its mesh embedded, in metres.
`export.ThreeMFItems` writes several bodies as one 3MF file of named objects, each moved by a rigid `export.ThreeMFItem.Placement`, for a whole build in the assembled pose or pulled apart.
`export.PNG` draws a body as an orthographic, flat-shaded PNG image from a `export.View`, such as `export.IsometricView`. `export.PNGBodies` draws several bodies together in one image, each in its own colour when `export.View.Colors` gives them. Set `export.View.Outlines` to draw the visible feature edges over the shading, `export.View.HiddenEdges` for the hidden ones as dashes, `export.View.Shadows` for the key light's shadows, `export.View.ShadowSoftness` for soft ones, `export.View.Gloss` for a specular highlight, `export.View.Opacity` for translucent bodies, `export.View.BackColor` for the colour of faces looking away, or `export.View.FieldOfView` for a perspective camera, which draws outlines, hidden edges and shadows too.
`export.GCodePerimeters` writes the outline of each print layer of a body, and rectilinear infill inside it when the spec sets an infill spacing, as G-code from an `export.GCodeSpec` (layer height, line width, filament diameter, feed rate, an optional infill spacing, solid layers, a perimeter inset, a retraction, nozzle and bed temperatures, a fan, a skirt and start and end code). `export.DXF` writes the feature edges of a body as an ASCII DXF drawing along an `export.DrawingView`, with hidden edges on their own dashed layer. `export.DXFViews` lays several views of a body in a row on one sheet. `export.DXF` and its siblings draw a whole circle that faces the view as a `CIRCLE` and an arc as an `ARC`. `export.DXFSheet` draws them at a stated scale on paper of a stated size, inside a border with a title block of a title, up to six labelled fields and the scale, with the overall size of each view, the diameter of each whole circle and the radius of each whole arc as dimensions, and a centre cross on each whole circle and arc when `CentreLines` is set. `export.DXFSection` draws the cut of a body by a plane with its outline and hatching, as a section view.
`export.MeshSTL`, `export.MeshOBJ`, `export.MeshGLTF`, `export.MeshPNG` and `export.MeshDXF` write a `decad.MeshBody` in the same formats, as it is, with no tolerance.
`export.STEP` writes planes, cylinders, cones, spheres and tori as analytic surfaces when every face of the body allows it, and otherwise writes the body's mesh facets. A face whose surface is not proven to be exactly its tagged surface, such as a drafted wall, a loop chamfer's corner cone, a revolved cone or a loft's cap, sends the body to facets.
A fillet's horn torus corner, which closes on its axis at a vertex, such as a pocket mouth's, is written as a torus. A
whole sphere or ring with no edge on it, such as a revolved ball, is written as facets.
`export.STEP` writes a solid with a cavity as a `BREP_WITH_VOIDS` of its outer shell and the cavity shells, and a body of several lumps as one solid per lump.
`export.STEPItems` writes several bodies as one STEP assembly, and `export.NewSTEPItemsFile` builds it in memory. Each `export.STEPItem` places a body by a rigid `Placement` under a unique `Name`, and may set a `Color` and a chord `Tolerance`. Each distinct body is one named part, written as `export.STEP` writes it, and each item is one named occurrence, so a receiving CAD system opens named components in their positions. A held faceted body that refuses the call's tolerance is written at its own minimum mesh bound, which keeps all its facets; `export.STEPItemsFile.Parts` reports the tolerance each body used. A non-zero `STEPItem.Tolerance` is used as given and its refusal is returned. `decad.ReadSTEP` does not read an assembly.
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
| `decad.ErrContainerLayout` | `ContainerReport.Err` found a body in the root or an empty container. |
| `decad.ErrRetiredBody` | A body was already consumed by a feature. |
| `decad.ErrConsumed` | `Do` was called on a feature command that already ran. |
| `decad.ErrMalformedMesh`, `decad.ErrMeshTooLarge` | An STL or OBJ file was malformed, or held more than the import limits allow. |
| `decad.ErrForeignProfile`, `decad.ErrStaleProfile`, `decad.ErrInvalidProfile`, `decad.ErrUnrecordableProfile` | A sketch profile was rejected. |

`Tessellate` of a faceted body at a tolerance below its minimum mesh bound returns a `*decad.MeshBoundError`, which
also satisfies `errors.Is(err, decad.ErrUnsupported)`. Read it with `errors.As`: `Floor()` is the smallest accepted
tolerance and `Requested()` the refused one. Every tolerance at or above `Floor()` restates the same held facets.

## Test helpers

The `decadtest` package offers assertions such as `decadtest.IsManifold`, `decadtest.HasStatus` and
`decadtest.Agree` for tests that build and verify bodies.
