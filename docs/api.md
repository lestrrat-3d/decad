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
| `decad.Sweep` | A section moved along a `*decad.Path`. |
| `decad.Loft` | A ruled solid between two profiles on different planes. |
| `decad.LoftChain` | A ruled sheet between two open chains. |
| `decad.LoftSections` | A solid through three or more profiles. |
| `decad.LoftFaces` | A solid through ordered cap faces. |
| `decad.LoftFacesToConeTangent` | A face loft that ends tangent to a cone. |
| `decad.LoftFromPoint` | A solid from an apex point to a profile. |
| `decad.LoftBetween` | A solid between two ends, each a profile (`decad.ProfileEnd`) or a point (`decad.PointEnd`). |
| `decad.Coil` | A profile moved along a screw motion, such as a thread or spring. |
| `decad.ConeFromFace` | The cone that extends a face's straight meridian. |

### Extents

`decad.Extent` is what `decad.Extrude` accepts: `decad.Distance`, `decad.Symmetric`, `decad.TwoSided`,
`decad.ThroughAll` and `decad.ToFace`. A `decad.Direction` is `decad.Along` or `decad.Against` the sketch normal.
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
| `decad.Chamfer` | Bevels selected edges. `decad.WithAsymmetricChamfer` sets a second distance. Selecting the whole outer cap loop of a prism whose section is a smooth chain of fit splines bevels it within a measured ratio of the ideal chamfer, as a cut. |
| `decad.FilletChord` | Rounds convex or concave edges between two planar faces with an arc stated by its chord length. |
| `decad.ChamferAngle` | Bevels convex or concave edges between two planar faces by a distance along a reference face and an angle from that face. |
| `decad.Shell` | Removes the selected faces and lines the rest with a wall. `decad.WithShellSense` picks `decad.Inward` or `decad.Outward`. |
| `decad.Draft` | Leans selected walls about a `decad.NeutralPlane` (`decad.NeutralFace`). |
| `decad.Hole` | Cuts a round hole, optionally drafted with `HoleSpec.Draft` or tapped with a `decad.HoleThread`, at a point on a planar face, or radially into a cylindrical one, through or blind, with an optional `decad.Counterbore` or `decad.Countersink` head. A `decad.RectangularHole` or `decad.SlotHole` shape cuts that cross-section instead, without a head. |
| `decad.Thread` | Cuts an ISO metric thread, or a trapezoidal one from a `decad.ThreadForm`, with an optional lead-in chamfer, right- or left-handed, into a cylindrical face, a rod or a bore, from its pitch, along the whole face or a stated length of it. |
| `decad.Boss` | Raises a round boss on a planar face, with an optional blind pilot hole that may carry a counterbore, a countersink or a thread, and a fillet round its root. |
| `decad.Rib` | Thickens one straight sketch line, extrudes it by an `decad.Extent` and joins it to the body. `decad.WithRibOverrun` buries its ends in the walls, and `decad.WithRibDraft` leans its walls. |
| `decad.RibArc` | Thickens one sketch arc and joins it to the body as a rib, like `decad.Rib` for a line. |
| `decad.Web` | Adds one such rib for each of several lines in one step. |
| `decad.WebPath` | Adds one rib for each line and arc of a run, in one step, so a polyline or a slot-shaped run becomes one rib. |
| `decad.TextProfiles` | Draws lines of text from a TrueType font (Go Regular by default) as closed sketch profiles for `decad.Emboss`. |
| `decad.Emboss` | Raises or engraves closed profiles on a planar face. `decad.EmbossRaised` or `decad.EmbossEngraved` picks the mode, `decad.WithEmbossDraft` leans the walls and `decad.WithEmbossRimFillet` rounds the rim. |
| `decad.Thicken` | Turns a sheet into a solid. `decad.WithThickenSide` picks the side. |
| `decad.Offset` | Builds a second sheet at a normal distance and leaves the source live. |

## Booleans

| Function | Result |
|---|---|
| `decad.Union` | The volume enclosed by two or more bodies. |
| `decad.Cut` | The target minus the tool. |
| `decad.Intersect` | The volume common to both bodies. |
| `decad.Split` | One solid per arranged target cell. |

Two full `decad.Revolve` bodies about one axis combine exactly, with no `Faceted` face, when their meridians
are lines, arcs or circles: a sphere cut by a coaxial disk or shaft, or two coaxial spheres, keep their
sphere, torus, cone, cylinder and plane faces (an off-axis circle meridian gives torus faces). A shared arc or
circle span between the operands, or a contact at a pole or an edge, is not joined this way.

A two-body `Union`, `Cut` or `Intersect` that would be refused because an operand is the mesh result of earlier Booleans is retried from
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
| `decad.Copy` | Returns a new live body and leaves the source live. `decad.WithCopyPlacement` and `decad.WithCopyRotation` place it. A rotated copy of a mesh Boolean result keeps the retained plane, cylinder, sphere and torus surfaces of its faces; its other faces read `Faceted`. |
| `decad.WithChainTolerance` | A `decad.New` option. Sets the coarsest bound, as a length, that a facet of an earlier boolean's result may carry where a later boolean touches it. The default is the pair's own chord tolerance, and a contact on a coarser facet is refused. A value admits contacts up to it and the result's bound grows to match. |
| `decad.WithGapTolerance` | A `decad.New` option. Sets how far, as a length, the evaluator may move a vertex of a faceted result, or the second operand of a boolean whose faces lie in one plane, off its exact place to keep the operation from refusing. The default is 0.01 mm, and the move is added to the result's bound. |
| `decad.WithParams` | A `decad.New` option. Makes the document read parameters from a `sketch/param` table, such as `sketch.World.Params()`. |
| `Document.Param`, `Document.Expr`, `Document.ParamInt` | Read a named parameter, an expression or a whole-number parameter as the value a feature takes. Editing the table and running the builder again rebuilds the model. |
| `Document.Bindings` | Lists every parameter read in call order: name, source text, value and direct dependencies. |
| `decad.ExactOnly` | A `decad.New` option. Refuses any operation that only such a move would allow. |
| `decad.Transform` | Moves a body by an `r3.Transform` and retires the input. |
| `decad.ScaledAt` | Scales a supported body uniformly about an anchor point. A scaled mesh Boolean result keeps the retained plane, cylinder, sphere and torus surfaces of its faces with their radii scaled; its other faces read `Faceted`. |
| `decad.Patterned` | Returns one body holding every instance of a `decad.PatternSpec`. |
| `decad.PathPatternSpaced` | Returns copies of a body at a stated arc-length distance along a `decad.Path`, as `decad.PathPatternCopies` does for a count. |
| `decad.PathPatternCopies` | Returns copies of a body spaced at equal arc length along a `decad.Path`, keeping its orientation, or turned to follow the path with `decad.WithPathFollow`, about its start or a `decad.WithPathPivot` point. |
| `decad.PatternCopies` | Returns the extra instances as separate bodies. |
| `decad.PatternJoin` | Repeats a feature by a `decad.PatternSpec` and joins every instance to a target body in one Union. |
| `decad.Arrange` | Moves bodies to lie side by side on a flat bed, each resting on its surface and a gap apart; `decad.WithArrangeTurns` lets it turn them a quarter turn to fit and `decad.WithArrangeAngle` turns each to the angle of least footprint. |
| `decad.Mirrored` | Returns the mirror image across a `decad.MirrorPlane` and retires the input. |
| `decad.MirroredCopy` | Returns the mirror image and leaves the source live. |

Pattern layouts are `decad.LinearPattern` and `decad.CircularPattern`. Mirror planes are `decad.MirrorFrame` and
`decad.MirrorFace`. `decad.WithJoin` joins a mirror to its source.

To repeat one feature and join the copies to a base, call `decad.PatternJoin`, or call `decad.PatternCopies` and pass the
feature and its copies to one `decad.Union` with the base. When the feature and the base are extrusions between the same two planes and the
pattern keeps those planes (a turn about the sweep axis, or a slide in the plane), the result is one extrusion of the
joined section, and its bound includes the rounding of the placed copies. A pattern that tilts the planes is refused.

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
| Geometry | `Face.Surface`, `Edge.Curve`, `Face.NormalAt`, `Edge.Length`, `Vertex.Position` |
| Cross-section | `Body.Section` returns a sketch and its profiles on a plane. `Body.SectionOutline` returns the closed outlines of the verified mesh cut by a plane, for bodies `Body.Section` refuses. |
| Mesh | `Body.Tessellate` returns a `*decad.Mesh` with its chord error bound. |

`Body.MassProperties` uses certified source moments for some strictly
contained prism Cuts when they produce tighter readings that agree with the
verified mesh. Other faceted results continue to use their verified mesh.

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

`decad.ReadSTL`, `decad.ReadOBJ`, `decad.ReadThreeMF` and `decad.ReadSTEP` read a triangle mesh file or a planar STEP solid into a `decad.MeshBody`. A `decad.MeshBody` is not a
`decad.Body`: no feature or Boolean takes it. It reports whether its triangles close into a surface, and its bounds,
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
the mesh closed. `decad.ReduceMesh` thins a mesh body by clustering its vertices into cells of a stated size. `decad.ScaleMesh`
stretches a mesh body by a separate factor along each axis about a point; a negative factor mirrors it.

## Export

`export.STEP`, `export.STL`, `export.OBJ`, `export.ThreeMF` and `export.GLTF` write a body to an `io.Writer` at a chord tolerance.
`export.GLTF` writes a glTF 2.0 JSON file with its mesh embedded, in metres.
`export.PNG` draws a body as an orthographic, flat-shaded PNG image from a `export.View`, such as `export.IsometricView`. `export.PNGBodies` draws several bodies together in one image, each in its own colour when `export.View.Colors` gives them. Set `export.View.Outlines` to draw the visible feature edges over the shading, `export.View.HiddenEdges` for the hidden ones as dashes, or `export.View.FieldOfView` for a perspective camera.
`export.DXF` writes the feature edges of a body as an ASCII DXF drawing along an `export.DrawingView`, with hidden edges on their own dashed layer. `export.DXFViews` lays several views of a body in a row on one sheet. `export.DXFSheet` draws them at a stated scale on paper of a stated size, inside a border with a title block, with the overall size of each view as a dimension.
`export.MeshSTL`, `export.MeshOBJ`, `export.MeshGLTF`, `export.MeshPNG` and `export.MeshDXF` write a `decad.MeshBody` in the same formats, as it is, with no tolerance.
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
