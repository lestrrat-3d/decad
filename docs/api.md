# decad API reference

This is the user-facing reference for the `decad` module. It names what the library offers and how the pieces fit.
Exact signatures and full behaviour live in the Go doc comments:
[pkg.go.dev/github.com/lestrrat-3d/decad](https://pkg.go.dev/github.com/lestrrat-3d/decad).
Runnable, output-checked usage lives in [`examples/`](../examples).

Root-package identifiers carry the `decad.` prefix. `apitest/api_reference_test.go` checks that each one exists.

## Contents

- [Packages](#packages)
- [The modeling loop](#the-modeling-loop)
- [Conventions](#conventions)
- [Data structures](#data-structures)
- [Documents and bodies](#documents-and-bodies)
- [Building bodies](#building-bodies)
- [Modifying bodies](#modifying-bodies)
- [Booleans](#booleans)
- [Copies, patterns, mirrors and placement](#copies-patterns-mirrors-and-placement)
- [Sheets and surfaces](#sheets-and-surfaces)
- [Selecting topology](#selecting-topology)
- [Reading bodies](#reading-bodies)
- [Verification](#verification)
- [Motion, linkages and contact](#motion-linkages-and-contact)
- [Export](#export)
- [Errors](#errors)
- [Test helpers](#test-helpers)

## Packages

| Package | Purpose |
|---|---|
| `decad` | Documents, bodies, features, selectors, measurement and verification. |
| `decad/export` | Writes a body as STEP, STL, OBJ or 3MF. |
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

## Data structures

### Document and body

A `decad.Document` owns the live set of bodies. A `decad.Body` is an immutable boundary representation. It holds
topology, the geometry attached to that topology, and the error bounds of its readings. A body is never a triangle
mesh and never exposes indices; faces, edges and vertices are pointers you read from the body.

### Topology

Topology is a tree from the body down to its vertices.

```
Body -> Lump -> FaceShell -> Face -> Loop -> CoEdge -> Edge -> Vertex
```

`Body.Lumps` returns the connected solid pieces of a body. `Lump.Shells` returns its `decad.FaceShell` values, and
`FaceShell.Faces` returns the faces of one shell. `FaceShell.IsOpen` is true when the shell has a free edge, and
`FaceShell.IsVoid` is true when the shell bounds an internal cavity. The two questions are independent.

`Face.Loops` returns the boundary loops of a face. `Loop.IsOuter` marks the outer loop, and any other loop is a hole.
`Loop.CoEdges` returns the directed uses of edges in walk order. A `decad.CoEdge` is one use of an edge by a loop. Its
`CoEdge.Start` and `CoEdge.End` follow the loop walk, and `CoEdge.IsForward` says whether that walk agrees with the
edge's own `Edge.Start` to `Edge.End` direction. An edge shared by two faces appears as one `decad.Edge` with two
co-edges, so `Edge.Faces` returns the faces on either side. `Edge.IsFree` marks an edge with only one face, which
is how a sheet's rim shows.

`Body.Faces`, `Body.Edges` and `Body.Vertices` return the same elements as flat lists. `Face.Origins` returns the
`decad.FeatureRef` values that name the feature role that made a face, such as `capStart` or a side wall. Selectors
such as `decad.FaceCreatedBy` match on that role.

### Surfaces and curves

`Face.Surface` returns a `decad.Surface` and `Edge.Curve` returns a `decad.Curve`. Both are sealed sets, so a type
switch needs a default case.

| Surface | Fields |
|---|---|
| `decad.Plane` | `Frame`, an `r3.Frame` whose U and V axes span the plane. |
| `decad.Cylinder` | `Origin` on the axis, unit `Axis` and `Radius`. |
| `decad.Cone` | `Origin`, `Axis` toward growing radius, `Radius` at the origin and `HalfAngle`. |
| `decad.Sphere` | `Center` and `Radius`. |
| `decad.Torus` | `Center`, `Axis`, `Major` radius and `Minor` tube radius. |
| `decad.NURBSSurface` | A free-form face. Its control net is private. |
| `decad.Faceted` | A polygonal face. `Bound` limits how far its polygons sit from the surface they stand for. |

`Surface.Kind` returns the matching `decad.SurfaceKind`, such as `decad.KindPlane` or `decad.KindFaceted`.
Edge curves are `decad.Line3`, `decad.Circle3`, `decad.Arc3`, `decad.Ellipse3`, `decad.NURBSCurve` and
`decad.FacetedCurve`.

### Measurements

A `decad.Measurement` is a `units.Value`, a `decad.Exactness` and an absolute error bound. `decad.VecMeasurement` does
the same for a point or direction, and `decad.Box` does it for an axis-aligned bound. A `decad.Approximate` reading is
correct to within its bound.

`decad.MassProperties` holds a mass, a centre and an inertia, each with its bound. `decad.QuantityVec` holds three
components of one physical kind. `decad.Point2` is a plane-local coordinate in millimetres, and `r3.Vec` is a world
coordinate in millimetres.

### Meshes

`Body.Tessellate` returns a `*decad.Mesh`, which is an output. `Mesh.Vertices` and `Mesh.Triangles` hold the triangles
as positions and index triples. `Mesh.Bound` limits how far the mesh can sit from the body's boundary.
`Mesh.BoundaryVerified` and `Mesh.VolumeVerified` say which proofs the mesh carries, and `Mesh.SourceFaces` lists the
faces the triangles come from.

### Reports

`Document.Verify` returns a `*decad.Report`. The report holds the effective `Request`, one `decad.BodyReport` per body,
the `Interferences` and `Clearances` between bodies, the `Diagnostics` that explain any verdict, and an overall
`decad.Status`. A `decad.BodyReport` holds that body's `Status`, `Validity`, `Topology` (lump and void counts), area,
bounds and the optional wall, undercut and concave-radius results. A `decad.Diagnostic` names a
`decad.DiagnosticCode`, the body or pair it concerns, the observed and required values, and a message.

### Paths and profiles

A `*decad.Path` is an immutable curve built by `decad.NewPath` from a start point and `decad.PathSegment` values, such
as `decad.LineTo` and `decad.ArcThrough`. Section-fed features read their 2D input from the `sketch` module: a closed
`*sketch.Profile` or an open `*sketch.Chain`. `decad.MeasureProfile` returns a `decad.MeasuredProfile` for 2D area,
centroid and second moments without building a body.

### Linkages

A `*decad.Linkage` is a tree of `*decad.Link` values rooted at `Linkage.Ground`. Joints join a link to its parent, and
`Linkage.Close` adds a loop-closing joint. A `decad.Drive` lists the `decad.JointSweep` values that move the joints.

## Documents and bodies

| Identifier | Meaning |
|---|---|
| `decad.New` | Creates an empty `decad.Document`. |
| `Document.Bodies` | Lists the live bodies. |
| `decad.Remove` | Retires a body from its document. |
| `decad.Body` | An immutable solid or sheet. |
| `decad.BodyKind` | `decad.BodySolid` or `decad.BodySheet`. `Body.Kind` and `Body.IsSolid` read it. |

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
| `decad.WithLoftAlignment` | `decad.Loft` |
| `decad.WithMitredJoins`, `decad.WithSectionScale`, `decad.WithSweepTwist` | `decad.Sweep` |
| `decad.WithLeftHand` | `decad.Coil` |
| `decad.WithSurfaceResult` | Section features: build a sheet instead of a solid. |

## Modifying bodies

Each of these returns a new body and retires its input.

| Function | Effect |
|---|---|
| `decad.Fillet` | Rounds selected edges with a tangent arc of a given radius. |
| `decad.Chamfer` | Bevels selected edges. `decad.WithAsymmetricChamfer` sets a second distance. |
| `decad.Shell` | Removes the selected faces and lines the rest with a wall. `decad.WithShellSense` picks `decad.Inward` or `decad.Outward`. |
| `decad.Draft` | Leans selected walls about a `decad.NeutralPlane` (`decad.NeutralFace`). |
| `decad.Thicken` | Turns a sheet into a solid. `decad.WithThickenSide` picks the side. |
| `decad.Offset` | Builds a second sheet at a normal distance and leaves the source live. |

## Booleans

| Function | Result |
|---|---|
| `decad.Union` | The volume enclosed by two or more bodies. |
| `decad.Cut` | The target minus the tool. |
| `decad.Intersect` | The volume common to both bodies. |
| `decad.Split` | One solid per arranged target cell. |

A boolean never takes a result parameter and never changes an operand. It retires its operands and returns the
result. A geometric failure is a `*decad.BooleanError`; read its `Code` with `errors.As`. Booleans that cannot be
proven exact may return a faceted body whose measurements are `decad.Approximate` with a proven bound.

## Copies, patterns, mirrors and placement

| Function | Effect |
|---|---|
| `decad.Copy` | Returns a new live body and leaves the source live. `decad.WithCopyPlacement` and `decad.WithCopyRotation` place it. |
| `decad.Transform` | Moves a body by an `r3.Transform` and retires the input. |
| `decad.ScaledAt` | Scales a supported body uniformly about an anchor point. |
| `decad.Patterned` | Returns one body holding every instance of a `decad.PatternSpec`. |
| `decad.PatternCopies` | Returns the extra instances as separate bodies. |
| `decad.Mirrored` | Returns the mirror image across a `decad.MirrorPlane` and retires the input. |
| `decad.MirroredCopy` | Returns the mirror image and leaves the source live. |

Pattern layouts are `decad.LinearPattern` and `decad.CircularPattern`. Mirror planes are `decad.MirrorFrame` and
`decad.MirrorFace`. `decad.WithJoin` joins a mirror to its source.

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
| Position | `Body.LocatePoint`, `Body.DistanceToPoint`, `Face.DistanceToPoint` |
| Topology | `Body.Lumps`, `Body.Shells`, `Body.Faces`, `Body.Edges`, `Body.Vertices`, `Face.Loops`, `Edge.Faces` |
| Geometry | `Face.Surface`, `Edge.Curve`, `Face.NormalAt`, `Edge.Length`, `Vertex.Position` |
| Cross-section | `Body.Section` returns a sketch and its profiles on a plane. |
| Mesh | `Body.Tessellate` returns a `*decad.Mesh` with its chord error bound. |

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

## Export

`export.STEP`, `export.STL`, `export.OBJ` and `export.ThreeMF` write a body to an `io.Writer` at a chord tolerance.
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
| `decad.ErrRetiredBody` | A body was already consumed by a feature. |
| `decad.ErrForeignProfile`, `decad.ErrStaleProfile`, `decad.ErrInvalidProfile`, `decad.ErrUnrecordableProfile` | A sketch profile was rejected. |

## Test helpers

The `decadtest` package offers assertions such as `decadtest.IsManifold`, `decadtest.HasStatus` and
`decadtest.Agree` for tests that build and verify bodies.
