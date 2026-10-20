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

### Type index

Every exported type of the `decad` package appears in the tables below, grouped by area. The kind column says whether
a type is a struct you read fields from, a sealed interface whose variants are listed, an enumeration, or a
configuration option.

#### Model and topology

| Type | Kind | Holds | Obtained from |
|---|---|---|---|
| `decad.Document` | struct | The live set of bodies and the producer identities behind provenance. | `decad.New` |
| `decad.Body` | struct | An immutable solid or sheet. | A feature function |
| `decad.BodyKind` | enum | `decad.BodySolid` or `decad.BodySheet`. | `Body.Kind` |
| `decad.Lump` | struct | A connected solid piece of a body. | `Body.Lumps` |
| `decad.FaceShell` | struct | A connected set of faces, open or void. | `Lump.Shells`, `Body.Shells` |
| `decad.Face` | struct | A surface bounded by loops. | `Body.Faces`, `FaceShell.Faces` |
| `decad.Loop` | struct | One boundary loop of a face. | `Face.Loops` |
| `decad.CoEdge` | struct | One directed use of an edge by a loop. | `Loop.CoEdges` |
| `decad.Edge` | struct | A curve between two vertices, with its adjacent faces. | `Body.Edges`, `Face.Edges` |
| `decad.Vertex` | struct | A topological point. | `Body.Vertices`, `Edge.Start` |
| `decad.FeatureRef` | struct | The stable role that made a body, face or edge. | `Face.Origins`, `Body.Origin` |
| `decad.HeldTopology` | struct | A body's lump and void counts. | `BodyReport.Topology` |

#### Geometry

| Type | Kind | Holds | Obtained from |
|---|---|---|---|
| `decad.Surface` | sealed interface | Face geometry: one of the surface structs below. | `Face.Surface` |
| `decad.SurfaceKind` | enum | Names a surface variant, such as `decad.KindPlane`. | `Surface.Kind` |
| `decad.Plane` | struct | A frame whose U and V axes span the plane. | `Face.Surface` |
| `decad.Cylinder` | struct | Origin, axis and radius. | `Face.Surface` |
| `decad.Cone` | struct | Origin, axis, radius and half angle. | `Face.Surface` |
| `decad.Sphere` | struct | Center and radius. | `Face.Surface` |
| `decad.Torus` | struct | Center, axis, major radius and minor radius. | `Face.Surface` |
| `decad.NURBSSurface` | struct | Marks a free-form face. The control net is private. | `Face.Surface` |
| `decad.Faceted` | struct | Marks a polygonal face, with a bound on its displacement from the true surface. | `Face.Surface` |
| `decad.Curve` | sealed interface | Edge geometry: one of the curve structs below. | `Edge.Curve` |
| `decad.Line3` | struct | A straight edge between its vertices. | `Edge.Curve` |
| `decad.Circle3` | struct | Center, axis and radius. | `Edge.Curve` |
| `decad.Arc3` | struct | Center, axis and radius, swept counter-clockwise from start to end. | `Edge.Curve` |
| `decad.Ellipse3` | struct | Center, axis, major direction and semi-axes. | `Edge.Curve` |
| `decad.NURBSCurve` | struct | Marks a free-form edge. The control points are private. | `Edge.Curve` |
| `decad.FacetedCurve` | struct | A polygonal edge, with a bound on its displacement from the true curve. | `Edge.Curve` |
| `decad.FilletMiter3` | struct | The intersection edge of two analytic fillet patches. | `Edge.Curve` |
| `decad.Mesh` | struct | Triangles as positions and index triples, with a chord bound. | `Body.Tessellate` |
| `decad.Path` | struct | An immutable ordered spatial curve in millimetres. | `decad.NewPath` |
| `decad.PathSegment` | sealed interface | One segment of a path: `decad.LineTo` or `decad.ArcThrough`. | `Path.Segments` |
| `decad.LineTo` | struct | A straight segment to `End`. | The caller |
| `decad.ArcThrough` | struct | A circular arc through a point to `End`. | The caller |
| `decad.Point2` | struct | A plane-local coordinate in millimetres. | The caller |
| `decad.PoseSegment` | struct | A motion between two placements, relative to the body's current placement. | The caller |

#### Measurements

| Type | Kind | Holds | Obtained from |
|---|---|---|---|
| `decad.Measurement` | struct | A value, its exactness and an absolute error bound. | `Body.Volume`, `Body.Area`, `Edge.Length` |
| `decad.VecMeasurement` | struct | A position or direction with its exactness and bound. | `Body.Centroid`, `Vertex.Position` |
| `decad.Box` | struct | An axis-aligned bound with its own exactness and bound. | `Body.Bounds` |
| `decad.Exactness` | enum | `decad.Exact` or `decad.Approximate`. | `Measurement.Exactness` |
| `decad.MassProperties` | struct | Mass, center and inertia of a solid. | `Body.MassProperties` |
| `decad.InertiaReading` | struct | The symmetric inertia tensor about the mass center in world axes. | `MassProperties.Inertia` |
| `decad.QuantityVec` | struct | Three components of one physical kind. | The caller |
| `decad.MeasuredProfile` | struct | A snapshot of a validated profile for plane-local measurement. | `decad.MeasureProfile` |
| `decad.SecondMoments` | struct | Second moments of area about the plane origin. | `MeasuredProfile.SecondMoments` |
| `decad.PointLocation` | enum | Inside, outside, on the boundary or undecided. | `Body.LocatePoint` |

#### Extents, axes and sections

| Type | Kind | Holds | Obtained from |
|---|---|---|---|
| `decad.Section` | type constraint | A closed `*sketch.Profile` or an open `*sketch.Chain`. | The caller |
| `decad.Extent` | sealed interface | A linear extent for `decad.Extrude`. | The caller |
| `decad.SideExtent` | sealed interface | One side of a `decad.TwoSided`. | The caller |
| `decad.Distance` | struct | A distance and a direction. | The caller |
| `decad.Symmetric` | struct | A distance each way, or in total. | The caller |
| `decad.TwoSided` | struct | An independent extent on each side. | The caller |
| `decad.DistanceSide` | struct | One side of a two-sided extent: a distance. | The caller |
| `decad.ThroughAll` | struct | A sweep to the far side of every live body. | The caller |
| `decad.ThroughAllSide` | struct | The through-all variant for one side of a two-sided extent. | The caller |
| `decad.ToFace` | struct | A sweep that stops at a face of another body, with a signed offset. | The caller |
| `decad.Direction` | enum | `decad.Along` or `decad.Against`. | The caller |
| `decad.AngularExtent` | sealed interface | An angular extent for `decad.Revolve`. | The caller |
| `decad.SideAngular` | sealed interface | One side of a `decad.TwoSidedAngle`. | The caller |
| `decad.FullRevolution` | struct | A full turn. | The caller |
| `decad.AngleExtent` | struct | An angle and a direction. | The caller |
| `decad.SymmetricAngle` | struct | An angle each way, or in total. | The caller |
| `decad.TwoSidedAngle` | struct | An independent angle on each side. | The caller |
| `decad.AngleSide` | struct | One side of a two-sided angle. | The caller |
| `decad.ToFaceAngular` | struct | A revolve that stops at a face of another body. | The caller |
| `decad.Axis` | sealed interface | A revolve or coil axis. | The caller |
| `decad.SketchLine` | struct | An axis line in the sketch plane. | The caller |
| `decad.ConstructionAxis` | struct | A world-space point and a direction. | The caller |
| `decad.EdgeAxis` | struct | A linear edge of a body, selected by query. | The caller |
| `decad.LoftSection` | struct | One recorded section of a smooth `decad.LoftSections` feature. | The caller |
| `decad.ConeTangentEnd` | struct | The cone that the last section of `decad.LoftFacesToConeTangent` touches. | The caller |

#### Selection, patterns and placement

| Type | Kind | Holds | Obtained from |
|---|---|---|---|
| `decad.EdgeSelector` | interface | An unresolved edge query accepted by fillet and chamfer. | `decad.Edges` |
| `decad.EdgeQuery` | struct | Branches of edge predicates plus a count assertion. | `decad.Edges` |
| `decad.EdgePredicate` | struct | One clause of an edge query. | `decad.Convex`, `decad.Free` and others |
| `decad.FaceSelector` | interface | An unresolved face query. | `decad.Faces` |
| `decad.FaceQuery` | struct | Branches of face predicates plus a count assertion. | `decad.Faces` |
| `decad.FacePredicate` | struct | One clause of a face query. | `decad.Planar`, `decad.Walls` and others |
| `decad.SelectionError` | struct | A failed query, with the count after each clause. | A failing feature |
| `decad.PredicateResidual` | struct | The running count after one query clause. | `SelectionError.Residuals` |
| `decad.SelectorKind` | enum | Whether a selector picks edges or faces. | `SelectionError.Kind` |
| `decad.PatternSpec` | sealed interface | A pattern layout. | The caller |
| `decad.LinearPattern` | struct | A direction, a step and a count. | The caller |
| `decad.CircularPattern` | struct | A center, an axis and a count. | The caller |
| `decad.MirrorPlane` | sealed interface | A plane to mirror across. | The caller |
| `decad.MirrorFrame` | struct | A plane given by a frame. | The caller |
| `decad.MirrorFace` | struct | A planar face of a body, selected by query. | The caller |
| `decad.NeutralPlane` | interface | The plane a draft tilts faces about. | The caller |
| `decad.NeutralFace` | struct | A cap face of the body being drafted. | The caller |
| `decad.NeutralFrame` | struct | A plane given by a frame. | The caller |
| `decad.TrimSide` | enum | `decad.KeepOutside` or `decad.KeepInside`. | The caller |
| `decad.ShellSense` | enum | `decad.Inward` or `decad.Outward`. | The caller |
| `decad.OffsetSide` | enum | The side of a sheet an offset takes. | The caller |
| `decad.ThickenSide` | enum | The side of a sheet that receives material. | The caller |

#### Verification

| Type | Kind | Holds | Obtained from |
|---|---|---|---|
| `decad.Report` | struct | Request, body reports, interferences, clearances, diagnostics and status. | `Document.Verify` |
| `decad.VerifyRequest` | struct | The effective settings of one verification call. | `Report.Request` |
| `decad.BodyReport` | struct | One body's status, validity, topology counts and survey results. | `Report.ForBody` |
| `decad.Status` | enum | `decad.Sound`, `decad.Suspect`, `decad.Violating`, `decad.Interfering`, `decad.Unsound` or `decad.Unverified`. | `Report.Status` |
| `decad.Diagnostic` | struct | A structured reason for a verdict. | `Report.Diagnostics` |
| `decad.DiagnosticCode` | enum | A diagnostic's stable reason code. | `Diagnostic.Code` |
| `decad.DiagnosticPair` | struct | Two bodies named in a pair finding. | `Diagnostic.Pair` |
| `decad.ReadingKind` | enum | The bounded quantity a diagnostic carries. | `Diagnostic.Reading` |
| `decad.SurveyKind` | enum | The optional survey a diagnostic concerns. | `Diagnostic.Survey` |
| `decad.ValidityResult` | struct | A body's held-boundary verdict. | `BodyReport.Validity` |
| `decad.ValidityOutcome` | enum | The validity verdict. | `ValidityResult.Outcome` |
| `decad.ScalarReading` | struct | A measurement with its tolerance verdict. | `BodyReport.Area` |
| `decad.VectorReading` | struct | A vector measurement with its tolerance verdict. | `BodyReport` readings |
| `decad.BoundsReading` | struct | A box measurement with its tolerance verdict. | `BodyReport.Bounds` |
| `decad.RegionReadings` | struct | A solid's volume and centroid. | `BodyReport.Region` |
| `decad.ToleranceResult` | struct | One reading's tolerance verdict. | `ScalarReading.Tolerance` |
| `decad.ToleranceState` | enum | A reading's verdict against the relative tolerance. | `ToleranceResult` |
| `decad.WallRequest` | struct | The effective wall-thickness settings. | `VerifyRequest.Wall` |
| `decad.WallResult` | struct | One body's wall survey. | `BodyReport.Wall` |
| `decad.UndercutRequest` | struct | The effective pull direction. | `VerifyRequest.Undercut` |
| `decad.UndercutResult` | struct | One body's undercut survey. | `BodyReport.Undercut` |
| `decad.ConcaveRadiusResult` | struct | One body's concave-radius survey. | `BodyReport.ConcaveRadius` |
| `decad.Coverage` | enum | An undercut survey's face-membership result. | `UndercutResult` |
| `decad.Assessment` | enum | A verdict against a stated survey requirement. | Survey results |
| `decad.ScalarOutcome` | enum | A whole-body scalar survey's result. | Survey results |
| `decad.Interference` | struct | A proven overlap between two bodies, with its volume. | `Report.Interferences` |
| `decad.Clearance` | struct | A measured gap between two bodies. | `Report.Clearances` |
| `decad.BooleanError` | struct | The typed failure of a boolean. | `decad.Union`, `decad.Cut`, `decad.Intersect` |
| `decad.BooleanErrorCode` | enum | The fine reason a boolean failed. | `BooleanError.Code` |

#### Motion, linkages and contact

| Type | Kind | Holds | Obtained from |
|---|---|---|---|
| `decad.Motion` | sealed interface | A one-parameter rigid motion. | The caller |
| `decad.Between` | struct | A motion joining two placements along the shorter screw path. | The caller |
| `decad.RigidDriftSegment` | struct | A center that moves linearly while its orientation rotates. | The caller |
| `decad.PairPath` | interface | One body's motion during a two-body sweep. | The caller |
| `decad.MotionRequest` | struct | The effective settings of a motion check. | `MotionReport.Request` |
| `decad.MotionReport` | struct | The path report of `Document.VerifyMotion`. | `Document.VerifyMotion` |
| `decad.MotionInterval` | struct | The certificate between two adjacent poses. | `MotionReport` |
| `decad.IntervalOutcome` | enum | What a motion interval proves. | `MotionInterval` |
| `decad.Collision` | struct | A proven overlap at one pose. | `MotionReport` |
| `decad.PoseResult` | struct | One evaluated pose and its pair findings. | `MotionReport` |
| `decad.Linkage` | struct | A tree of links joined by joints. | `decad.NewLinkage` |
| `decad.Link` | struct | A rigid set of bodies and the joint that attaches it to its parent. | `Linkage.Ground`, `Linkage.Links` |
| `decad.Joint` | interface | How a link moves against its parent. | `Link` |
| `decad.Revolute`, `decad.Prismatic` | struct | A rotation about an axis, or a translation along a direction. | The caller |
| `decad.RevoluteJoint`, `decad.PrismaticJoint` | struct | The joint a link carries, with its value. | `Link.Revolute`, `Link.Prismatic` |
| `decad.JointLimits` | struct | A joint's working range. | The caller |
| `decad.JointConfiguration` | struct | Every link's joint value and world pose. | `Linkage.Configuration` |
| `decad.LinkageLoop` | struct | A loop-closing joint and the links it ties together. | `Linkage.Close` |
| `decad.LoopBar` | struct | The length that a loop holds a link's two pins apart. | `LinkageLoop` |
| `decad.Drive` | slice | A one-parameter motion of a linkage. | The caller |
| `decad.JointSweep` | struct | The waypoints of one link's joint. | The caller |
| `decad.Schedule` | struct | A drive prepared for repeated pose queries. | `Linkage.Schedule` |
| `decad.LinkagePose` | struct | Every link's pose at one parameter value. | `Linkage.PoseAt` |
| `decad.LinkagePoseResult` | struct | One evaluated pose and its pair findings. | `LinkageReport` |
| `decad.LinkageReport` | struct | The drive report of `Document.VerifyLinkage`. | `Document.VerifyLinkage` |
| `decad.LinkCollision` | struct | A proven overlap at one linkage pose. | `LinkageReport` |
| `decad.JointBox` | slice | A box of joint values. | The caller |
| `decad.JointRange` | struct | One link's range of joint values. | The caller |
| `decad.JointBoxRequest` | struct | The effective settings of a joint-box check. | `JointBoxReport` |
| `decad.JointBoxReport` | struct | The cell subdivision and verdict. | `Document.VerifyJointBox` |
| `decad.JointCell` | struct | One closed cell of joint values. | `JointBoxReport` |
| `decad.JointCellResult` | struct | One leaf cell and its center's findings. | `JointBoxReport` |
| `decad.JointBoxCollision` | struct | A proven overlap at one evaluated center. | `JointBoxReport` |
| `decad.CellOutcome` | enum | What a cell result proves. | `JointCellResult` |
| `decad.ContactRequest` | struct | The maximum position and normal error a manifold may publish. | The caller |
| `decad.ContactReport` | struct | A read-only pair result at two poses. | `Document.ContactPair` |
| `decad.ContactRelation` | enum | The proven relation of the two occupied sets. | `ContactReport` |
| `decad.ContactReason` | enum | Why a relation or manifold is undecided or absent. | `ContactReport` |
| `decad.ContactManifold` | struct | A reduction of the certified contact patch. | `ContactReport` |
| `decad.ContactPoint` | struct | Two boundary witnesses and their normal. | `ContactManifold` |
| `decad.ContactFeature` | struct | An original topological feature. | `ContactPoint` |
| `decad.SweepRequest` | struct | The time search bound and contact resolution. | The caller |
| `decad.SweepReport` | struct | The first certified event or the earliest unresolved span. | `Document.SweepPair` |
| `decad.SweepEvent` | struct | The path relation at one sampled instant. | `SweepReport` |
| `decad.SweepSample` | struct | A query pose and its transferred pair finding. | `SweepReport` |
| `decad.SweepInstant`, `decad.SweepInterval` | struct | A dyadic fraction of the duration, or an interval of it. | `SweepReport` |
| `decad.SweepOutcome` | enum | The certified relation over the path. | `SweepReport` |
| `decad.SweepCause` | enum | Why a continuous claim was not proved. | `SweepReport` |
| `decad.SweepStartPolicy` | enum | What to prove when the bodies start touching. | `SweepRequest` |
| `decad.SweepContactTrack` | struct | The source geometry and affine motion of a touching prefix. | `SweepReport` |
| `decad.SweepDeparture` | struct | Certified separation after an initial touch. | `SweepReport` |
| `decad.SweptBox` | struct | A box enclosing every point a body occupies along a path. | `Document.SweptBox` |

#### Options

Option types configure one feature each. The `With...` constructors in [Building bodies](#building-bodies) and the
other sections return them.

| Type | Configures |
|---|---|
| `decad.ExtrudeOption` | `decad.Extrude` |
| `decad.RevolveOption` | `decad.Revolve` |
| `decad.SweepOption` | `decad.Sweep` |
| `decad.LoftOption`, `decad.ChainLoftOption` | `decad.Loft`, `decad.LoftChain` |
| `decad.CoilOption` | `decad.Coil` |
| `decad.FilletOption`, `decad.ChamferOption`, `decad.FilletChamferOption` | `decad.Fillet`, `decad.Chamfer` |
| `decad.ShellOption` | `decad.Shell` |
| `decad.DraftOption` | `decad.Draft` |
| `decad.OffsetOption`, `decad.ThickenOption` | `decad.Offset`, `decad.Thicken` |
| `decad.CopyOption`, `decad.MirrorOption` | `decad.Copy`, `decad.Mirrored` |
| `decad.SurfaceResultOption` | Every feature that `decad.WithSurfaceResult` reaches. |
| `decad.TessellateOption` | `Body.Tessellate` |
| `decad.Verification` | How much of the tessellation proof runs, set by `decad.WithVerification`. |
| `decad.VerifyOption`, `decad.WallOption` | `Document.Verify`, `decad.WithMinWallThickness` |
| `decad.MotionOption` | `Document.VerifyMotion`, `Document.VerifyLinkage` |
| `decad.JointBoxOption`, `decad.JointOption` | `Document.VerifyJointBox`, joints |
| `decad.DocumentOption` | `decad.New` |

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
