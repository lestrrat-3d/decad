# decad data structures

This page describes the values that decad functions take and return. It starts with an index of every exported type,
then explains the model, topology, geometry, measurement and report structures in prose.
The functions that build and read these values are in the [API reference](api.md).

## Contents

- [Type index](#type-index)
- [Document and body](#document-and-body)
- [Topology](#topology)
- [Surfaces and curves](#surfaces-and-curves)
- [Measurement values](#measurement-values)
- [Meshes](#meshes)
- [Reports](#reports)
- [Paths and profiles](#paths-and-profiles)
- [Linkages](#linkages)

## Type index

Every exported type of the `decad` package appears in the tables below, grouped by area. The kind column says whether
a type is a struct you read fields from, a sealed interface whose variants are listed, an enumeration, or a
configuration option.

### Model and topology

| Type | Kind | Holds | Obtained from |
|---|---|---|---|
| `decad.Document` | struct | The live set of bodies, provenance identities and operation records. | `decad.New` |
| `decad.Container` | struct | A named, nestable group of sketches, planes and bodies of one document. | `Document.Root`, `Container.NewChild` |
| `decad.Binding` | struct | One parameter read: name, source text, value and direct dependencies. | `Document.Bindings` |
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
| `decad.OperationHistory` | struct | An immutable snapshot of successful modeling calls. | `Document.OperationHistory` |
| `decad.Operation` | struct | One public call, its kind, body inputs and outputs. | `OperationHistory.Operations` |
| `decad.OperationInput` | struct | One body argument, role and consumed or referenced use. | `Operation.Inputs` |
| `decad.InputUse` | enum | `InputConsumed` or `InputReferenced`. | `OperationInput.Use` |
| `decad.HistoryCoverage` | enum | `HistoryUnknown` or `HistoryComplete` for a face map. | `FaceHistory.Coverage` |
| `decad.FaceHistory` | struct | Known successor faces and correspondence coverage. | `Operation.FaceSuccessors` |
| `decad.OperationKind` | enum | The public modeling or selector-resolution entry point. | `Operation.Kind` |

### Geometry

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

### Measurements

| Type | Kind | Holds | Obtained from |
|---|---|---|---|
| `decad.Measurement` | struct | A value, its exactness and an absolute error bound. | `Body.Volume`, `Body.Area`, `Edge.Length` |
| `decad.VecMeasurement` | struct | A position or direction with its exactness and bound. | `Body.Centroid`, `Vertex.Position` |
| `decad.Box` | struct | An axis-aligned bound with its own exactness and bound. | `Body.Bounds` |
| `decad.Exactness` | enum | `decad.Exact` or `decad.Approximate`. | `Measurement.Exactness` |
| `decad.MassProperties` | struct | Mass, center and inertia of a solid, with certified source moments used for some contained prism Cuts when they improve the verified-mesh reading. | `Body.MassProperties` |
| `decad.InertiaReading` | struct | The symmetric inertia tensor about the mass center in world axes. | `MassProperties.Inertia` |
| `decad.QuantityVec` | struct | Three components of one physical kind. | The caller |
| `decad.MeasuredProfile` | struct | A snapshot of a validated profile for plane-local measurement. | `decad.MeasureProfile` |
| `decad.SecondMoments` | struct | Second moments of area about the plane origin. | `MeasuredProfile.SecondMoments` |
| `decad.PointLocation` | enum | Inside, outside, on the boundary or undecided. | `Body.LocatePoint` |

### Extents, axes and sections

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
| `decad.LoftEnd` | struct | One end of a `decad.LoftBetween`: a profile or a point. | The caller |
| `decad.LoftSplit` | struct | One recorded segment of one loop and the parameter fractions at which `decad.WithLoftSplits` cuts it. | The caller |
| `decad.ConeTangentEnd` | struct | The cone that the last section of `decad.LoftFacesToConeTangent` touches. | The caller |
| `decad.ClosestPoint` | struct | The nearest point of a body or face to a query point, its distance reading and how far it can lie from the true surface. | `Body.ClosestPoint`, `Face.ClosestPoint` |
| `decad.MeshBody` | struct | An imported triangle mesh: vertices, triangles and what they say. Not a `decad.Body`. | `decad.ReadSTL`, `decad.ReadOBJ`, `decad.ReadThreeMF`, `decad.NewMeshBody` |
| `decad.MeshImportOption` | option set | A limit on `decad.ReadSTL`, `decad.ReadOBJ` or `decad.ReadThreeMF`: `decad.WithMaxMeshTriangles` or `decad.WithMaxMeshBytes`. | The caller |
| `decad.MeshReduceReport` | struct | The counts and the greatest vertex move `decad.ReduceMesh` reports: vertices merged, triangles dropped. | `decad.ReduceMesh` |
| `decad.MeshRepairOption` | option set | A repair step of `decad.RepairMesh`: `decad.WithMeshWeld`, `decad.WithMeshOrient` or `decad.WithMeshFillHoles`. | The caller |
| `decad.MeshRepairReport` | struct | The counts `decad.RepairMesh` changed: vertices welded, triangles dropped and flipped, loops filled or skipped, triangles added. | `decad.RepairMesh` |
| `decad.MeshReport` | struct | The counts a mesh body reads from its triangles: edges by use, shells, zero-area triangles, closed and inverted. | `MeshBody.Report` |
| `decad.PathPatternOption` | option set | A setting on `decad.PathPatternCopies`: `decad.WithPathFollow` turns each copy to follow the path. | The caller |
| `decad.ArrangeOption` | option set | A setting on `decad.Arrange`: `decad.WithArrangeTurns` lets bodies turn a quarter turn to fit. | The caller |
| `decad.ArrangeBed` | struct | The flat area `decad.Arrange` fills: a corner, a width, a depth and a gap. | The caller |
| `decad.ThreadSpec` | struct | One thread: the cylindrical face to cut, the pitch, the hand and an optional length. | The caller |
| `decad.BossSpec` | struct | One boss: the face it stands on, a world point, its diameter and height, an optional pilot and an optional draft. | The caller |
| `decad.BossBore` | struct | A boss's blind pilot hole: diameter and depth from the boss's top, an optional head and an optional thread. | The caller |
| `decad.SectionOutline` | struct | The closed outlines where a body's verified mesh meets a plane, and how far they lie from the true surface. | `Body.SectionOutline` |
| `decad.HoleSpec` | struct | One hole: the face, a world point on it, the diameter, a depth, an optional head, an optional shape, an optional thread and an optional draft. | The caller |
| `decad.HoleThread` | struct | The thread of a tapped hole: pitch, hand and an optional length. | The caller |
| `decad.HoleShape` | sealed interface | The cross-section of a hole that is not round. | The caller |
| `decad.RectangularHole` | struct | A hole with a rectangular bore: width, height and a turn about the hole axis. | The caller |
| `decad.SlotHole` | struct | A hole with a stadium bore: length, width and a turn about the hole axis. | The caller |
| `decad.HoleDepth` | sealed interface | How deep a hole runs. | The caller |
| `decad.ThroughHole` | struct | A hole that runs out of the far side. | The caller |
| `decad.BlindHole` | struct | A hole that stops at a depth below the face. | The caller |
| `decad.HoleHead` | sealed interface | The widening at a hole's mouth. | The caller |
| `decad.Counterbore` | struct | A flat-bottomed cylindrical recess. | The caller |
| `decad.TextSpec` | struct | A line of lettering: the text, the font size, where its baseline starts, an optional tolerance and an optional font. | The caller |
| `decad.EmbossMode` | enum | `decad.EmbossRaised` or `decad.EmbossEngraved`. | The caller |
| `decad.EmbossOption` | option set | A setting on `decad.Emboss`: `decad.WithEmbossDraft` leans the relief's walls. | The caller |
| `decad.Countersink` | struct | A conical recess with an included angle. | The caller |

### Selection, patterns and placement

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

### Verification

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
| `decad.OperationDiagnosticError` | interface | An error that returns structured operation diagnostics. | `errors.As` |
| `decad.OperationError` | struct | An original error plus structured operation reasons. | Modeling calls |
| `decad.OperationDiagnostic` | struct | One operation failure or successful-call warning. | Errors, `Body.OperationDiagnostics` |
| `decad.OperationDiagnosticCode` | enum | A stable failure or warning reason, including shifted Booleans, chain allowances, Stitch merges and positive-reach repairs. | `OperationDiagnostic.Code` |
| `decad.OperationDiagnosticSeverity` | enum | Unknown, warning or failure. | `OperationDiagnostic.Severity` |
| `decad.OperationStage` | enum | The broad stage where a diagnostic occurred. | `OperationDiagnostic.Stage` |
| `decad.OperationSubject` | struct | The public argument location associated with a diagnostic. | `OperationDiagnostic.Subject` |
| `decad.OperationChangeMechanism` | enum | The kind of accepted geometry adjustment. | `OperationChange.Mechanism` |
| `decad.OperationChange` | struct | A bounded geometry adjustment and its allowed reach. | `OperationDiagnostic.Change` |
| `decad.OperationChainDetail` | struct | The three values used by the local chain-depth gate. | `OperationDiagnostic.Chain` |
| `decad.OperationWorkLimit` | struct | An existing resource limit and its observed count. | `OperationDiagnostic.WorkLimit` |

### Motion, linkages and contact

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

### Options

Option types configure one feature each. The `With...` constructors in [Building bodies](api.md#building-bodies) and the
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
| `decad.RibOption` | `decad.Rib`, `decad.Web` |
| `decad.BooleanOption` | `decad.Cut`, `decad.Intersect` |
| `decad.CopyOption`, `decad.MirrorOption` | `decad.Copy`, `decad.Mirrored` |
| `decad.SurfaceResultOption` | Every feature that `decad.WithSurfaceResult` reaches. |
| `decad.TessellateOption` | `Body.Tessellate` |
| `decad.Verification` | How much of the tessellation proof runs, set by `decad.WithVerification`. |
| `decad.VerifyOption`, `decad.WallOption` | `Document.Verify`, `decad.WithMinWallThickness` |
| `decad.MotionOption` | `Document.VerifyMotion`, `Document.VerifyLinkage` |
| `decad.JointBoxOption`, `decad.JointOption` | `Document.VerifyJointBox`, joints |
| `decad.DocumentOption` | `decad.WithGapTolerance`, `decad.ExactOnly`, `decad.WithChainTolerance`, `decad.WithParams` |

## Document and body

A `decad.Document` owns the live set of bodies and records successful public
calls. `Document.OperationHistory` returns an immutable snapshot whose records
keep the input and output body handles. A `decad.Body` is an immutable boundary
representation. It holds topology, the geometry attached to that topology, and
the error bounds of its readings. A body is never a triangle mesh and never
exposes indices; faces, edges and vertices are pointers you read from the body.

`Body.OperationDiagnostics` returns successful-call warnings. A shifted Boolean,
a raised chain allowance and a tolerant Stitch merge each report the adjustment
and its measured bound. The slice is empty when the operation produced no
warning.

## Topology

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

## Surfaces and curves

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

## Measurement values

A `decad.Measurement` is a `units.Value`, a `decad.Exactness` and an absolute error bound. `decad.VecMeasurement` does
the same for a point or direction, and `decad.Box` does it for an axis-aligned bound. A `decad.Approximate` reading is
correct to within its bound.

`decad.MassProperties` holds a mass, a centre and an inertia, each with its bound. `decad.QuantityVec` holds three
components of one physical kind. `decad.Point2` is a plane-local coordinate in millimetres, and `r3.Vec` is a world
coordinate in millimetres.

## Meshes

`Body.Tessellate` returns a `*decad.Mesh`, which is an output. `Mesh.Vertices` and `Mesh.Triangles` hold the triangles
as positions and index triples. `Mesh.Bound` limits how far the mesh can sit from the body's boundary.
`Mesh.BoundaryVerified` and `Mesh.VolumeVerified` say which proofs the mesh carries, and `Mesh.SourceFaces` lists the
faces the triangles come from.

## Reports

`Document.Verify` returns a `*decad.Report`. The report holds the effective `Request`, one `decad.BodyReport` per body,
the `Interferences` and `Clearances` between bodies, the `Diagnostics` that explain any verdict, and an overall
`decad.Status`. A `decad.BodyReport` holds that body's `Status`, `Validity`, `Topology` (lump and void counts), area,
bounds and the optional wall, undercut and concave-radius results. A `decad.Diagnostic` names a
`decad.DiagnosticCode`, the body or pair it concerns, the observed and required values, and a message.

## Paths and profiles

A `*decad.Path` is an immutable curve built by `decad.NewPath` from a start point and `decad.PathSegment` values, such
as `decad.LineTo` and `decad.ArcThrough`. Section-fed features read their 2D input from the `sketch` module: a closed
`*sketch.Profile` or an open `*sketch.Chain`. `decad.MeasureProfile` returns a `decad.MeasuredProfile` for 2D area,
centroid and second moments without building a body.

## Linkages

A `*decad.Linkage` is a tree of `*decad.Link` values rooted at `Linkage.Ground`. Joints join a link to its parent, and
`Linkage.Close` adds a loop-closing joint. A `decad.Drive` lists the `decad.JointSweep` values that move the joints.

