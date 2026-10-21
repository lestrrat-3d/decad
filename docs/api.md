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
| `decad.Hole` | Cuts a round hole at a point on a planar face, through or blind, with an optional `decad.Counterbore` or `decad.Countersink` head. |
| `decad.Rib` | Thickens one straight sketch line, extrudes it by an `decad.Extent` and joins it to the body. `decad.WithRibOverrun` buries its ends in the walls. |
| `decad.Web` | Adds one such rib for each of several lines in one step. |
| `decad.Emboss` | Raises or engraves closed profiles on a planar face. `decad.EmbossRaised` or `decad.EmbossEngraved` picks the mode. |
| `decad.Thicken` | Turns a sheet into a solid. `decad.WithThickenSide` picks the side. |
| `decad.Offset` | Builds a second sheet at a normal distance and leaves the source live. |

## Booleans

| Function | Result |
|---|---|
| `decad.Union` | The volume enclosed by two or more bodies. |
| `decad.Cut` | The target minus the tool. |
| `decad.Intersect` | The volume common to both bodies. |
| `decad.Split` | One solid per arranged target cell. |

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
