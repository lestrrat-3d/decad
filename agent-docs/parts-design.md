# Parts design

The package lands in the PRs of §13, and each PR carries the part of this document it implements. P1 and E1 have landed: `Kind`, `Material`, `Part`, `Joint`, `Axis`, `Set`, axis matching (§7.1), `Explode`, `Graph` and
`export.ThreeMFItems`. P2 adds `Views`. Everything else here is design only until its PR.

**`github.com/lestrrat-3d/decad/parts` is a generic schematic and exploded view of a build made of separate bodies.**
Its two primary outputs are:

1. **An exploded view.** Every part gets a rigid placement that pulls it away from its neighbour along the joint that
   holds it (a pin out along its axis, a plate off along its contact face), with leader segments that show where each
   part came from and label anchors to name it.
2. **A schematic.** A connection diagram of parts as nodes and typed joints as edges (pin in bore, pin in slot, face
   contact, contact, keepout, tie), plus a view of each part on its own: a pose, its bounds, a label anchor and the
   points where its joints attach.

Around those sit the joint model they are drawn from, a check of that model against the geometry, a bill of materials
and a one-page Markdown summary. Every answer is data: transforms, points, faces, `units.Value` quantities and
`decad.Measurement` readings. Drawing belongs to the consumer (§11).

`parts` knows nothing of how a part is made. Nothing in this design, and nothing in decad, checks whether a part can
be printed or machined (§1).

## Question router

Navigation only; each named section owns its topic.

| Question | Section |
|---|---|
| Why this package, and what it will not do | §1 Purpose and non-goals |
| The name and the imports | §2 Package |
| The exported types and functions | §3 Public API |
| Runnable usage sketches | §4 Worked examples |
| Exploded layout | §5 Exploded view |
| Connection diagram and part views | §6 Schematic |
| Joint axes, checking, inference, the linkage bridge | §7 Joints |
| Bill of materials and summary | §8 Text outputs |
| Which existing code it reuses, and the 3MF writer | §9 Reuse |
| The Module boundary gate | §10 Gate analysis |
| What solidlens and kinetograph need | §11 Consumers |
| Tests, fixtures, CI shards | §12 Test plan |
| PR order, sizes and parallel work | §13 Phasing |
| What may break, and what is unknown | §14 Risks and open questions |
| How the gallery adopts it, and where printing checks live | §15 Gallery adoption |

## 1. Purpose and non-goals

The gallery's crossbow (`decad-gallery` `models/crossbow`, 51 parts in one assembled pose) shows the gap. From
assembled renders its author could not tell what each piece looks like or how the pieces join. The model carries
hand-written stand-ins for what this package provides:

| Gallery workaround | Replaced by |
|---|---|
| `Part{Name, Group, Motion, Body}` and string groups | `parts.Part`, grouped by the body's `Container` (§3.1) |
| no exploded view or connection diagram | `Set.Explode`, `Set.Graph`, `Set.Views` (§5, §6) |
| `designedContacts`, a map of pair names that may touch | declared joints and `Set.Check`'s unexplained-contact list (§7.2) |
| `internal/meshgap`, a boundary-gap screen | `Document.Verify`'s pair partition, read by `Set.Check` (§7.2) |
| joints stated again for each motion check | `Set.Linkage`, built from the same joints (§7.4) |
| `threemf.go`, a private multi-object 3MF writer | `export.ThreeMFItems` (§9.1) |

| Non-goal | Why |
|---|---|
| Manufacturing checks of any process: print orientation, overhangs, supports, bridges, thin strips, bed or build-volume size, plates | They are not decad's business. The gallery keeps them in its own tests or a gallery tool (§15). |
| Instancing (`Component`/`Occurrence`), a part tree with its own transforms | `agent-docs/api-design.md` §13 keeps assemblies a non-goal. A `Set` names placed bodies of one document and adds no placement of its own, as a `Container` does not (`agent-docs/container-design.md`). |
| Proving an assembly sequence | Exploded offsets are a layout. The pull path is checked by boxes only (§5). |
| Snap-fit deflection, stress, fatigue | Structural analysis is outside decad (`agent-docs/missing-features.md` "Also consider"). Joint kinds carry no load model. |
| Drawing, labels, colours, leader lines | The consumer draws (§11). decad returns placements, anchor points and segments. |
| A second geometry kernel | Every geometric reading comes from an existing public query (§9). |

Scope was cut from the original request in four places. Snap fits are not a joint kind: their geometry is a contact
at rest, so they are a `Contact` joint with a name. Press fits are a `PinInBore` with a negative clearance. Lap joints
are not a kind: a lap is a `FaceContact` plus one `PinInBore` per pin, shown as separate edges. Coplanar-face
inference is deferred (§7.3).

## 2. Package

**Import path `github.com/lestrrat-3d/decad/parts`.** A plain package in the root module; no new `go.mod`, so
`agent-docs/modules.md` is unchanged. It is the only new package of this design.

The name is the noun the user and the gallery already use: the crossbow's model is a list of `Part`s, the exploded
view and the schematic are views of those parts, and the bill of materials is a parts list. Rejected names:

| Name | Why not |
|---|---|
| `assembly` | It names the `api-design.md` §13 non-goal and invites instancing requests. |
| `schematic`, `explode` | Each names one of the two primary outputs; both are drawn from the same part and joint model. |
| `bom` | It covers the parts list only. |

**Imports.** Arrows point at what a package imports.

```text
  parts ──► decad ──► sketch ──► r3 ──► units
    │
    └────► decad/internal/jointinfer, decad/internal/explodeplan
```

`parts` does not import `export`. Nothing in `decad`, `export`, `sketch`, `r3` or `units` imports `parts`.

`parts` sits above the root as `dynamics` does (`agent-docs/collision-dynamics-design.md` §1): every geometric fact it
needs is a public query that already states its own proof (`Tessellate`, `Mesh.SourceFaces`, `Face.Surface`,
`Face.NormalAt`, `Body.Bounds`, `Body.Volume`, `Document.Verify`, `Document.SweptBox`, `Document.ContainerOf`). It
never needs a body's private payload, so it adds no root adapter. `api-design.md` §2 keeps types that the root API
names in the root package; the root never names a `parts` type, as it never names a `dynamics` type.

**Builders and queries.** `Set` is a builder like `Linkage` (`api-design.md` §6.3): `Add` and `Join` edit the `Set`,
never the document. Every other entry point is a query: it builds, retires and registers no body, records no
operation history and leaves `Document.Bodies()` as it found it. A `Set` is not safe for concurrent mutation. Queries
on one `Set` may run concurrently while nothing mutates it; each keeps its state in locals.

## 3. Public API

Doc comments below are the intended comments; implementation PRs may tighten wording, never meaning.

### 3.1 Parts and sets

```go
// Kind says how a part is obtained. The zero value is reserved; Add refuses it.
type Kind int

const (
    KindUnset    Kind = iota
    Manufactured // made for this build, by any process
    Purchased    // bought: a band, a bearing, a screw
)

// Material names what a part is made of. Name is a free label ("PETG",
// "steel"). Density is a units.Density, or the zero Value when unknown; the
// bill of materials then omits the mass.
type Material struct {
    Name    string
    Density units.Value
}

// Part is one piece of the build.
type Part struct {
    // Name is unique within its Set and non-empty.
    Name string
    Kind Kind
    // Body is the part as placed in the assembled pose. A Manufactured part
    // MUST have one; a Purchased part may (a modeled band) or may not (a screw
    // that is only listed). Every body of a Set belongs to one document.
    Body     *decad.Body
    Material Material
    // Quantity is how many of a body-less part the build needs. A part with a
    // Body is one piece; its Quantity MUST be 0 or 1.
    Quantity int
    // Spec groups identical parts in the bill of materials ("pin 4 x 60",
    // "rubber band 2 x 60"). Parts sharing a non-empty Spec are one row.
    Spec string
}

// Set is the parts of one build and the joints between them. It is a builder:
// its methods edit the Set, never the document.
type Set struct{ /* private */ }

func NewSet(doc *decad.Document) *Set
func (s *Set) Document() *decad.Document
func (s *Set) Add(p Part) error
func (s *Set) Parts() []Part                     // in Add order
func (s *Set) Part(name string) (Part, error)    // decad.ErrDegenerate for an unknown name
func (s *Set) Join(j Joint) error
func (s *Set) Joints() []Joint                   // in Join order
func (s *Set) Group(name string) (string, error) // the part body's container path, "" for the root
```

`Add` refuses, with `decad.ErrDegenerate`: an empty or repeated name, `KindUnset`, a `Manufactured` part without a
body, a sheet body, a body already in the `Set`, and a `Quantity` outside its rule. A body of another document is
`decad.ErrForeignBody`. Liveness is checked by each query, as `VerifyLinkage` does
(`agent-docs/linkage-check-design.md` §3), so a `Set` built before a later modelling step reports
`decad.ErrRetiredBody` at its next query instead of holding a stale handle silently.

**Group is the container.** A part's group is the path of `Document.ContainerOf(part.Body)`, joined with `/`. There
is no separate group string: the document already owns grouping (`agent-docs/container-design.md`), and a second copy
would drift. A body-less part has the group `""`.

### 3.2 Joints

```go
// JointKind is the geometric form of a connection. The zero value is reserved.
type JointKind int

const (
    JointUnset  JointKind = iota
    PinInBore   // a convex cylinder of A inside a coaxial concave cylinder of B
    PinInSlot   // a convex cylinder of A between two parallel planar faces of B
    FaceContact // planar faces of A and B rest on each other: a lap, a butt
    Contact     // A and B touch by design in an unspecified way: a snap at rest
    Keepout     // A and B must stay MinGap apart
    Tie         // a declared connection with no geometric check: a band on a post
)

// Motion is what the joint lets A do against B. Fixed is the zero value.
type Motion int

const (
    Fixed Motion = iota
    Turns          // about the joint axis
    Slides         // along Joint.Direction, or along a slot
    TurnsAndSlides // about and along the joint axis
)

// Joint connects part A to part B. A is the inserted or moving member (the pin,
// the slider), B the receiver.
type Joint struct {
    Name   string // unique within the Set; empty names get "A~B"
    Kind   JointKind
    A, B   string // part names
    Motion Motion
    // Axis is the author's statement of a PinInBore's or PinInSlot's axis, in
    // assembled world coordinates, or nil. The faces' own axis is used when
    // they show one (§7.1); Axis is the fallback where they do not, and Check
    // reports a declared axis the faces contradict.
    Axis *Axis
    // Clearance is the declared radial clearance of a PinInBore or the
    // declared side clearance of a PinInSlot, a signed Length: negative is a
    // press fit. The zero Value declares none, and Check reports the measured
    // clearance without judging it.
    Clearance units.Value
    // MinGap is a Keepout's least allowed gap, a Length > 0.
    MinGap units.Value
    // Direction is a Slides joint's direction for a FaceContact, in assembled
    // world coordinates. Zero elsewhere.
    Direction r3.Vec
}

// Axis is a line: a point on it and a unit direction.
type Axis struct{ Point, Dir r3.Vec }
```

`Join` refuses with `decad.ErrDegenerate`: an unknown part, `A == B`, `JointUnset`, a repeated name, a `Keepout`
without a positive `MinGap`, a `Motion` the kind cannot have (`Keepout`, `Contact` and `Tie` are `Fixed`;
`FaceContact` with `Slides` needs a nonzero `Direction`), an `Axis` on a kind without one or with a zero direction,
and a geometric kind naming a body-less part. A quantity of the wrong kind is `decad.ErrUnitKind`; a non-finite one
`decad.ErrNotFinite`. `Join` normalises `Axis.Dir`. A second joint between the same two parts needs an explicit `Name`: an empty one
would repeat `A~B`.

### 3.3 Entry points

```go
// Explode places every part along its joints for an exploded view (§5). It
// reads the joints' axes from the faces (§7.1) and needs no Check.
func (s *Set) Explode(ctx context.Context, opts ...ExplodeOption) (*Exploded, error)

func WithExplodeGround(name string) ExplodeOption  // default: the manufactured part of largest volume
func WithExplodeGap(gap units.Value) ExplodeOption // default 10 mm past full disengagement

// Graph is the connection diagram: parts as nodes, joints as typed edges
// (§6.1). r may be nil; with one, each edge carries its check status.
func (s *Set) Graph(r *JointReport) Graph

// Views returns the pose and points for drawing each part alone (§6.2).
func (s *Set) Views(ctx context.Context, opts ...ViewOption) (*Views, error)

func WithPartPose(part string, t r3.Transform) ViewOption // a caller's pose for one part

// Check compares every declared joint with the geometry and lists every pair
// of parts that touches, overlaps or comes within the contact tolerance
// without a joint that explains it (§7.2).
func (s *Set) Check(ctx context.Context, opts ...CheckOption) (*JointReport, error)

// Infer lists the joints the geometry suggests between the parts (§7.3). It
// adds nothing to the Set.
func (s *Set) Infer(ctx context.Context, opts ...CheckOption) ([]InferredJoint, error)

func WithVerifyReport(r *decad.Report) CheckOption     // reuse a Verify the caller ran
func WithFitTolerance(tol units.Value) CheckOption     // default 0.05 mm
func WithContactTolerance(tol units.Value) CheckOption // default 0.05 mm
func WithAxisTolerance(a units.Value) CheckOption      // default 0.5°
func WithMaxFit(fit units.Value) CheckOption           // largest clearance matched; default 1 mm

// BOM is the bill of materials (§8.1).
func (s *Set) BOM(ctx context.Context) (*BOM, error)

// Linkage builds a decad.Linkage from the Set (§7.4).
func (s *Set) Linkage(ground string, r *JointReport) (*decad.Linkage, []Joint, error)

// Summary is the agent-readable text form of the outputs (§8.2). Every field
// but Set is optional.
type Summary struct {
    Set      *Set
    Joints   *JointReport
    Exploded *Exploded
}

func (s Summary) WriteMarkdown(w io.Writer) error
```

`Explode`, `Views` and `Infer` take the matching tolerances through the same options as `Check`
(`WithFitTolerance`, `WithAxisTolerance`, `WithMaxFit`); their option types accept them.

Every entry point refuses a nil context, `Set` or body with `decad.ErrDegenerate`, validates options before reading
`ctx`, and returns `ctx.Err()` unchanged after cancellation with no result. A retired body is `decad.ErrRetiredBody`.
A nil option is `decad.ErrDegenerate`; the last of two equal options wins. `parts` defines no new sentinel: every
refusal is one of `api-design.md` §12's, so callers branch as they already do.

`Summary` has no extension hook. A caller who wants its own sections on the page writes them to the same `io.Writer`
after `WriteMarkdown`.

### 3.4 Result types

```go
type JointReport struct {
    Request     CheckRequest
    Joints      []JointResult   // one per declared joint, in Join order
    Unexplained []PairFinding   // pairs that touch, overlap or come near with no joint
    Undecided   []PairFinding   // pairs Verify could not decide
    Inferred    []InferredJoint // inferred matches no declared joint covers
}

type JointResult struct {
    Joint    Joint
    Status   JointStatus        // Confirmed, Contradicted, Undecided, NotChecked
    Relation decad.ContactRelation
    Gap      *decad.Measurement
    Measured *decad.Measurement // radial or side clearance read from the faces
    Axis     *Axis              // the axis used (§7.1)
    AxisFrom AxisSource         // Faces, Declared or None
    FacesA   []*decad.Face
    FacesB   []*decad.Face
    Reason   string             // why it is not Confirmed; for people, not for branching
}

type InferredJoint struct {
    Joint    Joint // Kind PinInBore or PinInSlot, Motion Fixed, Axis and Clearance as matched
    Measured decad.Measurement
    FacesA   []*decad.Face
    FacesB   []*decad.Face
}

type PairFinding struct {
    A, B     string
    Relation decad.ContactRelation
    Gap      *decad.Measurement // Separated and Touching
    Overlap  *decad.Measurement // Overlapping, when Verify bounded the volume
}
```

Until `Check` lands (P4), `JointReport` holds only `Joints`, and `JointResult` only `Joint` and `Status`: the part `Graph`
reads.

`Exploded` is in §5, `Graph` and `Views` in §6, `BOM` in §8. Enumerations reserve zero as "not evaluated", as
`report.Coverage` does, so a zero record never reads as an answer, and each has a `String` method printing the
lower-case words the examples show.

## 4. Worked examples

Each `// Output:` block is the contract the PR that adds the example meets.

**Exploded view and diagram, P1** (`examples/parts_explode_example_test.go`). A 20 x 20 x 10 mm plate has a Ø6.4
bore, and a Ø6 pin runs through it from z = −2 to z = 15. The pin is joined as a `PinInBore`. `Explode` reads the
axis from the two cylinder faces and pulls the pin out along +Z: that way it travels 12 mm to clear the plate and the
other way 15 mm (E3), and it moves 10 mm farther (E4).

```go
package examples_test

import (
    "context"
    "fmt"
    "os"

    "github.com/lestrrat-3d/decad"
    "github.com/lestrrat-3d/decad/parts"
    "github.com/lestrrat-3d/r3"
)

// A Set names the parts of a build and the joints between them. Explode pulls
// each part away from its neighbour along the joint that holds it, and Graph
// draws which part joins which, as a Mermaid diagram.
func Example_parts_explode() {
    ctx := context.Background()
    doc := decad.New()
    plate, err := extrudeBlock(doc, 0, 0, 20, 20, 0, 10)
    if err != nil {
        fmt.Printf("failed to build the plate: %s\n", err)
        return
    }
    bore, err := extrudeCircle(doc, r3.Vec{X: 10, Y: 10}, 3.2, -1, 12)
    if err != nil {
        fmt.Printf("failed to build the bore tool: %s\n", err)
        return
    }
    if plate, err = decad.Cut(ctx, plate, bore); err != nil {
        fmt.Printf("failed to bore the plate: %s\n", err)
        return
    }
    pin, err := extrudeCircle(doc, r3.Vec{X: 10, Y: 10}, 3, -2, 17)
    if err != nil {
        fmt.Printf("failed to build the pin: %s\n", err)
        return
    }

    set := parts.NewSet(doc)
    for _, p := range []parts.Part{
        {Name: "plate", Kind: parts.Manufactured, Body: plate},
        {Name: "pin", Kind: parts.Manufactured, Body: pin},
    } {
        if err := set.Add(p); err != nil {
            fmt.Printf("failed to add %s: %s\n", p.Name, err)
            return
        }
    }
    if err := set.Join(parts.Joint{Kind: parts.PinInBore, A: "pin", B: "plate", Motion: parts.Turns}); err != nil {
        fmt.Printf("failed to join: %s\n", err)
        return
    }
    ex, err := set.Explode(ctx, parts.WithExplodeGround("plate"))
    if err != nil {
        fmt.Printf("failed to explode: %s\n", err)
        return
    }
    for _, p := range ex.Parts {
        fmt.Printf("%s: offset (%g, %g, %g)\n", p.Name, p.Offset.X, p.Offset.Y, p.Offset.Z)
    }
    if err := set.Graph(nil).WriteMermaid(os.Stdout); err != nil {
        fmt.Printf("failed to write the diagram: %s\n", err)
        return
    }
    // Output:
    // plate: offset (0, 0, 0)
    // pin: offset (0, 0, 22)
    // flowchart LR
    //   plate["plate"]
    //   pin["pin"]
    //   pin -- "pin in bore, turns" --> plate
}
```

`extrudeCircle(doc, centre, radius, z0, height)` joins `extrudeBlock` in `examples/helpers_test.go` in P1. The exact
Mermaid text is fixed by P1's `WriteMermaid` tests (§6.1); the lines above are its shape for a set whose parts all sit
in the root container.

**Joint check, P4** (`examples/parts_joints_example_test.go`). The same pin and plate, joined with
`Clearance: units.Millimeters(0.2)` and checked with `Set.Check`, printing `pin~plate: confirmed, radial clearance
0.20 mm` and `0 unexplained contacts`.

## 5. Exploded view

```go
type Exploded struct {
    Ground   string
    Parts    []ExplodedPart // in Set order, body-less parts omitted
    Leaders  []Leader
    Overlaps [][2]string    // part pairs whose exploded boxes still meet
}

type ExplodedPart struct {
    Name      string
    Offset    r3.Vec       // translation after the assembled pose
    Placement r3.Transform // the same, as a transform
    Depth     int          // joint steps from the ground; -1 when no joint reaches it
    Anchor    r3.Vec       // label anchor: exploded box centre
    Pull      PullSource   // AxisFromFaces, AxisDeclared, FaceNormal, BoxCentres, None
}

type Leader struct {
    Joint    string
    From, To r3.Vec // exploded coordinates: the port on A, the port on B
}
```

| Rule | Decision |
|---|---|
| E1 | The joint graph is walked breadth-first from the ground, joints in `Join` order, so each part has one parent joint; a part reached twice keeps its first parent. Parts no joint reaches keep a zero offset and `Depth` −1. |
| E2 | A part's direction from its parent: a `PinInBore` or `PinInSlot` pulls along its axis, from the faces when §7.1 matches them, else the declared `Axis`, else as a `Contact`; a `FaceContact` along the contact face's normal, away from the parent; `Contact` and `Tie` along the line between the two box centres. `Pull` records which was used, so a viewer can flag a guessed direction. |
| E3 | For each sense along the direction, the travel is the distance that takes the part's box, at its parent's offset, clear of the exploded box of its parent and of every other part it is joined to that the walk has already placed. A sense is free when `Document.SweptBox` along that travel is `StrictlyDisjoint` from the box of every part not joined to it, each at its current offset (a part not yet placed sits where it was assembled); strictly disjoint boxes prove the path clear. Parts joined to the moving part never block it. A free sense beats a blocked one, then the shorter travel wins, then the sense whose largest direction component is positive. |
| E4 | The offset is the chosen travel plus `WithExplodeGap`, along the chosen sense, added to the parent's offset. A `FaceContact` has one sense, away from the parent. |
| E5 | `Leaders` join the two ports of each joint (§6.2 V3) in exploded coordinates, so a consumer can draw the dotted line a pin travels. |
| E6 | `Overlaps` reports exploded box pairs that still meet. It is advice for the consumer, not a proof. |

A matched axis carries its point at the middle of the engagement, which is the joint's port (V3). `Explode` needs no `Verify`. It reads faces, boxes and swept boxes only, so it works on a model whose pair check is
slow or undecided (§14).

## 6. Schematic

### 6.1 Connection diagram

```go
type Graph struct {
    Nodes []Node // in Set order
    Edges []Edge // in Join order
}

type Node struct {
    Part, Group string
    Kind        Kind
}

type Edge struct {
    Joint  string
    A, B   string
    Kind   JointKind
    Motion Motion
    Status JointStatus // NotEvaluated when Graph was given no report
}

func (g Graph) WriteMermaid(w io.Writer) error
```

`WriteMermaid` writes a left-to-right `flowchart`: one subgraph per non-root group, a rectangle for a manufactured
part and a rounded node for a purchased one, and one edge per joint labelled with its kind, its motion when not
fixed, and its status when known. Mermaid is text an agent reads directly and GitHub renders, so decad needs no graph
layout. A drawn layout belongs to the consumer.

### 6.2 Part views

`Views` returns, for each part with a body, what a renderer needs to draw that part alone on a part sheet.

```go
type Views struct{ Parts []PartView } // in Set order, body-less parts omitted

type PartView struct {
    Part      string
    Placement r3.Transform // assembled coordinates to view coordinates
    Bounds    decad.Box    // the body's box in view coordinates
    Anchor    r3.Vec       // label anchor: the top centre of Bounds
    Ports     []Port       // where each joint attaches, in view coordinates
}

type Port struct {
    Joint string
    Other string // the part at the joint's other end
    Point r3.Vec // see V3
    Dir   r3.Vec // the axis or the face's outward normal; zero when none is known
}
```

| Rule | Decision |
|---|---|
| V1 | `Placement` is the translation that puts the body's box centre at the origin, keeping the assembled orientation, or the caller's `WithPartPose` transform. A part sheet then shows every part the way it was modelled, side by side. |
| V2 | `Bounds` is the box of the body's mesh vertices moved by `Placement`, grown by the mesh bound, the body's box bound and the rotation's rounding (eight units in the last place of the largest coordinate), as Arrange G4 grows its footprints (`agent-docs/arrange-design.md`). For a pure translation it is the body's `Bounds` moved, with its own exactness. A pose that is a reflection or not a valid transform is `decad.ErrDegenerate`, as is a pose naming a part that is unknown or has no body. |
| V3 | A port's `Point` is the midpoint of the matched axis over the engagement, or the declared axis's point nearest the box overlap centre, or the contact face's centre for a `FaceContact`, or else the centre of the two parts' box overlap (or of the shortest segment between their boxes when they do not overlap); it is the part's own box centre when the other part has no body. `Dir` is the axis, or the part's own outward face normal for a `FaceContact` (the two ends of one contact get opposite directions); it is zero for the box fallback. |

Views pose each part for drawing, not for making. `parts` offers no "lay it flat" or "best side down" frame, because
choosing a side for a part to rest on is a process decision. A caller that wants another pose passes it with
`WithPartPose`.

## 7. Joints

### 7.1 Axes from the faces

`Explode`, `Views`, `Check` and `Infer` share one face-matching step in `internal/jointinfer`, run on plain cylinder
and plane records that `parts` reads through `Face.Surface`, `Face.NormalAt`, `Mesh.SourceFaces` and `Body.Bounds`.

| Rule | Decision |
|---|---|
| M1 | Candidate pairs are parts whose boxes, grown by `MaxFit`, meet. |
| M2 | A cylindrical face is convex when its outward normal at one of its mesh vertices points away from its axis, concave when toward. `Face.NormalAt` answers for a `Cylinder` tag, retained ones included. |
| M3 | `PinInBore`: a convex cylinder of A and a concave one of B whose axes are parallel within `AxisTolerance`, whose axis distance is within `FitTolerance`, whose radii satisfy `R − r ≤ MaxFit`, and whose axial extents overlap by a positive length. `Measured` is `R − r`, its bound the radius bounds and the axis distance. |
| M4 | `PinInSlot`: a convex cylinder of A and two planar faces of B parallel to its axis within `AxisTolerance`, with normals opposed and pointing at the axis, at distances whose sum less `2r` is at most `MaxFit`. `Measured` is that sum less `2r`. |
| M5 | A mesh Boolean result keeps a `Cylinder` or `Plane` tag on a face only where faceted source retention proved it (`agent-docs/faceted-source-retention-design.md`, the `RetainedTag` faces); a displaced retained tag matches with its bound charged. A face whose `Surface` is `Faceted` never matches, and `Face.NormalAt` refuses it with `ErrUnsupported`, so it is never asked for a normal here. A joint whose faces do not match falls back to its declared `Axis`, then to box centres (E2). |
| M6 | Two or more matches for one declared joint: the one with the longest axial overlap wins, then the smallest `Measured`. Matches of one cylinder split into several faces join into one match. |
| M7 | `FaceContact`: a planar face of A and one of B with opposed normals within `AxisTolerance`, planes within `MaxFit` of each other, and a positive overlap of their vertex boxes projected onto the plane; the largest overlap wins. The overlap is the box rectangle, so it never under-reports the shared area. |
| M8 | Faces are read from a mesh made with `VerifyNone` at a chord tolerance of 0.05 mm, then 0.5 mm, then 5 mm, whichever the body first accepts; the mesh gives only the vertices of each face. A face's reading bound is the bound `Face.NormalAt` reports, taken in millimetres for every length test it takes part in. A body no tolerance meshes reads as having no matchable face. |

### 7.2 Checking

`Check` reads one pair partition and judges each declared joint against it and against the faces.

| Rule | Decision |
|---|---|
| C1 | The partition is `Document.Verify(ctx, decad.WithClearances())`, or the report given by `WithVerifyReport`. A pair of parts is `Overlapping` when it is an interference row, `Touching` when a clearance row's `Gap` is an `Exact` zero, `Separated` with its gap otherwise, and `Undecided` when it is in neither list. Verify covers every live body of the document; pairs with a non-part body are ignored. |
| C2 | `PinInBore` and `PinInSlot`: the faces of M3 or M4 must match (else `Contradicted`, "no coaxial cylinder pair"), unless a face that would take part reads `Faceted`, which makes it `Undecided`. A declared `Axis` more than `AxisTolerance` or `FitTolerance` from the matched one is `Contradicted`. With a declared `Clearance`, `Measured` within `FitTolerance` of it is required. A non-negative declared clearance with an `Overlapping` pair is `Contradicted`. A negative one (press fit) expects `Overlapping` or `Touching`. |
| C3 | `FaceContact` and `Contact`: `Touching`, or `Separated` with a gap upper end at most `ContactTolerance`, confirms; `Overlapping` contradicts; anything else is `Undecided`. |
| C4 | `Keepout`: `Separated` with a gap lower end at least `MinGap` confirms; `Touching`, `Overlapping` or a gap upper end below `MinGap` contradicts. |
| C5 | `Tie`: `NotChecked`. |
| C6 | `Unexplained`: every part pair that is `Overlapping`, `Touching` or `Separated` with a gap upper end at most `ContactTolerance`, and that no declared joint names. This is the gallery's `designedContacts` check as a decad answer. |
| C7 | `Undecided` lists every part pair Verify left undecided; `JointReport` never treats one as clear. |
| C8 | `Inferred` lists `Infer`'s matches whose pair no declared joint names (from P6; empty before it). |

`Check` returns a report whenever it ran. A `Contradicted` joint is a finding, not an error.

### 7.3 Inference

`Infer` runs M1–M5 over every candidate pair and returns each match as a `PinInBore` or `PinInSlot` joint with its
measured clearance. Coplanar `FaceContact` inference is deferred: it needs the overlap area of two planar faces'
trims, a 2D question for `sketch` (a private scene of both loops), and no consumer needs it before the gallery's
declared laps are checked. Touching pairs without a cylinder match appear in `Unexplained` instead.

### 7.4 Moving joints and the linkage check

`Check` judges the assembled pose only. Motion is `VerifyLinkage`'s (`agent-docs/linkage-check-design.md`), which
`parts` consumes and does not repeat. `Set.Linkage(ground, r)` builds the linkage:

| Rule | Decision |
|---|---|
| K1 | Parts joined by `Fixed` joints form one link, found by union-find over those joints. The ground's link is `Linkage.Ground()`. |
| K2 | Walking the remaining joints breadth-first from the ground in `Join` order: a `Turns` joint becomes `Link.Revolute` about its axis from `r`; a `Slides` joint becomes `Link.Prismatic` along its slot direction (the slot planes' direction normal to the pin axis) or its `Direction`; `TurnsAndSlides` becomes the revolute-and-prismatic chain `linkage-check-design.md` §1 names. |
| K3 | Every joint between bodies of different links is declared with `Linkage.DeclareJointContact`. |
| K4 | A joint that would close a loop, or whose axis `r` neither matched nor took from a declaration, is returned unplaced, for the caller's `Linkage.Close` or for repair. Body-less parts take no part. |

One `Join` then feeds the exploded view, the diagram, the static check and the motion check.

## 8. Text outputs

### 8.1 Bill of materials

```go
type BOM struct{ Rows []BOMRow } // manufactured rows first, then purchased; each by Spec, then name

type BOMRow struct {
    Spec     string // or the part name when Spec is empty
    Parts    []string
    Kind     Kind
    Quantity int
    Material Material
    Volume   *decad.Measurement // solid volume of one part; nil for a body-less part
    Mass     *decad.Measurement // Volume times Density; nil without a density
}
```

Parts sharing a `Spec` must agree: their volumes must overlap within their bounds, or `BOM` returns
`decad.ErrDegenerate` naming the two parts. That check only falsifies the caller's claim that the parts are the same.
`Mass` is the solid body's mass.

### 8.2 Summary

`Summary.WriteMarkdown` writes, in this order: the bill of materials table; the joint table with statuses and reasons
when `Joints` is set; `Unexplained` and `Undecided` pairs; the exploded depth and pull table when `Exploded` is set;
the Mermaid diagram. Numbers print with their bounds. This is the agent's primary view of the schematic: it reads a
page, not pixels. Images come from the consumer (§11).

## 9. Reuse

| Need | Existing facility | New code |
|---|---|---|
| Cylinder and plane geometry and normals | `Face.Surface`, `Face.NormalAt` | the matching of M3, M4 |
| Triangles and their source faces, the mesh bound | `Body.Tessellate`, `Mesh.SourceFaces`, `Mesh.Bound` | none |
| Pull-path clearance for explode | `Document.SweptBox`, `SweptBox.StrictlyDisjoint` | the sense choice of E3 |
| Pair partition, gaps, interference | `Document.Verify` with `WithClearances` (`agent-docs/clearance-design.md`, `agent-docs/interference-design.md`) | the mapping of C1 |
| Motion of joints | `decad.Linkage`, `Link.Revolute`, `Link.Prismatic`, `Linkage.DeclareJointContact`, `VerifyLinkage` | the bridge of §7.4 |
| Grouping | `Document.ContainerOf` | none |
| Volume, mass | `Body.Volume`, `units` arithmetic | none |
| Transforms | `r3.Translation`, `Then`, `Apply`, `ApplyDir` | none |

New internal packages, each taking plain data (`r3.Vec`, slices, floats in millimetres):

| Package | Holds |
|---|---|
| `internal/jointinfer/` | cylinder and slot matching of M1–M6 on plain cylinder and plane records |
| `internal/explodeplan/` | the breadth-first walk, travels and offsets of E1–E4, and the port points of V3 |

### 9.1 The multi-object 3MF writer

`export` gains one generic writer, so a caller can write the whole build, assembled or exploded, as one 3MF of named
objects. It knows nothing of parts or of any process, and `parts` does not call it.

```go
// ThreeMFItem is one object of a 3MF package: a body placed by a rigid motion.
type ThreeMFItem struct {
    Name      string
    Body      *decad.Body
    Placement r3.Transform
}

// ThreeMFItems writes several bodies as one deterministic 3MF Core package:
// one named object and one build item each, vertices moved by the item's
// Placement. A body named by several items is tessellated once. A Placement
// that reflects is decad.ErrDegenerate, because it would turn the mesh inside
// out. The writer's other rules are ThreeMF's.
func ThreeMFItems(ctx context.Context, w io.Writer, items []ThreeMFItem, tol units.Value,
    opts ...decad.TessellateOption) error
```

Vertices are moved before writing rather than through the 3MF `transform` attribute, so every reader places them the
same way. `agent-docs/3mf-export-design.md` gains a section for it in PR E1. With each `ExplodedPart.Placement` as an
item's placement it writes the exploded view as a file any 3MF viewer opens.

## 10. Gate analysis

`agent-docs/module-boundary.md` must pass before implementation. This section is the evidence each PR quotes.

### 10.1 Reusable CAD behavior

| Gate item | Evidence |
|---|---|
| Input class stated | Axis matching: faces whose `Surface` is a `Plane` or `Cylinder`, exact or retained with a bound. Explode and views: any live solid body with bounds and a boundary-proved tessellation. Checking: any pair `Verify` partitions. 3MF items: any bodies `ThreeMF` writes, under any proper rigid placement. |
| Behavior by geometric invariants | Coaxiality and radius difference (M3), opposed parallel planes (M4), box travel and swept-box disjointness (E3), box overlap centres (V3). None names a part. |
| No part-specific or process-specific dispatch | No route reads a name, a fixture, a gallery model or a manufacturing process. Part names are labels only. |
| Independent cases | §12 lists a pin and plate, a press fit, a slider in a groove, a lap with two pins, a pin through two plates, two touching blocks, a keepout pair, a faceted-face joint and a rotated copy; each varies the geometry its rule reads. The crossbow is an acceptance run in the gallery, never a decad fixture (`agent-docs/testing.md` "Test scope"). |

### 10.2 Root package placement

| Gate item | Evidence |
|---|---|
| Private algorithms in `internal/` | §9's new algorithms live in the two internal packages, which take plain data. |
| No handle passed to reach data | `parts` unwraps `*decad.Body` and `*decad.Face` through public queries into plain records before calling an internal package. |
| Root adapters | None. Neither `parts` nor `export.ThreeMFItems` adds a file to the root package. |
| Exported types | In `parts` and `export`, both above the root (§2). |

### 10.3 House rules this design keeps

- Every scalar is a `units.Value` or a `decad.Measurement`; coordinates are `r3.Vec` (`api-design.md` §5.1, §5.2).
- No triangle, index or bare float leaves the package: joints name faces, views and placements name parts
  (`api-design.md` §3).
- No operation mutates the document; `Set` is a builder (`api-design.md` §6.3).
- Every check that reads the geometry either falsifies a declaration or states an interval.
- No 2D question is answered in decad; the one that would be (coplanar overlap, §7.3) is deferred to `sketch`.
- Nothing in the design belongs to a manufacturing process.

## 11. Consumers

decad returns data; drawing is the consumer's. Nothing in decad imports `solidlens` or `kinetograph`, and none needs
to: solidlens's `TriangleSource` is `Vertices() []r3.Vec` and `Triangles() [][3]int`, which `*decad.Mesh` satisfies.

| View | Built from |
|---|---|
| Exploded view | each part's mesh moved by `ExplodedPart.Placement`, `Leaders` as dotted lines, names at `Anchor` |
| Exploded animation | kinetograph: one translation track per part from zero to `Offset` |
| Connection diagram | the Mermaid text, or any graph renderer over `Graph` |
| Part sheet | one tile per `PartView`: the part's mesh moved by `Placement`, framed by `Bounds`, the name at `Anchor`, a callout per `Port` naming `Other` |
| Exploded 3MF | `export.ThreeMFItems` with the exploded placements |

What solidlens lacks, checked against its `main` at `77287d4`; this is a note for the solidlens repository, not work
in this design:

1. An exported projection, such as `Camera.Project(settings, point)` returning pixel x, y, depth and a visible flag,
   so a caller draws labels and leader lines on the returned `*image.RGBA` with `x/image/font`. Text stays out of
   `Scene`. This is the one blocking need: without it a label cannot be put at a part.
2. An orthographic camera with a view height in millimetres, for part sheets drawn to scale. Perspective works
   meanwhile.
3. An optional translation per `Model`, for exploded views. A wrapper `TriangleSource` that offsets vertices works
   meanwhile, so this is the lowest priority.

kinetograph needs nothing new: a translation track per part plays the explosion, and `linkage` posing already has its
bridge (`agent-docs/linkage-check-design.md` §2.4).

## 12. Test plan

Tests use `testify/require`, `t.Context()` and external test packages (`parts_test`, `export_test`), and assert on
computed geometry: offsets, axes, clearances, gaps, coordinates. Fixtures are the smallest that reach each rule.

| Test | Shows | PR |
|---|---|---|
| `TestSetRefusals` | every `Add` and `Join` refusal; `Group` from containers | P1 |
| `TestMatchPinInBore` | the pin and plate match with axis (10, 10, z) and `Measured` 0.2; an off-axis pin and a 2 mm-clearance pin do not | P1 |
| `TestMatchPinInSlot` | a Ø4 pin in a 4.4 mm groove matches with `Measured` 0.4; walls not parallel to the pin do not | P1 |
| `TestMatchFaceted` | a joint whose bore reads `Faceted` falls back to its declared `Axis`, then to box centres; no `NormalAt` call on the faceted face | P1 |
| `TestExplode` | the pin and plate (§4); a lap with two pins: pins along their axes by travel plus gap, plates apart along the face normal; a pin through two plates whose −Z path is blocked takes +Z by `SweptBox` | P1 |
| `TestExplodeUnreached` | a part no joint reaches keeps a zero offset and `Depth` −1 | P1 |
| `TestGraphMermaid` | the lap's diagram: subgraphs per container, node shapes per kind, one labelled edge per joint, parsed line by line | P1 |
| `TestViews` | centring, `WithPartPose`, ports at a pin's axis midpoint and at a box-overlap centre | P2 |
| `TestThreeMFItems` | object names, moved vertices, one tessellation for a repeated body, reflection refusal, determinism | E1 |
| `TestBOM`, `TestSummaryMarkdown` | the lap's rows, `Spec` grouping and its volume falsifier; the page's sections in order | P3 |
| `TestCheckContactKinds` | `FaceContact`, `Contact`, `Keepout`, `Tie` confirm and contradict cases | P4 |
| `TestCheckUnexplained` | two touching blocks without a joint appear in `Unexplained`; with a `FaceContact` they do not | P4 |
| `TestCheckPins` | Ø6 in Ø6.4 `Confirmed` 0.2; declared 0.3 `Contradicted`; a declared axis 1 mm off `Contradicted`; a press fit declared −0.05 confirms and +0.1 contradicts | P4 |
| `TestCheckVerifyReuse` | `WithVerifyReport` gives the same report as the internal `Verify` | P4 |
| `TestLinkage` | a hinge and a slider become a revolute and a prismatic link with declared contacts; a loop joint comes back unplaced; `VerifyLinkage` runs on the result | P5 |
| `TestInfer` | a lap with two pins infers both pins and nothing else; a rotated copy infers through retained tags (M5) | P6 |

Examples: `examples/parts_explode_example_test.go` (P1), `examples/parts_views_example_test.go` (P2),
`examples/export_3mf_items_example_test.go` (E1), `examples/parts_summary_example_test.go` (P3) and
`examples/parts_joints_example_test.go` (P4), each with a verified `// Output:` block.

**Acceptance case: the crossbow, in the gallery.** Per `agent-docs/testing.md` "Test scope", the 51-part model stays
in `decad-gallery`. Each phase is accepted when its real output passes through the real consumer there (§15): P1 by
an exploded image and a diagram of all 51 parts, P2 by a part sheet, P4 by the whole assembly's `Check`, P5 by its
linkage.

**CI shards** (`agent-docs/testing.md`). `parts/` is a new package: the CI workflow's race job must name it, or
`TestCIWorkflowRaceShardsCoverEveryPackage` fails; it joins the `./decadtest/ ./export/ ./internal/decaderr/ ./parts/` job
(the support-package job and shard 0), moving to its own runner only if measured time asks. The new internal packages fall under `./internal/...`. No
root-package test is added, so `.github/test-shards.txt` and `test-shards-apitest.txt` stay unchanged. The examples
run in the independent examples job. Run `go test . ./apitest/ -run '^TestCI'` before each push.

## 13. Phasing

The order puts a real exploded view and connection diagram of the crossbow first. Each PR carries its section of this
document, its layout rows, its `docs/api.md` and `docs/data-structures.md` entries, its examples, and the Module
boundary gate evidence in its description. P1 also carries this document. Sizes are rough line counts with tests.

| PR | Content | Needs | Size |
|---|---|---|---|
| P1 | this document; `Kind`, `Material`, `Part`, `Joint`, `Axis`, `Set`, `Add`, `Join`; `internal/jointinfer` axis matching (M1–M6); `Explode` with `internal/explodeplan`; `Graph` and `WriteMermaid`; explode example; CI job entry | — | 1,300 |
| E1 | `export.ThreeMFItems` and its section in `agent-docs/3mf-export-design.md` | — | 300 |
| P2 | `Views` and ports; views example | P1 | 350 |
| P3 | `BOM`, `Summary`; summary example | P1 | 500 |
| P4 | `Check` over `Verify` (C1–C7), `JointReport`, edge statuses in `Graph`; joints example | P1 | 800 |
| P5 | `Set.Linkage` | P4 | 450 |
| P6 | `Infer` and `JointReport.Inferred` | P4 | 300 |

**Parallel work.** P1 and E1 start together. After P1, P2, P3 and P4 run in parallel. After P4, P5 and P6 run in
parallel. `Summary` prints the joint table from whichever of P3 and P4 lands second.

P1 is the largest PR because the exploded view needs the axis matching; splitting it further would ship a `Set`
nobody can look at. If review asks for a smaller first PR, P1 splits into P1a (`Set`, `Join`, `Graph`,
`WriteMermaid`: the diagram, 500) and P1b (`internal/jointinfer`, `Explode`: the exploded view, 800).

Before P1 grows past its first case, `Explode` runs on the real 51-part crossbow and the count of joints whose `Pull`
is `BoxCentres` (no axis from faces or declaration) is recorded. Before P4 starts, one `Verify` with
`WithClearances` runs over the same model and its time and undecided pair count are recorded (§14). If either run
fails, expansion of that PR stops and the blocker is reported.

## 14. Risks and open questions

| Risk or unknown | What settles it |
|---|---|
| A face of a mesh Boolean result keeps a `Cylinder` tag only where retention proved it; other faces read `Faceted`. The crossbow's moving parts are placed by rotations about +Y, so their bores may read as displaced retained tags or as `Faceted`, and M3 may not match them. | P1's crossbow run counts `Pull` sources. A declared `Axis` covers each unmatched pin; the exploded view stays right, and `Pull` shows which axes were stated rather than read. |
| `Verify` over 51 faceted parts may be slow, or leave many pin and bore pairs undecided. The gallery screens pairs with its own `meshgap` and never calls `Verify` on the crossbow, which suggests one of the two. A mostly-`Undecided` `Check` would not replace `designedContacts`. | The pre-P4 run. The exploded view and diagram do not depend on it. If undecided pairs dominate, the fix is in the clearance kernel's faceted coverage, a decad change, not a `parts` fallback. |
| E3's box test is coarse: a long pin's swept box may meet a neighbour's box though the pin's path is clear. | E3 then falls back to the shorter travel, and `Overlaps` reports any exploded boxes that still meet. |
| The request mentioned a joint-chain checking program on branch `feat-joint-chain-check`. No such branch or worktree exists; the work merged as #665, "check joint chains with VerifyLinkage", and is the linkage check §7.4 consumes. | Nothing further, unless another branch was meant. |

## 15. Gallery adoption

The gallery model changes after the matching decad PRs merge and the gallery bumps decad. The exploded view, the
schematic, the connection diagram and the part sheets come from `parts` only:

1. After P1: the crossbow's `Assembly` builds a `parts.Set` (printed parts `Manufactured` with a material label, the
   bands `Purchased`) and joins its connections with kinds, declaring `Axis` on pins whose faces read `Faceted`. The
   gallery's `tools/render/` draws the exploded view from `Set.Explode` through solidlens and writes the Mermaid
   diagram into the README.
2. After E1: `threemf.go` is deleted; the model writes its 3MF with `export.ThreeMFItems` in the assembled pose, and
   an exploded 3MF with the exploded placements.
3. After P2: `tools/render/` draws a part sheet from `Set.Views`, one tile per part, ports as callouts.
4. After P3: the model writes the `Summary` page beside the README.
5. After P4: `designedContacts` is deleted; `TestPoseClearance` asserts `Unexplained` is empty and no joint is
   `Contradicted`. `internal/meshgap` stays only for checks `Verify` cannot yet decide, each listed against a decad gap.
6. After P5: the model's motion checks read their joints from the same `Set`.

**Printing checks stay in the gallery.** `TestPrintSize` (every printed part within a 200 mm cube),
`TestPrintWithoutSupports` (down-facing faces only on the bed in a chosen build direction) and the pins' "lying on
their side" check remain gallery tests in `models/crossbow`, built on public decad queries (`Body.Bounds`,
`Body.Tessellate`, `Mesh.SourceFaces`). If several gallery models need them, they move to a gallery package such as
`decad-gallery/internal/printcheck`. Print plates, if the gallery wants them, are `decad.Arrange` on copies of the
parts plus `export.ThreeMFItems`. Nothing in decad hosts a printing check.
