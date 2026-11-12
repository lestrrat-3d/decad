# Parts design

The package lands in the PRs of §13, and each PR carries the part of this document it implements. P1 and E1 have landed: `Kind`, `Material`, `Part`, `Joint`, `Axis`, `Set`, axis matching (§7.1), `Explode`, `Graph` and
`export.ThreeMFItems`. P2 added `Views`, P3 added `BOM` and `Summary`, and P4a adds `Check` for `PinInBore`,
`PinInSlot`, `FaceContact` and `Tie` joints and the box-screened list of pairs no joint names (§7.2), and P4b adds
point witnesses, `Contact` and `Keepout` checks, `WithWitnessSearch` and `WithVerifyReport`. P5 added `Set.Linkage`, and P6 adds
`Infer` and `JointReport.Inferred`. P7 adds staged explosion with per-stage, per-part and overall distance control (§5.2
to §5.6): the whole design has landed.

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
| `internal/meshgap`, a boundary-gap screen | `Set.Check`'s box screen and point witnesses (§7.2) |
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
`Face.NormalAt`, `Body.Bounds`, `Body.Volume`, `Body.DistanceToPoint`, `Document.SweptBox`, `Document.ContainerOf`). It
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
    // Pull (P7) is the direction in which A leaves B in an exploded view, in
    // assembled world coordinates, or zero. On a PinInBore or PinInSlot it
    // picks the sense along the axis; on any other kind it is the direction
    // (§5.1 E2). Its length does not matter.
    Pull r3.Vec
}

// Axis is a line: a point on it and a unit direction.
type Axis struct{ Point, Dir r3.Vec }
```

`Join` refuses a non-finite `Pull` with `decad.ErrNotFinite`. `Explode` refuses, with `decad.ErrDegenerate` naming the
joint, a `Pull` on a `PinInBore` or `PinInSlot` that lies within `AxisTolerance` of normal to the axis it uses, since it
then names no sense.

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

// P7 (§5.2–§5.6):
func WithExplodeStages(stages ...Stage) ExplodeOption           // ordered stages; unnamed parts form a last stage
func WithPartGap(part string, gap units.Value) ExplodeOption    // one part's gap, over its stage's
func WithExplodeScale(scale units.Value) ExplodeOption          // a Dimensionless multiplier on every offset; default units.Scalar(1)

// Hidden lists the pairs of exploded parts where one may hide the other seen
// along view, by their projected boxes (§5.6). It moves nothing.
func (e *Exploded) Hidden(view r3.Vec) ([]Occlusion, error)

// Graph is the connection diagram: parts as nodes, joints as typed edges
// (§6.1). r may be nil; with one, each edge carries its check status.
func (s *Set) Graph(r *JointReport) Graph

// Views returns the pose and points for drawing each part alone (§6.2).
func (s *Set) Views(ctx context.Context, opts ...ViewOption) (*Views, error)

func WithPartPose(part string, t r3.Transform) ViewOption // a caller's pose for one part

// Check judges every declared joint against the faces and bounds of its two
// parts, and lists the pairs of parts whose boxes meet with no joint naming
// them (§7.2). It is a structural check: it never calls Document.Verify, and
// it does not prove that two parts' interiors are disjoint.
func (s *Set) Check(ctx context.Context, opts ...CheckOption) (*JointReport, error)

// Infer lists the joints the geometry suggests between the parts (§7.3). It
// adds nothing to the Set.
func (s *Set) Infer(ctx context.Context, opts ...CheckOption) ([]InferredJoint, error)

func WithFitTolerance(tol units.Value) CheckOption     // default 0.05 mm
func WithContactTolerance(tol units.Value) CheckOption // default 0.25 mm (§7.2 G5)
func WithAxisTolerance(a units.Value) CheckOption      // default 0.5°
func WithMaxFit(fit units.Value) CheckOption           // largest clearance matched; default 1 mm
func WithWitnessSearch() CheckOption                   // also seek a contact witness for each unexplained pair (§7.2 U2)
func WithVerifyReport(r *decad.Report) CheckOption     // read a Verify the caller already ran (§7.2 R1)

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
    Request     CheckRequest    // the effective tolerances and options
    Joints      []JointResult   // one per declared joint, in Join order
    Unexplained []PairFinding   // part pairs whose boxes meet and that no joint names (§7.2 U1)
    Inferred    []InferredJoint // inferred matches no declared joint covers (from P6)
}

type CheckRequest struct {
    FitTolerance, ContactTolerance, AxisTolerance, MaxFit units.Value
    WitnessSearch bool
    VerifyReport  bool       // a WithVerifyReport report was read
    Meshes        []PartMesh // one per part with a body, in Set order (§7.2 G0, G5)
}

type PartMesh struct {
    Part      string
    Tolerance units.Value // the chord tolerance the part was read at
    Bound     units.Value // that mesh's bound
    Verified  bool        // VerifyAll held, so point distances were available
}

type JointResult struct {
    Joint    Joint
    Status   JointStatus        // Confirmed, Contradicted, Undecided, NotChecked
    Cause    Cause              // why it is not Confirmed; CauseNone when it is
    Axis     *Axis              // PinInBore, PinInSlot: the axis used (§7.1)
    AxisFrom AxisSource         // Faces, Declared or None
    Measured *decad.Measurement // PinInBore: R − r; PinInSlot: width − 2r; FaceContact: the planes' distance
    Engaged  *decad.Measurement // PinInBore, PinInSlot: axial overlap of the matched faces
    Gap      *decad.Measurement // Contact, Keepout: the interval the box gap and any witness prove (§7.2 G1)
    Witness  *r3.Vec            // a point the contact or overlap claim was read at
    FacesA   []*decad.Face      // the faces of A the rule read
    FacesB   []*decad.Face
    Verified *decad.ContactRelation // the pair's relation in a WithVerifyReport report; nil without one
    Reason   string             // the numbers behind Cause, for people, not for branching
}

// Cause names why a joint is not Confirmed.
type Cause int

const (
    CauseNone          Cause = iota
    CauseNoMatch             // no face pair of the joint's form exists, and every face near the pair was read
    CauseClearance           // the measured clearance is not the declared one
    CauseAxis                // the matched axis is not the declared one
    CauseInterference        // a pin larger than its bore with no declared press fit, or Verify's overlap
    CauseApart               // the parts are proven farther apart than the joint allows
    CauseTooClose            // a keepout pair is proven closer than MinGap
    CauseUnreadFace          // a face near the pair reads Faceted or the body would not mesh
    CauseNoWitness           // no point proves the contact or the face overlap
    CauseNeedsPairProof      // only a pair proof (Verify) could decide it
)

type InferredJoint struct {
    Joint    Joint // Kind PinInBore or PinInSlot, Motion Fixed, Axis and Clearance as matched
    Measured decad.Measurement
    FacesA   []*decad.Face
    FacesB   []*decad.Face
}

type PairFinding struct {
    A, B     string
    Overlap  decad.Box              // where the two boxes, grown by ContactTolerance and their bounds, meet
    Near     *decad.Measurement     // WithWitnessSearch: a proven gap upper bound at most ContactTolerance
    Verified *decad.ContactRelation // the pair's relation in a WithVerifyReport report; nil without one
}
```

The `AxisSource` values are named `AxisSourceNone`, `AxisSourceFaces` and
`AxisSourceDeclared`, because `AxisFromFaces` and `AxisDeclared` already name `PullSource` values.

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

**Joint check, P4a** (`examples/parts_joints_example_test.go`). The same pin and plate, joined with
`Clearance: units.Millimeters(0.2)` and checked with `Set.Check`, printing three lines: `pin~plate: confirmed`,
`radial clearance 0.20 mm, engaged 10 mm` and `0 unexplained pairs`. The pin passes through the bore, so the plate's
box and the pin's box meet; the joint names that pair, so it is not unexplained. A second joint declared with
`Clearance: units.Millimeters(0.3)` would print `contradicted (clearance)`.

**Staged explosion, P7** (`examples/parts_explode_stages_example_test.go`). A housing of two 40 x 20 x 10 mm halves,
split at z = 0, holds a 12 x 6 x 8 mm lever in a pocket; one Ø4 pin runs along Z through both halves and the lever.
The one-stage explosion lifts the top half off but leaves the lever inside the bottom half's box. Three stages take
the top off, lift the lever out of the opened housing, then pull the pin.

```go
package examples_test

import (
    "context"
    "fmt"

    "github.com/lestrrat-3d/decad"
    "github.com/lestrrat-3d/decad/parts"
    "github.com/lestrrat-3d/r3"
    "github.com/lestrrat-3d/units"
)

// Stages take an assembly apart in order. Each stage's parts move along their
// joints, or along a stated direction, and a ClearAll stage moves them until
// they clear every part still standing, so a mechanism comes out of the
// housing that encloses it instead of staying inside it.
func Example_parts_explode_stages() {
    ctx := context.Background()
    doc := decad.New()
    centre := r3.Vec{X: 20, Y: 10}
    // block is a box with a pocket and a Ø4.2 bore cut into it.
    block := func(x0, y0, x1, y1, z0, h float64, pocket [6]float64, bz0, bh float64) (*decad.Body, error) {
        b, err := extrudeBlock(doc, x0, y0, x1, y1, z0, h)
        if err != nil {
            return nil, err
        }
        if pocket != ([6]float64{}) {
            tool, err := extrudeBlock(doc, pocket[0], pocket[1], pocket[2], pocket[3], pocket[4], pocket[5])
            if err != nil {
                return nil, err
            }
            if b, err = decad.Cut(ctx, b, tool); err != nil {
                return nil, err
            }
        }
        bore, err := extrudeCircle(doc, centre, 2.1, bz0, bh)
        if err != nil {
            return nil, err
        }
        return decad.Cut(ctx, b, bore)
    }
    bottom, err := block(0, 0, 40, 20, -10, 10, [6]float64{10, 4, 30, 16, -6, 7}, -11, 12)
    if err != nil {
        fmt.Printf("failed to build the bottom half: %s\n", err)
        return
    }
    top, err := block(0, 0, 40, 20, 0, 10, [6]float64{10, 4, 30, 16, -1, 7}, -1, 12)
    if err != nil {
        fmt.Printf("failed to build the top half: %s\n", err)
        return
    }
    lever, err := block(14, 7, 26, 13, -4, 8, [6]float64{}, -5, 10)
    if err != nil {
        fmt.Printf("failed to build the lever: %s\n", err)
        return
    }
    pin, err := extrudeCircle(doc, centre, 2, -12, 24)
    if err != nil {
        fmt.Printf("failed to build the pin: %s\n", err)
        return
    }

    set := parts.NewSet(doc)
    for _, p := range []parts.Part{
        {Name: "housing-bottom", Kind: parts.Manufactured, Body: bottom},
        {Name: "housing-top", Kind: parts.Manufactured, Body: top},
        {Name: "lever", Kind: parts.Manufactured, Body: lever},
        {Name: "pin", Kind: parts.Manufactured, Body: pin},
    } {
        if err := set.Add(p); err != nil {
            fmt.Printf("failed to add %s: %s\n", p.Name, err)
            return
        }
    }
    for _, j := range []parts.Joint{
        {Kind: parts.FaceContact, A: "housing-top", B: "housing-bottom"},
        {Kind: parts.PinInBore, A: "pin", B: "housing-top"},
        {Kind: parts.PinInBore, A: "pin", B: "housing-bottom"},
        {Kind: parts.PinInBore, A: "pin", B: "lever", Motion: parts.Turns},
    } {
        if err := set.Join(j); err != nil {
            fmt.Printf("failed to join: %s\n", err)
            return
        }
    }

    // One stage: the lever comes off its pin but stays in the bottom half's box.
    flat, err := set.Explode(ctx, parts.WithExplodeGround("housing-bottom"))
    if err != nil {
        fmt.Printf("failed to explode: %s\n", err)
        return
    }
    fmt.Printf("one stage: %d overlap(s) %v\n", len(flat.Overlaps), flat.Overlaps)

    staged, err := set.Explode(ctx, parts.WithExplodeGround("housing-bottom"), parts.WithExplodeStages(
        parts.Stage{Name: "shell", Parts: []string{"housing-top"}},
        parts.Stage{Name: "mechanism", Parts: []string{"lever"}, Clear: parts.ClearAll, Direction: r3.Vec{Z: 1}},
        parts.Stage{Name: "pins", Parts: []string{"pin"}, Clear: parts.ClearAll, Gap: units.Millimeters(10)},
    ))
    if err != nil {
        fmt.Printf("failed to explode in stages: %s\n", err)
        return
    }
    for _, p := range staged.Parts {
        path := p.Path.String()
        if p.Path == parts.PathEnclosed {
            path += " by " + p.Blocker
        }
        fmt.Printf("%s: stage %q, offset (%.0f, %.0f, %.0f), path %s\n",
            p.Name, p.Stage, p.Offset.X, p.Offset.Y, p.Offset.Z, path)
    }
    fmt.Printf("staged: %d overlap(s), %d short\n", len(staged.Overlaps), len(staged.Short))
    // Output:
    // one stage: 1 overlap(s) [[housing-bottom lever]]
    // housing-bottom: stage "", offset (0, 0, 0), path none
    // housing-top: stage "shell", offset (0, 0, 10), path enclosed by lever
    // lever: stage "mechanism", offset (0, 0, 34), path enclosed by housing-bottom
    // pin: stage "pins", offset (0, 0, -32), path clear
    // staged: 0 overlap(s), 0 short
}
```

The numbers follow from §5.3. The top half lifts off its face contact by its 10 mm gap (S4). The lever rises along
+Z until its box is 10 mm clear of both halves' boxes, the top half's now at z 10 to 20, so its bottom sits at
z 30 and its offset is 34 (S5); the pin is excluded from its standing set, because the pin leaves later. The pin
then needs 32 mm downward against 60 mm upward, past the lifted lever, to clear every standing box by 10 mm, and
takes the shorter (S5, E3). The housing and the lever start inside each other's boxes, so their paths read
`enclosed` (S7).

## 5. Exploded view

```go
type Exploded struct {
    Ground   string
    Parts    []ExplodedPart // in Set order, body-less parts omitted
    Leaders  []Leader
    Overlaps [][2]string    // part pairs whose exploded boxes still meet
    // P7:
    Stages []string    // the stage names in the order they ran; "" is the last, implicit stage (S1)
    Scale  units.Value // the WithExplodeScale multiplier, Dimensionless
    Short  []string    // parts whose Clearance is below their gap (S9), in Set order
}

type ExplodedPart struct {
    Name      string
    Offset    r3.Vec       // translation after the assembled pose
    Placement r3.Transform // the same, as a transform
    Depth     int          // joint steps from the ground; -1 when no joint reaches it
    Anchor    r3.Vec       // label anchor: exploded box centre
    Pull      PullSource   // AxisFromFaces, AxisDeclared, FaceNormal, BoxCentres, PullDeclared, StageDirection, None
    // P7:
    Stage     string       // the last stage that moved it; "" for the ground and the implicit last stage
    Dir       r3.Vec       // the unit direction of that stage's move; zero when it did not move
    Travel    units.Value  // the distance that stage's clearing rule needed (S4, S5), before the gap and scale
    Gap       units.Value  // the gap asked for there: the part's WithPartGap, else the stage's, else WithExplodeGap
    Moved     units.Value  // the length of its own offset from where its parent ended, all stages and scale included
    Clearance units.Value  // after the last stage: the least box distance to any part it is not joined to; zero when boxes meet
    Path      PathState    // PathClear, PathBlocked, PathEnclosed, PathStuck, PathNone (S7)
    Blocker   string       // the part that blocked or enclosed the path, or that Clearance was read against
}

type Leader struct {
    Joint    string
    From, To r3.Vec // exploded coordinates: the port on A, the port on B
    // Separation (P7) is the distance between the two parts' exploded boxes,
    // zero when they meet: how far a pin ended up from the part it passes
    // through, which Clearance (joined parts excluded) does not say.
    Separation units.Value
}
```

### 5.1 One stage: the rules every explosion keeps

| Rule | Decision |
|---|---|
| E1 | The joint graph is walked breadth-first from the ground, joints in `Join` order, so each part has one parent joint; a part reached twice keeps its first parent. Parts no joint reaches keep a zero offset and `Depth` −1. |
| E2 | A part's direction from its parent: a `PinInBore` or `PinInSlot` pulls along its axis, from the faces when §7.1 matches them, else the declared `Axis`, else as a `Contact`; a `FaceContact` along the contact face's normal, away from the parent; `Contact` and `Tie` along the line between the two box centres. A declared `Joint.Pull` (P7) replaces the face normal and the box-centre line, and on an axis kind fixes the sense, so E3 compares one sense only; `Pull` then reads `PullDeclared`. `Pull` records which was used, so a viewer can flag a guessed direction. A stage's `Direction` replaces all of these for its members, and `Pull` reads `StageDirection` (§5.3 S3). |
| E3 | For each sense along the direction, the travel is the distance that takes the part's box, at its parent's offset, clear of the exploded box of its parent and of every other part it is joined to that the walk has already placed. A sense is free when `Document.SweptBox` along that travel is `StrictlyDisjoint` from the box of every part not joined to it, each at its current offset (a part not yet placed sits where it was assembled); strictly disjoint boxes prove the path clear. Parts joined to the moving part never block it. A free sense beats a blocked one, then the shorter travel wins, then the sense whose largest direction component is positive. |
| E4 | The offset is the chosen travel plus `WithExplodeGap`, along the chosen sense, added to the parent's offset. A `FaceContact` has one sense, away from the parent. |
| E5 | `Leaders` join the two ports of each joint (§6.2 V3) in exploded coordinates, so a consumer can draw the dotted line a pin travels. |
| E6 | `Overlaps` reports exploded box pairs that still meet. It is advice for the consumer, not a proof. |

A matched axis carries its point at the middle of the engagement, which is the joint's port (V3). `Explode` needs no `Verify`. It reads faces, boxes and swept boxes only, so it works on a model whose pair check is
slow or undecided (§14).

### 5.2 Why one stage is not enough

E3 and E4 move each part just clear of its parent and the joined parts already placed, plus one gap. A part nested
inside an enclosure is joined to its pins, not to the enclosure's other walls, so it moves clear of its pin and stays
inside the enclosure. On the crossbow (51 parts, ground `receiver-right`) the release nut, sear, trigger bar, hooks,
carriage and links end up inside the boxes of the receiver halves, the side plates and the stock, and the gallery
replaced `Explode`'s offsets with a hand-written table of 40 offsets to show them (dated illustration, 2026-11). One
global gap cannot fix it: raising it moves the enclosure and the mechanism alike.

Exploded-view tools solve this with ordered steps: take the shell off, lift the mechanism out, then pull its pins.
P7 adds exactly that, with each stage's parts clearing either their joined neighbours (as today) or everything still
standing, and with directions still taken from the joints unless a stage states one.

### 5.3 Stages

```go
// Stage is one step of a staged explosion. Its parts move in it, in the order
// of the joint walk (E1); every other part stays where earlier stages left it.
type Stage struct {
    Name   string   // non-empty and unique
    Parts  []string // part names
    Groups []string // container paths (Set.Group); every part in one is a member
    // Gap is how far past clearing each member moves; the zero Value means
    // WithExplodeGap's gap.
    Gap units.Value
    // Clear says what the members must clear: ClearJoined (the zero value,
    // E3's rule) or ClearAll (S5).
    Clear Clearing
    // Direction, when not zero, moves every member along it instead of along
    // its joint (Pull reads StageDirection). Its length does not matter.
    Direction r3.Vec
    // Together moves the members as one rigid group by one offset (S6).
    Together bool
}

type Clearing int

const (
    ClearJoined Clearing = iota // clear the parent and the joined parts already moved (E3)
    ClearAll                    // clear every part that stays put, by the gap (S5)
)
```

| Rule | Decision |
|---|---|
| S1 | Stages run in the order given. Parts no stage names, other than the ground, form a last stage named `""` with `ClearJoined`, `WithExplodeGap`'s gap and no direction. With no stages every part is in that last stage, which is the one-stage explosion of §5.1 unchanged. |
| S2 | Each member keeps its parent joint and direction from the joint walk E1 and E2; the walk is the same whatever the stages. A part moves only in the stages that name it: until its first, it stays where it was assembled, and between stages it stays where the last one left it. A part named by several stages moves in each, from where the previous one left it, so a group lifted as one in one stage can be taken apart in the next. |
| S3 | A member starts from its parent's offset when its parent is a member of the same stage (it is carried, as in §5.1), else from its own current offset. With a `Direction`, a member moves along that direction in its one sense; without one, along E2's direction in the senses E2 allows. |
| S4 | `ClearJoined`: the travel is E3's, computed against the parent and the joined parts already moved in this or an earlier stage, at their current offsets. The offset is the travel plus the gap, as E4. |
| S5 | `ClearAll`: the travel is the least `t ≥ 0` at which the member's box, moved from its start by `t` along the sense, overlaps no box of the **standing** parts, each grown by the gap on every side. Standing parts are all parts except the member itself, the members of its stage not yet moved in it (they leave too), and the parts joined to it that a later stage moves (they come out after it, as a pin leaves its bore). The overlap intervals of `t` against each standing box are read exactly from the box faces and merged; `t` is the least point at or after zero outside their union. The gap is already inside the test, so the offset is that `t` alone; `Travel` reports `t` less the gap, floored at zero: grown boxes close the holes between standing boxes that the plain boxes leave, so a part is not left in a space narrower than its gap, and `Moved − Travel` is the gap. When no `t` up to 10⁶ mm clears (a direction along a box that is longer than that, or a direction that is zero), the member stays at its start with `PathStuck`. |
| S6 | `Together`: the members move by one offset. The moving box is the union of the members' boxes at their current offsets; the direction is the stage's `Direction`, else E2's direction of the first member in walk order whose parent is not a member; the clearing rule is the stage's, applied to the union box with the members excluded from the standing set. Members keep their relative places, so a later stage can take the group apart. |
| S7 | `Path`, for each move: `PathClear` when the `Document.SweptBox` of the moving box along its travel is `StrictlyDisjoint` from the box of every standing part not joined to it, as E3 proves; `PathEnclosed` when the start box already meets such a part's box, so a box test cannot decide whether the part can leave it (a mechanism inside a housing); `PathBlocked` when the start is clear and the sweep meets one; `PathStuck` from S5; `PathNone` for a part that did not move, the zero value. The standing parts of a `ClearJoined` move are every part but the member and those joined to it; of a `ClearAll` move, S5's set less the parts joined to it. The path is swept to the travel without the gap. `Blocker` names the first such part in `Set` order, a part that starts inside the member's box before one the sweep meets. The sense choice of E3 applies with `PathClear` counting as free. |
| S8 | `WithPartGap` replaces its part's gap in whatever stage moves it. `WithExplodeScale(s)` multiplies every final offset by `s`, a positive `Dimensionless` value, after the last stage: a uniform spread about the assembled layout, so the ground stays put unless a stage moved it. `Moved` is the length of the part's last move, from where that move started to where it ended, scale included; `Travel` and `Gap` are not scaled. A part that did not move has a zero `Gap`, so it is never short. Scaling by more than 1 can bring two parts that moved the same way closer, so `Overlaps`, `Clearance` and `Short` are read after scaling. |
| S9 | After scaling, each part's `Clearance` is the least distance between its exploded box and the exploded box of any part it is not joined to, zero when they meet, with `Blocker` naming that part when its move has none. `Short` lists the parts whose `Clearance` is below their `Gap` by more than 10⁻⁹ of the gap (rounding of the box arithmetic), so a caller sees which parts the stages did not separate enough and against what. A part joined to every other part has the zero `Clearance` and is never short. |
| S10 | Refusals, before `ctx` is read: an empty or repeated stage name, a stage with no members, an unknown part or group, a part named twice in one stage (directly or through a group), the ground in a stage without a `Direction` (with one, the ground moves by its gap along it, so two halves can open symmetrically), a non-finite `Direction`, a gap that is not a non-negative finite `Length`, and a scale that is not a positive finite `Dimensionless` value; each is `decad.ErrDegenerate`, `decad.ErrUnitKind`, `decad.ErrNegativeMagnitude` or `decad.ErrNotFinite` as `api-design.md` §12 sorts them. A part without a body in a stage, and a gap of a part no Set has, are `decad.ErrDegenerate` too; a group names its descendants as well, and a group with no part with a body is unknown. |

`Leaders` (E5) join the two ports of each joint at the final offsets, so a pin's leader runs from its exploded place
back to the bore it left whatever stage moved it.

### 5.4 What a stage proves and what it costs

Every clearing decision is a box decision. `ClearAll` proves that the member's exploded box is apart from each
standing part's box by at least the gap at the end of its stage, and later stages and the scale may change that,
which S9 reports. `PathClear` proves the straight path clear of every part not joined to it, box against box. Nothing
proves that a part can leave an enclosure along its path: `PathEnclosed` says the box test cannot tell, which is the
usual case for a mechanism inside a housing, and the exploded picture is a layout, not an assembly sequence (§1).

Cost: S5 is one interval per standing part per sense, so `n²` box comparisons per stage, about 2,600 for 51 parts. S7
adds one `SweptBox` per standing part per sense, the same calls §5.1 makes today, at about 30 ms for the crossbow's
one stage (dated illustration). The standing boxes are computed once per stage. Larger distances cost nothing extra:
`t` comes from the interval union, not from stepping.

### 5.5 Output and determinism

`Explode` is deterministic: stages run in the given order, members in walk order, senses and ties as E3, and every
quantity is computed from boxes and offsets in a fixed order. Equal inputs give equal `Exploded` values and therefore
equal `export.ThreeMFItems` bytes. `Stages`, `Scale` and the P7 fields of `ExplodedPart` record what was asked
(`Gap`, `Stage`, `Dir`) and what came out (`Travel`, `Moved`, `Clearance`, `Path`, `Blocker`), so the gallery can
assert a separation instead of reading pixels.

Nothing else changes with stages. `Views` and `Graph` do not read `Exploded`. `Summary` prints the new columns
(§8.2). `export.ThreeMFItems` takes each `ExplodedPart.Placement` as before, and `Leaders` follow the final offsets.

### 5.6 What stays the caller's, and the rejected view-driven explosion

The caller chooses the stages (which parts form the shell, which group lifts out, which way), the camera, the labels,
the drawing of `Leaders`, and any animation between stages; kinetograph can play a stage-by-stage explosion from the
offsets because each stage's offsets are a prefix of the result (a part not yet moved has a zero own offset).

**A view-driven explosion is rejected.** Pushing parts that hide others along a viewing direction would tie the
layout to one camera, while the same `Exploded` feeds several views, the exploded 3MF and the summary; the gallery
draws the crossbow's one explosion from four cameras (an overall view, left, right and top). Occlusion is
the renderer's knowledge (perspective, field of view, image size), and minimising it has no unique answer, so the
result would shift with small model edits. Moving a part along its joint does not even help when the joint runs along
the view. Instead `Exploded.Hidden(view)` reports, without moving anything, each pair whose boxes, projected
orthographically along `view`, overlap with one nearer the viewer:

```go
type Occlusion struct {
    Front, Back string
    Covered     units.Value // Dimensionless: the share of Back's projected box that Front's covers
}
```

A pair absent from the list cannot hide one another; a listed pair may. The caller then adds or changes a stage. A
zero or non-finite `view` is `decad.ErrDegenerate` or `decad.ErrNotFinite`.

### 5.7 Blocking between two tessellated parts

`internal/explodeblock` decides, from tessellations alone, whether part B stands in the way of part A when A moves
toward a unit direction `d`. It takes plain vertices, triangles and `Mesh.Bound`; it touches no body and has no public
API. Box-only blocking cannot work: a shaft's box overlaps a bearing's box in both senses, whatever the bore.

A `Projection` is the per-mesh half. It projects the triangles onto the plane perpendicular to `d` and keeps each
corner's height along `d`; triangles parallel to `d` have no shadow and are dropped. It is immutable, so one projection
serves many goroutines and many identical bodies (an `Instance` adds a translation). `Block(A, B)` is the per-pair half:

- Over every overlap of a projected triangle of A with one of B, the height difference `zB - zA` is affine. A part of the
  overlap where it exceeds 1e-6 mm is a witness: B stands ahead of A there. Faces at equal height (touching, coplanar)
  give no witness.
- With `s = bound(A) + bound(B)`, the pair is **Blocked** when a witness region survives an erosion by `s / 2`, that is
  when it is wider than `s`. Its witness is a point on A's surface and the height difference there.
- The pair is **Free** when no triangle of B that reaches ahead of the mover (highest corner above the mover's lowest by
  more than the epsilon) lies within `s` of a triangle of A in the plane. That is the same test with A's shadow dilated
  by `s`.
- Anything else is **Undecided**: the only witnesses are narrower than `s`, as in a press fit with a gap below
  `2 x Bound`. A layout reads Undecided as free along the part's own axis, so a tight fit never deadlocks a removal
  search. The result is layout-grade, not a proof.

Blocking between two parts does not depend on which others are present. `BuildTable` tests every ordered pair for one
direction (optionally in parallel; the table is identical either way) after a prefilter on shadow boxes and on position
along `d`. `Directions` lists candidate directions: given axes in both senses, then the six world axes.

### 5.8 Derived removal order

`WithDerivedExplode` derives the order and direction of every part no `WithExplodeStages` entry names from the
tessellations, with no joint walk. The result is layout-grade: a layout, not a proof. Explicit stages run first and
unchanged; their parts count as already gone. `internal/explodeorder` owns the search and `parts/explode_derive.go` the
adapter that feeds it and turns its rounds into stages.

```go
type Removal struct {
    Direction r3.Vec // the only direction the part may leave by; its length is ignored
    After     []string // parts that must leave in an earlier round
    Before    []string // parts this part leaves before
    Group     string   // parts with the same Group leave as one rigid group
}

func WithDerivedExplode() ExplodeOption
func WithRemoval(part string, r Removal) ExplodeOption // needs WithDerivedExplode
```

| Rule | Decision |
|---|---|
| D1 | Candidate directions are `explodeblock.Directions` of the joint axes read from the faces (both senses), the declared `Joint.Pull` vectors and the `WithRemoval` directions, then the six world axes, merged within the axis tolerance. |
| D2 | Each body is read once through the face cache, which keeps its triangles and `Mesh.Bound`; a body that will not tessellate is blocked as its box. One `explodeblock.Projection` serves every part sharing a body, per direction, and `BuildTable` runs the pair tests in parallel. A `Blocked` verdict blocks. `Free` and `Undecided` do not: an `Undecided` pair counts as free, so a press fit with a gap below `2 x Bound` never deadlocks the search. |
| D3 | A round removes every part with no blocker among the parts still present along some candidate. All blockers are read against the parts present at the start of the round, so the parts of one round may leave in any order. |
| D4 | When no part can leave, the blocking graph of each direction is cut into strongly connected components. A component with no blocker outside it and more than one part leaves as one rigid group (a `Together` stage), the smallest first. This covers a captive nut staying with its housing. A component holding the ground never leaves. Each round removes at least one part, so the search terminates. |
| D5 | Among the free directions of a part or group: a joint axis or pull first, then the side of the ground-relative outward direction (the group's box centre minus the ground's, dotted with the direction), then the shorter box travel, then a positive largest component, then the earlier candidate. The search is deterministic. |
| D6 | Inside a round the part whose trailing face along its direction is lower goes closer in: it moves first, and the next part clears its moved box. |
| D7 | Each round becomes `explodeplan.Stage` entries named `removal N` (`removal N.2` for a further stage of the same round): consecutive single parts leaving the same way share one `ClearAll` stage, a group has its own `Together` stage, and `Dir` is the chosen direction. `explodeplan.Run` is unchanged and gives the distances, leaders and overlaps. Derived parts run with no parent in the stage, so a part is never carried with the part that happens to be its joint parent. |
| D8 | `Removal.Direction` fixes the only direction a part may use. `After` and `Before` order rounds strictly. `Group` joins parts into one unit; its members may not be fixed to different directions. A name that is unknown, has no body, is the ground or is already moved by an explicit stage is `decad.ErrDegenerate`, as is `WithRemoval` without `WithDerivedExplode`. |
| D9 | A part with no free direction is `decad.ErrDegenerate` naming the part and, per direction, the parts that block it. A constraint that makes a removable part wait for one that cannot leave, an order cycle and a group fixed to two directions are `decad.ErrDegenerate` naming the parts. |

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

With a `JointReport` (P4a), an edge's label ends with its status and, when not `Confirmed`, its cause in brackets
(`pin in bore, turns, contradicted (clearance)`). A `Contradicted` edge is drawn with a crossed end (`--x`) and an
`Undecided` one dotted (`-.->`), so the problem joints stand out in a rendered diagram as well as in its text. A
`NotChecked` edge keeps the plain arrow.

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
| M7 | `FaceContact`: a planar face of A and one of B with opposed normals within `AxisTolerance`, planes within `MaxFit` of each other, and a positive overlap of their vertex boxes projected onto the plane; the largest overlap wins. The overlap is the box rectangle, so it never under-reports the shared area; it is enough to pick a pull direction, and `Check` confirms shared area with a witness instead (§7.2 J3). |
| M8 | Faces are read from a mesh made with `VerifyNone` at a chord tolerance of 0.05 mm, then 0.1, 0.2, 0.5 and 5 mm, whichever the body first accepts (the 0.1 and 0.2 mm rungs come with P4a, so a body that refuses 0.05 mm, as a model built with a 0.1 mm chain tolerance does, is not read at 0.5 mm); the mesh gives only the vertices of each face. A face's reading bound is the bound `Face.NormalAt` reports, taken in millimetres for every length test it takes part in. A body no tolerance meshes reads as having no matchable face. |

### 7.2 Checking

**`Check` is a structural check.** For each declared joint it asks one question: is the declaration consistent with
the faces and bounds of its two parts? It reads face tags, mesh vertices, body boxes and point distances, all public
queries with stated bounds, and never calls `Document.Verify`. It does not prove that two parts' interiors are
disjoint; that is a pair proof, `Verify`'s, which `WithVerifyReport` can feed in when the caller already has one.

`Check` takes this form because the pair proof does not decide a real build's joints. Measured on the crossbow's 51
parts (2026-11, a dated illustration): `Verify(WithClearances)` took 9 min 57 s and left 771 of 1,275 pairs
undecided, 64 of them declared joint pairs. Of those 64, 59 involve faceted mesh Boolean parts; about 250 undecided
pairs in all involve the rivets (a `Union` of two cylinders) or the disc (a chamfered cylinder with no clearance
reader); 22 are face-on-face contacts. A clearance-kernel fix for oblique pins in bores, designed separately, decides
one of the 64. A joint check built on the pair proof would leave most joints `Undecided`.

#### 7.2.1 Shared readings

| Rule | Decision |
|---|---|
| G0 | Every part's faces are read once per call, as M1–M8 read them, from the mesh at the first chord tolerance of M8's ladder the body accepts **with `decad.VerifyAll`**. Point distances (G2) need that verification, and a body holds one cached tessellation, so reading faces and distances from the same tolerance and level tessellates each body once. A body that refuses `VerifyAll` at every rung is read at `VerifyNone` for its faces, and its point distances are unavailable. |
| G1 | **Box gap.** The lower end of a pair's gap is the separation of the two `Body.Bounds` boxes along the axis that separates them most, less both boxes' `Bound`s, and zero when the boxes meet. It is sound: the bodies lie inside their boxes. |
| G2 | **Witness.** An upper end of a pair's gap comes from points. For any point `p`, the gap between A and B is at most `A.DistanceToPoint(p)` plus `B.DistanceToPoint(p)`, each taken at its upper end (the triangle inequality; a distance is zero for a point inside or on the solid). `p` runs over up to 64 mesh vertices of A nearest B's box, then up to 64 of B nearest A's box, at G0's tolerance. The least sum is the witness and its point `Witness`. The search finds a witness when one exists among the sampled points; finding none proves nothing. A search stops at the first point whose sum is already below the threshold it serves (`ContactTolerance`, or `MinGap` for a keepout), so `Gap` then reports that point's sum, not the least of all 128. Joints and unexplained pairs are searched on up to `GOMAXPROCS` goroutines. |
| G5 | **Resolution.** A witness cannot prove a gap smaller than the two distance readings' own widths, which grow with the meshes' bounds (about the chord tolerance G0 settled on, 0.1 mm for a body that refuses 0.05 mm). `JointReport.Request` records each part's tolerance and mesh bound. A `Contact` whose two parts' bounds together exceed `ContactTolerance` is `Undecided` (`CauseNoWitness`, `Reason` "below resolution") without a search. The default `ContactTolerance` of 0.25 mm leaves room for two parts meshed at 0.1 mm; a caller who needs a finer contact claim states a smaller tolerance and accepts more `Undecided` contacts. |
| G3 | **Faces near a pair.** A face is near the pair when its vertex box, grown by `MaxFit`, meets the other part's box. A rule that needs a cylinder or plane and finds none among the near faces reports `CauseNoMatch` only when every near face of both parts was read (tagged `Plane` or `Cylinder` and answered `Face.NormalAt`); one near face that reads `Faceted`, or a body that would not mesh, makes it `CauseUnreadFace` and the joint `Undecided`. |
| G4 | Interval tests read both ends: a reading `[m − b, m + b]` inside the allowed interval confirms, one outside it contradicts, and one that straddles its edge is `Undecided` with the cause of the test it failed to settle. |

#### 7.2.2 Per kind

| Rule | Kind | Confirmed | Contradicted | Undecided |
|---|---|---|---|---|
| J1 | `PinInBore` | M3 matches; `Engaged`, the axial overlap of the matched faces, has a positive lower end; with a declared `Clearance` `c`, `Measured` lies within `c ± FitTolerance`; without one, `Measured` is at least zero; a declared `Axis` lies within `AxisTolerance` and `FitTolerance` of the matched one | no match and every near face read (`CauseNoMatch`, `Reason` naming the closest cylinder pair's radii and axis distance); `Measured` outside `c ± FitTolerance` (`CauseClearance`); `R − r` below zero with no negative `c` declared (`CauseInterference`); a declared axis off the matched one (`CauseAxis`); G1's box gap above `MaxFit` (`CauseApart`) | a near face unread (`CauseUnreadFace`); a straddling interval (G4) |
| J2 | `PinInSlot` | M4 matches; positive `Engaged` along the axis; the declared clearance and axis tests of J1 on `width − 2r` | as J1, with "no pin between parallel walls" for `CauseNoMatch` | as J1 |
| J3 | `FaceContact` | M7 matches with the planes' distance `Measured` at most `FitTolerance`, and an **overlap witness**: the centroid of a triangle of A's face lies inside a triangle of B's face in projection onto the plane, farther than `β_A + β_B` from that triangle's edges, or the same with A and B swapped | no opposed planar pair near and every near face read (`CauseNoMatch`); the nearest opposed pair's `Measured` lower end above `FitTolerance` (`CauseApart`) | a near face unread (`CauseUnreadFace`); planes coincide but no overlap witness (`CauseNoWitness`) |
| J4 | `Contact` | a G2 witness with an upper end at most `ContactTolerance` | G1's lower end above `ContactTolerance` (`CauseApart`) | no witness (`CauseNoWitness`), or point distances unavailable (`CauseUnreadFace`) |
| J5 | `Keepout` | G1's lower end at least `MinGap` | a G2 witness below `MinGap` (`CauseTooClose`) | neither (`CauseNeedsPairProof`) |
| J6 | `Tie` | — | — | — (`NotChecked`) |

`Gap` reports the interval G1 and G2 prove for `Contact` and `Keepout`: `[box gap, witness]`, with `Value` its
midpoint and `Bound` its half width; with no witness its upper end is the largest distance between the two boxes.

J3's overlap witness replaces M7's box-rectangle overlap for confirming: two L-shaped faces whose vertex boxes overlap
but whose faces do not would pass M7 and must not be confirmed. Its points are mesh triangles of the two faces, read
in projection on the shared plane; that is a point-in-triangle test on mesh chords with a stated bound, not a 2D
question about recorded entities, so `sketch` is not asked.

#### 7.2.3 Pairs no joint names

| Rule | Decision |
|---|---|
| U1 | `Unexplained` lists every pair of parts with bodies whose boxes, grown by `ContactTolerance` and both boxes' bounds, meet, and that no declared joint names, with the region where the grown boxes meet. Boxes meeting is a screen, not a finding: it bounds the work and says where to look. |
| U2 | With `WithWitnessSearch`, each `Unexplained` pair also runs G2 and carries `Near` when a witness proves its gap at most `ContactTolerance`: an undeclared contact, proven. Pairs without `Near` may be apart. A caller gates on `Near` entries, not on the list's length. |
| U3 | Pairs whose boxes do not meet are never listed: G1 proves them apart by more than `ContactTolerance`. |

#### 7.2.4 A Verify report, when the caller has one

| Rule | Decision |
|---|---|
| R1 | `WithVerifyReport(r)` sets `Verified` on each joint and `Unexplained` pair to the pair's relation in `r`: `Overlapping` for an interference row, `Touching` for a clearance row whose `Gap` is an `Exact` zero, `Separated` for another clearance row, `Undecided` for a pair in neither list. |
| R2 | It can only settle what the structural rules left open or contradict what they confirmed. `Overlapping` contradicts any joint but a press fit (`CauseInterference`). A `Separated` gap at least `MinGap` confirms a `Keepout` that J5 left `Undecided`. A `Touching` relation, or a gap at most `ContactTolerance`, confirms a `Contact` J4 left `Undecided`. An `Unexplained` pair `r` proves `Separated` beyond `ContactTolerance` is dropped. |
| R3 | `Check` never calls `Verify` itself. A report of another document, or one older than a body of the `Set` (it does not name that body), is `decad.ErrDegenerate`. |

`Check` returns a report whenever it ran. A `Contradicted` joint is a finding, not an error. `Inferred` lists `Infer`'s
matches whose pair no declared joint names (from P6; empty before it).

#### 7.2.5 What stays undecided, and why

| Case | Why `Check` cannot decide it |
|---|---|
| A pin, slot or face joint where a near face reads `Faceted` | The face carries no plane or cylinder tag, so there is no radius or plane to compare; a mesh fit would be a guess, and a guess cannot confirm. A declared `Axis` still drives `Explode`. |
| A `FaceContact` whose planes coincide but whose sampled triangles give no overlap witness | Thin or interleaved faces can coincide in plane without sharing area; the witness test is the only cheap proof of shared area. |
| A `Contact` with no witness among the sampled vertices | The 64-vertex sample can miss the touching spot on a large part. |
| A `Keepout` whose boxes come within `MinGap` and whose sampled vertices stay farther | Only a pair proof bounds the gap from below inside the boxes. |
| Any claim that two parts' interiors are disjoint | That is `Verify`'s pair proof; `Check` does not make it. |

#### 7.2.6 Cost

`Check` costs one `VerifyAll` tessellation per part (G0), the face matching `Explode` already runs (25 ms for the
crossbow's 51 parts, on cached meshes), one box test per pair, and at most 256 `DistanceToPoint` calls (128 points,
two bodies each) per `Contact` or `Keepout` joint, and per `Unexplained` pair under `WithWitnessSearch`. Each call
reads the body's cached verified mesh. There is no pair proof and no all-pairs distance. The witness search runs
pairs concurrently, since `DistanceToPoint` is a read of immutable bodies.

### 7.3 Inference

`Infer` runs M1–M5 over every candidate pair, in both directions (a pin of either part in a receiver of the other),
and returns each match as a `PinInBore` or `PinInSlot` joint with its measured clearance. Each match of a pair is
returned, not only the best of M6: a joint name is `A~B`, with `#2`, `#3` for further matches of one pair. It reads
faces from the `VerifyNone` ladder of M8, as `Explode` does. Coplanar `FaceContact` inference is deferred: it needs the overlap area of two planar faces'
trims, a 2D question for `sketch` (a private scene of both loops), and no consumer needs it before the gallery's
declared laps are checked. Touching pairs without a cylinder match appear in `Unexplained` instead.

### 7.4 Moving joints and the linkage check

`Check` judges the assembled pose only. Motion is `VerifyLinkage`'s (`agent-docs/linkage-check-design.md`), which
`parts` consumes and does not repeat. `Set.Linkage(ground, r)` builds the linkage:

| Rule | Decision |
|---|---|
| K1 | Parts joined by `Fixed` pin, slot and face joints form one link, found by union-find over those joints; a `Contact`, `Tie` or `Keepout` joint holds nothing rigid. The ground part's group is `Linkage.Ground()`, whose bodies belong to no link and stay static. |
| K2 | Walking the remaining joints breadth-first from the ground in `Join` order: a `Turns` joint becomes `Link.Revolute` about its axis from `r`; a `Slides` joint becomes `Link.Prismatic` along its slot direction (the axis crossed with the slot wall's normal), its `Direction`, or for a pin in a bore its axis. The axis is `JointResult.Axis` from `r`, else the joint's declared `Axis`; `r` may be nil. `TurnsAndSlides` is unplaced: the revolute-and-prismatic chain of `linkage-check-design.md` §1 needs an intermediate link that carries a body, and a part joint supplies none. |
| K3 | Every `PinInBore`, `PinInSlot`, `FaceContact`, `Contact` and `Tie` joint between bodies of different links, or of a link and a static body, is declared with `Linkage.DeclareJointContact`, once per pair. A `Keepout` is not: its parts must stay apart. |
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

Parts sharing a `Spec` must agree: they must have the same `Kind` and material name, and their volumes must overlap
within their bounds, or `BOM` returns `decad.ErrDegenerate` naming the two parts. That check only falsifies the
caller's claim that the parts are the same. A part with a body counts one and a body-less part counts its `Quantity`.
`Mass` is the solid body's mass: the volume and its bound times the density. `Add` refuses a density that is not a
finite, non-negative `units.Density` (`decad.ErrUnitKind`, `decad.ErrNotFinite`, `decad.ErrNegativeMagnitude`).

### 8.2 Summary

`Summary.WriteMarkdown` writes, in this order: the bill of materials table; the joint table with statuses and reasons
when `Joints` is set; the `Unexplained` pairs; the exploded depth and pull table when `Exploded` is set; the Mermaid
diagram. Numbers print with their bounds. The joint table lists the Set's joints with the status the report gives;
the cause, the evidence (`Measured`, `Engaged`, `Gap`, faces read) and the `Unexplained` table come with `Check`
(P4a), and `Near` with P4b. From P7 the exploded table adds the stage, the moved distance, the clearance, the path
state and its blocker for each part, and lists `Short`. `WriteMarkdown` takes no context: it reads volumes and faces
only.
This is the agent's primary view of the schematic: it reads a
page, not pixels. Images come from the consumer (§11).

## 9. Reuse

| Need | Existing facility | New code |
|---|---|---|
| Cylinder and plane geometry and normals | `Face.Surface`, `Face.NormalAt` | the matching of M3, M4 |
| Triangles and their source faces, the mesh bound | `Body.Tessellate`, `Mesh.SourceFaces`, `Mesh.Bound` | none |
| Pull-path clearance for explode | `Document.SweptBox`, `SweptBox.StrictlyDisjoint` | the sense choice of E3, the path states of S7 |
| Stage membership by group | `Set.Group` over `Document.ContainerOf` | none |
| Gap bounds for a pair | `Body.Bounds` with its `Bound`, `Body.DistanceToPoint` on the body's cached verified mesh (`agent-docs/point-containment-design.md`) | the box gap of G1, the vertex sample of G2 |
| Pair proof, when the caller has one | a `decad.Report` from `Document.Verify` with `WithClearances` (`agent-docs/clearance-design.md`), passed by `WithVerifyReport` | the reading of R1 |
| Motion of joints | `decad.Linkage`, `Link.Revolute`, `Link.Prismatic`, `Linkage.DeclareJointContact`, `VerifyLinkage` | the bridge of §7.4 |
| Grouping | `Document.ContainerOf` | none |
| Volume, mass | `Body.Volume`, `units` arithmetic | none |
| Transforms | `r3.Translation`, `Then`, `Apply`, `ApplyDir` | none |

New internal packages, each taking plain data (`r3.Vec`, slices, floats in millimetres):

| Package | Holds |
|---|---|
| `internal/jointinfer/` | cylinder and slot matching of M1–M6 on plain cylinder and plane records |
| `internal/explodeplan/` | the breadth-first walk, travels and offsets of E1–E4, and the port points of V3; from P7 the stage runner of S1–S9 (interval union, union boxes, clearances, scale) and the projected-box overlap of `Hidden`, all on plain boxes, with the swept-box question still asked through its `Blocker` interface |
| `internal/jointcheck/` | the per-kind rules J1–J5 and G1, G3–G5 on plain matches, boxes and distance intervals; the vertex sample of G2; the overlap witness of J3; the unexplained screen of U1 (P4a) |

`parts` keeps only the adapters: it reads faces, boxes and meshes, calls `Body.DistanceToPoint` for the sample points
`internal/jointcheck` chooses, and hands the intervals back.

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
same way. `agent-docs/format-io-design.md` gains a section for it in PR E1. With each `ExplodedPart.Placement` as an
item's placement it writes the exploded view as a file any 3MF viewer opens.

## 10. Gate analysis

`agent-docs/module-boundary.md` must pass before implementation. This section is the evidence each PR quotes.

### 10.1 Reusable CAD behavior

| Gate item | Evidence |
|---|---|
| Input class stated | Axis matching: faces whose `Surface` is a `Plane` or `Cylinder`, exact or retained with a bound. Explode and views: any live solid body with bounds and a boundary-proved tessellation. Checking: the same faces for pin, slot and face joints, and any solid with bounds, plus a `VerifyAll` tessellation for point witnesses, for contact and keepout joints. 3MF items: any bodies `ThreeMF` writes, under any proper rigid placement. |
| Behavior by geometric invariants | Coaxiality and radius difference (M3), opposed parallel planes (M4), box travel and swept-box disjointness (E3), box clearance intervals along a direction and box distances (S5, S9), projected box overlap (`Hidden`), box overlap centres (V3), box gaps and point-distance witnesses (G1, G2), plane distance and a shared-area witness (J3). None names a part. |
| No part-specific or process-specific dispatch | No route reads a name, a fixture, a gallery model or a manufacturing process. Part names are labels only. |
| Independent cases | §12 lists a pin and plate, a press fit, a slider in a groove, a lap with two pins, a pin through two plates, two touching blocks, two L-shaped plates whose boxes overlap but whose faces do not, a keepout pair with meeting boxes, a faceted-face joint, a rotated copy, and for P7 a housing holding a lever, a group lifted as one, a direction that never clears and a scaled layout; each varies the geometry its rule reads. Stage membership is the caller's naming, never a route the package chooses by part. The crossbow is an acceptance run in the gallery, never a decad fixture (`agent-docs/testing.md` "Test scope"). |

### 10.2 Root package placement

| Gate item | Evidence |
|---|---|
| Private algorithms in `internal/` | §9's new algorithms live in the three internal packages, which take plain data. |
| No handle passed to reach data | `parts` unwraps `*decad.Body` and `*decad.Face` through public queries into plain records before calling an internal package. |
| Root adapters | None. Neither `parts` nor `export.ThreeMFItems` or `export.STEPItems` (`agent-docs/step-export-design.md` §10) adds a file to the root package. |
| Exported types | In `parts` and `export`, both above the root (§2). |

### 10.3 House rules this design keeps

- Every scalar is a `units.Value` or a `decad.Measurement`; coordinates are `r3.Vec` (`api-design.md` §5.1, §5.2).
- No triangle, index or bare float leaves the package: joints name faces, views and placements name parts
  (`api-design.md` §3).
- No operation mutates the document; `Set` is a builder (`api-design.md` §6.3).
- Every check that reads the geometry either falsifies a declaration or states an interval.
- No 2D question about recorded entities is answered in decad: J3's overlap witness is a point-in-triangle test on
  mesh chords with stated bounds, and coplanar-face inference (§7.3), which would need the trims' overlap, is
  deferred to `sketch`.
- `Check` never runs a pair proof, so it never claims one: no status means "interiors disjoint" unless a caller's
  `Verify` report says so (R2).
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
| `TestCheckPins` | Ø6 in Ø6.4: `Confirmed`, `Measured` 0.2, `Engaged` 10; declared 0.3 `Contradicted` (`CauseClearance`); a declared axis 1 mm off `Contradicted` (`CauseAxis`); a pin 1 mm off the bore's axis `Contradicted` (`CauseNoMatch`) with `Reason` naming both radii; a Ø6.1 pin in a Ø6 bore declared −0.05 `Confirmed`, undeclared `Contradicted` (`CauseInterference`); a pin 5 mm clear of the plate `Contradicted` (`CauseApart`) | P4a |
| `TestCheckPinInSlot` | a Ø4 pin in a 4.4 mm groove `Confirmed` 0.4; walls not parallel to the pin `Contradicted` | P4a |
| `TestCheckUnread` | a bore whose face reads `Faceted` (a rotated copy past retention) makes its pin joint `Undecided` (`CauseUnreadFace`), never `Contradicted`; its declared `Axis` is reported | P4a |
| `TestCheckFaceContact` | two stacked blocks `Confirmed` with a witness on the shared plane; blocks 0.5 mm apart `Contradicted` (`CauseApart`); two L-shaped plates on one plane whose vertex boxes overlap but whose faces do not stay `Undecided` (`CauseNoWitness`), never `Confirmed` | P4a |
| `TestCheckUnexplained` | touching blocks with no joint are listed with their box overlap; with a `FaceContact` they are not; blocks 1 mm apart are not listed (U3) | P4a |
| `TestCheckNoVerify` | `Check` on the pin and plate leaves the document's history and live set as they were, and its result is the same with and without `WithVerifyReport` when the report adds nothing | P4a |
| `TestGraphStatus` | edge labels end with status and cause; contradicted edges use `--x`, undecided ones `-.->` | P4a |
| `TestCheckContact` | a block resting on another `Confirmed` with a witness whose `Gap` upper end is within 0.25 mm; blocks 2 mm apart `Contradicted` (`CauseApart`); a contact between parts meshed coarser than the tolerance allows `Undecided` ("below resolution", G5) | P4b |
| `TestCheckKeepout` | blocks 5 mm apart with `MinGap` 2 `Confirmed` by boxes; L-shaped parts whose boxes meet but whose nearest points are 1 mm apart `Contradicted` (`CauseTooClose`) by a witness; the same parts 3 mm apart `Undecided` (`CauseNeedsPairProof`), then `Confirmed` with a `WithVerifyReport` report | P4b |
| `TestCheckWitnessSearch` | `WithWitnessSearch` sets `Near` on an unexplained touching pair and not on a pair whose boxes meet while its bodies stay 1 mm apart | P4b |
| `TestLinkage` | a hinge and a slider become a revolute and a prismatic link with declared contacts; a loop joint comes back unplaced; `VerifyLinkage` runs on the result | P5 |
| `TestInfer` | a lap with two pins infers both pins and nothing else; a rotated copy infers through retained tags (M5) | P6 |
| `TestExplodeStagesDefault` | no stages gives the offsets of P1's `TestExplode` exactly, with every P7 field filled: `Stage` "", `Gap` 10 mm, `Moved` the offset's length | P7 |
| `TestExplodeStagesHousing` | the §4 housing: the one-stage overlap, the three stage offsets, `Travel` 24 and `Moved` 34 for the lever, `PathEnclosed` with both blockers, zero `Overlaps` and `Short` | P7 |
| `TestExplodeClearAllIntervals` | a box moving through a row of three standing boxes with gaps of 3, 8 and 30 mm between them and a 5 mm gap asked: it stops in the 30 mm space, not the 8 mm one, at the travel the interval union gives | P7 |
| `TestExplodeTogether` | two parts lifted as one keep their relative place; a later stage takes them apart from there; `Leaders` follow the final offsets | P7 |
| `TestExplodeStuck` | a stage direction along which a box never leaves a longer standing box reads `PathStuck` and leaves the part at its start | P7 |
| `TestExplodePartGapAndScale` | `WithPartGap` changes one part's offset only; scale 2 doubles every offset, recomputes `Clearance`, and puts two parts that moved the same way into `Short` | P7 |
| `TestExplodeStageRefusals` | every S10 refusal, each with its sentinel | P7 |
| `TestExplodeHidden` | looking down −Z at the housing before and after staging: the top half covers the whole lever (`Covered` 1) before, nothing covers it after; a zero view is refused | P7 |
| `TestExplodeDeterminism` | two runs with the same stages give equal `Exploded` values and equal `export.ThreeMFItems` bytes | P7 |
| `TestExplodeJointPull` | a `Contact` with `Pull` +Y moves along +Y instead of the box-centre line and reads `PullDeclared`; a `Pull` on a pin picks the sense E3 would not; a `Pull` normal to the pin's axis is refused; a non-finite one is refused at `Join` | P7 |
| `TestExplodeGroundStage` | the ground in a stage with a `Direction` moves by its gap, so two halves open symmetrically; without a `Direction` it is refused | P7 |
| `TestExplodeLeaderSeparation` | the housing's pin leaders read the distance between the pin's box and each half's box; a pin left in its bore reads zero | P7 |

Examples: `examples/parts_explode_example_test.go` (P1), `examples/parts_views_example_test.go` (P2),
`examples/export_3mf_items_example_test.go` (E1), `examples/parts_summary_example_test.go` (P3),
`examples/parts_joints_example_test.go` (P4a) and `examples/parts_explode_stages_example_test.go` (P7), each with a
verified `// Output:` block.

**Acceptance case: the crossbow, in the gallery.** Per `agent-docs/testing.md` "Test scope", the 51-part model stays
in `decad-gallery`. Each phase is accepted when its real output passes through the real consumer there (§15): P1 by
an exploded image and a diagram of all 51 parts, P2 by a part sheet, P4a and P4b by the whole assembly's `Check`, P5
by its linkage.

**`Check` on the crossbow.** The model declares 82 joints: 46 `PinInBore`, 4 `PinInSlot`, 15 `FaceContact`, 9
`Contact` and 8 `Tie` (the bands). The figures below are dated illustrations from P1's acceptance run (2026-11) and
the model's layout, not measurements of `Check`:

| Kind | Joints | Expected | Basis |
|---|---|---|---|
| `PinInBore` | 46 | 40 `Confirmed`, 6 `Contradicted` (`CauseApart`) | `Explode` read 40 of the 46 axes from the faces. The other 6 pair a rivet with a receiver part it does not pass through (rivets 1 and 2 against the front receiver, rivet 3 against the two rear halves), from layout coordinates tens of millimetres apart, so their boxes are apart. They are joints the model declares wrongly, which `Check` should report. |
| `PinInSlot` | 4 | 4 `Confirmed` | all 4 axes came from the faces |
| `FaceContact` | 15 | at least 7 `Confirmed`; the rest `Confirmed` or `Undecided` (`CauseUnreadFace`, `CauseNoWitness`) | `Explode` pulled 7 parts along a matched contact face; the other 8 contacts were not a part's first joint, so P1 did not record whether they match |
| `Contact` | 9 | most `Confirmed` | parts at rest against each other by design; the count depends on G5's resolution at the model's 0.1 mm meshes and is the main unknown |
| `Tie` | 8 | 8 `NotChecked` | by J6 |

**Targets.** Of the 74 checked joints, at least 51 `Confirmed` (40 + 4 + 7), the 6 misdeclared rivet joints
`Contradicted`, no `Contradicted` joint the gallery cannot trace to a modelling error, and every `Undecided` one
carrying a cause. Time: P4a's structural rules under 5 s for the whole model after its parts are built (the face
matching alone takes 25 ms on cached meshes; `VerifyAll` tessellation of 51 parts dominates); P4b's witnesses under 60 s
more with `WithWitnessSearch`, against 9 min 57 s for `Verify`. A miss on either target stops the PR and is reported
with the measurement.

**Staged explosion on the crossbow (P7).** The gallery's hand-tuned layout (`StagedExplosion` in
`models/crossbow/schematic_explode.go`, dated illustration, 2026-11) shows what the stages must reproduce. The
one-stage `Explode` left the receiver halves, side plates and links in front of the mechanism; it pulled the hooks,
trigger, limbs and bands, whose joints are contacts or ties, along box-centre lines; 15 exploded box pairs met. The
hand layout opens the receiver symmetrically and sends the side layers out along ±Y in the order the parts come off
(receiver half 30 mm, side plate 70, hook 110, follower 140, link 170, hook pin 205; rivets 105 past the plate),
lifts the trigger train (95 mm), carriage (55), disc (110) and lever (150) along +Z, sends the bow and fore-end
forward along +X and the butt back. It leaves 2 meeting box pairs, both band strands, and every pin and slot joint's
pin at least 20 mm from the part it passes through (58 joints pass). Its two stated needs were a distance scale or
per-group distance, and a pull direction on a contact or tie joint; `WithExplodeScale`, `Stage.Gap`, `WithPartGap`
and `Joint.Pull` answer them.

The stages that reproduce it, ground `receiver-right` opened by its own stage, with `Joint.Pull` declared on the
contact and tie joints of the hooks, trigger, limbs and bands (±Y by side for the hooks, +Z for the trigger, ±Y then
+X for the limbs) and on the receiver halves' face contact:

| Stage | Members | Rule | Expected separation |
|---|---|---|---|
| open | `receiver-right` | `Direction` −Y, gap 30 mm | the ground moves 30 mm (S10) |
| shell | `receiver-left`, `stock-rear`, `receiver-front`, the three stock pins | `ClearJoined`, gap 30 mm, joint directions | each 30 mm past the part it left |
| plates | both side plates and the six rivets | `ClearAll`, gap 15 mm, joint axes, sense from `Pull` | every box 15 mm from every standing box |
| hooks | both hooks | `ClearAll`, gap 15 mm, `Pull` ±Y | outside the plates |
| followers | both followers | `ClearAll`, gap 15 mm, joint axes | outside the hooks |
| links | both links | `ClearAll`, gap 15 mm, `Pull` ±Y | outside the followers |
| hook pins | both hook pins | `ClearAll`, gap 15 mm, joint axes | outside the links |
| train | release nut, sear, trigger, trigger bar, safety and their five pins | `Together`, `ClearAll`, gap 20 mm, `Direction` +Z | the group's box 20 mm above everything standing |
| top | carriage, disc, lever, lug and lever pins | `ClearAll`, gap 20 mm, `Direction` +Z | each 20 mm above everything standing |
| train pins | the train's five pins again | `ClearAll`, gap 10 mm, joint axes | each 10 mm out of the lifted train |
| "" | the bow, its pins and posts, the band strands | `ClearJoined`, `WithExplodeGap`, `Pull` on the limbs | forward of the fore-end |

Eleven stages for 51 parts replace 40 hand-set offsets; every distance comes from the boxes, so a model edit moves
the layout with it. Targets, each read from `Exploded`, not from an image:

- `Overlaps` holds no pair of two `Manufactured` parts after the last stage; pairs with a band strand are listed and
  accepted, because a strand runs between parts that move apart. The hand layout's 2 is the bar to match.
- Every `PinInBore` and `PinInSlot` `Leader` has `Separation` at least 20 mm, as the hand layout achieves.
- `Short` names no part of the open, shell, plates, hooks, followers, links, hook pins, train or top stages.
- `Hidden` along each of the gallery's four exploded cameras reports no `Covered` of 1 for the release nut, sear,
  trigger bar, hooks, carriage or links.
- `Explode` with the eleven stages takes under 0.5 s on the built model (one stage takes about 30 ms, dated
  illustration), and two runs give equal exploded 3MF bytes.

The table is a plan, not a measurement. The first P7 crossbow run (2026-11, ground `receiver-right`, the gallery's own
`Schematic` joints with `Pull` declared) reached every target with thirteen stages: the table's eleven, with the
train's five parts moving one by one (`Together` kept the interlocked release nut and sear overlapping, so it is not
used for the train) and the bow in two stages of its own (risers and limbs along +X, then their pins on their axes),
and with every gap 20 mm except the shell's 10 for `receiver-front` by `WithPartGap`. It left no two manufactured parts
meeting (two band-strand pairs meet), every pin and slot leader at least 20 mm, no `Hidden` cover of 1 on the release
nut, sear, trigger bar, hooks, carriage or links along the gallery's four cameras, `Short` naming only the bow pins and
the band strands, and the largest offset 287 mm (the lug pin; the hand layout's 205 mm). Explode took 38 ms and two
runs wrote equal 3MF bytes.

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
| E1 | `export.ThreeMFItems` and its section in `agent-docs/format-io-design.md` | — | 300 |
| P2 | `Views` and ports; views example | P1 | 350 |
| P3 | `BOM`, `Summary`; summary example | P1 | 500 |
| P4a | the §7.2 design delta; `internal/jointcheck`; `Check` for `PinInBore`, `PinInSlot`, `FaceContact` and `Tie` (G0, G1, G3, G4, J1–J3, J6), `Unexplained` by boxes (U1, U3), `JointReport`, `CheckRequest`, `Cause`; M8's finer rungs; edge statuses in `Graph`; the joint table's cause and evidence columns in `Summary`; joints example | P1–P3 (landed) | 900 |
| P4b | point witnesses (G2, G5), `Contact` and `Keepout` (J4, J5), `WithWitnessSearch` (U2), `WithVerifyReport` (R1–R3) | P4a | 600 |
| P5 | `Set.Linkage` | P4a | 450 |
| P6 | `Infer` and `JointReport.Inferred` | P4a | 300 |
| P7 | the §5.2–§5.6 design delta; `Stage`, `Clearing`, `PathState`, `Occlusion`; `Joint.Pull` and `PullDeclared`; `WithExplodeStages`, `WithPartGap`, `WithExplodeScale`; the stage runner in `internal/explodeplan` (S1–S9), the ground in a stage with a `Direction`; the P7 fields of `Exploded`, `ExplodedPart` and `Leader`; `Exploded.Hidden`; the exploded table's new columns in `Summary`; staged example | P1–P6 (landed) | 850 |

P1 to P6 have landed. P7 is one PR; `Hidden` (about 100 lines with its test) can leave it for a follow-up if review
asks for a smaller one.

**Parallel work.** P4a comes first: it decides the pin, slot and face joints, 65 of the crossbow's 74 checked joints,
without a single distance query. After P4a, P4b, P5 and P6 run in parallel; none edits another's rules. P5 reads
only `JointResult.Axis`, which P4a fills.

P1 is the largest PR because the exploded view needs the axis matching; splitting it further would ship a `Set`
nobody can look at. If review asks for a smaller first PR, P1 splits into P1a (`Set`, `Join`, `Graph`,
`WriteMermaid`: the diagram, 500) and P1b (`internal/jointinfer`, `Explode`: the exploded view, 800).

Before P1 grows past its first case, `Explode` runs on the real 51-part crossbow and the count of joints whose `Pull`
is `BoxCentres` (no axis from faces or declaration) is recorded. Before P4a grows past the pin case, `Check` runs on
the real crossbow and its time and status counts are recorded against §12's targets; before P4b grows past the
`Contact` case, the same for the witnesses. Before P7 grows past the housing case, the gallery's crossbow runs the
stages of §12 and its `Overlaps`, `Short` and timing are recorded. If a run misses its target, expansion of that PR
stops and the blocker is reported.

## 14. Risks and open questions

| Risk or unknown | What settles it |
|---|---|
| A face of a mesh Boolean result keeps a `Cylinder` tag only where retention proved it; other faces read `Faceted`. The crossbow's moving parts are placed by rotations about +Y, so their bores may read as displaced retained tags or as `Faceted`, and M3 may not match them. | P1's crossbow run counts `Pull` sources. A declared `Axis` covers each unmatched pin; the exploded view stays right, and `Pull` shows which axes were stated rather than read. |
| `Verify` is slow and mostly undecided on a real build (9 min 57 s, 64 of 82 crossbow joint pairs undecided), so `Check` does not use it. Its pair proof stays the only proof that interiors are disjoint. | `Check` is structural (§7.2). Faster, more decisive pair proofs are clearance-kernel work, a decad change outside `parts`; `WithVerifyReport` reads one when a caller has it. |
| `DistanceToPoint`'s enclosure width on a faceted part is unknown; if it is much wider than the mesh bound, G5 leaves most `Contact` joints `Undecided`. | P4b's first crossbow run measures the witness sums for the 9 contacts. |
| The cost of `VerifyAll` tessellation and of `DistanceToPoint` per call on the crossbow's parts is unmeasured. The gallery already meshes all 51 parts with `VerifyAll` at 0.1 mm in its tests. | The P4a and P4b crossbow runs against §12's targets. |
| A 64-point sample per side can miss a small touching spot, leaving a true contact `Undecided`. | `CauseNoWitness` says so; raising the sample is a constant to revisit with the P4b measurement. |
| E3's box test is coarse: a long pin's swept box may meet a neighbour's box though the pin's path is clear. | E3 then falls back to the shorter travel, and `Overlaps` reports any exploded boxes that still meet. |
| Boxes are a poor stand-in for long, diagonal or L-shaped parts: a crossbow link's box covers much empty space, so `ClearAll` can push parts far past what the eye needs, and `Overlaps` can list pairs that do not touch. | The P7 crossbow run shows how wide the result gets. Clearing against meshes instead of boxes is a later option if boxes prove too coarse; P7 does not add it. |
| `ClearAll` along an axis can send a part through the far side of a wide enclosure; the move is legal, and `PathEnclosed` says the box test could not judge the path. | A stage `Direction` or an earlier stage that opens the enclosure; the caller chooses. |
| A `Pull` declared on a joint states the author's intent, not geometry; a wrong one sends a part the wrong way. | `Pull` reads `PullDeclared` in the output, and the `Overlaps`, `Short` and `Separation` readings show a bad result. |
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
5. After P4a: the six rivet joints `Check` contradicts are fixed in the model (dropped, or made `Contact` where a rivet
   head does rest on the part). `designedContacts` is deleted; `TestPoseClearance` asserts no joint is `Contradicted`
   and records the `Undecided` count and the `Unexplained` pairs. After P4b, with `WithWitnessSearch`, it asserts that
   no `Unexplained` pair has `Near` set. `internal/meshgap` stays for the poses the cycle tests sweep, where the
   gallery needs a gap screen on every pair at every step, which `Check` does not do.
6. After P5: the model's motion checks read their joints from the same `Set`.
7. After P7: `StagedExplosion` and its offset table are deleted; the model declares `Pull` on the joints of §12's
   plan and passes its stages to `Set.Explode`,
   the exploded image and the exploded 3MF read the same `Exploded`, and `TestSchematicExplode` asserts the §12
   targets on `Overlaps`, leader `Separation`, `Short` and `Hidden`.

**Printing checks stay in the gallery.** `TestPrintSize` (every printed part within a 200 mm cube),
`TestPrintWithoutSupports` (down-facing faces only on the bed in a chosen build direction) and the pins' "lying on
their side" check remain gallery tests in `models/crossbow`, built on public decad queries (`Body.Bounds`,
`Body.Tessellate`, `Mesh.SourceFaces`). If several gallery models need them, they move to a gallery package such as
`decad-gallery/internal/printcheck`. Print plates, if the gallery wants them, are `decad.Arrange` on copies of the
parts plus `export.ThreeMFItems`. Nothing in decad hosts a printing check.
