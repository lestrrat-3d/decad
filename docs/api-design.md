# Public API Design

The design of decad's public API: a headless CAD engine an agent uses to model a
part, prove it sound, and only then write real CAD software code (a Fusion
add-in). This document is the contract. No public API lands that contradicts it.

## 1. What decad is answerable for

decad is **not** a CAD kernel competing with Fusion. It is a **proxy that must
be right about a specific set of questions**:

- Does this profile close, and does it sweep into the solid I meant?
- Is the body watertight? Manifold? Does it self-intersect?
- What is its volume, area, centroid, bounding box?
- Do these bodies interfere, and by how much? What is the clearance?
- Is any wall thinner than the tool that has to cut it?

The failure mode to fear is not *limited*. It is **confidently wrong**. Fusion
ships confidently-wrong: its physical properties carry a ±1% error margin by
default, and its `MinimumRadiusAnalysis` / `DraftAnalysis` / `AccessibilityAnalysis`
classes — which look exactly like the primitives an agent wants — are empty
classes that return no data at all. An agent that trusts those is worse off than
one that never asked.

**Every answer decad gives states how much it can be trusted.**

## 2. Architecture: recipe, evaluator, result

Three layers. The split is what makes an exact-kernel future reachable.

```
Recipe      declarative, exact, kernel-independent, serializable
            sketch profiles + features + selectors + parameters
   |
   v
Evaluator   swappable
            v1: analytic where free, tessellation-backed booleans, exactness flagged
            vN: full analytic B-rep
   |
   v
Result      Body -> Lump -> Shell -> Face -> Loop -> Edge -> Vertex
            every measurement carries Exactness
```

**The recipe is never approximate.** "Extrude this closed profile 10mm, then
fillet the convex vertical edges at 2mm" is an exact statement of intent and stays
true forever. Approximation lives only in the evaluator — the one thing we intend
to replace.

So vN is **not a migration**. It is a second evaluator over the same recipe.
Existing models re-evaluate and get better answers.

The recipe is also **the thing that translates into Fusion code** — it is the
library's actual deliverable, so it is a first-class inspectable value.

### 2.1 Why the boolean is the only place exactness dies

Feature-generated faces are analytically exact **by construction, with no
intersection math**: extrude a straight sketch line and the face *is* a plane;
extrude an arc and it *is* a cylinder; revolve a line and it *is* a cone. That
falls out of the sketch we already have — free, and already proven closed by
`sketch`'s solver.

Combining two bodies is the hard part. Cylinder ∩ cylinder at an arbitrary angle
is a quartic space curve with no closed form, so even production kernels store it
as a fitted approximation — and then decide **topology** (does this curve pass
*through* that vertex, or 1e-14 beside it?) from floating-point sign tests.

A flipped sign does not give a slightly wrong answer. It gives a **nonsensical**
one: a solid with a hole in its skin. The error is discrete, not continuous. This
is why kernels ship *tolerant topology* (Fusion exposes `BRepEdge.isTolerant` — a
production kernel admitting in its public API that its geometry does not meet),
and why every CAD user has seen "cannot perform boolean operation".

decad therefore computes booleans on a tessellated representation, where robust
algorithms are a solved problem, and **marks what it touched**.

## 3. Forward-compatibility invariants

These are cheap now and expensive to retrofit. They are the upgrade path.

| # | Invariant | Rationale |
|---|---|---|
| 1 | **NEVER expose triangles as the representation.** `Tessellate(tol)` is an output; the public vocabulary is `Body → Face → Edge → Vertex` even while the backing is approximate. | A public `Triangles()` is a one-way door; callers depend on it and vN can never remove it. |
| 2 | **Every measurement carries `Exactness`, from the first commit.** | Makes the upgrade **monotonic, not breaking**: callers already branch on `Approximate`; in vN that branch stops being taken and nobody's code changes. A bare `float64` today means adding exactness later breaks every call site. |
| 3 | **Selectors, never indices.** | An exact kernel produces a different (more correct) face/edge decomposition. If `Edges()[3]` is the API, index order becomes a de facto contract and vN breaks every model. |
| 4 | **Booleans are pure functions.** `Union(a, b) (*Body, error)` — never Fusion's in-place `booleanOperation(target, tool, type) -> bool`. | A pure signature lets the implementation be swapped with zero API churn. |
| 5 | **Imported meshes are a separate type.** A `MeshBody` never claims to be a solid B-rep. | For imported triangle soup, no future kernel can recover exactness. Keeping it separate stops approximate-forever geometry from contaminating the type we promise to make exact. |

**What will still churn**, stated honestly: face/edge *counts* may change under an
exact kernel (canonicalize aggressively — merge coplanar faces — so v1 counts
already match the analytic answer); numeric outputs shift (`12.9997 → 13.0000`, so
tests compare with tolerances, never goldens); and vN's surface set is a superset,
so a `switch` on `Surface` MUST have a `default` branch.

## 4. What we take from Fusion, and what we reject

Loosely based on the Fusion SDK. The odd parts are thrown away deliberately.

### Keep

| Idea | Why |
|---|---|
| B-rep topology `Body → Lump → Shell → Face → Loop → Edge → Vertex`, co-edges as half-edges | The right model, and it makes agent traversal code map 1:1 onto Fusion's. |
| Face geometry as a **tagged surface** (`Plane`/`Cylinder`/`Cone`/`Sphere`/`Torus`/`NURBS`), not everything-is-NURBS | Preserves intent: "this hole is exactly Ø6", not "about 5.997". |
| `Profile` as a derived, planar, one-outer-loop-plus-N-holes region back-referencing its source curves | Already exactly what `sketch.Profile` is. |
| Feature owns the faces it created (`Feature.faces`) | The basis of provenance and of selectors. |
| `isTangentChain` / rule-fillets — *selection as a rule, not a list* | Fusion's own admission that pointer-selection does not survive a rebuild. We take the lesson further: selectors are first-class. |
| `containsApproximation` on projected outlines | The single best idea in the API. Fusion does it in one place; decad does it everywhere as `Exactness`. |

### Reject

| Fusion artifact | Why it exists | decad instead |
|---|---|---|
| `ObjectCollection` (+ `XxxList` + `Xxxs`) | COM cannot marshal native lists | `[]T`, variadic |
| `createInput()` → `setByX()` → `add(input)` ceremony | No overloading/named args in COM; mirrors modal GUI dialogs | One call, options struct / functional options |
| `ValueInput` (`createByReal(2)` for a taper silently means **2 radians**) | GUI expression fields pushed into the modeling API | `units.Value` — typed quantities that cannot be misread |
| Proxies: `nativeObject` / `assemblyContext` / `createForAssemblyContext` | Avoiding explicit transform arguments | Component-local coords + **explicit** transforms |
| `Base.isValid` (means "not deleted", NOT geometrically valid) | Live-kernel handles dying under you | Immutable bodies. NEVER ship an `IsValid()` with that meaning. |
| `healthState` + `errorOrWarningMessage`; `bool` returns; `-1` sentinels + `GetLastError()` | COM HRESULT conventions; deferred recompute | `(T, error)` |
| `core.Base` as a parameter type ("planarEntity") | No sum types in COM IDL | Sealed interfaces |
| Booleans folded into every feature (`FeatureOperations`) with `participantBodies = nil` meaning "guess" | Convenient in a UI | **Explicit** boolean ops with an explicit target |
| Mutable `Point3D`/`Vector3D`/`Matrix3D` with in-place mutators | COM reference objects | `r3.Vec` immutable value structs |
| GUI state on the geometry model (`isLightBulbOn` ×8, `displayBounds`, `opacity`, `Timeline.play()`) | The API is a scripting shim over the GUI | Keep view state out entirely |
| `LowCalculationAccuracy` default (±1%) | UI responsiveness | Accuracy is explicit; verification NEVER defaults to loose |
| `boundingBox` (knowingly loose) beside `preciseBoundingBox` | Legacy | One `Bounds()`, tight. |
| `volume == 0` meaning both "empty" and "not a solid" | Sloppiness | `(Measurement, error)` |

## 5. Foundations

### 5.1 Units — no naked floats

Every physical quantity crossing the API is a `units.Value` from
`github.com/lestrrat-3d/sketch/units`. We do NOT invent a parallel unit system.

```go
body, err := doc.Extrude(prof, decad.Distance(units.Millimeters(10)))
```

`units.Millimeters(6)` can never silently mean centimeters — which is precisely
the trap in `ValueInput.createByReal`, where the value is always internal units
(cm, radians) regardless of the document. A wrong-`Kind` value (an angle where a
length is wanted) is an **error**, not a coercion.

### 5.2 Coordinates — r3, never hand-rolled

`r3.Vec` and `r3.Frame` only. `Frame` is orthonormal, so the inverse is the
transpose, never a matrix solve.

> **Open, and a dependency-side decision:** `r3` has no general **rigid transform**
> type — `Frame` covers plane-local↔world only. Placing a body at an arbitrary pose
> needs rotation + translation. This belongs in `r3` by its own charter ("what
> *acts on* 3-space"), so the proposal is to add `r3.Transform` upstream rather
> than hand-roll one here. **Blocks body placement; needs a decision before that
> lands.**

### 5.3 Exactness — the load-bearing type

```go
type Exactness int

const (
    Exact Exactness = iota // analytic; the number is the truth
    Approximate            // tessellation-derived; Bound holds the error
)

// Measurement is a quantity plus how far it can be trusted.
type Measurement struct {
    Value     units.Value
    Exactness Exactness
    Bound     float64 // absolute error bound in base units; 0 when Exact
}
```

Every measurement returns one:

```go
vol, err := body.Volume()  // v1 after a boolean: {12.9997mm³, Approximate, 1e-3}
                           // vN:                 {13.0000mm³, Exact,       0}
```

## 6. The document and bodies

Immediate mode. **The agent's Go function is the feature tree** — re-running
`MakeBracket(height)` with a new height *is* the rebuild. A Fusion add-in script is
imperative too, so immediate-mode Go resembles the target artifact more closely
than a timeline API would. We do not build an interpreter for a language the agent
already has.

```go
type Document struct{ /* ... */ }

func New(opts ...DocumentOption) *Document

func (d *Document) Bodies() []*Body            // live bodies
func (d *Document) Recipe() Recipe             // the exact record of intent
func (d *Document) Verify(ctx context.Context, opts ...VerifyOption) *Report
```

`Body` is **immutable**; every operation returns a new one, and the input body is
retired from the document.

```go
type Body struct{ /* ... */ }

func (b *Body) Lumps() []*Lump    // disjoint pieces; len > 1 means disconnected
func (b *Body) Shells() []*Shell  // Shell.IsVoid() marks an internal cavity
func (b *Body) Faces() []*Face
func (b *Body) Edges() []*Edge
func (b *Body) Vertices() []*Vertex

func (b *Body) IsSolid() bool
func (b *Body) Bounds() Box       // tight, always
func (b *Body) Volume() (Measurement, error)   // error when not a solid — never 0
func (b *Body) Area() (Measurement, error)
func (b *Body) Centroid() (r3.Vec, Exactness, error)

func (b *Body) Origin() FeatureRef  // which feature created this body
```

### 6.1 Topology

```go
type Face struct{ /* ... */ }

func (f *Face) Surface() Surface   // sealed; see below
func (f *Face) Loops() []*Loop     // Loop.IsOuter() distinguishes outer from holes
func (f *Face) Edges() []*Edge
func (f *Face) Area() (Measurement, error)
func (f *Face) NormalAt(p r3.Vec) (r3.Vec, error)
func (f *Face) Origin() FeatureRef // provenance: the feature that created it

type Edge struct{ /* ... */ }

func (e *Edge) Curve() Curve
func (e *Edge) Faces() []*Face     // len != 2 on a closed body means NON-MANIFOLD
func (e *Edge) Start() *Vertex
func (e *Edge) End() *Vertex
func (e *Edge) Length() (Measurement, error)
func (e *Edge) IsConvex() bool
```

`Surface` and `Curve` are **sealed** interfaces (unexported method), matching the
house idiom in `sketch.Entity` — the answer to Fusion's `core.Base`-as-anything.

```go
type Surface interface {
    Kind() SurfaceKind
    surface() // sealed
}

type Plane struct    { Frame r3.Frame }
type Cylinder struct { Origin, Axis r3.Vec; Radius units.Value }
type Cone struct     { Origin, Axis r3.Vec; Radius, HalfAngle units.Value }
type Sphere struct   { Center r3.Vec; Radius units.Value }
type Torus struct    { Center, Axis r3.Vec; Major, Minor units.Value }
type NURBSSurface struct { /* ... */ }

// Faceted is the honest v1 variant: a face a boolean produced, whose analytic
// identity is gone. Its presence is exactly why a measurement reads Approximate.
type Faceted struct{ /* ... */ }
```

A `switch` on `Surface` MUST carry a `default` — vN adds variants.

## 7. The sketch seam

`sketch` answers every 2D question; decad consumes the answer and NEVER
re-derives it.

```go
w := sketch.NewWorld()
s, _ := w.CreateSketch(w.XY())
s.CreateRectangle(0, 0, 100, 60)
s.Solve(ctx)

// sketch has already proven this closes, is fully constrained, is extrudable.
prof := s.Profiles()[0]

doc := decad.New()
body, err := doc.Extrude(prof, decad.Distance(units.Millimeters(10)))
```

`doc.Extrude` REJECTS a `sketch.Profile` whose `Valid` is false — a
self-intersecting or degenerate region is never silently swept. The plane's
`r3.Frame` lifts the plane-local profile into world space.

## 8. Features

v1 vocabulary, deliberately small: **Extrude, Revolve, Union/Cut/Intersect,
Fillet, Chamfer, Shell**. Sweep and Loft are deferred.

```go
func (d *Document) Extrude(p *sketch.Profile, e Extent, opts ...ExtrudeOption) (*Body, error)
func (d *Document) Revolve(p *sketch.Profile, axis Axis, a Extent, opts ...RevolveOption) (*Body, error)
```

Booleans are **explicit and pure** — not folded into every feature with an
ambient, implicitly-chosen target:

```go
func Union(a, b *Body) (*Body, error)
func Cut(target, tool *Body) (*Body, error)
func Intersect(a, b *Body) (*Body, error)
```

Modify operations return a new body:

```go
func (b *Body) Fillet(sel EdgeSelector, r units.Value) (*Body, error)
func (b *Body) Chamfer(sel EdgeSelector, d units.Value) (*Body, error)
func (b *Body) Shell(sel FaceSelector, thickness units.Value) (*Body, error)
```

### 8.1 Extent — illegal states unrepresentable

Fusion has three mutually-exclusive `setXxxExtent` methods with no enforcement,
and `add()` fails at runtime. decad makes exclusivity structural:

```go
type Extent interface{ extent() }

type Distance   struct { D units.Value }
type Symmetric  struct { D units.Value; FullLength bool }
type TwoSided   struct { One, Two Extent }
type ThroughAll struct { Dir Direction }
type ToFace     struct { Face *Face; Offset units.Value }
```

Options carry the rest (`WithTaper(units.Degrees(3))`), via
`github.com/lestrrat-go/option/v3` — the house functional-options library, and a
new approved dependency.

## 9. Selectors — intent, not identity

The topological-naming problem, in Fusion's own words: `tempId` is valid only
"as long as the owning BRepBody is not modified in any way", and entity tokens
"should never be compared". Handles do not survive an edit, and index order is not
stable. Its `isTangentChain` flag is the workaround; we take the lesson properly.

**A feature is given a query, never a pointer.**

```go
type EdgeSelector interface {
    SelectEdges(*Body) ([]*Edge, error)
}

func Edges(preds ...EdgePredicate) EdgeSelector
func Faces(preds ...FacePredicate) FaceSelector

// Predicates compose.
func Convex() EdgePredicate
func Concave() EdgePredicate
func ParallelTo(v r3.Vec) EdgePredicate
func LongerThan(l units.Value) EdgePredicate
func CreatedBy(f FeatureRef) EdgePredicate   // provenance
func Circular() EdgePredicate

func Planar() FacePredicate
func Cylindrical() FacePredicate
func NormalTo(v r3.Vec) FacePredicate
```

```go
body, err = body.Fillet(
    decad.Edges(decad.Convex(), decad.ParallelTo(r3.NewVec(0, 0, 1))),
    units.Millimeters(2),
)
```

**A selector that matches nothing is an error, loudly.** A fillet that silently
filleted zero edges is exactly the class of bug decad exists to catch. Cardinality
is assertable:

```go
decad.Edges(decad.Convex()).Exactly(4)   // errors unless it matches 4
decad.Edges(decad.Convex()).AtLeast(1)
```

This is also why the decad code and the eventual Fusion code stay structurally
parallel: a real Fusion script must pick edges by geometric predicate too.

## 10. Verification — the product

The 3D counterpart of `sketch.Verify`. One non-mutating call; a rich report; a
single-value `Status` with a fixed severity precedence; and one bit an agent gates
on. Deliberately mirrors `sketch.VerificationReport` / `WorldVerificationReport`.

```go
func (d *Document) Verify(ctx context.Context, opts ...VerifyOption) *Report

type Report struct {
    Bodies        []*BodyReport
    Interferences []Interference // pairwise overlap, with the overlap VOLUME
    Clearances    []Clearance    // opt-in: minimum gap between bodies
    Status        Status
}

func (r *Report) Trustworthy() bool // the single bit to gate on

type BodyReport struct {
    Body             *Body
    Solid            bool
    Watertight       bool
    Manifold         bool          // every edge bounds exactly 2 faces
    SelfIntersecting bool
    Lumps            int           // > 1 == disconnected pieces
    Voids            int           // internal cavities (Shell.IsVoid)
    Volume           Measurement
    Area             Measurement
    Centroid         r3.Vec
    Bounds           Box
    Exactness        Exactness     // the weakest link across this body

    // Opt-in, expensive:
    MinWallThickness *Measurement  // WithMinWallThickness(tool)
    Undercuts        []*Face       // WithPullDirection(v)
    MinRadius        *Measurement  // WithMinRadius() — can the endmill reach?
}
```

Fusion answers **none** of `Watertight` (with diagnostics), `Manifold`,
`SelfIntersecting`, `MinWallThickness` (B-rep), `Undercuts`, or `MinRadius`. That
gap is decad's mandate.

`Status` follows a fixed precedence so one check gates on the dominant condition:

```go
type Status int

const (
    Sound   Status = iota // solid, watertight, manifold, no self-intersection
    Suspect               // sound, but an answer is Approximate beyond tolerance
    Unsound               // not a valid solid
)
```

`Trustworthy()` is false for an unsound body, an unresolved interference, or an
approximation coarser than the caller's tolerance — even when the geometry "looks"
fine.

## 11. Export and translation

```go
func (b *Body) Tessellate(tol units.Value) (*Mesh, error) // an OUTPUT, not the representation
func (b *Body) STL(w io.Writer, opts ...STLOption) error
func (b *Body) OBJ(w io.Writer, opts ...OBJOption) error
```

**Fusion codegen is out of scope for v1.** `Document.Recipe()` exposes the exact
record of intent as inspectable data; emitting a Fusion add-in from it is a
follow-up (and possibly the agent's job, not the library's). The recipe is designed
to make that mechanical.

## 12. Errors and concurrency

- `(T, error)`. NEVER a `bool` return, a `-1` sentinel, or a deferred health state.
- Sentinel/typed errors for the cases an agent must branch on: `ErrNoMatch`
  (selector matched nothing), `ErrCardinality`, `ErrNotSolid`, `ErrDegenerate`,
  `ErrBooleanFailed`, `ErrInvalidProfile`, `ErrUnitKind`.
- `Body` is immutable → safe to read from many goroutines.
- `Document` owns mutable state and is NOT safe for concurrent mutation. `Verify`
  is non-mutating and safe.

## 13. Non-goals for v1

Assemblies (`Component`/`Occurrence` instancing — bodies with explicit transforms
cover interference and clearance without the DAG), a feature tree / timeline /
rollback, sweep and loft, STEP, sheet metal, mesh import, GUI or view state of any
kind, and Fusion code generation.
