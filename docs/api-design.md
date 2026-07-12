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
| 3 | **Selectors, never indices.** `Body.Faces()` / `Edges()` / `Vertices()` exist for **traversal and inspection only**; no feature, selector, or recipe ever accepts an index or a bare topology pointer. | An exact kernel produces a different (more correct) face/edge decomposition. If `Edges()[3]` is the API, index order becomes a de facto contract and vN breaks every model. |
| 4 | **Booleans never mutate their operands, and take no target-out parameter.** `Union(a, b) (*Body, error)` — never Fusion's in-place `booleanOperation(target, tool, type) -> bool`. | A signature free of in-place mutation lets the implementation be swapped with zero API churn. |
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
| GUI state on the geometry model (`isLightBulbOn` ×16, `displayBounds`, `opacity`, `Timeline.play()`) | The API is a scripting shim over the GUI | Keep view state out entirely |
| `LowCalculationAccuracy` default (±1%) | UI responsiveness | Accuracy is explicit; verification NEVER defaults to loose |
| `boundingBox` (knowingly loose) beside `preciseBoundingBox` | Legacy | One `Bounds()`, as tight as the evaluator can prove — and it reports its own exactness. The sin is being loose *without saying so*. |
| `volume == 0` meaning both "empty" and "not a solid" | Sloppiness | `(Measurement, error)` |

## 5. Foundations

### 5.1 Units — no naked floats

Every physical quantity crossing the API is a `units.Value` from
`github.com/lestrrat-3d/sketch/units`. We do NOT invent a parallel unit system.

```go
body, err := doc.Extrude(prof, decad.Distance{D: units.Millimeters(10)})
```

`units.Millimeters(6)` can never silently mean centimeters — which is precisely
the trap in `ValueInput.createByReal`, where the value is always internal units
(cm, radians) regardless of the document. A wrong-`Kind` value (an angle where a
length is wanted) is an **error**, not a coercion.

`units.Kind` today is Dimensionless / Length / Angle — deliberately limited to the
kinds a 2D sketch needs. decad needs Area, Volume, Mass and Density as well, and
the wrong-`Kind`-is-an-error rule means they cannot be faked. See §5.3.

### 5.2 Coordinates — r3, never hand-rolled

`r3.Vec` and `r3.Frame` only. `Frame` is orthonormal, so the inverse is the
transpose, never a matrix solve.

**The one deliberate exception to §5.1.** An `r3.Vec` *position* — `Box.Min`,
`Box.Max`, `Body.Centroid`, `Cylinder.Origin`, every point the API returns or
accepts — is a **length in the base unit, millimetres**. It is not a `units.Value`
and never becomes one: a vector of three typed quantities cannot be added, scaled,
dotted or crossed without unwrapping it at every step, which makes coordinate math
unusable and pushes callers back to hand-rolling. (An `r3.Vec` used as a *direction*
— `Cylinder.Axis`, `NormalTo(v)` — is dimensionless, and carries no unit at all.)

Vectors carry the unit **by convention**; scalars carry it **in the type**. §5.1
governs scalars, and this is the whole of the carve-out — a bare `float64` scalar
quantity is still forbidden anywhere in the API.

### 5.3 Dependency-side decisions (blockers)

Two capabilities decad's API depends on do not exist in its dependencies yet. Both
belong upstream by charter, and both are resolved upstream — decad does not
hand-roll around either.

| Gap | Proposal | Blocks |
|---|---|---|
| **`r3` has no rigid transform type.** `Frame` covers plane-local↔world only; placing a body at an arbitrary pose needs rotation + translation. | Add `r3.Transform` upstream. It *acts on* ℝ³, which is r3's charter. | Body placement — `Body.Placed` (§8), and with it the explicit-transform answer to assemblies (§13). |
| **`sketch/units` has no Area / Volume / Mass / Density kinds.** `units.Kind` is Dimensionless / Length / Angle only, so a `units.Value` cannot hold `12.9997 mm³`. | Add those kinds upstream to `sketch/units` — it is a first-party module — so `Measurement.Value` stays a single `units.Value` and the no-parallel-unit-system rule holds. | Every volume / area / mass measurement — i.e. most of `Verify`. |

Until the units kinds land, `Measurement` **cannot be implemented as specified**,
and decad will **not** work around it by inventing a parallel unit system.

The units gap is **wider than the measured value alone**. `Measurement.Bound` and
`Box.Bound` are `units.Value` too, carrying the same `Kind` as the quantity they
bound — the error bound on a volume is itself a volume, so it needs the Volume kind
as much as the volume does. So does `Interference.Volume` (§6.2). The blocker
covers the bound, not just the number.

### 5.4 Exactness — the load-bearing type

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
    Bound     units.Value // absolute error bound, same Kind as Value; zero when Exact
}
```

`Bound` carries the same `Kind` as `Value` — the error bound on a volume is a
volume. It is never a bare `float64`; invariant #2 and §5.1 admit no exception
here.

Every measurement returns one:

```go
vol, err := body.Volume()  // v1 after a boolean: {12.9997mm³, Approximate, 1e-3mm³}
                           // vN:                 {13.0000mm³, Exact,       0mm³}
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

**A `*Body` carries its owning `*Document`.** This is what lets a boolean keep the
signature invariant #4 demands — no target-out parameter, no operand mutated —
while `Document.Bodies()` stays truthful: `Union(a, b)` reaches the document
*through* `a`, so retiring the operands and registering the result happen inside
the operation, with no `*Document` argument and no caller bookkeeping. Operands
themselves are untouched: retiring is a change of *document* membership, not of
the body, which stays immutable and readable. The rule is uniform — every
operation that consumes a body (the booleans of §8, the `Body.Fillet` /
`Chamfer` / `Shell` modify ops, and `Body.Placed`) retires its input body or
bodies from the document and registers the body it returns. A retired body
remains readable, but it is gone from `Document.Bodies()` and
`Document.Verify()` never reports on it.

It follows that **bodies from different documents cannot be combined**.
`Union(a, b)` where `a` and `b` have different owners has no defined result — which
document would own it? — and is `ErrForeignBody` (§12).

```go
type Body struct{ /* ... */ }

func (b *Body) Document() *Document // the document that owns this body

func (b *Body) Lumps() []*Lump    // disjoint pieces; len > 1 means disconnected
func (b *Body) Shells() []*Shell  // Shell.IsVoid() marks an internal cavity
func (b *Body) Faces() []*Face
func (b *Body) Edges() []*Edge
func (b *Body) Vertices() []*Vertex

func (b *Body) IsSolid() bool
func (b *Body) Bounds() (Box, error)
func (b *Body) Volume() (Measurement, error)   // error when not a solid — never 0
func (b *Body) Area() (Measurement, error)
func (b *Body) Centroid() (r3.Vec, Exactness, error)

func (b *Body) Origin() FeatureRef  // which feature created this body
```

A bounding box is a measurement, so it carries the same trust metadata every other
measurement does — a v1 box around a curved body produced by a boolean is bounded
by `Faceted` faces and is therefore *not* tight, and says so:

```go
type Box struct {
    Min, Max  r3.Vec      // lengths in millimetres — the §5.2 carve-out
    Exactness Exactness
    Bound     units.Value // absolute error bound, Kind Length; zero when Exact
}
```

Invariant #2 covers **measurements**. `IsSolid() bool` and `IsConvex() bool` are
**predicates**, not measurements: they are decided by the evaluator, not
approximated by it, so they stay bare bools.

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

### 6.2 Supporting types

The rest of the vocabulary the signatures above and below name. Shapes given here
are load-bearing; the rest are deferred, not undecided.

**`Recipe`** — the exact record of intent, and per §2 the library's actual
deliverable, so it is a real value, not a handle. An ordered, immutable list of
steps; each step is an exact statement of intent — the feature kind, its profile
and parameters, and the *selectors* (never resolved pointers, never indices) it
was given. It is declarative and kernel-independent: nothing in a `Recipe` names a
face, an edge, a tessellation or an evaluator, which is what lets a second
evaluator re-run it and what makes emitting Fusion code from it mechanical (§11).

```go
type Recipe struct {
    Steps  []Step         // ordered, immutable; the model, exactly as meant
    Params []Param        // the named quantities the steps are written against
}

type Step struct {
    Op        OpKind          // Extrude / Revolve / Union / Cut / Intersect / Fillet / Chamfer / Shell
    Profile   *sketch.Profile // the source region, when the op has one
    Extent    Extent          // set for Extrude; nil for Revolve
    Angular   AngularExtent   // set for Revolve; nil for Extrude
    Axis      Axis            // revolve only
    Selectors []Selector      // the edge / face queries, unresolved
    Values    []units.Value   // radii, distances, thicknesses
}

type Param struct {
    Name  string
    Value units.Value
}

type OpKind int
```

`Extent` and `Angular` are the two disjoint extent sets of §8.1, and **exactly one
of them is non-nil, keyed to `Op`**: `Extrude` sets `Extent` and leaves `Angular`
nil; `Revolve` sets `Angular` and leaves `Extent` nil; every other `Op` leaves both
nil. That is what lets a `Recipe` encode a revolve at all — an `AngularExtent` is
not assignable to an `Extent` field, by design.

`Selector` is the sealed root of the selector vocabulary, so a `Step` never holds
an `any` — Fusion's `core.Base`-as-anything is rejected in §4, and `Recipe` is the
last place to reintroduce it:

```go
// Selector is what a Step may carry: an unresolved edge or face query.
type Selector interface{ selector() }
```

Every `EdgeSelector` and `FaceSelector` implementation is a `Selector`; in
particular `*EdgeQuery` and `*FaceQuery` (§9) satisfy all three.

**`Axis`** — sealed; what a revolve may spin about.

```go
type Axis interface{ axis() }

type SketchLine   struct{ /* ... */ } // a line in the source sketch
type ConstructionAxis struct{ /* ... */ } // an explicit axis in the document
type EdgeAxis     struct{ Edge EdgeSelector } // a linear edge, selected — never a pointer
```

**`Interference` / `Clearance`** — the pairwise results of §10. Both name the two
bodies and carry their quantity as a `Measurement`, so both report their own
exactness like everything else.

```go
// Interference: the two bodies overlap, and by how much.
type Interference struct {
    A, B   *Body
    Volume Measurement // the overlap volume; needs the Volume kind — see §5.3
}

// Clearance: the two bodies do not overlap, and how close they come.
type Clearance struct {
    A, B *Body
    Gap  Measurement // minimum distance between the two surfaces
}
```

The rest are deferred:

```go
type Direction struct{ /* ... */ }     // a sense along a sketch normal: up / down / both
type FeatureRef struct{ /* ... */ }    // an opaque handle to the feature that created a body or face
type Mesh struct{ /* ... */ }          // a triangle mesh; an OUTPUT of Tessellate, never the representation
type Curve interface{ curve() }        // sealed, like Surface: Line / Circle / Arc / Ellipse / NURBSCurve / FacetedCurve
type SurfaceKind int                   // the discriminant a Surface reports: Plane / Cylinder / Cone / Sphere / Torus / NURBS / Faceted
type EdgePredicate struct{ /* ... */ } // one clause of an EdgeQuery; the §9 constructors return these
type FacePredicate struct{ /* ... */ } // one clause of a FaceQuery
```

The remaining topology of §2's chain, each deferred but for the one method that
carries its weight:

```go
// Lump is a connected solid piece of a body. Body.Lumps() returning more than one
// means the body is disconnected.
type Lump struct{ /* ... */ }

// Shell is a connected set of faces.
type Shell struct{ /* ... */ }
func (s *Shell) IsVoid() bool // true when the shell bounds an internal cavity

// Loop is a boundary loop of a face.
type Loop struct{ /* ... */ }
func (l *Loop) IsOuter() bool // the outer boundary; false for a hole

// Vertex is a topological point.
type Vertex struct{ /* ... */ }
func (v *Vertex) Position() r3.Vec // a length in millimetres — the §5.2 carve-out

// MeshBody is imported triangle soup. It NEVER claims to be a solid B-rep —
// invariant #5 — and mesh import is a v1 non-goal (§13).
type MeshBody struct{ /* ... */ }
```

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
body, err := doc.Extrude(prof, decad.Distance{D: units.Millimeters(10)})
```

`doc.Extrude` REJECTS a `sketch.Profile` whose `Valid` is false — a
self-intersecting or degenerate region is never silently swept. The plane's
`r3.Frame` lifts the plane-local profile into world space.

## 8. Features

v1 vocabulary, deliberately small: **Extrude, Revolve, Union/Cut/Intersect,
Fillet, Chamfer, Shell**. Sweep and Loft are deferred.

```go
func (d *Document) Extrude(p *sketch.Profile, e Extent, opts ...ExtrudeOption) (*Body, error)
func (d *Document) Revolve(p *sketch.Profile, axis Axis, a AngularExtent, opts ...RevolveOption) (*Body, error)
```

Booleans are **explicit** — not folded into every feature with an ambient,
implicitly-chosen target — and they never mutate an operand or take a target-out
parameter:

```go
func Union(a, b *Body) (*Body, error)
func Cut(target, tool *Body) (*Body, error)
func Intersect(a, b *Body) (*Body, error)
```

No `*Document` appears in those signatures, and none is needed: a `*Body` carries
its owning document (§6), so each call retires its operands and registers its
result inside the document that owns them. The operands are themselves unchanged
— invariant #4 — and `Document.Bodies()` and `Document.Verify()` stay truthful.
Operands owned by different documents are `ErrForeignBody`.

Modify operations return a new body, retiring the receiver, on the same terms:

```go
func (b *Body) Fillet(sel EdgeSelector, r units.Value) (*Body, error)
func (b *Body) Chamfer(sel EdgeSelector, d units.Value) (*Body, error)
func (b *Body) Shell(sel FaceSelector, thickness units.Value) (*Body, error)
```

Placement is a body operation on the same terms — it retires the receiver and
registers the placed body — and is **blocked on `r3.Transform` landing upstream**
(§5.3):

```go
func (b *Body) Placed(t r3.Transform) (*Body, error) // blocked on r3.Transform (§5.3)
```

This is the whole of the "explicit transforms" story: a body is positioned by an
argument the caller states, never by an ambient assembly context (§4). Until
`r3.Transform` exists, `Placed` **cannot be implemented as specified** — it is not
in the v1 vocabulary above, and no `OpKind` records it — and decad will not
hand-roll a transform type around the gap.

### 8.1 Extent — illegal states unrepresentable

Fusion has three mutually-exclusive `setXxxExtent` methods with no enforcement,
and `add()` fails at runtime. decad makes exclusivity structural:

Extrude takes a **linear** extent:

```go
type Extent interface{ extent() }

// SideExtent is what ONE side of a two-sided extent may be. Sealed, and narrower
// than Extent: implemented ONLY by Distance, ThroughAllSide and ToFace.
type SideExtent interface{ sideExtent() }

type Distance   struct { D units.Value }               // Extent AND SideExtent
type ToFace     struct { Face FaceSelector; Offset units.Value } // Extent AND SideExtent
type ThroughAll struct { Dir Direction }               // Extent ONLY
type ThroughAllSide struct{}                           // SideExtent ONLY
type Symmetric  struct { D units.Value; FullLength bool }        // Extent ONLY
type TwoSided   struct { One, Two SideExtent }                   // Extent ONLY
```

Revolve takes an **angular** extent, its own sealed set, with the same two-tier
split:

```go
type AngularExtent interface{ angularExtent() }

// SideAngular is what ONE side of a two-sided angular extent may be. Sealed:
// implemented ONLY by AngleExtent and ToFaceAngular.
type SideAngular interface{ sideAngular() }

type AngleExtent    struct { A units.Value }           // AngularExtent AND SideAngular; one-sided
type ToFaceAngular  struct { Face FaceSelector }       // AngularExtent AND SideAngular
type SymmetricAngle struct { A units.Value; FullLength bool } // AngularExtent ONLY
type TwoSidedAngle  struct { One, Two SideAngular }           // AngularExtent ONLY
type FullRevolution struct{}                                  // AngularExtent ONLY
```

**The nesting is the point.** A side of a two-sided extent is a *single direction
of travel*, so only the variants that mean one are admissible there. Because
`Symmetric`, `TwoSided`, `SymmetricAngle`, `TwoSidedAngle` and `FullRevolution`
implement only the outer interface and not the inner one,
`TwoSided{One: TwoSided{...}}`, `TwoSided{One: Symmetric{...}}` and
`TwoSidedAngle{One: FullRevolution{}}` do not compile. Illegal states are
unrepresentable structurally, not rejected at runtime.

`ThroughAll` and `ThroughAllSide` are split for the same reason, **by role**.
Standalone, "through all" must say which way it runs, so `ThroughAll` carries a
`Direction` (up / down / both) and satisfies `Extent` only. As a *side* of a
`TwoSided` it must not: the side already **is** a single direction of travel, so a
second, possibly contradicting direction on the same value —
`TwoSided{One: ThroughAll{Dir: both}}` — is precisely the illegal state this
section abolishes. `ThroughAllSide` therefore carries no direction, satisfies
`SideExtent` only, and takes its sense from the side it occupies. Neither form is
assignable where the other belongs.

The two sets are **deliberately disjoint**: no linear extent satisfies
`AngularExtent` and no angular extent satisfies `Extent`. That is what makes
"revolve 90mm" unrepresentable rather than a runtime error.

`ToFace` and `ToFaceAngular` take a **`FaceSelector`, never a `*Face`** — invariant
#3 and §9's rule hold inside features too, not merely at their edges. The selector
MUST resolve to **exactly one** face: that is what `FaceQuery.Exactly(1)` is for.
Resolving to zero faces or to more than one is `ErrCardinality` (§12) — "extrude up
to that face" is meaningless when "that face" is four faces.

Options carry the rest (`WithTaper(units.Degrees(3))`), via
`github.com/lestrrat-go/option/v3` — the house functional-options library, and an
approved dependency.

## 9. Selectors — intent, not identity

The topological-naming problem, in Fusion's own words: `tempId` is valid only
"as long as the owning BRepBody is not modified in any way", and entity tokens
"should never be compared". Handles do not survive an edit, and index order is not
stable. Its `isTangentChain` flag is the workaround; we take the lesson properly.

**A feature is given a query, never a pointer.**

Features accept the **interfaces**; the constructors return the **concrete query
types** that implement them and that carry the cardinality assertions. Both
interfaces embed the sealed `Selector` root (§6.2), so every selector a feature
accepts is a value a `Recipe` can record.

```go
type EdgeSelector interface {
    Selector
    SelectEdges(*Body) ([]*Edge, error)
}

type FaceSelector interface {
    Selector
    SelectFaces(*Body) ([]*Face, error)
}

func Edges(preds ...EdgePredicate) *EdgeQuery
func Faces(preds ...FacePredicate) *FaceQuery

type EdgeQuery struct{ /* ... */ }

func (q *EdgeQuery) SelectEdges(*Body) ([]*Edge, error)
func (q *EdgeQuery) Exactly(n int) *EdgeQuery   // errors at resolve unless exactly n match
func (q *EdgeQuery) AtLeast(n int) *EdgeQuery

type FaceQuery struct{ /* ... */ }

func (q *FaceQuery) SelectFaces(*Body) ([]*Face, error)
func (q *FaceQuery) Exactly(n int) *FaceQuery
func (q *FaceQuery) AtLeast(n int) *FaceQuery

// Both queries seal into Selector (§6.2), which is what a Recipe Step stores.
func (q *EdgeQuery) selector()
func (q *FaceQuery) selector()

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
`Status` at both the body and the document level, aggregated by a fixed severity
precedence; and one bit an agent gates on. Deliberately mirrors
`sketch.VerificationReport` / `WorldVerificationReport`.

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
    Body              *Body
    Status            Status       // Sound / Suspect / Unsound — this body only
    Solid             bool
    Watertight        bool
    Manifold          bool         // every edge bounds exactly 2 faces
    SelfIntersecting  bool
    Lumps             int          // > 1 == disconnected pieces
    Voids             int          // internal cavities (Shell.IsVoid)
    Volume            Measurement
    Area              Measurement
    Centroid          r3.Vec       // millimetres — the §5.2 carve-out
    CentroidExactness Exactness    // mirrors Body.Centroid() (§6); invariant #2
    Bounds            Box
    Exactness         Exactness    // the weakest link across this body

    // Opt-in, expensive:
    MinWallThickness  *Measurement // WithMinWallThickness(tool)
    Undercuts         []*Face      // WithPullDirection(v)
    MinRadius         *Measurement // WithMinRadius() — can the endmill reach?
}
```

Fusion answers **none** of `Watertight` (with diagnostics), `Manifold`,
`SelfIntersecting`, `MinWallThickness` (B-rep), `Undercuts`, or `MinRadius`. That
gap is decad's mandate.

`Status` is one type used at two levels:

```go
type Status int

const (
    Sound       Status = iota // every body sound; nothing approximate beyond tolerance
    Suspect                   // sound, but an answer is Approximate beyond the caller's tolerance
    Interfering               // bodies overlap
    Unsound                   // some body is not a valid solid
)
```

- **`BodyReport.Status`** is per-body: `Sound` (solid, watertight, manifold, no
  self-intersection), `Suspect` (sound, but one of its answers is `Approximate`
  beyond the caller's tolerance), or `Unsound` (not a valid solid). A body is never
  `Interfering` — interference is a property of a *pair*, not of a body.
- **`Report.Status`** is the document-level aggregate.

Aggregation is by **severity precedence — worst wins**:

**`Unsound` > `Interfering` > `Suspect` > `Sound`**

Concretely, `Report.Status` is:

| Condition | `Report.Status` |
|---|---|
| any `BodyReport.Status == Unsound` | `Unsound` |
| else, `len(Interferences) > 0` | `Interfering` |
| else, any `BodyReport.Status == Suspect` | `Suspect` |
| else | `Sound` |

`Report.Trustworthy()` is true **only** at `Report.Status == Sound`. An unsound
body, an unresolved interference, or an approximation coarser than the caller's
tolerance each make it false — even when the geometry "looks" fine.

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
  (a selector carrying no cardinality assertion matched nothing), `ErrCardinality`
  (a cardinality assertion failed), `ErrForeignBody` (an operation was handed bodies
  owned by different documents), `ErrNotSolid`, `ErrDegenerate`, `ErrBooleanFailed`,
  `ErrInvalidProfile`, `ErrUnitKind`.
- **`ErrCardinality` takes precedence at zero matches.** A failed cardinality
  assertion is `ErrCardinality` **even when the selector matched nothing** — and
  that covers both the explicit assertions, `Exactly(n)` / `AtLeast(n)` (§9), and
  the *implicit* exactly-one of `ToFace` / `ToFaceAngular` (§8.1). `ErrNoMatch` is
  reserved for the one remaining case: a selector that asserts no cardinality at
  all and resolves to zero entities.
- `Body` is immutable → safe to read from many goroutines.
- `Document` owns mutable state and is NOT safe for concurrent mutation. `Verify`
  is non-mutating and safe.

## 13. Non-goals for v1

Assemblies (`Component`/`Occurrence` instancing and the DAG that comes with it), a
feature tree / timeline / rollback, sweep and loft, STEP, sheet metal, mesh import,
GUI or view state of any kind, and Fusion code generation.

The assemblies non-goal rests on a plan, not on a capability already in hand:
interference and clearance (§10) are computed between **explicitly-placed bodies**
— `Body.Placed(t r3.Transform)` (§8) — which needs no instancing graph. That plan
is **gated on `r3.Transform` landing** (§5.3). Until it does, a body sits where its
profile put it, and multi-body interference is limited to what that reaches.
