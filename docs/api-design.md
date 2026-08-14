# Public API Design

The design of decad's public API: a headless CAD engine an agent uses to model a
part, prove it sound, and only then write real CAD software code (a Fusion
add-in). This document is the contract. No public API lands that contradicts it.

Companion designs carry the deep ends of this contract: the recording
contract at the sketch seam — the trim contract, the recording IR, and
`ErrUnrecordableProfile` — is specified in `docs/sketch-seam-design.md`; strict
recipe decoding, validation, versioning, resource limits, and atomic evaluation
are specified in `docs/recipe-replay-design.md`; how verification judges every
bounded result — the report, the tolerance gate, and the noise floor — is
specified in `docs/verification-design.md`; the mesh contract, per-payload
chording, proof bounds, and boolean handoff are specified in
`docs/tessellation-design.md`; exact cup plus certificate-backed faceted
proofs are specified in `docs/payload-verification-design.md`; and how
verification proves overlap and bounds its volume without consuming either body
is specified in `docs/interference-design.md`. `CLAUDE.md`
lists every current design document and its owner.

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
            analytic profiles + planes + features + operands + selectors + options + quantities
   |
   v
Evaluator   swappable
            v1: analytic where free, tessellation-backed booleans, exactness flagged
            vN: full analytic B-rep
   |
   v
Result      Body -> Lump -> Shell -> Face -> Loop -> CoEdge -> Edge -> Vertex
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

Stored recipes are executable model input, not JSON-shaped documentation.
`EncodeRecipe` writes one canonical envelope after bounded full validation,
`DecodeRecipe` strictly decodes one versioned envelope, `Recipe.Validate`
independently proves every stored profile and checks the operation/reference
graph without building a body, and `Evaluate` runs a bounded private snapshot
through a package-owned evaluator into a new `Document`. Whole-recipe failure
exposes no partial document. The complete contract is
`docs/recipe-replay-design.md`.

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

decad therefore computes a boolean on a tessellated representation, where robust
algorithms are a solved problem, and **marks what it touched** — unless the pair
reduces analytically, which needs no intersection math either. §8 names the one
admitted class, and `docs/prism-boolean-design.md` owns it.

## 3. Forward-compatibility invariants

These are cheap now and expensive to retrofit. They are the upgrade path.

| # | Invariant | Rationale |
|---|---|---|
| 1 | **NEVER expose triangles as the representation.** `Tessellate(tol)` is an output; the public vocabulary is `Body → Face → Edge → Vertex` even while the backing is approximate. | A public `Triangles()` is a one-way door; callers depend on it and vN can never remove it. |
| 2 | **Every measurement carries `Exactness`, from the first commit.** | Makes the upgrade **monotonic, not breaking**: callers already branch on `Approximate`; in vN that branch stops being taken and nobody's code changes. A bare `float64` today means adding exactness later breaks every call site. |
| 3 | **Selectors, never topology indices.** `Body.Faces()` / `Edges()` / `Vertices()` exist for **traversal and inspection only**; no feature, selector, or recipe ever names a face, an edge or a vertex by index or by a bare topology pointer. (A `Recipe`'s `StepRef` (§6.2) is not one: it indexes the recipe's own steps, which no evaluator reorders.) | An exact kernel produces a different (more correct) face/edge decomposition. If `Edges()[3]` is the API, index order becomes a de facto contract and vN breaks every model. |
| 4 | **Booleans never mutate their operands, and take no target-out parameter.** `Union(a, b) (*Body, error)` — never Fusion's in-place `booleanOperation(target, tool, type) -> bool`. | A signature free of in-place mutation lets the implementation be swapped with zero API churn. |
| 5 | **Imported meshes are a separate type.** A `MeshBody` never claims to be a solid B-rep. | For imported triangle soup, no future kernel can recover exactness. Keeping it separate stops approximate-forever geometry from contaminating the type we promise to make exact. |

**What will still churn**, stated honestly: face/edge *counts* may change under an
exact kernel (canonicalize under evaluator §3's rules, so v1 counts
already match the analytic answer); numeric outputs shift (`12.9997 → 13.0000`, so
tests compare with tolerances, never goldens); and vN's surface set is a superset,
so a `switch` on `Surface` MUST have a `default` branch.

## 4. What we take from Fusion, and what we reject

Loosely based on the Fusion SDK. The odd parts are thrown away deliberately.

### Keep

| Idea | Why |
|---|---|
| B-rep topology `Body → Lump → Shell → Face → Loop → CoEdge → Edge → Vertex`, with co-edges as directed edge uses | The right model, and it makes agent traversal code map 1:1 onto Fusion's. |
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
`github.com/lestrrat-3d/units` — the same module `sketch` uses for its own
dimensional values, so a quantity never changes type crossing the seam. We do
NOT invent a parallel unit system.

```go
body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
```

`units.Millimeters(6)` can never silently mean centimeters — which is precisely
the trap in `ValueInput.createByReal`, where the value is always internal units
(cm, radians) regardless of the document. A wrong-`Kind` value (an angle where a
length is wanted) is an **error**, not a coercion.

`units.Kind` is a vector of dimension exponents — length, mass, angle — so kinds
compose (`Kind.Mul` / `Div` / `Pow`) rather than being enumerated; a composition
past the exponent range saturates and is marked overflowed, stickily, so it can
never pass for a real kind. The named kinds — Dimensionless, Length, Area,
Volume, Angle, Mass, Density, MomentOfInertia, SecondMomentOfArea — each have a
registered base unit, and they cover every quantity decad measures: a
`units.Value` holds `12.9997 mm³` as readily as a length, and the
wrong-`Kind`-is-an-error rule means no kind can be faked.

### 5.2 Coordinates — r3, never hand-rolled

`r3.Vec`, `r3.Frame` and `r3.Transform` only. `Frame` is orthonormal, so the
inverse is the transpose, never a matrix solve.

`r3.Transform` is a rigid motion of ℝ³ — an orthogonal linear map plus a
translation, an isometry by construction: scale, shear and projection are
unrepresentable. It is built with `Identity` / `Translation` / `Rotation` /
`RotationAround` / `Reflection` / `FromFrame` / `FromBasis` — angles are typed
`units.Value`, so a bare `2` can never silently mean radians — and applied with
`Apply` (a point) and `ApplyDir` (a direction; normals transform like
directions, with no inverse transpose anywhere). Every constructor but
`Identity`, and the derivations `Then` (composition) and `Inverse`, return
`(Transform, error)` and validate what they produce, so a `Transform` that
exists is a real rigid motion; `Inverse` is exact — the transpose, never a
solve. Reflections are representable (det = −1) and `Transform.IsReflection`
reports them, because a reflected solid has inverted face normals. `Basis()`
and `Translation()` read a transform out as plain vectors and `FromBasis`
rebuilds it, which is how a placement survives encoding (§6.2). It is what
`Body.Placed` (§8) takes.

**The one deliberate exception to §5.1.** An `r3.Vec` *position* — `Box.Min`,
`Box.Max`, a `VecMeasurement.Value` (§5.3), `Cylinder.Origin`, every point the API
returns or accepts — is a **length in the base unit, millimetres**. It is not a `units.Value`
and never becomes one: a vector of three typed quantities cannot be added, scaled,
dotted or crossed without unwrapping it at every step, which makes coordinate math
unusable and pushes callers back to hand-rolling. (An `r3.Vec` used as a *direction*
— `Cylinder.Axis`, `NormalTo(v)` — is dimensionless, and carries no unit at all.)

The carve-out is about **coordinates**, not about the vector type: a plane-local
2D coordinate — `Point2` (`docs/sketch-seam-design.md`), the `(u, v)` of a
recorded profile — is a length in millimetres on exactly the same terms, and for
exactly the same reason.

Vectors carry the unit **by convention**; scalars carry it **in the type**. §5.1
governs scalars, and this is the whole of the carve-out — a bare `float64` scalar
quantity is still forbidden anywhere in the API. A *curve parameter* is not a
scalar quantity: a spline's knots and weights, a recorded segment's parameter
range (`TStart`/`TEnd`), and a conic's fullness `Rho` — from which a rational
quadratic's apex weight derives as `w = Rho/(1−Rho)` (all
`docs/sketch-seam-design.md`) — are dimensionless indices into a
parameterisation, not measurements of anything, and §5.1 does not reach them.

### 5.3 Exactness — the load-bearing type

```go
type Exactness int

const (
    Exact Exactness = iota // proved exactly representable; number is truth
    Approximate            // bounded numerical/tessellated; Bound holds error
)

// Measurement is a scalar quantity plus how far it can be trusted.
type Measurement struct {
    Value     units.Value
    Exactness Exactness
    Bound     units.Value // absolute error bound, same Kind as Value; zero when Exact
}

// VecMeasurement is a computed coordinate or direction, with how far it can be
// trusted. It is what the API returns wherever the evaluator computes a vector.
type VecMeasurement struct {
    Value     r3.Vec      // a position in millimetres (§5.2), or a unit direction
    Exactness Exactness
    Bound     units.Value // absolute error bound; Kind Length for a position,
                          // Dimensionless for a direction. Zero when Exact.
}
```

`Measurement.Bound` carries the same `Kind` as `Value` — the error bound on a
volume is a volume. It is never a bare `float64`; invariant #2 and §5.1 admit no
exception here.

`VecMeasurement.Bound` obeys the same rule, and its `Kind` therefore **follows its
`Value`**. The bound is the radius of the ball around `Value` that the true vector
is proven to lie in, so it is a magnitude of the same quantity the vector is. For a
**position** that is a distance: `Kind` Length, millimetres. For a **direction** it
is the deviation of a unit vector from the true unit vector, and a direction is
dimensionless (§5.2) — so the bound is `Dimensionless` (`units.Scalar`), and typing
it as a length would be exactly the wrong-`Kind` coercion §5.1 forbids: it would
hand a millimetre tolerance a quantity that is not a length. The verification
gate (`docs/verification-design.md`) is stated over both, and needs no exponent
to be.

Every measurement returns one:

```go
vol, err := body.Volume()  // v1 after a boolean: {12.9997mm³, Approximate, 1e-3mm³}
                           // vN:                 {13.0000mm³, Exact,       0mm³}
```

`Measurement`, `VecMeasurement` and `Box` (§6) are the **three and only three**
bounded results the API returns. Every one of them carries a `Bound`; how
verification judges each against the caller's tolerance is specified in
`docs/verification-design.md`, and every one of them is judged — the gate is
total because this set is.

## 6. The document and bodies

Immediate mode. **The agent's Go function is the feature tree** — re-running
`MakeBracket(height)` with a new height *is* the rebuild. A Fusion add-in script is
imperative too, so immediate-mode Go resembles the target artifact more closely
than a timeline API would. We do not build an interpreter for a language the agent
already has.

```go
type Document struct{ /* ... */ }

func New(opts ...DocumentOption) *Document

func EncodeRecipe(w io.Writer, r Recipe, opts ...EncodeRecipeOption) error
func DecodeRecipe(r io.Reader, opts ...DecodeRecipeOption) (Recipe, error)
func (r Recipe) Validate(opts ...ValidateRecipeOption) error
func Evaluate(ctx context.Context, r Recipe, opts ...EvaluateOption) (*Document, error)

func (d *Document) Bodies() []*Body            // live bodies
func (d *Document) Recipe() Recipe             // the exact record of intent
func (d *Document) Verify(ctx context.Context, opts ...VerifyOption) (*Report, error)
```

`Body` is **immutable**; every operation returns a new one, and the input body is
retired from the document.

`Evaluate` always owns a new document. It never appends into a caller's
document, and any failure returns a nil document. Evaluator selection is an
option on `Evaluate`; evaluator identity never enters `Recipe`. See
`docs/recipe-replay-design.md` §§1/5.

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
`Document.Verify()` never reports on it. The two copies of §8 —
`Body.Duplicate` and `Body.PlacedCopy` — are the deliberate exception: they
**depend on** the source without consuming it, exactly as an `Extrude` depends
on the body a `ToFace` names (§6.2), so the source stays live and the copy is a
new body beside it.

**A retired body is no longer part of the model, so no operation takes one.** It is
readable — its measurements still answer — but handing it to a boolean, to a modify
op, or to an extent that names a body (`ToFace`, §8.1) is `ErrRetiredBody` (§12).
A recipe that stopped an extrude at the face of a body the model no longer contains
would not re-evaluate, and §11's emission would have no face to name.

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
func (b *Body) Centroid() (VecMeasurement, error)

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

Invariant #2 covers **measurements**, and the line is drawn once, here, so no
implementation has to guess where it falls:

- **A quantity the evaluator computes is a measurement, and it carries both an
  `Exactness` and a `Bound`.** A scalar is a `Measurement`; a coordinate or a
  direction is a `VecMeasurement`; a bounding box is a `Box`. That covers a volume,
  an area, a length, a bounding box, a centroid, a vertex position, a face normal —
  those three shapes are the whole of it, and none of them is unbounded. A vertex
  position produced by a v1 tessellation-backed boolean is approximated exactly as
  its volume is, and says by how much.
- **Only a predicate the evaluator *decides* is exempt** — `Body.IsSolid`,
  `Edge.IsConvex`, `Loop.IsOuter`, `Shell.IsVoid`. These are answers, not
  approximations of answers, so they stay bare bools.

The §5.2 carve-out is a carve-out from the **units** rule (§5.1) only: it says an
`r3.Vec` coordinate is a length in millimetres rather than a `units.Value`. It says
nothing about invariant #2, and exempts nothing from it — a coordinate is still a
measurement, so it is returned as a `VecMeasurement`, `Exactness` and `Bound` and
all. A computed vector never crosses the API as a bare `r3.Vec`.

### 6.1 Topology

```go
type Face struct{ /* ... */ }

func (f *Face) Surface() Surface   // sealed; see below
func (f *Face) Loops() []*Loop     // Loop.IsOuter() distinguishes outer from holes
func (f *Face) Edges() []*Edge
func (f *Face) Area() (Measurement, error)
func (f *Face) NormalAt(p r3.Vec) (VecMeasurement, error) // a computed direction: a measurement
func (f *Face) Origins() []FeatureRef // provenance: every feature role that created it — canonicalization
                                      // may merge coplanar faces, and a merged face carries ALL contributing
                                      // roles; FaceCreatedBy matches on any of them

type Loop struct{ /* ... */ }

func (l *Loop) IsOuter() bool
func (l *Loop) CoEdges() []CoEdge // directed boundary in walk order
func (l *Loop) Edges() []*Edge    // compatibility view: same order, direction omitted

type CoEdge struct{ /* ... */ }

func (ce CoEdge) Edge() *Edge
func (ce CoEdge) Start() *Vertex  // start in this loop's walk
func (ce CoEdge) End() *Vertex    // end in this loop's walk
func (ce CoEdge) IsForward() bool // true when the walk matches Edge.Start → Edge.End

type Edge struct{ /* ... */ }

func (e *Edge) Curve() Curve
func (e *Edge) Faces() []*Face     // len != 2 on a closed body means NON-MANIFOLD
func (e *Edge) Start() *Vertex
func (e *Edge) End() *Vertex
func (e *Edge) Length() (Measurement, error)
func (e *Edge) IsConvex() bool
```

**A `CoEdge` is one directed use of a shared `Edge` by one `Loop`.**
`CoEdge.Start()` and `End()` follow the loop walk. `CoEdge.IsForward()` is true
when that walk matches `Edge.Start()` to `Edge.End()`, and false when it runs
from `Edge.End()` to `Edge.Start()`. `Loop.CoEdges()` is the complete boundary
traversal. `Loop.Edges()` remains the undirected compatibility view: it returns
the same shared edge pointers in the same order and omits only the per-loop
direction. Both methods return copied slices, preserving topology immutability.

**`Edge.IsConvex` is the WALKED-BOUNDARY convexity, not the 3D material angle
across the edge.** Every profile is walked with the material on the left — the
outer loop counter-clockwise, every hole clockwise — and an edge reports the
sense of that walk. A **junction** edge, where two walls meet, is convex when the
walk turns left there, which is also the material angle. A **rim** edge, a wall's
copy in a cap plane, takes the sense of the wall it runs along: a circular wall
by its own turn (counter-clockwise convex, clockwise concave), a straight wall —
which turns not at all — by the role of its loop (outer convex, hole concave).
The on-axis edge shared by both caps of a partial revolve is convex when the
sweep is under a half turn. So **a hole's rim edges are concave**, as are those of
a concave round bitten out of the outer boundary, even though the material across
such a rim is a plain quarter-turn wedge a chamfer could take. It is a decided
answer either way — `Concave()` selects those rims, `Convex()` never does.

`Surface` and `Curve` are **sealed** interfaces (unexported method), matching the
house idiom in `sketch.Entity` — the answer to Fusion's `core.Base`-as-anything.

```go
type Surface interface {
    Kind() SurfaceKind
    surface() // sealed
}

// The variant types. The SurfaceKind discriminant a Surface reports is a
// separate, KindXxx-prefixed constant set (§6.2) — a surface variant and its
// kind never share a name.
type Plane struct    { Frame r3.Frame }
type Cylinder struct { Origin, Axis r3.Vec; Radius units.Value }
type Cone struct     { Origin, Axis r3.Vec; Radius, HalfAngle units.Value }
type Sphere struct   { Center r3.Vec; Radius units.Value }
type Torus struct    { Center, Axis r3.Vec; Major, Minor units.Value }
// NURBSSurface is a free-form face's geometry. `docs/spline-design.md` §7 owns
// its construction, its exactness and its v1 shape.
type NURBSSurface struct { /* private */ }

// Faceted is the honest v1 variant: a face a boolean produced, whose public
// analytic identity is gone. Bound encloses two-sided displacement between the
// held polygons and their true boundary patches under the evaluator's internal
// local correspondence (payload verification §5).
type Faceted struct{ /* ... */ }
```

`Curve` is `Surface`'s 1-D analog, sealed the same way, `Edge.Curve()`'s return
type:

```go
type Curve interface{ curve() } // sealed

type Line3 struct   { /* ... */ }
type Circle3 struct { /* ... */ }
type Arc3 struct    { /* ... */ }
// NURBSCurve is a free-form edge's geometry, NURBSSurface's 1-D analog
// (docs/spline-design.md §7).
type NURBSCurve struct{ /* private */ }
// FacetedCurve is a boolean-built edge's chord chain, Faceted's 1-D analog.
type FacetedCurve struct{ Bound units.Value }
```

**Surface parameters carry no `Exactness`, and this is not an exception to
invariant #2.** An analytic `Surface` variant's parameters are exact for the
surface it names. A face whose built geometry is that surface is exact; a tagged
analytic variant may instead be a bounded stand-in and carry its departure in
computed measurements (`docs/modify-reach-design.md` §8.3). A face whose held
geometry is faceted is `Faceted`, and `Faceted` is the flag. So a `Cylinder.Axis`
or a `Sphere.Center` is an exact parameter of its named surface, while
`Face.NormalAt(p)` — a quantity the evaluator *computes*, on a face that may be
a bounded analytic stand-in — is a measurement and reports its `Exactness` like
every other. It answers for the five analytic variants, and `NormalAt` on a
`NURBSSurface` or on a `Faceted` face is `ErrUnsupported` (`docs/spline-design.md`
§7); the faceted reading lands with the faceted certificate stage, whose internal
source certificate is what bounds the true patch normals
(`docs/payload-verification-design.md` §5.4, §13). A positional `Faceted.Bound`
alone does not imply a normal bound (`docs/payload-verification-design.md`
§5/§8), and no certificate details enter the public API.

<!-- The NormalAt sentence is claim + pointer, which the authoring rule in
~/.claude/docs/agent-instructions.md sanctions for a non-owning site: "One full
derivation per why, at the owning site; every repeat becomes claim + pointer."
Its never-restate rule bans the pointer-WITH-GLOSS shape, and no clause of spline
§7's derivation is unpacked here — not the (u, v) root-find, not the faceted
answer's union of held-facet certificates, not the other variants' own
closed-form bound, not the undercut survey reading normals off the payload
walk. Two sentences of the same claim+pointer shape already ship in this
file: the Faceted.Bound sentence immediately above, and the
DiagUnsupportedPairContact sentence in §8's boolean-error taxonomy, which even
names the owner's constant. This reading of the authoring rule is the project's
settled one. -->

A `switch` on `Surface` MUST carry a `default` — vN adds variants.

### 6.2 Supporting types

The rest of the vocabulary the signatures above and below name. Shapes given here
are load-bearing; the rest are deferred, not undecided.

**`Recipe`** — the exact record of intent, and per §2 the library's actual
deliverable, so it is **a real value, not a handle**: it holds no pointer into a
live `*Document` and none into a live `*sketch.Sketch`, and it stays true after
either has moved on. An ordered, immutable list of steps; each step is an exact
statement of intent — the feature kind, the bodies it depends on, its profile and
the plane that profile lies in, its extent, its options, its quantities, and the
*selectors* (never resolved pointers, never topology indices) it was given. It is
declarative and kernel-independent: nothing in a `Recipe` names a face, an edge, a
tessellation or an evaluator, which is what lets a second evaluator re-run it and
what makes emitting Fusion code from it mechanical (§11).

**The completeness rule, and it is a rule, not a hope.** A `Recipe` MUST be
sufficient to (a) re-evaluate the model from scratch under any evaluator, and
(b) emit equivalent CAD code. Every input an operation takes — its operands, **every
body its extent or its axis names**, its profile, **the plane that profile lies
in**, its extent, its selectors, its options, its quantities — MUST be recordable in
its `Step`. **An operation whose inputs a `Step` cannot record does not ship**, and
an *input* a `Step` cannot record is **rejected at the call**, never recorded
approximately — that is what `ErrUnrecordableProfile` (§12) is. This is what §2's "the exact record of intent" costs: a `Recipe` that
re-evaluates to a *different* model than the one it was recorded from is not the
deliverable §2 claims — it would make vN a silently different model rather than a
better answer to the same one, and would make the mechanical Fusion emission of
§11 emit the wrong feature.

```go
type Recipe struct {
    Steps []Step // ordered, immutable; the model, exactly as meant
}

// StepRef refers to the Step that produced a body. It is NOT a topology index.
type StepRef int

type Step struct {
    Op        OpKind          // Extrude / Revolve / Loft / Union / Cut / Intersect / Fillet / Chamfer / Shell / Placed / Duplicate / PlacedCopy
    Inputs    []StepRef       // the bodies this step depends on. Cut is [target, tool].
    Profile   ProfileRecord   // Extrude / Revolve / Loft ("from" section) — decad's own analytic 2D record of the region
    Plane     PlaneRecord     // Extrude / Revolve / Loft ("from" section) — the sketch plane; lifts Profile into world space
    Extent    Extent          // Extrude
    Angular   AngularExtent   // Revolve
    Axis      Axis            // Revolve
    Placement TransformRecord // Placed / PlacedCopy — the rigid motion, recorded as vectors; zero for Duplicate
    Selectors []Selector      // Fillet / Chamfer / Shell — the edge / face queries, unresolved
    Opts      StepOpts        // per-op options; nil when the op takes none
    Values    []units.Value   // radii, distances, thicknesses
}

type OpKind int
```

**The `Recipe` owns its geometry.** A `Step` holds no `*sketch.Profile` and no
`r3.Frame`: it records the region **structurally**, in decad's own plane-local
types, at the moment the feature is called — a live profile is a handle into a
mutable sketch, and §2 says the recipe is a value. `ProfileRecord` and
`PlaneRecord` — the structural record of the region and of the sketch plane
that lifts it into world space — are specified, with the `CurveSegment`
vocabulary they are built from (one variant per `sketch` entity kind, whole
and trimmed alike) and the whole-versus-`Partial` recording rules, in
`docs/sketch-seam-design.md`.

A placement is recorded on the same terms. `r3.Transform`'s fields are
unexported, so a `Step` that stored one would silently drop the motion; decad
converts, it does not reference:

```go
// TransformRecord is a rigid placement, as four vectors: it survives encoding,
// which an r3.Transform does not. EX, EY, EZ are the transformed world basis
// (r3.Transform.Basis()), T the translation. r3.FromBasis rebuilds the
// transform, snapping encoding drift straight and rejecting anything that is
// not an isometry.
type TransformRecord struct {
    EX, EY, EZ r3.Vec // the images of the world axes — dimensionless directions
    T          r3.Vec // the translation, millimetres (§5.2)
}
```

**Serializability is a rule, not an aspiration.** Every type reachable from a
`Recipe` MUST be encodable and decodable: exported fields only; no foreign
interface; no foreign type whose fields decad cannot see. decad's own sealed
interfaces — `Extent`, `AngularExtent`, `SideExtent`, `SideAngular`, `Axis`,
`Selector`, `StepOpts`, `CurveSegment` — are **closed variant sets decad owns**, so
decad ships their codec: each encodes as a tagged object and decoding dispatches on
the tag. That is precisely what decad cannot do for `sketch.Entity`, which is why
the entity never enters a `Step`. The rule is transitive, so it reaches the query
types too (§9): an `EdgeQuery` / `FaceQuery` is recorded content — its predicates
and its cardinality assertion — and a predicate that cannot be encoded does not
ship, exactly as an option that cannot be recorded does not. `units.Value`
carries its own text form — `"10 mm"`, the magnitude and the unit's registered
symbol, an exact bit-for-bit round trip — and refuses to write what cannot be
read back: an unnamed or overflowed kind, a non-finite magnitude. Every quantity
a `Step` records therefore encodes.

**The root wire format is versioned and strict.** Canonical JSON is
`{"format":"decad.recipe","version":2,"steps":[...]}`. Unversioned and
version-1 input are accepted under the version-1 grammar and re-encode as
canonical version 2. A version-1-only decoder rejects a version-2 envelope
with `ErrUnsupportedRecipeVersion` before step dispatch. `json.Marshal` runs
full recipe validation under default limits; `EncodeRecipe` runs the same
encoder under explicit limits for trusted larger recipes. Invalid Unicode,
unknown versions, unknown fields, duplicate keys, trailing values, malformed
operation shapes, invalid references and configured resource-limit overruns
reject. The in-memory `Recipe` keeps no version field: format metadata is not
design intent.
`docs/recipe-replay-design.md` §§2–7 is normative.

**A recipe is evaluable, not merely encodable.** `Recipe.Validate` checks every
operation's required/forbidden fields, independently proves every stored
profile through a private `sketch` arrangement, checks every reachable value,
and checks every backward reference + the body-liveness state machine.
`Evaluate` applies selected recipe limits while deep-copying + normalizing the
recipe, validates it, then walks steps in order through the selected
package-owned evaluator. Immediate feature calls and replay share the same
recorded-step helpers; a second implementation is forbidden. A valid intent
beyond the selected evaluator's reach remains `ErrUnsupported`.

**A `Step` holds no `*Body` either.** A body reference in a `Step` is a `StepRef` —
the step that produced the body — and that is what makes `Inputs` a graph. The
extents and the axis that name a body (`ToFace`, `ToFaceAngular`, `EdgeAxis`, §8.1)
hold a `BodyRef`, which is either form; recording a step substitutes the `StepRef`
for the `*Body` the caller passed, so what a `Recipe` carries is only ever values:

```go
// BodyRef names the body a selector resolves against. A live *Body is one, which
// is what a caller passes; a StepRef is one, which is what a recorded — or a
// decoded — Recipe holds. One field, and the codec maps between them.
type BodyRef interface{ bodyRef() }

func (*Body) bodyRef()
func (StepRef) bodyRef()
```

A `StepRef` handed to a *feature call*, where a live body is required, is
`ErrUnresolvedBody` (§12).

**`Plane` is what makes an `Extrude` step complete.** A `sketch.Profile` is
plane-local 2D — its boundary is `(u, v)` in the frame of the sketch plane, and it
back-references no plane of its own. A `Step` that recorded only the profile would
therefore record the same bytes for the same rectangle extruded on XY and on XZ, and
re-evaluating it could not know which solid was meant. Recording the sketch plane is
what the completeness rule demands, and it is why §8's `Extrude` and `Revolve` take
the sketch (§7).

**Every `Step` produces exactly one body**, so a `StepRef` names it without
ambiguity, and `Inputs` is what makes the recipe a graph rather than a list of
unrelated features. `Inputs` records every body a step **depends on**, which is not
always a body it consumes:

- `Fillet`, `Chamfer`, `Shell` and `Placed` depend on one — the body they modify
  or place, which they consume;
- `Duplicate` and `PlacedCopy` (§8) depend on one — the source they copy — and
  consume **none**: the source's `StepRef` is recorded in `Inputs`, the source
  stays live, and the copy is a new body. This is the same depend-without-consume
  the `ToFace` extrude below uses, applied to a body-to-body copy;
- the booleans depend on two, and consume both. **`Cut`'s `Inputs` order is
  `[target, tool]`** — the two roles are asymmetric and order is the only thing that
  distinguishes them;
- `Extrude` and `Revolve` consume no body, and leave `Inputs` empty **only when
  their extent and their axis name none**. A `ToFace`, a `ToFaceAngular` or an
  `EdgeAxis` names one (§8.1), and the extrude genuinely depends on it — the solid
  it produces is a function of that body's geometry — so **that body's `StepRef` is
  recorded in `Inputs`**, in extent order first and axis second, deduplicated. A
  `TwoSided{One: ToFace{Body: a…}, Two: ToFace{Body: b…}}` records both. Without
  this the recipe would not be a complete graph: a second evaluator would reach the
  extrude with no way to know which body's face it stops at.
- `Loft` depends on and consumes no body. Its increment-1 form has no
  body-relative stop, so its `Inputs` is empty.
- **`ThroughAll` and `ThroughAllSide` depend on bodies they do not name.** Their
  stops are the far sides of the live bodies the sweep meets, so the dependency is
  ambient at the CALL but must never be ambient in the RECORD: the feature call
  resolves which bodies actually bound the stops and records **each stop body's
  `StepRef` in `Inputs`**, in stop order along the sweep (after any named-extent
  refs, deduplicated like the rest). Replay then reaches the same stops explicitly
  — a recipe whose through-all depended on "whatever happened to be live" would
  re-evaluate to a different model in a different document state, which the
  completeness rule forbids.

Depending on a body is **not** consuming it: `Extrude`, `Revolve`, and `Loft`
retire nothing, and the body a `ToFace` names stays live in `Document.Bodies()`.
§6's retire rule is unchanged, and lists exactly the operations it covers.

**A `StepRef` is not the index invariant #3 forbids.** Invariant #3 forbids indices
as *topology* selectors — `Edges()[3]` — because an exact kernel decomposes a body
into different faces and edges. A `StepRef` names a *step in this recipe*, and the
step list is the recipe's own content, not the evaluator's output: step 2 is step 2
under every evaluator, forever. Step references are stable by construction;
topology indices are not. That is the whole of the distinction.

`Extent` and `Angular` are the two disjoint extent sets of §8.1, and **at most one
of them is non-nil, keyed to `Op`**: `Extrude` sets `Extent` and leaves `Angular`
nil; `Revolve` sets `Angular` and leaves `Extent` nil; every other `Op` leaves both
nil. That is what lets a `Recipe` encode a revolve at all — an `AngularExtent` is
not assignable to an `Extent` field, by design.

**Parameterisation is the caller's Go function, and there is no parameter
sublanguage.** `Step.Values` holds literal quantities, and a `Recipe` binds no
names. §6 already settles this: the agent's Go function *is* the feature tree, and
re-running `MakeBracket(height)` with a new height *is* the rebuild — it emits a
new `Recipe` with new values. decad does not build an interpreter for a language
the agent already has.

**`StepOpts`** — the feature options a `Step` carries, sealed and typed, one struct
per `OpKind` that has options. Not `[]any`, not a stringly key/value map: §4 rejects
`core.Base`-as-anything, and a recipe is the last place to smuggle it back in.

```go
type StepOpts interface{ stepOpts() } // sealed

type ExtrudeOpts struct { Taper units.Value; /* ... */ }
type RevolveOpts struct { /* ... */ }
type FilletOpts  struct { TangentChain bool }
type AsymmetricChamferOpts struct {
    Reference FaceSelector
    Other     units.Value
}
type ChamferOpts struct {
    TangentChain bool
    Asymmetric  *AsymmetricChamferOpts
}
type ShellOpts struct {
    Sense      ShellSense
    NoOpenings bool
}
type LoftOpts struct {
    Profile2  ProfileRecord // the "to" section
    Plane2    PlaneRecord   // the "to" section's plane
    Alignment []int         // per-loop segment-rotation offset; absent means every offset is 0
}
```

`docs/recipe-replay-design.md` §3.2 owns required `StepOpts` wire payload
fields and their absent/null rules. The decoder reads required fields through
presence-aware pointer wire fields, then constructs the value-form variant.
Feature-call defaults are materialized in the recorded `StepOpts`, so canonical
output always carries the required field.

The completeness rule applied to options: **every `ExtrudeOption`,
`RevolveOption`, `FilletOption`, `ChamferOption`, `ShellOption`, and
`LoftOption` MUST be representable in the corresponding `…Opts` struct.** An
option with nowhere to land in the recipe does not ship — a tapered extrude that
round-tripped as an untapered one would be exactly the lossy record the
completeness rule forbids.

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
// EdgeAxis is a linear edge, selected — never a pointer. Body is what Edge
// resolves against: a Revolve is handed no body, so the axis must name its own.
type EdgeAxis struct{ Body BodyRef; Edge EdgeSelector }
```

`EdgeAxis.Edge` MUST resolve to **exactly one** linear edge of `Body`
(`EdgeQuery.Exactly(1)`); zero or many is `ErrCardinality` (§12), and a non-linear
edge is `ErrDegenerate`. `Body` MUST be a live body of the same `Document` the
`Revolve` is called on — another document's is `ErrForeignBody`, a retired one is
`ErrRetiredBody` — and its `StepRef` is recorded in the step's `Inputs`.

**`Interference` / `Clearance`** — the pairwise results of §10. Both name the two
bodies and carry their quantity as a `Measurement`, so both report their own
exactness like everything else. `docs/interference-design.md` owns the
read-only overlap proof; `docs/clearance-design.md` owns the disjointness and
minimum-gap proof.

```go
// Interference: the two bodies overlap, and by how much.
type Interference struct {
    A, B   *Body
    Volume Measurement // the overlap volume, Kind Volume
}

// Clearance: the two bodies do not overlap, and how close they come.
type Clearance struct {
    A, B *Body
    Gap  Measurement // minimum distance between the two surfaces
}
```

The rest are deferred:

```go
type FeatureRef struct{ /* ... */ }    // an opaque handle to the feature that created a body or face
type Mesh struct{ /* ... */ }          // a triangle mesh; an OUTPUT of Tessellate, never the representation
type EdgePredicate struct{ /* ... */ } // one clause of an EdgeQuery; the §9 constructors return these
type FacePredicate struct{ /* ... */ } // one clause of a FaceQuery
```

**`SurfaceKind`** — the discriminant a `Surface` reports (§6.1). Its constants are
**`Kind`-prefixed**, because the unprefixed names are already the surface variant
types and a package has one namespace:

```go
type SurfaceKind int

const (
    KindPlane SurfaceKind = iota
    KindCylinder
    KindCone
    KindSphere
    KindTorus
    KindNURBS
    KindFaceted
)
```

`Direction` — the enumerated sense a standalone extent carries — is *not* deferred:
it is specified in full in §8.1, and declared there.

The remaining topology of §2's chain, each deferred but for the one method that
carries its weight:

```go
// Lump is a connected solid piece of a body. Body.Lumps() returning more than one
// means the body is disconnected.
type Lump struct{ /* ... */ }

// Shell is a connected set of faces.
type Shell struct{ /* ... */ }
func (s *Shell) IsVoid() bool // true when the shell bounds an internal cavity

// Vertex is a topological point.
type Vertex struct{ /* ... */ }
func (v *Vertex) Position() VecMeasurement // millimetres (§5.2); a computed coordinate, so it is bounded (§6)

// MeshBody is imported triangle soup. It NEVER claims to be a solid B-rep —
// invariant #5 — and mesh import is a v1 non-goal (§13).
type MeshBody struct{ /* ... */ }
```

## 7. The sketch seam

`sketch` answers every 2D question; decad consumes the answer and NEVER
re-derives it. The recording contract at this seam — the trim contract
(`TStart`/`TEnd`/`TExact`), the recording IR a `Step` carries, and
`ErrUnrecordableProfile`'s full semantics — is specified in
`docs/sketch-seam-design.md`; this section states the feature-call rules built
on it.

```go
w := sketch.NewWorld()
s, _ := w.CreateSketch(w.XY())
s.CreateRectangle(0, 0, 100, 60)
s.Solve(ctx)

// sketch has already proven this region closes: it is a valid, extrudable profile.
prof := s.Profiles()[0]

doc := decad.New()
body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
```

**A feature takes the sketch and the profile, in that order.** A
`sketch.Profile`'s geometry is pure plane-local 2D — an outer loop and its holes
in `(u, v)` — so the profile alone does not place the region in space: the plane
is the sketch's. `s.Plane()` is the construction plane the sketch is drawn on,
and `s.Plane().Frame()` is the orthonormal `r3.Frame` that lifts the plane-local
profile into world space. A `Step` records that frame as a `PlaneRecord` —
origin, u, v — because an `r3.Frame` does not survive encoding (§6.2), and the
frame is what the profile normal — the sense `Direction.Along` means for a
linear extent (§8.1) — is read from.

**`p` MUST be an unaltered current profile of `s`, and decad records only a
fresh sketch-authenticated match.** `Profile.Sketch()` MUST be `s`.
Every boundary entity MUST be non-nil and present in `s.Entities()`; a foreign
boundary entity is `ErrForeignProfile` (§12). decad then calls `s.Profiles()`
and requires every exported field of `p` to exactly match one fresh current
profile, including entity identities, boundary order/ranges, polylines, holes,
area and validity. It records the fresh match, never caller-mutable `p`. No
match is `ErrInvalidProfile` (§12).

A `*sketch.Profile` is freshly allocated by every `Profiles()` call, so pointer
membership cannot authenticate it. Exact snapshot matching preserves the
profile value while obtaining a trusted copy. `Profile.Sketch()` still proves
the source sketch; a different source is `ErrForeignProfile`. Either foreign
case could otherwise lift another sketch's plane-local coordinates through
`s`'s frame and silently place the wrong solid.

**A *stale* profile is detected, and rejected.** A profile is a snapshot: one
built before a later `Solve`, parameter edit or geometry change still holds
entities that belong to `s`, but its boundary is the old geometry, and sweeping
it would silently build the wrong part. `sketch` answers this too:
`Profile.Revision()` is the value of `Sketch.Revision()` — a fingerprint of
every input `Profiles()` reads, compared for equality only — at the moment the
profile was built, and `Profile.IsStale()` reports whether the sketch has
changed since. A feature handed a stale profile is `ErrStaleProfile` (§12); the
caller rebuilds with `s.Profiles()` and passes a current one.

`doc.Extrude` REJECTS a `sketch.Profile` whose `Valid` is false — a
self-intersecting or degenerate region is never silently swept. This early
check can only reject; `Valid == true` admits nothing until the complete
snapshot matches a fresh profile. `Profile.Valid` is `sketch`'s
region-validity answer, not one decad recomputes. Snapshot authentication is a
separate integrity gate, also reported as `ErrInvalidProfile`. The one further
rejection is not a validity judgement at all: a valid profile whose boundary
decad cannot record exactly — a profile containing a
`Partial` fragment `sketch` could not certify, `TExact == false` — whether the
arrangement sampled its cut or the whole-sketch gate withheld certification —
or one whose certified range the seam's falsifier disproves —
is `ErrUnrecordableProfile` (§12), because a `Step` that recorded the whole curve
where the caller drew a piece of it, or an uncertified range as an exact trim,
would be the lossy record the completeness rule (§6.2) forbids.

**The seam's read-outs are `sketch`'s answers, and its one check can only
falsify.** Admission of a `Partial` fragment is decided by `TExact` — `sketch`'s
own certification — and by nothing else; the one check decad runs is a
reject-only falsifier that can disprove the flag but never admit a fragment.
decad neither re-derives the trim nor infers it, and a fragment it cannot
record it **rejects** — it never repairs, projects or fits a point `sketch`
handed over, and it never solves for one. What decad reads of a
`BoundaryEdge`, why a residual test cannot be an admission gate, and the
whole-edge rules are specified in `docs/sketch-seam-design.md`.

Whether the *sketch* is fully constrained is a separate, sketch-level question: a
profile can close while the sketch still has degrees of freedom. It is not decad's
to answer — an agent that wants that guarantee gates on `sketch.Sketch.Verify`
before extruding. decad never re-derives it.

## 8. Features

v1 vocabulary, deliberately small: **Extrude, Revolve, Union/Cut/Intersect,
Fillet, Chamfer, Shell, Placed, Duplicate, PlacedCopy, Loft**. Sweep is
deferred. `docs/loft-design.md` owns `Loft`'s signature, its two-profile
correspondence rule, and its increment-1 scope.

```go
func (d *Document) Extrude(s *sketch.Sketch, p *sketch.Profile, e Extent, opts ...ExtrudeOption) (*Body, error)
func (d *Document) Revolve(s *sketch.Sketch, p *sketch.Profile, axis Axis, a AngularExtent, opts ...RevolveOption) (*Body, error)
```

Both take the **sketch** as well as the profile, because a `sketch.Profile`'s
geometry is plane-local and the plane is the sketch's (§7). `p` MUST be a
current, unaltered profile of `s`: another sketch's profile or a foreign
boundary entity is `ErrForeignProfile`, a stale one is `ErrStaleProfile`, and a
snapshot that does not match a fresh `s.Profiles()` result is
`ErrInvalidProfile` (§7/§12). decad
reads the plane through `s.Plane()` and its frame through `s.Plane().Frame()`, and
records the plane — as a `PlaneRecord`, and the profile as a `ProfileRecord` — in
the `Step` (§6.2), so the recipe stays complete and holds no live sketch.

Booleans are **explicit** — not folded into every feature with an ambient,
implicitly-chosen target — and they never mutate an operand or take a target-out
parameter:

```go
func Union(a, b *Body) (*Body, error)
func Cut(target, tool *Body) (*Body, error)
func Intersect(a, b *Body) (*Body, error)

func UnionContext(ctx context.Context, a, b *Body) (*Body, error)
func CutContext(ctx context.Context, target, tool *Body) (*Body, error)
func IntersectContext(ctx context.Context, a, b *Body) (*Body, error)
```

No `*Document` appears in those signatures, and none is needed: a `*Body` carries
its owning document (§6), so each call retires its operands and registers its
result inside the document that owns them. The operands are themselves unchanged
— invariant #4 — and `Document.Bodies()` and `Document.Verify()` stay truthful.
Operands owned by different documents are `ErrForeignBody`.

**Context variants cancel the complete Boolean before its atomic commit.**
They pass `ctx` through operand tessellation, exact-predicate classification,
cutting, stitching, mesh audit, and exact volume calculation. Cancellation
returns `ctx.Err()` unchanged and leaves both operands, the live-body set, and
the recipe unchanged. `Union` / `Cut` / `Intersect` are compatibility wrappers
over their `Context` variants with `context.Background()`; success keeps the
same consuming behavior and recorded step.

**A boolean failure is typed, because its three failures are three different
caller actions.** A boolean that produces no body, reaches a valid-model limit
this evaluator cannot handle, or breaks an internal invariant returns a
`BooleanError` — the operation, its operand `StepRef`s, and a branchable `Code`
— wrapping the sentinel `errors.Is` already branches on, so compatibility
holds:

```go
type BooleanError struct {
    Op     OpKind            // Union / Cut / Intersect
    Inputs []StepRef         // the operands, as the Step would record them; [target, tool] for Cut
    Code   BooleanErrorCode
    // Error()/Message carry the human text; Unwrap() returns the wrapped sentinel.
}

type BooleanErrorCode int

const (
    // BooleanEmpty is a NORMAL geometric outcome: the result encloses no
    // volume — a disjoint Intersect, an all-removing Cut. Wraps ErrBooleanFailed.
    BooleanEmpty BooleanErrorCode = iota
    // BooleanUnsupportedContact is a VALID model whose boolean geometry reaches
    // a limit: a curved-surface tangency or near-contact, a coplanar
    // face-on-face overlap, a grazing edge, an isolated-point pinch, or a
    // analytic prism-arrangement refusal wrapping ErrUnsupported.
    // Recipe-recordable, evaluator-staged.
    // Wraps ErrUnsupported.
    BooleanUnsupportedContact
    // BooleanEvaluatorFailure is an internal invariant break: the stitched
    // boundary did not close, no split line was found. Wraps ErrBooleanFailed.
    BooleanEvaluatorFailure
)
```

`errors.Is(err, ErrBooleanFailed)` still holds for `BooleanEmpty` and
`BooleanEvaluatorFailure`; `errors.Is(err, ErrUnsupported)` holds for
`BooleanUnsupportedContact`. `errors.As(err, &be)` then `be.Code` is the fine
branch. The three separate a caller's three moves: `BooleanEmpty` — the model is
sound and the operation asked for nothing (change the geometry, or drop the
call); `BooleanUnsupportedContact` — the model is valid but past this
evaluator's reach (choose a construction that does not lean on that limit, or
wait for vN); `BooleanEvaluatorFailure` — a bug to file. Mixing the
three under one sentinel is what makes `errors.Is(err, ErrBooleanFailed)` too
coarse to drive recovery, so the `Code` draws the line the sentinel cannot.

**A valid-model boolean limit is staged, not malformed.** A curved-surface or
coplanar contact this evaluator cannot classify, and an analytic
prism-arrangement refusal wrapping `ErrUnsupported`, are
`BooleanUnsupportedContact`, NEVER `ErrDegenerate`: the input names a real
solid, and the refusal is the evaluator's reach, not a zero or self-crossing
region. That
"never `ErrDegenerate`" scopes to the contact-classification and analytic
arrangement refusals. It does not reach a distinct case: a valid operand whose
boolean OUTPUT cannot be
chorded finely enough to tessellate (a bridge pinch, a stalled ear clip)
surfaces `errors.Is(err, ErrDegenerate)` publicly, and that is a retryable
coarse-chording `ErrDegenerate` on that operand — a finer chord tolerance may
clear it — not a `BooleanError`. So `ErrDegenerate` covers BOTH an input with
no usable geometry (its §12 meaning) AND the coarse-chording tessellation
refusal, while the typed `BooleanError` family covers the empty result, the
unclassifiable contact, and the evaluator-internal failure. This
sentinel mapping fixes the test impact mechanically: a valid-contact assertion
checks `errors.Is(err, ErrUnsupported)` where a coarser taxonomy would check
`ErrDegenerate`, and landing those assertion changes is the implementation PR's
work, not this contract's.

This is one taxonomy with `Verify` FOR A CONTACT THAT REACHES `Verify`'s
read-only boolean: an unsupported-contact pair the boolean gives its
unsupported-contact outcome leaves that pair `Suspect` with a
`DiagUnsupportedPairContact` diagnostic (`docs/verification-design.md` §1.1), so it
reads the same way whether a caller ran the boolean directly or reached it
through interference. A pair `Verify` resolves EARLIER never reaches that
outcome, and two resolvers run before the mesh boolean does. The coplanar
`Plane`×`Plane` contact certificate (`docs/clearance-design.md`) makes a
certified coplanar touch read as a touching/clearance result — an `Exact`-zero
gap, no interference. The read-only analytic `OpIntersect` dispatch
(`docs/prism-boolean-design.md`; `docs/interference-design.md` §3.4) measures an
admitted coplanar prism pair's overlap outright. Neither emits a
`DiagUnsupportedPairContact`.

**What each boolean refusal leaves the caller.**
"Choose a construction that does not lean on a tangent contact" names a move
without naming the move, so the two refusals a modelling caller actually meets
are stated here concretely.

**The coplanar contact.** `Union` admits co-directional, coplanar straight
prisms to the analytic reduction (`docs/prism-boolean-design.md`), so their
overlapping caps build rather than refuse. Every other operation, and every
`Union` outside that admitted class, stays on the mesh path. There, two bodies
extruded from ONE sketch plane to one end plane have coplanar caps by
construction, and where their footprints OVERLAP those caps share positive
area, which is the contact the boolean reads: `BooleanUnsupportedContact`.
Coplanar caps on their own are not a contact — two such bodies standing apart
in the plane share no cap area, and every boolean over them runs. What replaces
the tangent contact is an INTERIOR overlap: no face pair coplanar, and each
operand reaching into the other's interior, so every contact is a transversal
crossing the evaluator does classify. Both operands span the same interval
here, so a LATERAL displacement never reaches that state — moving one body
sideways leaves its caps in the two planes the other's caps lie in, and the
overlap stands wherever the footprints still meet. The displacement has to run
ALONG the sweep, and the caller owns what it costs: it changes the enclosed
solid, with an effect that depends on the operation and geometry. That
displacement is necessary but insufficient when the profiles retain coplanar
lateral faces: identical footprints still share lateral face area after their
caps separate and return `BooleanUnsupportedContact`. The caller has to change
the profile or otherwise prove that no face pair is coplanar. A displacement
leaves a union's enclosed solid unchanged only when the moved operand is wholly
inside the other body both before and after the displacement. Moving an operand
into containment removes any former protruding volume. A caller proving a part
against a model built elsewhere has to state the deviation it accepted; it is
not free.

**The chain depth.** A boolean's result is a `Faceted` body whose held `Bound`
composes from the operation that made it, while §9's chord tolerance for the next
pair is a fixed fraction of that pair's diameter. Where the composed bound is the
coarser of the two, feeding the result back in as an operand is refused before
any contact is examined — a plain `ErrUnsupported`, not a `BooleanError`, since
the operand cannot be re-tessellated finer than the boundary it holds. Where the
held bound stays under that tolerance the result is an ordinary operand and the
chain continues, so the comparison at each step is what limits a chain, not the
fact that an operand came out of a boolean. At every step, the result's held
bound recomposes both operand bounds through `rimDelta`, then adds the final
rounding displacement. A chained boolean whose first operand carries a bound
and whose second operand also carries a nonzero bound therefore raises the held
bound for the next pair; successive booleans do not keep that bound flat. Where
the comparison does refuse, it is geometry rather than an argument the caller got
wrong. The booleans take no tolerance parameter (§9), by the same decision that
puts the tolerance's whole effect on the result's proven `Bound`. The bound is readable rather than
merely printed in the refusal — `Body.Tessellate` at any tolerance the faceted
body already meets returns a `Mesh` whose `Bound` is that held bound — so a
caller can size the limit before planning a chain. A construction needing many
booleans over one part has to be reshaped, not retried.

Modify operations return a new body, retiring the receiver, on the same terms:

```go
func (b *Body) Fillet(sel EdgeSelector, r units.Value, opts ...FilletOption) (*Body, error)
func (b *Body) FilletContext(ctx context.Context, sel EdgeSelector, r units.Value, opts ...FilletOption) (*Body, error)
func (b *Body) Chamfer(sel EdgeSelector, d units.Value, opts ...ChamferOption) (*Body, error)
func (b *Body) ChamferContext(ctx context.Context, sel EdgeSelector, d units.Value, opts ...ChamferOption) (*Body, error)
func (b *Body) Shell(sel FaceSelector, thickness units.Value, opts ...ShellOption) (*Body, error)
func (b *Body) ShellContext(ctx context.Context, sel FaceSelector, thickness units.Value, opts ...ShellOption) (*Body, error)
```

The `Context` forms bound cancellation latency in their cancellable construction
and audit paths, including Shell's shared offset-construction, crossing, and
nesting audit. Cancellation returns `ctx.Err()` before commit, so the receiver
stays live and the recipe and document remain unchanged. The original methods
delegate with `context.Background()` and keep their existing behavior.

Modify reach options are recordable intent, not evaluator switches:

```go
type FilletChamferOption interface {
    FilletOption
    ChamferOption
}

func WithTangentChain() FilletChamferOption
func WithAsymmetricChamfer(reference FaceSelector, otherDistance units.Value) ChamferOption
func WithNoOpenings() ShellOption
```

`WithTangentChain` expands selected seeds only through proven analytic G1
continuations. `WithAsymmetricChamfer` applies positional `d` on `reference`
and `otherDistance` on the other adjacent face. `WithNoOpenings` is the only
shell form that accepts `sel == nil`; it conflicts with a non-nil selector.
`docs/modify-reach-design.md` owns exact receiver/target limits, refusal order,
payloads, and recipe encoding.

Placement is a body operation on the same terms — it retires the receiver and
registers the placed body:

```go
func (b *Body) PlacedContext(ctx context.Context, t r3.Transform) (*Body, error)
func (b *Body) Placed(t r3.Transform) (*Body, error)
```

This is the whole of the "explicit transforms" story: a body is positioned by an
argument the caller states — an `r3.Transform`, a rigid motion (§5.2) — never by
an ambient assembly context (§4). The zero `Transform{}` is invalid
(`Transform.IsValid`) and is `ErrDegenerate` (§12). The step records the motion
as a `TransformRecord` (§6.2). `PlacedContext` checks `ctx` through any
faceted-payload rebuild and returns its error without changing the document.
`Placed` is the compatibility wrapper using `context.Background()`.

**A body can be copied without consuming it.** `Placed` retires its receiver, so
modelling a part once and placing several instances — a bolt pattern, several
interference placements, one cutting tool reused for several holes — would mean
rebuilding the whole feature chain per instance. The two non-consuming copies
close that gap:

```go
func (b *Body) DuplicateContext(ctx context.Context) (*Body, error)
func (b *Body) Duplicate() (*Body, error)
func (b *Body) PlacedCopyContext(ctx context.Context, t r3.Transform) (*Body, error)
func (b *Body) PlacedCopy(t r3.Transform) (*Body, error)
```

Each returns a NEW live body and leaves the receiver **live**: the source is
depended on, never consumed. `Duplicate` re-registers the receiver's immutable
payload under a new step — identical, independent geometry, a fresh body
identity. `PlacedCopy(t)` re-evaluates the payload under the composed rigid
motion exactly as `Placed` does, so `Duplicate` is `PlacedCopy` with no motion;
the zero `Transform{}` is `ErrDegenerate` as in `Placed`, and `r3.Identity()` is
a valid no-op motion. A body this evaluator did not build is `ErrUnsupported`, as
`Placed`'s is. The context-taking variants check `ctx` throughout a faceted
rebuild and leave the source, live-body set, and recipe unchanged on
cancellation. `Duplicate` and `PlacedCopy` are compatibility wrappers using
`context.Background()`.

**A copy preserves geometry, so it preserves provenance.** A copy is the same
part at a new identity and position, and `FaceCreatedBy` (§9) must still find its
faces. The modify-design "a result's roles are its own record's, never inherited"
rule governs the geometry-CHANGING ops (`Fillet` / `Chamfer` / `Shell`), whose
rewrite makes the source's roles no longer name the same geometry; a copy changes
no geometry, so it does not re-derive provenance from scratch — it carries the
source's, reproduced from the same record:

- a **faceted** copy (a boolean's output) carries each face's UPSTREAM
  `FeatureRef` origins verbatim — they ride in the payload itself, so a boolean's
  preserved provenance survives the copy unchanged, and `FaceCreatedBy(ref)`
  matches a cut result's copy exactly as it matches the cut result. A carried
  upstream `FeatureRef` is a `FaceCreatedBy` provenance origin, not an own-step
  reference: a boolean's faces already carry the operands' upstream origins, not
  a role keyed to the boolean's own step (§9), so keeping it verbatim is outside
  the never-inherited rule — that rule governs the own-step references a copy DOES
  re-derive from its own record, `Body.Origin()` (the `body` role, below) and the
  analytic/modified feature roles;
- an **analytic** or **modified** copy (a prism, revolve, cup, tube) carries no
  separate upstream provenance — a prism cap is created by the prism step itself
  — so re-evaluation reproduces the body's own fixed roles (`capStart`, `capEnd`,
  `side(i, j)`, …) from the same record, keyed to the copy's own producing step
  (§6.1's rule that roles derive from the recorded step), and the copy's own
  `CapStart` / `CapEnd` / `FaceCreatedBy` resolve against it;
- the `body` role — `Body.Origin()` — is the copy's own step in every case.

Two new `OpKind`s — `OpDuplicate` and `OpPlacedCopy` — record the source's
`StepRef` in `Inputs` on the same terms `ToFace` and `EdgeAxis` record a
depended-on body (§6.2): the source's `StepRef` in `Inputs`, the source **not**
in the consumed set, so §6's retire rule never touches it. Each is a closed-set
member with its own named-text wire token in the `OpKind` codec — `OpDuplicate`
is `"duplicate"`, `OpPlacedCopy` is `"placed_copy"` — beside `"placed"` and the
rest (§6.2), so the constant order is never a serialization concern. A recipe
replay reproduces every copy deterministically — the copies are steps like any
other, and the source stays live for each.

**`Step.Placement` is keyed to the two placing ops.** `OpPlacedCopy` records its
motion as a `TransformRecord` in `Placement`, exactly as `OpPlaced` does, so
`Placement` is present (nonzero, valid) for both; `OpDuplicate` records no
motion, so its `Placement` is absent (the zero value), the same field-keying
discipline the extent/angular one-of already enforces (§6.2). `Placement` is
therefore present exactly for `OpPlaced` and `OpPlacedCopy` and forbidden on
every other op — the same required/forbidden-field discipline
`docs/recipe-replay-design.md` §3.2 states for `OpPlaced`. The stored-recipe
contract carries the copy ops in full: `docs/recipe-replay-design.md` holds the
`OpDuplicate` / `OpPlacedCopy` §3.2 shape rows (each `Inputs: 1`,
`consumed inputs: 0`, and the `Placement` present/absent rule above), their §4
liveness handling (the source `StepRef` depended on, never retired) and §5.1
replay dispatch — this API contract fixes the copy ops' shape, the replay design
fixes their schema.

### 8.1 Extent — illegal states unrepresentable

Fusion has three mutually-exclusive `setXxxExtent` methods with no enforcement,
and `add()` fails at runtime. decad makes exclusivity structural:

Extrude takes a **linear** extent:

```go
type Extent interface{ extent() }

// SideExtent is what ONE side of a two-sided extent may be. Sealed, and narrower
// than Extent: implemented ONLY by DistanceSide, ThroughAllSide and ToFace.
type SideExtent interface{ sideExtent() }

// Extent — standalone. A standalone extent is one-sided, so it carries its own
// sense as an enumerated Direction.
type Distance   struct { D units.Value; Dir Direction }   // Extent ONLY
type ThroughAll struct { Dir Direction }                  // Extent ONLY
type Symmetric  struct { D units.Value; FullLength bool } // Extent ONLY
type TwoSided   struct { One, Two SideExtent }            // Extent ONLY

// ToFace stops the sweep at a face of Body. Extent AND SideExtent; Offset is SIGNED.
// Body is what Face resolves against: an Extrude is handed no body, so the extent
// must name its own.
type ToFace struct { Body BodyRef; Face FaceSelector; Offset units.Value }

// SideExtent — one side of a TwoSided. The SIDE supplies the sense, so a side
// variant never carries a Direction.
type DistanceSide   struct { D units.Value }                     // SideExtent ONLY
type ThroughAllSide struct{}                                     // SideExtent ONLY
```

Revolve takes an **angular** extent, its own sealed set, with the same two-tier
split:

```go
type AngularExtent interface{ angularExtent() }

// SideAngular is what ONE side of a two-sided angular extent may be. Sealed:
// implemented ONLY by AngleSide and ToFaceAngular.
type SideAngular interface{ sideAngular() }

type AngleExtent    struct { A units.Value; Dir Direction }   // AngularExtent ONLY
type FullRevolution struct{}                                  // AngularExtent ONLY
type SymmetricAngle struct { A units.Value; FullLength bool } // AngularExtent ONLY
type TwoSidedAngle  struct { One, Two SideAngular }           // AngularExtent ONLY

// ToFaceAngular stops the revolve at a face of Body. AngularExtent AND SideAngular.
type ToFaceAngular struct { Body BodyRef; Face FaceSelector }

type AngleSide struct { A units.Value }                       // SideAngular ONLY
```

**The nesting is the point.** A side of a two-sided extent is a *single direction
of travel*, so only the variants that mean one are admissible there. Because
`Distance`, `ThroughAll`, `Symmetric`, `TwoSided`, `AngleExtent`, `SymmetricAngle`,
`TwoSidedAngle` and `FullRevolution` implement only the outer interface and not the
inner one, `TwoSided{One: TwoSided{...}}`, `TwoSided{One: Symmetric{...}}`,
`TwoSided{One: Distance{...}}` and `TwoSidedAngle{One: FullRevolution{}}` do not
compile. Illegal states are unrepresentable structurally, not rejected at runtime.

**Magnitude and direction are separate, and direction is role-scoped.** A magnitude
— `Distance.D`, `AngleExtent.A`, `Symmetric.D`, and the rest of the set §12
enumerates — is **non-negative**; a negative magnitude is `ErrNegativeMagnitude`
(§12), never a reversal. Sense is carried **only** by the enumerated `Direction`,
never as a sign on a number: a signed magnitude means every value has two ways to
say the same thing and one way to contradict itself.

Which value carries the `Direction` is decided by **role**, and every variant obeys
the same rule:

| Role | Sense comes from | Carries `Direction` |
|---|---|---|
| Standalone one-sided (`Distance`, `ThroughAll`, `AngleExtent`) | the value itself — nothing else states it | yes |
| A side of a `TwoSided` / `TwoSidedAngle` (`DistanceSide`, `ThroughAllSide`, `AngleSide`) | the side it occupies | **never** |
| Sense implied by a target (`ToFace`, `ToFaceAngular`) | the target face | never |
| Structurally two-sided (`Symmetric`, `SymmetricAngle`, `TwoSided`, `TwoSidedAngle`, `FullRevolution`) | the shape of the variant | never |

That is why `Distance` and `DistanceSide` — and `ThroughAll` and `ThroughAllSide`,
and `AngleExtent` and `AngleSide` — are distinct types rather than one type used in
two places. A side already **is** a single direction of travel, so a second,
possibly contradicting direction on the same value — `TwoSided{One: Distance{Dir:
…}}` — is precisely the illegal state this section abolishes. Neither form is
assignable where the other belongs.

**`ToFace.Offset` is not a magnitude**, and it is the one signed number in the
extent vocabulary. It is a **signed displacement along the target face's normal**.
The role table above still holds: the *sense* of a `ToFace` comes from the target
face, and the sign says only which **side** of that face the sweep stops on — a
positive offset overshoots it, a negative one stops short of it ("stop 2mm short of
that boss"). `ToFace` carries no `Direction` to reverse, so clamping the offset
non-negative would make one of those two ordinary intents unrepresentable.
`ErrNegativeMagnitude` therefore does **not** apply to `ToFace.Offset` (§12).

`Direction` is a two-valued enumeration. There is no "both": a sweep that runs both
ways is `Symmetric`, `TwoSided`, `SymmetricAngle` or `TwoSidedAngle`, stated
structurally.

```go
type Direction int

const (
    Along   Direction = iota // linear: with the sketch plane's normal (§7). angular: right-handed about the revolve axis.
    Against                  // the opposite sense
)
```

The two sets are **deliberately disjoint**: no linear extent satisfies
`AngularExtent` and no angular extent satisfies `Extent`. That is what makes
"revolve 90mm" unrepresentable rather than a runtime error.

`ToFace` and `ToFaceAngular` take a **`FaceSelector`, never a `*Face`** — invariant
#3 and §9's rule hold inside features too, not merely at their edges.

**A selector needs a body to resolve against, and the feature does not supply one.**
`FaceSelector.SelectFaces` takes a `*Body` (§9), but `Extrude` and `Revolve` take a
sketch and a profile — no body at all. So the extent names its own: `ToFace.Body`,
`ToFaceAngular.Body`, and `EdgeAxis.Body` (§6.2). At a feature call that `BodyRef`
is a live `*Body` — a `StepRef` there is `ErrUnresolvedBody` (§6.2) — and `Face` is
resolved as `Face.SelectFaces(Body)` and nothing else. The rules are the same
everywhere:

- the selector MUST resolve to **exactly one** face — that is what
  `FaceQuery.Exactly(1)` is for. Zero faces or more than one is `ErrCardinality`
  (§12); "extrude up to that face" is meaningless when "that face" is four faces;
- `Body` MUST be a live body of the same `Document` the feature is called on.
  Another document's body is `ErrForeignBody` (§12) — a face in another document's
  coordinates would stop the sweep somewhere the caller never named — and a body the
  document has retired is `ErrRetiredBody` (§6);
- the step **depends on** that body, so its `StepRef` is recorded in the step's
  `Inputs` (§6.2). It is not consumed and not retired.

Options carry the rest (`WithTaper(units.Degrees(3))`), via
`github.com/lestrrat-go/option/v3` — the house functional-options library, and an
approved dependency. **Every option MUST be representable in the corresponding
`…Opts` struct a `Step` records (§6.2).** An option a `Recipe` cannot round-trip
would make the recipe a lossy record of intent, which §2 does not permit; such an
option does not ship.

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
func NormalTo(v r3.Vec) FacePredicate          // planar face whose normal is parallel to v — EITHER sense
func Facing(v r3.Vec) FacePredicate            // planar face whose OUTWARD normal points ALONG v — ONE sense
func FaceCreatedBy(f FeatureRef) FacePredicate // provenance, the face analog of CreatedBy
```

`Convex()` and `Concave()` read `Edge.IsConvex` (§6.1): the walked-boundary
convexity, not the material angle across the edge. A hole's rim edges are
concave, so a fillet meant for them asks for `Concave()`.

**`Facing(v)` is the signed one-face predicate.** `NormalTo(v)` matches a planar
face on either normal sense, so a slab's two parallel caps both match `NormalTo(z)`
— the right answer when a caller wants both, the wrong one when they mean the top
only. `Facing(v)` matches only the face whose OUTWARD (material-leaving) normal
points along `v` — parallel to `v` **and** the same sense, a positive projection —
so `Faces(Planar(), Facing(z))` picks the single top cap that
`Faces(NormalTo(z))` returns as a pair. A zero or non-finite `v` is rejected at
resolve, as `NormalTo`'s is (§12). `Facing` is a closed-set variant like every
predicate, so it ships its own tagged codec entry (kind `"facing"`, a `dir`
payload decoded through a pointer wire field like `normal_to`) — recipe-stable.

**Typed role helpers keep the fixed roles out of string literals.** A provenance
selector names a `FeatureRef` (§6.1), and the roles a feature mints — `capStart`,
`capEnd`, `body` — are a public part of the contract, but hand-typing `"capStart"`
invites a typo the compiler cannot catch. The fixed roles get typed constructors:

```go
func CapStart(b *Body) FeatureRef // the start-cap role of b's producing step
func CapEnd(b *Body) FeatureRef   // the end-cap role of b's producing step
```

`FaceCreatedBy(CapStart(body))` selects the start cap without the caller writing
the string; each helper reads `b.Origin().Step` (§6) and pairs it with its fixed
role — `FeatureRef{Step: b.Origin().Step, Role: "capStart"}`. The body-role
reference is already `Body.Origin()`, so no helper duplicates it.

**The cap roles exist only where the producing step mints them.** `CapStart(b)` /
`CapEnd(b)` name `b`'s OWN producing step, so they resolve to a face only when
that step actually mints the fixed cap role: an extrude, a partial revolve, a
shell that built a tube — the analytic prisms and partial revolves whose
evaluator emits `capStart` / `capEnd` — an admitted analytic `Union`, which
rebuilds an analytic prism with fresh roles under its own step, and a `Placed` /
`PlacedCopy` / `Duplicate` of one, which re-mints those roles under the copy's
own step (the copy-provenance rule above). On a body whose step mints no such
role the helper still returns a well-formed `FeatureRef` — it just matches
nothing, an ordinary `ErrNoMatch` at resolve (or `ErrCardinality` under an
implicit exactly-one). That covers a mesh-path `Union` / `Cut` / `Intersect`
result (its faces carry the operands' UPSTREAM origins, not a cap role keyed to
the boolean's own step), a full revolution (no caps at all), and `CapEnd` on a
cup (a cup mints `capStart` and a `shellCap` pocket floor, no `capEnd`). To name
a cap that a mesh boolean's operand contributed, select
`FaceCreatedBy(CapStart(originalBody))` against the upstream body whose
provenance the mesh boolean preserved (§9), never `CapStart(meshBooleanResult)`.

The **indexed** `side(i, j)` roles stay positional and get no helper: a wall
is named by geometry — `Faces(Planar(), Facing(v))` — never by a hand-counted
loop/segment index, so a fillet that re-segments a section can never silently
invalidate a hand-typed number, and the one selection style that survives a
rebuild is the geometric one selectors exist to provide.

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

**A selector failure says which clause emptied the set.** A cardinality error
that reports only a count tells an agent its assumption was wrong but not how,
so it must reconstruct the query from source and probe it one clause at a time —
exactly the iterative work the API should make mechanical. `SelectEdges` /
`SelectFaces` return a `SelectionError` wrapping `ErrNoMatch` or `ErrCardinality`
(§12), so `errors.Is` still branches; the type carries what the agent needs to
repair the query directly:

```go
type SelectionError struct {
    Kind      SelectorKind        // edge or face
    Query     string              // the query's stable rendering — q.String()
    Body      *Body               // the body resolved against; Body.Origin() names its feature
    Expected  string              // the cardinality assertion, rendered: "exactly 4", "at least 1", or "any"
    Actual    int                 // the total match count
    Residuals []PredicateResidual // the running match count after each clause, in query order
    // Error()/Message carry the human text; Unwrap() returns the wrapped sentinel.
}

type PredicateResidual struct {
    Predicate string // the clause's stable rendering — "convex", "parallel_to(0,0,1)"
    Remaining int    // candidates still matching after this clause AND every clause before it
}

type SelectorKind int // EdgeSelectorKind / FaceSelectorKind; a stable String()
```

`Residuals` is the conjunction evaluated cumulatively in query order: `Remaining`
starts at the body's whole edge or face count and can only fall, and the clause
whose `Remaining` reaches zero is the one that emptied the set. It is diagnostic
enrichment only — WHICH entities a satisfiable query matches, and the final match
set, are exactly as §9 already defines them, and the residuals are computed only
on the failing path, so a resolving query pays nothing. Modify ops (`Fillet` /
`Chamfer` / `Shell`) return the `SelectionError` unchanged, still branchable.

**The implicit exactly-one callers report through the same `SelectionError`.**
`ToFace` and `ToFaceAngular` (their stop face) and `EdgeAxis` (its axis edge)
resolve their selector through `SelectFaces` / `SelectEdges` and then demand
exactly one match — the implicit exactly-one of §12. The rule turns on ONE
distinction — did the selector's OWN explicit assertion fail? A failed explicit
assertion is preserved unchanged; the implicit exactly-one owns every other
outcome that is not a single match, an unasserted zero or several OR a satisfied
explicit assertion whose count is not one:

- A selector whose OWN `.Exactly(n)` / `.AtLeast(n)` assertion FAILED never
  reaches the stop: `SelectFaces` / `SelectEdges` already returned its
  `SelectionError`, and the caller gets it UNCHANGED — its `Expected` reflects
  the caller's own assertion (`"exactly 2"` for an `.Exactly(2)` that matched
  one), never overwritten.
- On ANY SUCCESSFUL resolution — an unasserted query that matched several, OR an
  explicit assertion that was SATISFIED — if the resolved count is not exactly
  one, the stop returns a NEW `SelectionError` wrapping `ErrCardinality` with
  `Expected` rendered `"exactly 1"` and `Actual` the resolved count, PRESERVING
  the resolution's `Kind`, `Query`, `Body` and `Residuals`. A satisfied
  `.Exactly(2)` on a body with two planar faces resolves to two faces, and the
  stop — which needs one — turns that into `Expected "exactly 1"` / `Actual 2`,
  exactly as an unasserted three-match would.
- An unasserted resolution that matched nothing is an `ErrNoMatch` from
  `SelectFaces` / `SelectEdges`; the stop rewrites it the same way, into
  `ErrCardinality` with `Expected "exactly 1"` and `Actual 0`.

So every stop or axis that does not land exactly one face or edge reads
`ErrCardinality` with `Expected == "exactly 1"` and the same stable query
rendering — whether the count was zero, three, or a satisfied `.Exactly(2)` —
while a selector whose OWN assertion FAILED reads that assertion's count back;
and either way the agent repairs the query the same way whether it drove
`SelectFaces` directly or reached it through a stop or an axis.

**A query renders stably**, and the same rendering is what `SelectionError.Query`
and a verification `Diagnostic.Message` (`docs/verification-design.md` §1.1)
reuse:

```go
func (q *EdgeQuery) String() string
func (q *FaceQuery) String() string
```

The rendering is a canonical, deterministic function of the query's recorded
content, built from the codec's own tagged vocabulary (§6.2): equal recorded
queries render identically, and a query and its decoded round-trip render
identically. It is an identity for diagnostics and equality, not a parseable
format — the `Recipe` JSON codec is the round-trip channel.

`q.String()` is `<kind>(<pred>, <pred>, …)<cardinality>`:

- **kind** is `edges` for an `EdgeQuery`, `faces` for a `FaceQuery` — the codec's
  own selector tokens. (`SelectionError.Kind`'s `SelectorKind.String()` is the
  singular `edge` / `face`, naming the entity; the query prefix is plural and the
  error's kind field singular, deliberately distinct.)
- **preds** are the clauses in query order, `, `-separated; no clause renders
  `edges()`.
- **cardinality** is empty for no assertion, `.exactly(<n>)` for `Exactly(n)`,
  `.at_least(<n>)` for `AtLeast(n)` — the codec's `exactly` / `at_least` keys.
  (`SelectionError.Expected` renders the same assertion in prose — `exactly <n>`,
  `at least <n>`, or `any` for none — a human line, not this suffix.)

Each predicate renders by its codec kind token and payload:

| Predicate | Rendering |
|---|---|
| `Convex()` / `Concave()` / `Circular()` | `convex` / `concave` / `circular` |
| `Planar()` / `Cylindrical()` | `planar` / `cylindrical` |
| `ParallelTo(v)` | `parallel_to(<vec>)` |
| `NormalTo(v)` / `Facing(v)` | `normal_to(<vec>)` / `facing(<vec>)` |
| `LongerThan(l)` | `longer_than(<value>)` |
| `CreatedBy(f)` / `FaceCreatedBy(f)` | `created_by(<ref>)` / `face_created_by(<ref>)` |

with these payload forms:

- **`<vec>`** is `<x>,<y>,<z>` — comma-separated, no spaces, each coordinate
  first NORMALIZED so a negative zero renders `0` (`-0.0`, including
  `math.Copysign(0, -1)`, is replaced with `+0.0` before formatting), then
  written as the shortest round-tripping float
  (`strconv.FormatFloat(c, 'g', -1, 64)`), so `parallel_to(0,0,1)`. The
  normalization is load-bearing for the "equal recorded queries render
  identically" contract: `-0.0 == 0.0`, so two value-equal vectors — say two
  `Facing` predicates — must render the same, but `FormatFloat` writes a
  negative zero as `"-0"`; normalizing first keeps them identical. Rendering is
  total, so an unresolved query still prints: a non-finite component reads
  `NaN` / `+Inf` / `-Inf` as that formatter writes it, and the zero vector
  reads `0,0,0`.
- **`<value>`** is the `units.Value`'s own canonical text form — magnitude plus
  registered unit symbol, `"10 mm"` (§6.2) — so `longer_than(10 mm)` states kind
  and unit explicitly and round-trips.
- **`<ref>`** is a `FeatureRef` as `<step>:<role>` — the `StepRef` in decimal and
  the role QUOTED with `strconv.Quote`, so a role that itself holds parentheses or
  commas stays unambiguous: `created_by(3:"capStart")`, `created_by(2:"side(0,1)")`.
- a zero-value, kind-less predicate — which the constructors never produce, but a
  half-decoded query might carry — renders `<invalid>` rather than panicking.

So `Faces(Planar(), Facing(z)).Exactly(1)` renders
`faces(planar, facing(0,0,1)).exactly(1)`, and
`Edges(Convex(), ParallelTo(z)).Exactly(4)` renders
`edges(convex, parallel_to(0,0,1)).exactly(4)`.

This is also why the decad code and the eventual Fusion code stay structurally
parallel: a real Fusion script must pick edges by geometric predicate too.

## 10. Verification — the product

The 3D counterpart of `sketch.Verify`. One non-mutating call; a rich report; a
`Status` at both the body and the document level, aggregated by a fixed severity
precedence; and one bit an agent gates on. Deliberately mirrors
`sketch.VerificationReport` / `WorldVerificationReport`.

```go
func (d *Document) Verify(ctx context.Context, opts ...VerifyOption) (*Report, error)

func (r *Report) Trustworthy() bool // the single bit to gate on
```

The report vocabulary — `Report`, `BodyReport`, `Status` and its severity
precedence, and the `VerifyOption` set including `WithTolerance` — and the
tolerance gate that judges every `Exactness` and `Bound` the report carries are
specified in `docs/verification-design.md`. Its §1.2 also owns `Verify`'s
public cost warning and caller-deadline guidance. The non-mutating proof and
bounded volume behind every `Interference` row are specified in
`docs/interference-design.md`. `Status` reserves zero as `Unverified`, so a
zero-value `Report` fails `Trustworthy`; every report and body report returned
by `Verify` is explicitly initialized to a decided status. What the core
contract pins down:

- **`Verify` returns an `error`, and the report carries no health state.** The
  error is for the call that could not be made — a `WithTolerance` value of the
  wrong `Kind` is `ErrUnitKind`, a negative one is `ErrNegativeMagnitude`, a
  non-finite one is `ErrNotFinite` (§12) — and a `Report` is returned only when
  the verification actually ran.
  §12 admits no alternative: a `Report.Err` field an agent could forget to read
  is exactly the deferred health state §4 rejects in Fusion, and a `bool` is no
  better.
- **Every bounded result the report carries is judged.** A `Measurement`, a
  `VecMeasurement` or a `Box` (§5.3) whose `Bound` is beyond the caller's
  tolerance makes the report `Suspect` — on a body or on a pair, nothing is
  exempt — and `Report.Trustworthy()` is true only when the whole report is
  `Sound`.
- **`Verify` returns structured diagnostics.** On a report returned by
  `Verify`, `Report.Diagnostics` is one
  branchable `Diagnostic` per reason the report is not `Sound` — a reading
  beyond tolerance, an undecided validity, an undecided or staged pair, an
  undecided survey (named per survey) or clearance, a proven wall or undercut
  violation, an interference. The slice is empty exactly when
  the report is `Sound`, so an agent reads the reasons instead of reconstructing
  them. Every existing field and `Trustworthy()` are unchanged; the slice is
  additive. `docs/verification-design.md` §1.1 owns its shape.

Fusion answers **none** of `Watertight` (with diagnostics), `Manifold`,
`SelfIntersecting`, `MinWallThickness` (B-rep), `Undercuts`, or `MinRadius`. That
gap is decad's mandate.


## 11. Export and translation

```go
func (b *Body) Tessellate(tol units.Value) (*Mesh, error) // an OUTPUT, not the representation
func (b *Body) TessellateContext(ctx context.Context, tol units.Value) (*Mesh, error)
func (b *Body) STL(w io.Writer, opts ...STLOption) error
func (b *Body) OBJ(w io.Writer, opts ...OBJOption) error
```

`docs/tessellation-design.md` is normative for the mesh these calls consume and
return: shared boundary samples, two-sided `Mesh.Bound`, source-face mapping,
area slack, the symmetric-difference proof a boolean requires, and explicit
per-payload staging. A payload with no complete boundary proof is never
exported. A mesh without the separate occupied-volume proof is never admitted
to a boolean by an unproved generic bound.

`TessellateContext` passes `ctx` through chording, loop-clearance scans, cap
triangulation, mesh audits, and faceted restatement. Cancellation returns
`ctx.Err()` unchanged. `Tessellate` is its compatibility wrapper with
`context.Background()`.

**Fusion codegen is out of scope for v1.** `Document.Recipe()` exposes the exact
record of intent as inspectable data; emitting a Fusion add-in from it is a
follow-up (and possibly the agent's job, not the library's). The recipe is designed
to make that mechanical.

## 12. Errors and concurrency

- `(T, error)`. NEVER a `bool` return, a `-1` sentinel, or a deferred health state.
- Sentinel/typed errors for the cases an agent must branch on: `ErrNoMatch`
  (a selector carrying no cardinality assertion matched nothing), `ErrCardinality`
  (a cardinality assertion failed), `ErrForeignBody` (an operation was handed bodies
  owned by different documents, or an extent or axis named a body owned by another
  document, §8.1), `ErrForeignProfile` (a feature was handed a profile built from
  a different sketch than the one given, or a boundary entity the source sketch
  does not own, §7),
  `ErrStaleProfile` (a feature was handed a profile built before the sketch's
  current state — `Profile.IsStale`, §7), `ErrRetiredBody` (an
  operation, or an extent, was handed a body the document has retired, §6),
  `ErrUnresolvedBody` (a `StepRef` was passed as a `BodyRef` to a feature call,
  where a live `*Body` is required, §6.2), `ErrNegativeMagnitude` (a magnitude was
  given as a negative value; magnitudes are non-negative and sense is enumerated,
  §8.1), `ErrUnrecordableProfile` (a feature was handed a profile whose boundary
  contains a `Partial` fragment `sketch` could not certify — `TExact == false`,
  from a sampled arrangement cut or whole-sketch withholding — or one whose
  certified range the seam's falsifier disproves.
  A `Partial` fragment `sketch` certifies exact records as its entity's own
  variant with the certified range, and a whole edge of every kind records;
  full semantics in `docs/sketch-seam-design.md`),
  `ErrNotSolid`, `ErrDegenerate`, `ErrBooleanFailed`, `ErrInvalidProfile` (the
  profile is invalid, contains a nil boundary entity, or no longer exactly
  matches a fresh current profile from its source sketch),
  `ErrUnitKind`, `ErrNotFinite` (a non-finite `units.Value` magnitude or
  `r3.Vec` component handed as a parameter, a non-finite value derived while
  validating a Revolve axis whose input is otherwise finite, or a non-finite
  measurement or bound derived by an analytic evaluator — `units` construction
  admits a non-finite value and only its operations reject one, so the call
  must; option semantics in `docs/verification-design.md`),
  `ErrUnsupported` (the recipe records the intent exactly, but the current
  evaluator cannot yet build it — evaluator staging is explicit and rejected
  at the call, never silently approximated or narrowed;
  `docs/evaluator-design.md` §2), `ErrInvalidRecipe` (stored IR violates its
  profile/operation/reference contract), `ErrUnsupportedRecipeVersion` (the
  envelope names a format version this package cannot interpret), and
  `ErrResourceLimit` (recipe encoding, decoding, validation, or evaluation crossed
  an explicit ceiling). `RecipeError`
  carries root/step + field path and matches both `ErrInvalidRecipe` and its
  specific cause; `EvaluationError` carries step + op and unwraps evaluator or
  context failures. Full precedence is in `docs/recipe-replay-design.md` §6.
  `BooleanError` carries the op, the operand `StepRef`s and a `Code`
  (empty / unsupported-contact / evaluator-failure), wrapping `ErrBooleanFailed`
  (empty, evaluator-failure) or `ErrUnsupported` (unsupported-contact); a valid
  tangent or coplanar contact this evaluator cannot classify is
  unsupported-contact, **not** `ErrDegenerate` (§8). `SelectionError` carries the
  selector kind, the query's stable rendering, the body, the expected/actual
  cardinality and the per-clause residual counts, wrapping `ErrNoMatch` or
  `ErrCardinality` (§9).
- **`ErrUnitKind` covers exactly the wrong-`Kind` values.** A `units.Value` whose
  `Kind` is not the one the parameter takes: an angle where a length is wanted, and
  a `WithTolerance` value that is not `Dimensionless`
  (`docs/verification-design.md`). It is never a coercion (§5.1).
- **`ErrNegativeMagnitude` covers exactly the magnitudes.** Those are `Distance.D`,
  `DistanceSide.D`, `Symmetric.D`, `AngleExtent.A`, `AngleSide.A`,
  `SymmetricAngle.A`, every fillet and chamfer radius or distance, every shell
  thickness, the `LongerThan(l)` edge-predicate length (§9), the `WithTolerance`
  relative tolerance, the `WithMinWallThickness` tool size and its
  `WithDraftAllowance` draft allowance (§10, `docs/verification-design.md`), and
  the `Tessellate` tolerance (§11). Two `units.Value` parameters are **signed
  displacements, not magnitudes**, and are outside it: `ToFace.Offset`, which
  displaces along the target face's normal and whose sign says which side of that
  face the sweep stops on (§8.1); and `ExtrudeOpts.Taper` (`WithTaper`, §8.1),
  whose sign says which way the wall leans. Neither carries a `Direction` to reverse,
  so a negative value there is a legal intent, not an error.
- **`ErrCardinality` takes precedence at zero matches.** A failed cardinality
  assertion is `ErrCardinality` **even when the selector matched nothing** — and
  that covers both the explicit assertions, `Exactly(n)` / `AtLeast(n)` (§9), and
  the *implicit* exactly-one of `ToFace` / `ToFaceAngular` (§8.1) and of `EdgeAxis`
  (§6.2). `ErrNoMatch` is
  reserved for the one remaining case: a selector that asserts no cardinality at
  all and resolves to zero entities.
- `Body` is immutable → safe to read from many goroutines.
- `Document` owns mutable state and is NOT safe for concurrent mutation. `Verify`
  is non-mutating and safe.

## 13. Non-goals for v1

Assemblies (`Component`/`Occurrence` instancing and the DAG that comes with it), a
feature tree / timeline / rollback, sweep, STEP, sheet metal, mesh import,
GUI or view state of any kind, and Fusion code generation.

The assemblies non-goal rests on a capability in hand, not on an instancing
graph: interference and clearance (§10) are computed between
**explicitly-placed bodies** — `Body.Placed(t r3.Transform)` (§8) — which needs
no `Component`/`Occurrence` machinery.
