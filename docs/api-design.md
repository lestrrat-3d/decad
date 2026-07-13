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
            analytic profiles + planes + features + operands + selectors + options + quantities
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
| 3 | **Selectors, never topology indices.** `Body.Faces()` / `Edges()` / `Vertices()` exist for **traversal and inspection only**; no feature, selector, or recipe ever names a face, an edge or a vertex by index or by a bare topology pointer. (A `Recipe`'s `StepRef` (§6.2) is not one: it indexes the recipe's own steps, which no evaluator reorders.) | An exact kernel produces a different (more correct) face/edge decomposition. If `Edges()[3]` is the API, index order becomes a de facto contract and vN breaks every model. |
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
`Box.Max`, a `VecMeasurement.Value` (§5.4), `Cylinder.Origin`, every point the API
returns or accepts — is a **length in the base unit, millimetres**. It is not a `units.Value`
and never becomes one: a vector of three typed quantities cannot be added, scaled,
dotted or crossed without unwrapping it at every step, which makes coordinate math
unusable and pushes callers back to hand-rolling. (An `r3.Vec` used as a *direction*
— `Cylinder.Axis`, `NormalTo(v)` — is dimensionless, and carries no unit at all.)

The carve-out is about **coordinates**, not about the vector type: a plane-local
2D coordinate — `Point2` (§6.2), the `(u, v)` of a recorded profile — is a length
in millimetres on exactly the same terms, and for exactly the same reason.

Vectors carry the unit **by convention**; scalars carry it **in the type**. §5.1
governs scalars, and this is the whole of the carve-out — a bare `float64` scalar
quantity is still forbidden anywhere in the API. A *curve parameter* is not a
scalar quantity: a spline's knots, weights and parameter range, and a conic's
fullness `Rho` — the apex weight of a rational quadratic in disguise (§6.2) — are
dimensionless indices into a parameterisation, not measurements of anything, and
§5.1 does not reach them.

### 5.3 The trim contract at the sketch seam

Every capability decad's API consumes exists in its dependencies today — there
is no open dependency gap. The one contract subtle enough to state in full is
`sketch.BoundaryEdge`'s account of a boundary fragment, because §6.2's mapping
table, §7's seam rules and `ErrUnrecordableProfile` (§12) are all built on it.

A `BoundaryEdge` carries `Entity`, `Partial`, `Reversed` and `Polyline` — and
the trim itself:

- **`TStart` / `TEnd`** are the fragment's parameter range on `Entity`, as
  normalized `t` in `[0, 1]` in the entity's **natural** direction: `TStart <
  TEnd` always, and the range **never wraps** — a fragment of a closed curve
  that straddles the seam arrives as two edges. `Reversed` composes with that
  natural-direction range: it, and never the order of the pair, says the
  boundary walks the range backwards. A whole edge spans the entity's full
  domain.
- **`TExact`** reports whether that range is the true parameters on `Entity`
  rather than a sampling-accurate approximation, and its meaning is precise and
  checkable: evaluating `Entity` at `TStart` / `TEnd` reproduces the fragment's
  `Polyline` endpoints to machine precision, at both bounds.

**Which cuts `sketch` certifies exact is a property of the pair.** A cut bound
is exact only when `sketch`'s closed-form kernel placed it, and that kernel runs
only when **both** of the crossing's source curves are a line, a circle or an
arc — and then only for a line-involved crossing or a tangency between them.
Every other contact yields a **sampled** cut parameter: every curve/curve
crossing (circle × circle, circle × arc, arc × arc), and every contact involving
an ellipse, elliptical arc, conic, spline, closed spline, fit spline or NURBS —
**including a plain line** crossing, or tangent to, one of those. A sampled cut
yields `TExact == false` on every fragment it bounds.

**No residual test on a fragment's endpoints could stand in for the flag, at any
tolerance.** A `Polyline` is a **sample of the curve**: its vertices lie *on*
the curve. A sampled cut is the crossing of a **chord** between two such
vertices, so as the crossing approaches a sample vertex the cut point approaches
the curve, and its residual against the curve goes to zero. A sampled
circle/circle cut has been measured with a normalised radial residual of
**7.07e-10** — indistinguishable from an exact cut by any threshold. Exactly-cut
endpoints and sampled ones are therefore not separated populations, and an
endpoint-residual test is **unsound as an admission gate**: it is not a mistuned
test, it is a quantity that does not answer the question asked of it.

**The test is one-sided, and that asymmetry is the whole of what it is good for:**

| observation | what it proves |
|---|---|
| the residual is **large** | the endpoint does **not** lie where the range says it does, so the cut cannot have been computed exactly. A fragment claiming exactness here is **wrong**. **Sound.** |
| the residual is **small** | **nothing.** A sampled cut can lie arbitrarily close to the curve. **Unsound as an accept.** |

**A large residual is proof of inexactness. A small residual is proof of nothing.** A
check on it may therefore only ever **reject** a fragment; it may never admit one.

**So admission of a `Partial` fragment is decided by `TExact`, and by nothing
else.** `sketch` computed the arrangement that produced the fragment, so it is
the only party that knows the range and the only party that knows how it was
obtained. The range it computed **is** the trim; decad neither second-guesses
the flag nor re-derives it — re-deriving a 2D answer is what §7 forbids.

- `TExact == true` → the fragment records: the trimmed variant of the entity's
  own kind, built from the entity's defining data and `TStart` / `TEnd` (the
  table in §6.2). A circle crossed by a rectangle records.
- `TExact == false` → `ErrUnrecordableProfile` (§12). An approximate parameter
  range is never recorded as an exact analytic fragment, never recorded as the
  whole entity — a different region than the caller drew — and never repaired.
- **A residual check is retained purely as a one-sided falsifier, never as an
  admission gate.** If `TExact == true` and evaluating the source entity at the
  reported range does not reproduce the fragment's endpoints — the flag's own
  stated meaning — the flag is disproven: decad returns `ErrUnrecordableProfile`
  and the discrepancy is reported upstream as a `sketch` bug. The check can only
  ever **reject**. It never admits a fragment on its own, because a small
  residual proves nothing — that is the asymmetry above, and re-reading it as an
  accept is the unsound gate it forbids.

**`TExact` is per-fragment, and it is per-fragment because exactness is a property of the
crossing that produced the fragment — of *both* its source curves — never of the entity
the fragment lies on.** A circle cut by a rectangle edge is exact: both sources are
line/circle/arc, and the crossing is line-involved. The *same circle* cut by another
circle is sampled: both sources are curves. One entity, two fragments, two verdicts — so a
per-entity flag, or any rule keyed on the entity kind, would be wrong on one of them.

**The rule is about the pair, and no shorthand on one source survives it.** "Line-involved"
is not one either: a line crossing an *ellipse*, a conic, a spline or a NURBS is sampled,
because the other source is not a line, a circle or an arc. decad reads the flag on the
fragment it is recording, and derives it from nothing.

**Whole (non-`Partial`) edges never consult `TExact`.** A whole edge records
from the entity's own defining data — there is no trim to recover, so the flag
answers a question decad is not asking. The distinction bites on exactly one
kind: a whole `*EllipticalArc` edge reads `TExact == false`, because the arc's
endpoints are pinned to sketch points that lie on the parametric ellipse only
within solver tolerance, so evaluating the entity at its domain ends misses the
polyline ends by that tolerance. That is a fact about the pinning, **not
topology distrust**, and it must not affect whole-edge recording: an implementer
who "helpfully" gates whole edges on `TExact` would reject every whole
elliptical-arc boundary that the entity's own data records exactly.

**What records reaches exactly as far as `sketch`'s exact kernel does, and no further.**
That is a circle or arc fragment cut against a line, and a fragment of a tangency
among lines, circles and arcs. A fragment cut by anything else — an ellipse, elliptical
arc, conic, spline, closed spline, fit spline or NURBS on either side of the crossing, and
every curve/curve crossing — carries `TExact == false` and is `ErrUnrecordableProfile`.
That is not decad declining to record it; it is `sketch` reporting that the parameter is
sampled, and decad recording no fragment on a range it was told is approximate. Widening
that set is an upstream question about the arrangement, not an API question here.

### 5.4 Exactness — the load-bearing type

```go
type Exactness int

const (
    Exact Exactness = iota // analytic; the number is the truth
    Approximate            // tessellation-derived; Bound holds the error
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
hand a millimetre tolerance a quantity that is not a length. §10.1's gate is stated
over both, and needs no exponent to be.

Every measurement returns one:

```go
vol, err := body.Volume()  // v1 after a boolean: {12.9997mm³, Approximate, 1e-3mm³}
                           // vN:                 {13.0000mm³, Exact,       0mm³}
```

`Measurement`, `VecMeasurement` and `Box` (§6) are the **three and only three**
bounded results the API returns. Every one of them carries a `Bound`, which is
what lets §10.1's tolerance gate be total.

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
func (d *Document) Verify(ctx context.Context, opts ...VerifyOption) (*Report, error)
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

// The variant types. The SurfaceKind discriminant a Surface reports is a
// separate, KindXxx-prefixed constant set (§6.2) — a surface variant and its
// kind never share a name.
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

**Surface parameters carry no `Exactness`, and this is not an exception to
invariant #2.** An analytic `Surface` variant is `Exact` by construction: a face
whose geometry is not exact is `Faceted`, and `Faceted` is the flag. So a
`Cylinder.Axis` or a `Sphere.Center` is an exact parameter of an exact surface,
while `Face.NormalAt(p)` — a quantity the evaluator *computes*, on a face that may
be `Faceted` — is a measurement and reports its `Exactness` like every other.

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
    Op        OpKind          // Extrude / Revolve / Union / Cut / Intersect / Fillet / Chamfer / Shell / Placed
    Inputs    []StepRef       // the bodies this step depends on. Cut is [target, tool].
    Profile   ProfileRecord   // Extrude / Revolve — decad's own analytic 2D record of the region
    Plane     PlaneRecord     // Extrude / Revolve — the sketch plane; lifts Profile into world space
    Extent    Extent          // Extrude
    Angular   AngularExtent   // Revolve
    Axis      Axis            // Revolve
    Placement TransformRecord // Placed — the rigid motion, recorded as vectors
    Selectors []Selector      // Fillet / Chamfer / Shell — the edge / face queries, unresolved
    Opts      StepOpts        // per-op options; nil when the op takes none
    Values    []units.Value   // radii, distances, thicknesses
}

type OpKind int
```

**The `Recipe` owns its geometry.** A `Step` holds no `*sketch.Profile` and no
`r3.Frame`. It records the region **structurally**, in decad's own types, at the
moment the feature is called:

- A `*sketch.Profile` is a pointer into a live, mutable `*sketch.Sketch` — a
  *handle*, and §2 says the recipe is a value. Its `Entities` and its
  `BoundaryEdge.Entity` are `sketch.Entity`, an interface with unexported methods,
  which no decoder can reconstruct. And its `BoundaryEdge.Polyline` is a **densified
  sample** — a tessellation, which §2 says a `Recipe` never names. (Its first and
  last points are the edge's start and end. They are the one thing decad reads from
  it, and only to check — they are what §5.3's falsifier tests the recorded range
  against — never to record; see below.)
- `r3.Frame`'s fields are unexported: it marshals to `{}`, so a `Step` that stored
  one would silently drop the plane — the single field without which the step is
  incomplete. `r3.Transform`'s fields are unexported on the same terms, so a
  placement is recorded as a `TransformRecord`, read out through
  `Transform.Basis()` and `Transform.Translation()`.

So decad **converts, it does not reference**:

```go
// PlaneRecord is the sketch plane, as three vectors: it survives encoding, which
// an r3.Frame does not. Orthonormal, right-handed; the plane normal is U × V, and
// that normal is the sense Direction.Along means for a linear extent (§8.1).
type PlaneRecord struct {
    Origin r3.Vec // millimetres (§5.2)
    U, V   r3.Vec // the in-plane axes: the (u, v) a Point2 below is expressed in
}

// TransformRecord is a rigid placement, as four vectors: it survives encoding,
// which an r3.Transform does not. EX, EY, EZ are the transformed world basis
// (r3.Transform.Basis()), T the translation. r3.FromBasis rebuilds the
// transform, snapping encoding drift straight and rejecting anything that is
// not an isometry.
type TransformRecord struct {
    EX, EY, EZ r3.Vec // the images of the world axes — dimensionless directions
    T          r3.Vec // the translation, millimetres (§5.2)
}

// Point2 is a plane-local coordinate, a length in millimetres — §5.2's carve-out
// in the plane's own (u, v).
type Point2 struct{ U, V float64 }

// ProfileRecord is the region a Step extrudes or revolves: one outer loop and its
// holes, structural and plane-local. Not a sample, not a pointer, not a sketch.
type ProfileRecord struct {
    Outer LoopRecord
    Holes []LoopRecord
}

// LoopRecord is a closed, directed boundary loop: each segment's end is the next
// segment's start, and the last closes onto the first. A single closed segment —
// a circle, an ellipse, a closed spline — is a loop on its own. Outer loops run
// counter-clockwise in (u, v), holes clockwise.
type LoopRecord struct {
    Segments []CurveSegment
}

// CurveSegment is one curve of a loop, recorded structurally — never as a
// sample. Sealed, like Surface (§6.1).
// A variant records exactly the defining data of the curve the edge IS — the
// fields of the source entity's own geom value, in plane-local Point2 — so an
// evaluator reconstitutes the curve through sketch/geom and never re-derives it
// (§7). The variant is chosen by the entity and by whether the edge is whole:
// a whole edge records its entity's own data, and a Partial fragment sketch
// certifies exact (TExact, §5.3) records the trimmed variant of the same kind,
// its range from TStart/TEnd (see the table below).
type CurveSegment interface{ curveSegment() }

// The five analytic kinds.
type LineSeg    struct { Start, End Point2 }
type ArcSeg     struct { Center Point2; Radius units.Value; StartAngle, EndAngle units.Value; CCW bool }
type CircleSeg  struct { Center Point2; Radius units.Value; CCW bool }

// EllipseSeg is a whole ellipse. Rx and Ry are the semi-axes along the ellipse's
// own local x and y, and they are UNORDERED — geom.Ellipse does not enforce
// Rx >= Ry; the axes are simply the local x and y, and Rotation is the angle of
// that local frame. Naming them Major/Minor would oblige an implementer to
// normalise the pair, which is re-deriving 2D geometry (§7).
type EllipseSeg struct { Center Point2; Rx, Ry, Rotation units.Value; CCW bool }

type EllipticalArcSeg struct {
    Center               Point2
    Rx, Ry, Rotation     units.Value // local-x / local-y semi-axes, unordered; frame angle
    StartAngle, EndAngle units.Value // the swept range in the ECCENTRIC angle — the t of (Rx·cos t, Ry·sin t)
    CCW                  bool
}

// The five free-form kinds. Degree, knots, weights, a conic's fullness Rho and
// every T range are curve parameters, not quantities (§5.2): Rho is the apex
// weight w = Rho/(1-Rho) of a rational quadratic in disguise, of exactly the
// same class as a NURBS weight.

// SplineSeg covers sketch's Spline and NURBS alike — a Spline is the degree-3,
// clamped-uniform, unweighted case, which is its definition, not a fit.
type SplineSeg struct {
    Degree       int
    Control      []Point2
    Knots        []float64
    Weights      []float64
    TStart, TEnd float64
}

// ClosedSplineSeg is sketch's periodic uniform cubic B-spline: a closed curve
// that bounds a region on its own, so it is a whole LoopRecord by itself.
type ClosedSplineSeg struct {
    Control      []Point2
    CCW          bool
    TStart, TEnd float64 // the full period for a whole edge
}

// FitSplineSeg records the INTENT sketch was given: the points the curve
// interpolates. sketch's definition — a natural cubic with chord-length
// parameterisation through exactly these points — is the curve; decad records
// the points and NEVER runs the interpolation solve itself (§7).
type FitSplineSeg struct {
    Fit          []Point2
    TStart, TEnd float64
}

// ConicSeg is a rational quadratic Bezier: endpoints, the apex where the end
// tangents meet, and the fullness Rho in (0, 1) — Rho < 0.5 an ellipse arc,
// 0.5 a parabola, > 0.5 a hyperbola arc.
type ConicSeg struct {
    Start, Apex, End Point2
    Rho              float64
    TStart, TEnd     float64
}
```

Nine variants, and they cover **all ten** of `sketch`'s `Entity` implementations —
line, circle, arc, ellipse, elliptical arc, conic, spline, closed spline, fit
spline, NURBS — because `SplineSeg` covers `Spline` and `NURBS` alike. That is
`sketch`'s entity vocabulary exactly and entirely, and `sketch` puts every one of
those kinds on a profile boundary: the four open free-form curves as boundary edges,
the closed spline as a closed curve bounding a region on its own. So **every entity
`sketch` can put on a boundary has a record** — whole, and trimmed: a `Partial`
fragment records through the same nine variants, as the trimmed variant of its
entity's kind with the certified range, so the vocabulary needs no
fragment-specific kinds. What has no record is a `Partial` fragment whose cut
`sketch` samples — `TExact == false` (§5.3) — and such a profile is rejected
rather than recorded lossily. A
new entity kind upstream needs a new `CurveSegment` variant before decad accepts a
profile that uses it; there is no fallback to a sample.

**A whole edge records its entity. A `Partial` fragment records exactly when
`sketch` certifies its cut — `TExact == true` — and it records from
`TStart` / `TEnd`.** The range is `sketch`'s answer — normalized `t` in the
entity's natural direction, `TStart < TEnd`, never wrapping (§5.3) — and the
trimmed variant is built from the entity's own defining data plus that range.
Admission never depends on the entity's kind: the flag admits the fragment, and
the kind only selects which variant records it. Whole edges never consult the
flag (§5.3).

| `BoundaryEdge.Entity` | whole edge | `Partial` fragment — records when `TExact`; else **`ErrUnrecordableProfile`** (§12) |
|---|---|---|
| `*Line` | `LineSeg` | `LineSeg` — the line, trimmed to `TStart`/`TEnd` |
| `*Circle` | `CircleSeg` | `ArcSeg` — the circle's center and radius; the angles `2π·TStart` / `2π·TEnd` from +x |
| `*Arc` | `ArcSeg` — the entity's own angles | `ArcSeg` — the entity's own angles, narrowed to the fragment's fraction of the sweep |
| `*Ellipse` | `EllipseSeg` | `EllipticalArcSeg` — the ellipse's axes and frame; the eccentric range `2π·TStart` / `2π·TEnd` |
| `*EllipticalArc` | `EllipticalArcSeg` — the entity's own range | `EllipticalArcSeg` — the entity's own range, narrowed to the fragment's fraction of the sweep |
| the free-form five | the matching free-form variant, `TStart`/`TEnd` spanning the full domain | the matching free-form variant, `TStart`/`TEnd` the fragment's range |

**Every row records a certified fragment, and every row rejects a sampled one.**
The per-row parameter mapping — from `sketch`'s normalized `t` to each variant's
own parameter — is `sketch`'s published per-type contract
(`geom.BoundaryEdge`), applied mechanically; nothing is solved for, and no
point is ever inverted to a parameter — the range is handed over, not derived
(§7). Under `sketch`'s exact kernel the fragments that carry `TExact == true`
are those of a line, a circle or an arc cut within that family (§5.3), so those
are the rows the true column reaches today: a circle cut by a rectangle edge
records, and a circle cut by another circle — or an ellipse cut by anything,
including the line fragments that crossing leaves on the rectangle — is
`ErrUnrecordableProfile`, because its range is sampled.

Conversion is mechanical, and it happens once, in the feature call. decad walks
`p.Outer` and each loop of `p.Holes` and reads each `BoundaryEdge`'s source `Entity`
for its defining parameters. `Partial` selects the column of the table above, and
on a fragment `TExact` decides admission (§5.3). `Reversed` is **baked into the
segment** as its own orientation — endpoints swapped, `CCW` flipped, `TStart` and
`TEnd` swapped, so `TStart > TEnd` says the segment runs against the curve's
natural sense; `sketch` hands the range over in natural direction, so the swap is
decad's record of the walk, composed with that range. A `LoopRecord` therefore
carries no residual flags and no back-reference.

**What decad reads of `BoundaryEdge.Polyline`, and nothing more: `Polyline[0]` and
`Polyline[len-1]`, on the `Partial` fragments it admits, as the observations
§5.3's falsifier tests the certified range against.** They never enter a `Step`:
every recorded value is the entity's own defining data and the certified range,
so no sampled content reaches a `Recipe` through them, which is why §2's "a
`Recipe` never names a tessellation" holds without qualification. On a rejected
fragment the `Polyline` is not read at all; on a whole edge the entity's own data
is the record and the `Polyline` is never read either. No interior point of a
`Polyline` is ever read, and no `Polyline` enters a `Step`.

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

Depending on a body is **not** consuming it: `Extrude` and `Revolve` retire
nothing, and the body a `ToFace` names stays live in `Document.Bodies()`. §6's
retire rule is unchanged, and lists exactly the operations it covers.

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
type FilletOpts  struct { /* ... */ }
type ChamferOpts struct { /* ... */ }
type ShellOpts   struct { /* ... */ }
```

The completeness rule applied to options: **every `ExtrudeOption`,
`RevolveOption`, `FilletOption`, `ChamferOption` and `ShellOption` MUST be
representable in the corresponding `…Opts` struct.** An option with nowhere to land
in the recipe does not ship — a tapered extrude that round-tripped as an untapered
one would be exactly the lossy record the completeness rule forbids.

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
exactness like everything else.

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
type Curve interface{ curve() }        // sealed, like Surface: Line / Circle / Arc / Ellipse / NURBSCurve / FacetedCurve
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

// Loop is a boundary loop of a face.
type Loop struct{ /* ... */ }
func (l *Loop) IsOuter() bool // the outer boundary; false for a hole

// Vertex is a topological point.
type Vertex struct{ /* ... */ }
func (v *Vertex) Position() VecMeasurement // millimetres (§5.2); a computed coordinate, so it is bounded (§6)

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

**`p` MUST be a profile of `s`, and `sketch` answers that too:
`Profile.Sketch()` is the sketch the profile was built from.** A
`*sketch.Profile` is freshly allocated by every `Profiles()` call, so pointer
membership in `s.Profiles()` could never establish provenance — it would reject
the caller's own profile, including the one in the example above. The
back-reference is `sketch`'s own answer, and decad consumes it. A profile whose
`Sketch()` is not `s` is `ErrForeignProfile` (§12): it is expressed in a
different plane's coordinates, so lifting it through `s`'s frame would place it
silently, confidently, in the wrong place.

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
self-intersecting or degenerate region is never silently swept. `Profile.Valid` is
the whole of decad's *validity* gate, and it is `sketch`'s answer, not one decad
recomputes. The one further rejection is not a validity judgement at all: a valid
profile whose boundary decad cannot record exactly — a profile containing a
`Partial` fragment whose cut `sketch` reports sampled, `TExact == false`, or one
whose certified range §5.3's falsifier disproves —
is `ErrUnrecordableProfile` (§12), because a `Step` that recorded the whole curve
where the caller drew a piece of it, or an approximate range as an exact trim,
would be the lossy record §6.2 forbids.

**The seam's read-outs are `sketch`'s answers, and its one check can only
falsify.** Beyond an edge's source `Entity`, decad reads `Partial`, `Reversed`,
`TStart` / `TEnd` and `TExact`, and the two `Polyline` ends of an admitted
fragment — the observations §5.3's falsifier tests the range against (§6.2), and
nothing else. Admission is the flag's alone: an endpoint's distance from its
source curve does not say whether the cut that produced it was exact, because
`sketch`'s polyline vertices are samples *on* the curve (§5.3), so the residual
check rejects a provably wrong flag and admits nothing. decad neither re-derives
the trim nor infers it. A
fragment it cannot record it **rejects** — it never repairs, projects or fits a point
`sketch` handed over, and it never solves for one.

Whether the *sketch* is fully constrained is a separate, sketch-level question: a
profile can close while the sketch still has degrees of freedom. It is not decad's
to answer — an agent that wants that guarantee gates on `sketch.Sketch.Verify`
before extruding. decad never re-derives it.

## 8. Features

v1 vocabulary, deliberately small: **Extrude, Revolve, Union/Cut/Intersect,
Fillet, Chamfer, Shell, Placed**. Sweep and Loft are deferred.

```go
func (d *Document) Extrude(s *sketch.Sketch, p *sketch.Profile, e Extent, opts ...ExtrudeOption) (*Body, error)
func (d *Document) Revolve(s *sketch.Sketch, p *sketch.Profile, axis Axis, a AngularExtent, opts ...RevolveOption) (*Body, error)
```

Both take the **sketch** as well as the profile, because a `sketch.Profile`'s
geometry is plane-local and the plane is the sketch's (§7). `p` MUST be a
profile of `s` (`Profile.Sketch()`), and a current one: another sketch's profile
is `ErrForeignProfile`, a stale one `ErrStaleProfile` (§12). decad
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
```

No `*Document` appears in those signatures, and none is needed: a `*Body` carries
its owning document (§6), so each call retires its operands and registers its
result inside the document that owns them. The operands are themselves unchanged
— invariant #4 — and `Document.Bodies()` and `Document.Verify()` stay truthful.
Operands owned by different documents are `ErrForeignBody`.

Modify operations return a new body, retiring the receiver, on the same terms:

```go
func (b *Body) Fillet(sel EdgeSelector, r units.Value, opts ...FilletOption) (*Body, error)
func (b *Body) Chamfer(sel EdgeSelector, d units.Value, opts ...ChamferOption) (*Body, error)
func (b *Body) Shell(sel FaceSelector, thickness units.Value, opts ...ShellOption) (*Body, error)
```

Placement is a body operation on the same terms — it retires the receiver and
registers the placed body:

```go
func (b *Body) Placed(t r3.Transform) (*Body, error)
```

This is the whole of the "explicit transforms" story: a body is positioned by an
argument the caller states — an `r3.Transform`, a rigid motion (§5.2) — never by
an ambient assembly context (§4). The zero `Transform{}` is invalid
(`Transform.IsValid`) and is `ErrDegenerate` (§12). The step records the motion
as a `TransformRecord` (§6.2).

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
func (d *Document) Verify(ctx context.Context, opts ...VerifyOption) (*Report, error)

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
    Centroid          VecMeasurement // a computed coordinate, so it is bounded (§5.4)
    Bounds            Box
    Exactness         Exactness      // the weakest link across this body

    // Opt-in, expensive:
    MinWallThickness  *Measurement   // WithMinWallThickness(tool)
    Undercuts         []*Face        // WithPullDirection(v)
    MinRadius         *Measurement   // WithMinRadius() — can the endmill reach?
}
```

**`Verify` returns an `error`, and the report carries no health state.** The error
is for the call that could not be made — a `WithTolerance` value of the wrong
`Kind` is `ErrUnitKind`, a negative one is `ErrNegativeMagnitude` (§12) — and a
`Report` is returned only when the verification actually ran. §12 admits no
alternative: a `Report.Err` field an agent could forget to read is exactly the
deferred health state §4 rejects in Fusion, and a `bool` is no better.

Fusion answers **none** of `Watertight` (with diagnostics), `Manifold`,
`SelfIntersecting`, `MinWallThickness` (B-rep), `Undercuts`, or `MinRadius`. That
gap is decad's mandate.

### 10.1 Tolerance — what "beyond the caller's tolerance" means

`Suspect` and `Trustworthy()` turn on an approximation being *coarser than the
caller will accept*, so the caller must be able to say what they accept:

```go
func WithTolerance(rel units.Value) VerifyOption // Dimensionless; default units.Scalar(1e-3)
func WithMinWallThickness(tool units.Value) VerifyOption
func WithPullDirection(v r3.Vec) VerifyOption
func WithMinRadius() VerifyOption
```

**The tolerance is relative, and it is one number for every kind.** `rel` is the
largest error the caller will accept **as a fraction of the quantity being
measured**:

> A bounded result is **within tolerance** when `Bound <= rel × Ref`, where `Ref` is
> the result's **reference magnitude**.

One comparison, one number, no exponentiation. It is scale-invariant — a 1mm part
and a 1m part are judged on the same footing — which mirrors how `sketch` makes its
conditioning gate scale-invariant. `rel` is `Dimensionless` (`units.Scalar`); any
other `Kind` is `ErrUnitKind` (§12), never a coercion, and a negative `rel` is
`ErrNegativeMagnitude`. Both are returned from `Verify`, which is why it returns an
`error` (§10).

**The default is `units.Scalar(1e-3)` — three significant figures — and it is set
by what a tessellation-backed evaluator can actually prove.** A v1 boolean bounds a
volume by roughly `surface area × chord error`: a Ø20×10mm cylinder tessellated to a
1e-3 mm chord has an area of 1257 mm², so a bound of order 1.3 mm³ against a volume
of 3142 mm³ — a relative error of 4e-4. A default of `1e-6` would put every such
body — every *correct* such body — in `Suspect`, and a gate no honest evaluator can
pass has no content. `1e-3` passes it with room to spare, and is still an order of
magnitude tighter than the ±1% Fusion ships by default (§1) — and unlike Fusion's
figure it is a **proven bound**, not a nominal one. A caller who wants six figures
asks for them, and gets the honest `Suspect` an approximate evaluator owes them.

An **absolute** threshold cannot do this job. A `Bound` carries the `Kind` of the
quantity it bounds (§5.4), so an absolute length `t` has nothing to say about a
volume bound: raising it — `t²` for an area, `t³` for a volume — makes the gate
meaningless, because a linear error `t` propagates to a volume error of order
`area × t`, not `t³`. A Ø20×10mm cylinder tessellated to a 1mm chord carries a
volume bound of order 10³ mm³ while `t³` would admit 1 mm³, so *every* body touched
by a boolean would read `Suspect` at *every* tolerance, and the gate would have no
content at all. The ratio has content: it is exactly "how many significant figures
of this answer are real".

`Ref` is fixed per shape, and the report carries bounded results in exactly three
shapes (§5.4) — a `Measurement`, a `VecMeasurement`, a `Box` — so the table is
total:

| Bounded result | `Ref` |
|---|---|
| `Measurement` — a volume, an area, a length, a gap | `max(abs(Value), Quantum)` |
| `VecMeasurement`, a **direction** (`Bound` is `Dimensionless`, §5.4) | `1` — the magnitude of a unit vector |
| `VecMeasurement`, a **position**, and `Box` (`Bound` is a Length) | `Diag` |

**`Diag` is the bounding-box diagonal of the thing the result belongs to**, and
that is decided by ownership, not by convenience:

- a result on a `BodyReport` — `Volume`, `Area`, `Centroid`, `Bounds`,
  `MinWallThickness`, `MinRadius`, and every vertex position and face normal of that
  body — belongs to **one body**, and `Diag` is **that body's own** bounding-box
  diagonal, i.e. the diagonal of that `BodyReport.Bounds`. A body's answers are
  judged against the size of the body they are answers about;
- an `Interference.Volume` and a `Clearance.Gap` belong to a **pair**, so no single
  body's size is theirs to be judged against. For those, and only those, `Diag` is
  the diagonal of the **document's** bounding box — the box enclosing every live
  body. It is the one reference both members of the pair share.

A diagonal is a difference of coordinates, so `Diag` is translation-invariant in
either form — which is exactly what a position's reference must be.

`Scale` is `Diag` **raised to the dimension of the quantity's `Kind`**: `Diag`
for a length, `Diag²` for an area, `Diag³` for a volume, `1` for a dimensionless
quantity. And `Quantum` is `ε × Scale` with `ε = 1e-9` fixed — the noise level of the
thing being measured, the magnitude below which a quantity of that `Kind` is not
distinguishable from zero at that size. `ε` is a constant of the gate, not the
caller's knob: it is **not** `rel`, and `rel` never multiplies `Scale`.

Three things follow, and all three are rules:

- **The gate is genuinely relative.** For any quantity above the noise level — every
  volume, area, length and gap a real model measures — `Ref` **is** `abs(Value)`, and
  the test is exactly `Bound <= rel × abs(Value)`: how many significant figures of
  this answer are real. `Quantum` is a floor, not a scale factor, and it never
  loosens the test for a quantity that has a magnitude of its own. A `Scale`-sized
  reference would be an absolute threshold wearing a ratio's clothes — it would judge
  a volume against the *extent* of the body rather than against the volume itself. A
  100×100×0.001mm sliver has a `Diag` of 141.4mm and so a `Scale` of 2.83e6 mm³, but
  a volume of 10 mm³: against `Scale` a ±5 mm³ bound — a 50% error — would pass at
  every tolerance an evaluator can meet. Against its own 10 mm³ it is `Suspect` at
  the default, and it must be, and it is.
- **At and below the noise level the gate becomes absolute.** A zero clearance, or
  the volume of a degenerate body, has `|Value|` at or under `Quantum`, and a ratio
  to it is undefined or explosive. `Ref` collapses to `Quantum` there — and that is
  the whole of the near-zero rule, because it is the same formula: `Bound <= rel ×
  Ref` reads `Bound <= rel × Quantum`, an **absolute** threshold of `1e-12 × Scale`
  at the default `rel`. It is a real number and the reader can check it: a zero wall
  thickness on a body whose `Diag` is 1 mm has `Scale = 1 mm` and
  `Quantum = 1e-9 mm`, so the gate is `1e-12 mm`; a zero clearance in a document
  100 mm across has `Quantum = 1e-7 mm` and a gate of `1e-10 mm`. So a near-zero
  answer passes only with a bound that is, in practice, vanishingly tight. A
  tessellation does not produce one — an `Approximate` near-zero answer will
  essentially always read `Suspect` — while an `Exact` answer has a zero `Bound` and
  passes at the floor as it does everywhere else. That is the intent: a zero
  clearance reported as `0 ± 5mm` is untrustworthy and must be `Suspect`; a zero
  clearance known to `1e-12 mm` is not. A point-like body carries this to its limit:
  `Diag → 0`, so `Scale → 0`, so `Quantum → 0`, and the gate tightens to zero —
  only a zero `Bound`, i.e. an `Exact` answer, passes.
- **A coordinate is judged against `Diag` alone**, never against its own magnitude:
  the magnitude of a position is origin-dependent, and translating the model must
  never change the verdict. Because that `Diag` is the **owning body's**, the verdict
  is also scale-free: a centroid is judged against the size of the body whose centroid
  it is, so a 100mm bracket sharing a document with a 1.5m enclosure is judged against
  its own 173mm, and does not inherit a slack tolerance from the biggest thing in the
  document.

Worked, at the default `rel = 1e-3`, on two bodies:

| Body | `Diag` | `Volume` | `Bound` | `Ref` | `rel × Ref` | Verdict |
|---|---|---|---|---|---|---|
| a small boolean off-cut, ~2mm across | 3.5 mm | 8 mm³ | 5 mm³ (±62%) | 8 mm³ | 8e-3 mm³ | **`Suspect`** — 5 ≫ 8e-3 |
| a Ø20×10mm cylinder, 1e-3 mm chord | 30 mm | 3142 mm³ | 1.3 mm³ (±0.04%) | 3142 mm³ | 3.14 mm³ | `Sound` — 1.3 ≤ 3.14 |

`Quantum` is nowhere near either (4.3e-8 mm³ and 2.7e-5 mm³), so `Ref` is the volume
itself in both rows. The two are separated by three orders of magnitude in *relative*
error, which is the only thing that distinguishes them, and it is exactly what the
gate reads.

The gate has nothing to miss, because **every one of the three shapes carries a
`Bound`**:

- a `Measurement`, a `VecMeasurement` or a `Box` on a `BodyReport` that is beyond
  tolerance — `Volume`, `Area`, `Centroid`, `Bounds`, `MinWallThickness`,
  `MinRadius` — makes that `BodyReport` `Suspect`;
- an `Interference.Volume` or a `Clearance.Gap` beyond tolerance makes the `Report`
  `Suspect` directly — those two are properties of a *pair*, so there is no
  `BodyReport` for them to travel through, and a gap known to only one significant
  figure is not an answer the caller said they would accept.

Either path makes `Trustworthy()` false. `Exact` answers have a zero `Bound` and
can never trip it, at any tolerance. **Nothing in the report is exempt**, and the
`VecMeasurement` of §5.4 is what makes that true: a centroid or a vertex position
carries a bound like everything else, so a boolean that puts the centroid off by
more than `rel` of the body's own size cannot hide inside a `Sound` body — which is
the confidently-wrong failure §1 exists to prevent.

**That guarantee is relative, and it is stated relatively because that is what is
true.** The gate on a centroid is `Bound <= rel × Diag` with `Diag` the owning
body's bounding-box diagonal, so at the default `rel = 1e-3` a centroid bound
passes only when it is under one part in a thousand of that body's diagonal:

| Body | `Diag` | `rel × Diag` | a 1 mm centroid `Bound` reads |
|---|---|---|---|
| 100×100×100mm block | 173.2 mm | 0.173 mm | **`Suspect`** |
| 1200×800×600mm enclosure | 1562 mm | 1.562 mm | `Sound` |
| 1m cube | 1732 mm | 1.732 mm | `Sound` |

A millimetre is a coarse answer on a 100mm block and the gate says so. On a body a
metre across it is under one part in a thousand — three significant figures, which is
what the default tolerance *means* and what the caller asked for. A caller who needs
an absolute millimetre on a metre-scale body buys it with figures: `rel = 5e-4` puts
the 1m cube's gate at 0.87mm and the enclosure's at 0.78mm. What is ruled out
absolutely — at every tolerance, on every body — is the failure §1 names: an error
that is large *relative to the part it is an error about* sitting inside a `Sound`
report. Judging the centroid against the owning body rather than the document is what
makes that reading hold at any scale.

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
  self-intersection), `Suspect` (sound, but one of its answers is `Approximate` with
  a `Bound` beyond the tolerance of §10.1), or `Unsound` (not a valid solid). A body is never
  `Interfering` — interference is a property of a *pair*, not of a body.
- **`Report.Status`** is the document-level aggregate — over the bodies *and* over
  the pairwise results, which belong to no body.

Aggregation is by **severity precedence — worst wins**:

**`Unsound` > `Interfering` > `Suspect` > `Sound`**

Concretely, `Report.Status` is:

| Condition | `Report.Status` |
|---|---|
| any `BodyReport.Status == Unsound` | `Unsound` |
| else, `len(Interferences) > 0` | `Interfering` |
| else, any `BodyReport.Status == Suspect` | `Suspect` |
| else, any `Interference.Volume` or `Clearance.Gap` beyond tolerance (§10.1) | `Suspect` |
| else | `Sound` |

The last rung is what keeps the tolerance gate **total**: a bounded result that
hangs off the `Report` rather than off a `BodyReport` is gated exactly as every
other is, so a `Clearance.Gap` measured far coarser than the caller's tolerance can
never sit inside a `Sound` report. (Interference is caught by the rung above it as
well, and `Interfering` is the worse verdict; the rule is stated over both so that
nothing in the report is exempt.) Together with the `Suspect` rung above it, the
gate covers **every `Measurement`, every `VecMeasurement` and every `Box` the report
carries** — and per §5.4 those are all of them.

`Report.Trustworthy()` is true **only** at `Report.Status == Sound`. An unsound
body, an unresolved interference, or an approximation coarser than the caller's
tolerance — on a body or on a pair — each make it false, even when the geometry
"looks" fine.

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
  owned by different documents, or an extent or axis named a body owned by another
  document, §8.1), `ErrForeignProfile` (a feature was handed a profile built from
  a different sketch than the one given — `Profile.Sketch()`, §7),
  `ErrStaleProfile` (a feature was handed a profile built before the sketch's
  current state — `Profile.IsStale`, §7), `ErrRetiredBody` (an
  operation, or an extent, was handed a body the document has retired, §6),
  `ErrUnresolvedBody` (a `StepRef` was passed as a `BodyRef` to a feature call,
  where a live `*Body` is required, §6.2), `ErrNegativeMagnitude` (a magnitude was
  given as a negative value; magnitudes are non-negative and sense is enumerated,
  §8.1), `ErrUnrecordableProfile` (a feature was handed a profile whose boundary
  contains a `Partial` fragment `sketch` could not certify — `TExact == false`,
  its cut sampled — or one whose certified range fails §5.3's falsifier. A
  `Partial` fragment `sketch` certifies exact records as the trimmed variant of
  its entity's kind, and a whole edge of every kind records; §5.3/§6.2),
  `ErrNotSolid`, `ErrDegenerate`, `ErrBooleanFailed`, `ErrInvalidProfile`,
  `ErrUnitKind`.
- **`ErrUnitKind` covers exactly the wrong-`Kind` values.** A `units.Value` whose
  `Kind` is not the one the parameter takes: an angle where a length is wanted, and
  a `WithTolerance` value that is not `Dimensionless` (§10.1). It is never a
  coercion (§5.1).
- **`ErrNegativeMagnitude` covers exactly the magnitudes.** Those are `Distance.D`,
  `DistanceSide.D`, `Symmetric.D`, `AngleExtent.A`, `AngleSide.A`,
  `SymmetricAngle.A`, every fillet and chamfer radius or distance, every shell
  thickness, the `LongerThan(l)` edge-predicate length (§9), the `WithTolerance`
  relative tolerance (§10.1), the `WithMinWallThickness` tool size (§10.1), and the
  `Tessellate` tolerance (§11). Two `units.Value` parameters are **signed
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
feature tree / timeline / rollback, sweep and loft, STEP, sheet metal, mesh import,
GUI or view state of any kind, and Fusion code generation.

The assemblies non-goal rests on a capability in hand, not on an instancing
graph: interference and clearance (§10) are computed between
**explicitly-placed bodies** — `Body.Placed(t r3.Transform)` (§8) — which needs
no `Component`/`Occurrence` machinery.
