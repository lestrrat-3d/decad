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
`github.com/lestrrat-3d/sketch/units`. We do NOT invent a parallel unit system.

```go
body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
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

### 5.3 Dependency-side decisions (blockers)

Three capabilities decad's API depends on do not exist in its dependencies yet.
All three belong upstream by charter, and all three are resolved upstream — decad
does not hand-roll around any of them.

| Gap | Proposal | Blocks |
|---|---|---|
| **`r3` has no rigid transform type.** `Frame` covers plane-local↔world only; placing a body at an arbitrary pose needs rotation + translation. | Add `r3.Transform` upstream. It *acts on* ℝ³, which is r3's charter. | Body placement — `Body.Placed` (§8), and with it the explicit-transform answer to assemblies (§13). |
| **`sketch/units` has no Area / Volume / Mass / Density kinds.** `units.Kind` is Dimensionless / Length / Angle only, so a `units.Value` cannot hold `12.9997 mm³`. | Add those kinds upstream to `sketch/units` — it is a first-party module — so `Measurement.Value` stays a single `units.Value` and the no-parallel-unit-system rule holds. | Every volume / area / mass measurement — i.e. most of `Verify` — and, for the text form below, encoding a `Recipe` (§6.2). |
| **`sketch.BoundaryEdge` does not carry a fragment's parameter range.** `Partial` is a bare `bool`: it says *that* the edge is a sub-range of its `Entity`, never *which* sub-range. The entity gives the parameters of the whole curve; the fragment's extent survives only as its endpoints — and `BoundaryEdge` does not say whether `sketch` cut it analytically or at a sampled crossing, so it does not say whether those endpoints are on the curve. | Add `TStart` / `TEnd` (the fragment's parameter range on `Entity`) to `sketch.BoundaryEdge`. sketch already computes the split that produced the fragment, so it is the only party that knows the range exactly. | Recording **any partial fragment whose endpoints `sketch` did not compute exactly** — every fragment of a curve/curve crossing, and every fragment of an ellipse, conic, spline, closed spline, fit spline or NURBS (§6.2). With the range, **every** partial fragment records exactly and the endpoint test of §6.2 disappears. |

Until the units kinds land, `Measurement` **cannot be implemented as specified**,
and decad will **not** work around it by inventing a parallel unit system.

The units gap is **wider than the measured value alone**. `Measurement.Bound` and
`Box.Bound` are `units.Value` too, carrying the same `Kind` as the quantity they
bound — the error bound on a volume is itself a volume, so it needs the Volume kind
as much as the volume does. So does `Interference.Volume` (§6.2). The blocker
covers the bound, not just the number.

The same row also covers **a text form for `units.Value`**: its fields are
unexported and it marshals to `{}`, so a `Recipe` — which carries every quantity
it records as a `units.Value` — cannot round-trip until `units.Value` gains
`MarshalText` / `UnmarshalText` upstream. It is the same first-party module and
the same charter; decad does not hand-roll a shadow encoding of a foreign type.
This is what §6.2's serializability rule is waiting on, and nothing else is.

The `BoundaryEdge` gap decides which partial fragments decad can record, and the
line does **not** fall where the entity kinds do. A fragment's two endpoints are
`Polyline[0]` and `Polyline[len-1]` (§6.2), and they are the only record of the
trim. Whether they are the *exact* ends of the fragment is a property of **the
crossing that produced it**, not of the entity it is a fragment of. `sketch` says
so itself: its exact closed-form contact is authoritative "for line-involved
crossings and all tangencies", while "a curve split at a *sampled* crossing
(ellipse/spline/conic vs line, or curve/curve) has an approximate cut parameter",
and "curve/curve transverse crossings (both circle/arc) are deferred to the sampled
path".

| A fragment cut by | Its endpoints |
|---|---|
| a **line-involved** crossing — line × circle, line × arc: a circle cut by a rectangle edge — or **any tangency** | lie on the source curve to machine precision |
| a **curve/curve** crossing — circle × circle, circle × arc, arc × arc | lie on a **chord**, off the true curve by O(chord² / R) |
| any crossing involving an **ellipse, conic, spline, closed spline, fit spline or NURBS**, a line among them | lie on a **chord** |

A circle cut by a circle and an ellipse cut by a line are ordinary sketches, and
their fragments' endpoints are **not on the curve**. `BoundaryEdge` carries
`Entity`, `Partial`, `Reversed` and `Polyline` and nothing else, so decad cannot
ask which case it is in — but it can **test the evidence it was handed**, and it
does (§6.2): a partial fragment records **iff both endpoints lie on the source
entity**, to a normalised radial residual of `1e-9` — a scale-invariant test, stated
in full in §6.2. Passing, the endpoints invert to parameters in closed
form and the fragment records as the trimmed variant — a `Partial` `*Circle` as an
`ArcSeg`, a `Partial` `*Ellipse` as an `EllipticalArcSeg` — because a fragment of a
closed curve is an open one and it is the open variant that has the fields for a
trim. Failing, it is `ErrUnrecordableProfile` (§12) — **rejected, never recorded as
the whole entity**, which would be a different region than the caller drew, and
never recorded from a point that is not on the curve, which would be a different
curve.

A partial fragment of a free-form curve fails a second way as well, and would fail
even with endpoints on the curve: inverting a point to a spline, NURBS, fit-spline
or conic parameter is a **projection** — a 2D root-find, which is a 2D answer, which
decad never re-derives (§7). Those fragments are `ErrUnrecordableProfile` outright.

`TStart` / `TEnd` on `BoundaryEdge` retires all of it: the range `sketch` already
computed is the trim, every partial fragment records exactly — the circle cut by a
circle, every ellipse fragment, every free-form fragment — and §6.2's endpoint test
is deleted rather than loosened. Whole (non-`Partial`) edges of every kind are
unaffected and record today: there is no trim to recover.

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

`Dimensionless` exists in `sketch/units` today, so the direction bound is **not**
blocked by §5.3.

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
    Op        OpKind          // Extrude / Revolve / Union / Cut / Intersect / Fillet / Chamfer / Shell
    Inputs    []StepRef       // the bodies this step depends on. Cut is [target, tool].
    Profile   ProfileRecord   // Extrude / Revolve — decad's own analytic 2D record of the region
    Plane     PlaneRecord     // Extrude / Revolve — the sketch plane; lifts Profile into world space
    Extent    Extent          // Extrude
    Angular   AngularExtent   // Revolve
    Axis      Axis            // Revolve
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
  last points are the edge's start and end, and they are the one thing decad reads
  from it — under the endpoint test below, because `sketch` does not always cut a
  curve exactly.)
- `r3.Frame`'s fields are unexported: it marshals to `{}`, so a `Step` that stored
  one would silently drop the plane — the single field without which the step is
  incomplete.

So decad **converts, it does not reference**:

```go
// PlaneRecord is the sketch plane, as three vectors: it survives encoding, which
// an r3.Frame does not. Orthonormal, right-handed; the plane normal is U × V, and
// that normal is the sense Direction.Along means for a linear extent (§8.1).
type PlaneRecord struct {
    Origin r3.Vec // millimetres (§5.2)
    U, V   r3.Vec // the in-plane axes: the (u, v) a Point2 below is expressed in
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
// (§7). The variant is therefore chosen by the entity AND by whether the edge
// is the whole entity or a fragment of it: a fragment of a closed curve is an
// open one, and it is the open variant that has somewhere to put the trim.
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
the closed spline as a closed curve bounding a region on its own. So **every profile
`sketch` can produce has a record**, which is what the completeness rule demands. A
new entity kind upstream needs a new `CurveSegment` variant before decad accepts a
profile that uses it; there is no fallback to a sample.

**Which variant an edge records is a function of its entity *and* its `Partial`
flag**, because a fragment of a closed curve is not that closed curve. A circle
crossing a rectangle is the most ordinary split a 2D sketch produces, and it yields
a `BoundaryEdge{Entity: *Circle, Partial: true}`: an arc. `sketch` says so in its own
vocabulary — `geom.SplitCircleAt(c, p, q)` returns two `*Arc` — and decad records what
`sketch` says. A `CircleSeg` has no range to trim and never needs one; the trim lives
in the open variant, which has the fields for it.

| `BoundaryEdge.Entity` | whole edge | `Partial` fragment |
|---|---|---|
| `*Line` | `LineSeg` | `LineSeg` — the fragment's own two endpoints. **Always records**: a chord of a line *is* the line, so there is nothing for the endpoint test to catch |
| `*Circle` | `CircleSeg` | **`ArcSeg`** — same centre and radius; `StartAngle` / `EndAngle` from the endpoints — **if the endpoint test passes**; else `ErrUnrecordableProfile` (§12) |
| `*Arc` | `ArcSeg` — the entity's own angles | `ArcSeg` — trimmed angles — **if the endpoint test passes**; else `ErrUnrecordableProfile` |
| `*Ellipse` | `EllipseSeg` | **`EllipticalArcSeg`** — same centre, `Rx`, `Ry`, `Rotation`; eccentric angles from the endpoints — **if the endpoint test passes**; else `ErrUnrecordableProfile` |
| `*EllipticalArc` | `EllipticalArcSeg` — the entity's own range | `EllipticalArcSeg` — trimmed range — **if the endpoint test passes**; else `ErrUnrecordableProfile` |
| the free-form five | the matching free-form variant, `TStart`/`TEnd` spanning the full domain | `ErrUnrecordableProfile` — the trim needs a projection, which is a 2D answer decad never re-derives (§5.3). No test; there is no closed form to pass into |

**Only the `*Line` row promises a `Partial` fragment records unconditionally.** The
four analytic *curve* rows record **when the endpoint test passes**, and it passes
exactly when `sketch` cut the curve analytically. A `*Circle` cut by a rectangle
edge passes; the same circle cut by another circle does not, and neither does an
ellipse cut by anything (§5.3). Exactness is a property of **the crossing**, never
of the entity kind. A fragment that fails is `ErrUnrecordableProfile`, on the same
terms as a partial free-form curve.

Conversion is mechanical, and it happens once, in the feature call. decad walks
`p.Outer` and each loop of `p.Holes`, reads each `BoundaryEdge`'s source `Entity`
for its defining parameters, and **bakes the edge's flags into the segment**:
`Reversed` becomes the segment's own orientation — endpoints swapped, `CCW`
flipped, `TStart` and `TEnd` swapped, so `TStart > TEnd` says the segment runs
against the curve's natural sense — and `Partial` selects the trimmed variant per
the table above and becomes its trimmed range. A `LoopRecord` therefore carries no
residual flags and no back-reference.

**What decad reads of `BoundaryEdge.Polyline`, and nothing more: `Polyline[0]` and
`Polyline[len-1]`.** `Partial` is a bare bool — it says *that* the edge is a
fragment, never which fragment — and the source `Entity` holds the parameters of
the **whole** curve, so the trim exists nowhere else. Those two points are the
fragment's start and end, and they are the whole of the carve-out: no interior point
of `Polyline` is ever read, by decad or by anything a `Recipe` carries, and no
`Polyline` enters a `Step`.

**They are the fragment's start and end. They are not always *on the curve*.**
`sketch` cuts exactly at a line-involved crossing and at every tangency, and falls
back to its sampled path for curve/curve crossings and for every crossing involving
an ellipse, conic, spline or NURBS (§5.3). A sampled cut lands on a **chord** of the
polyline, not on the curve. `BoundaryEdge` does not record which path produced it,
so decad does not ask — it **tests**.

**The endpoint test.** A `Partial` fragment of an analytic *curve* — a circle, an
arc, an ellipse, an elliptical arc — is recordable **iff both of its endpoints lie
on the source entity**. decad evaluates that entity's own implicit residual at
`Polyline[0]` and `Polyline[len-1]`:

> Take the point into the entity's own frame — centre `C`, and for an ellipse its
> local axes — which is a subtraction and an orthonormal basis change, never a solve.
> The **normalised radial residual** is
>
> `ρ(p) = | sqrt((u/Rx)² + (v/Ry)²) − 1 |`
>
> with `Rx = Ry = R` on a circle or an arc, where it degenerates to
> `| ‖p − C‖ / R − 1 |`. It is zero exactly on the curve, it is dimensionless, and it
> is invariant under scaling the entity and the point together — so a threshold on it
> is **relative to the entity's own size**, at every scale, with no absolute length
> anywhere in it.
>
> **The fragment records iff `ρ ≤ 1e-9` at both endpoints.** Otherwise the profile is
> `ErrUnrecordableProfile` (§12).

**`1e-9` is not a delicate number, because the two populations are ten orders of
magnitude apart.** An analytic cut puts the endpoint on the curve to round-off: `ρ`
of order `1e-15`. A sampled cut puts it on a chord, off the curve by
`O(chord² / R)`: a `*sketch.Ellipse` cut by a rectangle gives `ρ = 7.0e-5`. The
threshold sits in the empty middle — six orders above the round-off floor, so no
exact cut is rejected; nearly five orders below the sampled residual, so no chord
point is mistaken for a curve point. `1e-9` is a constant of this test, not a
caller's knob, and it is unrelated to `WithTolerance` (§10.1), which governs
*measurements of a body*, not the *admissibility of an input*.

**And the test is sound, not merely a proxy for provenance.** It gates on the thing
that actually has to be true — the endpoint is on the curve — so a sampled cut that
happens to land on the curve is *right*, and recording it is right. decad never
needs to know which path `sketch` took.

**Passing the test, the trim is a closed-form read-out.** On a circle or an arc the
parameter is `atan2` about the known centre, and the fragment is an `ArcSeg`; on an
ellipse or an elliptical arc it is the eccentric angle about the known centre and
axes, and the fragment is an `EllipticalArcSeg`; on a line the endpoints *are* the
record, and a chord of a line is the line, so a `Partial` `*Line` needs no test at
all. Each is an evaluation of the entity's own parameterisation at a point `sketch`
handed over — not a solve.

**The test is not a re-derivation, and this is the whole of why.** decad is
**validating an input it was handed**, not computing a cut. It evaluates a known
formula at a given point and gets a number; it runs no iteration, solves for
nothing, and never produces a point or a parameter it was not given. It computes no
intersection, and where the evidence fails the test it does not repair it, project
it, or fit it — it **rejects** (§12). CLAUDE.md's rule forbids re-deriving a 2D
answer; checking that a 2D answer decad was handed is the one it claims to be is the
opposite of ignoring the seam.

On the **free-form five** there is nothing to test into: inverting a point to a
spline, NURBS, fit-spline or conic parameter is a projection — a 2D root-find, which
is a 2D answer, and decad re-derives none (§7). A `Partial` fragment of a free-form
curve is `ErrUnrecordableProfile` unconditionally. Whole (non-`Partial`) edges of
every kind — the overwhelming case, since a fragment exists only where a curve is
split at a bare crossing — record today, free-form ones with `TStart`/`TEnd` spanning
the curve's full domain. All of it is retired by `TStart`/`TEnd` on `BoundaryEdge`,
the third blocker of §5.3: with the range in hand every partial fragment records
exactly and this test is deleted.

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
ship, exactly as an option that cannot be recorded does not. The one thing
outstanding is a text form for `units.Value`, which belongs upstream (§5.3).

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

- `Fillet`, `Chamfer` and `Shell` depend on one — the body they modify, which they
  consume;
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

**A feature takes the sketch and the profile, in that order.** A `sketch.Profile`
is pure plane-local 2D geometry — an outer loop and its holes in `(u, v)` — and it
holds no back-reference to the sketch it came from, so it names no plane and no
frame. It cannot say *where in space* it is. The sketch can: `s.Plane()` is the
construction plane the sketch is drawn on, and `s.Plane().Frame()` is the
orthonormal `r3.Frame` that lifts the plane-local profile into world space. A
`Step` records that frame as a `PlaneRecord` — origin, u, v — because an `r3.Frame`
does not survive encoding (§6.2), and the frame is what the profile normal — the
sense `Direction.Along` means for a linear extent (§8.1) — is read from.

**`p` MUST be a profile of `s`, and the test is *entity membership*.** Every
`BoundaryEdge.Entity` of `p.Outer` and of every loop of `p.Holes` — and every
element of `p.Entities` — MUST be a member of `s.Entities()`. A profile that fails
that test is `ErrForeignProfile` (§12): it is expressed in a different plane's
coordinates, so lifting it through `s`'s frame would place it silently,
confidently, in the wrong place.

The test is entity membership and **not** pointer membership in `s.Profiles()`,
because `sketch.Sketch.Profiles()` **recomputes and reallocates** the profiles on
every call: the `*sketch.Profile` a caller holds is never pointer-equal to anything
a later `Profiles()` returns, so a pointer-membership rule would reject the caller's
own profile — including the one in the example above. The `Entity` pointers are the
stable identity: they are the sketch's own entities, and they survive every
`Profiles()` call.

**The honest limit of the check: it cannot detect a *stale* profile.** A profile
built before a later `Solve` still holds entities that belong to `s`, so it passes —
but its geometry is the pre-solve geometry. The caller MUST pass a profile from the
current solve. A profile→sketch back-reference, or a generation counter, would make
the check total; both belong upstream in `sketch`, and neither is a condition of
shipping — this is a limitation of the check, not a blocker (§5.3 lists the three
blockers, and this is not one of them).

`doc.Extrude` REJECTS a `sketch.Profile` whose `Valid` is false — a
self-intersecting or degenerate region is never silently swept. `Profile.Valid` is
the whole of decad's *validity* gate, and it is `sketch`'s answer, not one decad
recomputes. The one further rejection is not a validity judgement at all: a valid
profile whose boundary decad cannot record exactly — a partial fragment whose trim
range `sketch` does not expose and whose endpoints do not lie on the source curve,
or a partial fragment of a free-form curve at all (§5.3) — is
`ErrUnrecordableProfile` (§12), because a `Step` that recorded the whole curve
where the caller drew a piece of it, or a trim read off a point that is not on the
curve, would be the lossy record §6.2 forbids.

**The seam permits exactly one read-out, and one test, and neither is a
re-derivation.** Beyond an edge's source `Entity`, decad reads a fragment's two
endpoints — `Polyline[0]` and `Polyline[len-1]` (§6.2). `sketch` computes an exact
cut only for line-involved crossings and tangencies, so those endpoints lie on the
source curve only sometimes, and `BoundaryEdge` does not say when. decad therefore
**checks**: it evaluates the source entity's own implicit residual at each endpoint
(§6.2) and records the fragment only if both lie on the curve. That is arithmetic on
an answer `sketch` handed over — a *validation of an input*, computing no
intersection and solving for nothing — not a second computation of `sketch`'s
answer. Passing, the trim is a closed-form read-out of the entity's own
parameterisation at those points. Where the conversion would need a solve — any
free-form curve — decad does not do it, and where the test fails, decad does not
repair the point: it errors (§5.3).

Whether the *sketch* is fully constrained is a separate, sketch-level question: a
profile can close while the sketch still has degrees of freedom. It is not decad's
to answer — an agent that wants that guarantee gates on `sketch.Sketch.Verify`
before extruding. decad never re-derives it.

## 8. Features

v1 vocabulary, deliberately small: **Extrude, Revolve, Union/Cut/Intersect,
Fillet, Chamfer, Shell**. Sweep and Loft are deferred.

```go
func (d *Document) Extrude(s *sketch.Sketch, p *sketch.Profile, e Extent, opts ...ExtrudeOption) (*Body, error)
func (d *Document) Revolve(s *sketch.Sketch, p *sketch.Profile, axis Axis, a AngularExtent, opts ...RevolveOption) (*Body, error)
```

Both take the **sketch** as well as the profile, because a `sketch.Profile` is
plane-local and carries no plane of its own (§7). Every entity of `p` MUST be an
entity of `s`; a profile from another sketch is `ErrForeignProfile` (§12). decad
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
- **The near-zero case is still defined.** A zero clearance, or the volume of a
  degenerate body, has `|Value|` at or near zero, and a ratio to it is undefined or
  explosive. `Quantum` is what keeps the comparison total there. It engages only
  when the quantity is itself below the noise level — and an `Approximate`
  answer at the noise level, whose `Bound` is anything at all, then reads `Suspect`,
  which is the honest verdict: nothing has been proven about it. An `Exact` answer
  has a zero `Bound` and passes at the floor like everywhere else.
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
  document, §8.1), `ErrForeignProfile` (a feature was handed a profile with an
  entity that is not an entity of the given sketch, §7), `ErrRetiredBody` (an
  operation, or an extent, was handed a body the document has retired, §6),
  `ErrUnresolvedBody` (a `StepRef` was passed as a `BodyRef` to a feature call,
  where a live `*Body` is required, §6.2), `ErrNegativeMagnitude` (a magnitude was
  given as a negative value; magnitudes are non-negative and sense is enumerated,
  §8.1), `ErrUnrecordableProfile` (a feature was handed a profile whose boundary
  contains a partial fragment whose trim range `sketch` does not expose and decad
  cannot recover: a fragment of a free-form curve — conic, spline, closed spline,
  fit spline, NURBS — where the trim needs a projection; or a fragment of an
  analytic curve that fails the endpoint test, i.e. whose endpoints do not lie on
  its source entity, because `sketch` cut it at a sampled crossing, §5.3/§6.2),
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

The assemblies non-goal rests on a plan, not on a capability already in hand:
interference and clearance (§10) are computed between **explicitly-placed bodies**
— `Body.Placed(t r3.Transform)` (§8) — which needs no instancing graph. That plan
is **gated on `r3.Transform` landing** (§5.3). Until it does, a body sits where its
profile put it, and multi-body interference is limited to what that reaches.
