# Public API Design

The design of decad's public API: a headless CAD engine an agent uses to model a
part, prove it sound, and only then write real CAD software code (a Fusion
add-in). This document is the contract. No public API lands that contradicts it.

Companion designs carry the deep ends of this contract: the recording
contract at the sketch seam — the trim contract, the recording IR, and
`ErrUnrecordableProfile` — is specified in `agent-docs/sketch-seam-design.md`; feature
evaluation, topology construction, and atomic commits are specified in
`agent-docs/evaluator-design.md`; how verification judges every
bounded result — the report, the tolerance gate, and the noise floor — is
specified in `agent-docs/verification-design.md`; the mesh contract, per-payload
chording, proof bounds, and boolean handoff are specified in
`agent-docs/tessellation-design.md`; exact cup plus certificate-backed faceted
proofs are specified in `agent-docs/payload-verification-design.md`; and how
verification proves overlap and bounds its volume without consuming either body
is specified in `agent-docs/interference-design.md`. `agent-docs/layout/design-docs.md`
lists every current design document and its owner. Spatial-path recording,
rotation-minimizing frame transport, and the staged `Sweep` evaluator are
specified in `agent-docs/sweep-design.md`. The sheet body — a body that encloses no
material region — and the operations that build, fill and close one are
specified in `agent-docs/surface-design.md`.

Two design-only proposals extend operation observability without changing this
contract: `agent-docs/operation-history-design.md` covers evaluator proof
lineage, source-derived quantity calculations, and immutable execution records;
`agent-docs/operation-diagnostics-design.md` covers structured operation
failures and successful-call warnings.

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

## 2. Architecture: modeling program, evaluator, result

The caller's Go function is the feature tree. `Document` owns only the live body set and private provenance identities used by selectors. Each feature validates its inputs, evaluates geometry, and commits its result atomically. A rejected operation changes neither live-body membership nor provenance numbering.

Bodies are immutable values owned by one document. Features retire consumed bodies and return new bodies; read-only measurements remain available on retired bodies. The evaluator may improve without adding a second public model representation or persistence language.

Decad-owned types that callers name, construct, or inspect are defined in the
root package. Internal packages receive the data they need at the root
boundary; they do not own these public types behind root aliases. Types used
only to implement the evaluator remain internal. Shared foundation types from
`sketch`, `r3`, and `units` keep their own package ownership.

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
reduces analytically, which needs no intersection math either. A pair reduces
when both operands' sweeps are generated by the same one-parameter rigid
motion: their whole intersection is then the sweep of a 2D crossing `sketch`
computes and certifies — a straight line for two co-directional prisms, a
latitude circle for two co-axial revolves — so no space curve arises and every
topology decision is a comparison of flags rather than a floating-point sign
test on a fit. Two documents own that class between them:
`agent-docs/prism-boolean-design.md` for `Union`/`Cut`/`Intersect` over two solids,
and `agent-docs/surface-intersection-design.md` for `Trim`/`Extend`/`Split` where one
operand encloses no material. Every pair outside it takes the tessellated path,
or — where an operand is a sheet, whose mesh proves no occupied volume —
refuses at the call.

The tessellated path ends by rounding its exact vertices to float64, and a
rounded mesh can fold through itself. The final repair first tries the float
box of each exact point and then, within the document's gap tolerance, positions
up to that far away. Every decision stays exact over the values the repair ends
with, and the distance a vertex moved is part of its bound, so the tolerance
states a displacement and never makes the topology tolerant
(`agent-docs/gap-tolerance-design.md`).

## 3. Forward-compatibility invariants

These are cheap now and expensive to retrofit. They are the upgrade path.

| # | Invariant | Rationale |
|---|---|---|
| 1 | **NEVER expose triangles as the representation.** `Tessellate(ctx, tol)` is an output; the public vocabulary is `Body → Face → Edge → Vertex` even while the backing is approximate. | A public `Triangles()` is a one-way door; callers depend on it and vN can never remove it. |
| 2 | **Every measurement carries `Exactness`, from the first commit.** | Makes the upgrade **monotonic, not breaking**: callers already branch on `Approximate`; in vN that branch stops being taken and nobody's code changes. A bare `float64` today means adding exactness later breaks every call site. |
| 3 | **Selectors, never topology indices.** `Body.Faces()` / `Edges()` / `Vertices()` exist for **traversal and inspection only**; no feature or selector names a face, an edge or a vertex by index or by a bare topology pointer. | An exact kernel produces a different (more correct) face/edge decomposition. If `Edges()[3]` is the API, index order becomes a de facto contract and vN breaks every model. |
| 4 | **Booleans never mutate their operands, and take no target-out parameter.** `Union(ctx, first, others...) (*Body, error)` — never Fusion's in-place `booleanOperation(target, tool, type) -> bool`. | A signature free of in-place mutation lets the implementation be swapped with zero API churn. |
| 5 | **Imported meshes are a separate type.** A `MeshBody` never claims to be a solid B-rep. | For imported triangle soup, no future kernel can recover exactness. Keeping it separate stops approximate-forever geometry from contaminating the type we promise to make exact. |
| 6 | **Operations are free functions; queries are methods.** `Fillet(ctx, b, sel, r)`, not `b.Fillet(...)`; `b.Volume()` stays a method (§6.3). | A body-consuming call written as a method reads as an edit of the receiver. A free function states that every body it names is an input and that the result is a new body. |

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
| B-rep topology `Body → Lump → FaceShell → Face → Loop → CoEdge → Edge → Vertex`, with co-edges as directed edge uses | The right model, and it makes agent traversal code map 1:1 onto Fusion's. |
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
body, err := decad.Extrude(ctx, doc, s, prof, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
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
`Transform` (§8) takes.

**The one deliberate exception to §5.1.** An `r3.Vec` *position* — `Box.Min`,
`Box.Max`, a `VecMeasurement.Value` (§5.3), `Cylinder.Origin`, every point the API
returns or accepts — is a **length in the base unit, millimetres**. It is not a `units.Value`
and never becomes one: a vector of three typed quantities cannot be added, scaled,
dotted or crossed without unwrapping it at every step, which makes coordinate math
unusable and pushes callers back to hand-rolling. (An `r3.Vec` used as a *direction*
— `Cylinder.Axis`, `NormalTo(v)` — is dimensionless, and carries no unit at all.)

The carve-out is about **coordinates**, not about the vector type: a plane-local
2D coordinate — `Point2` (`agent-docs/sketch-seam-design.md`), the `(u, v)` of a
recorded profile — is a length in millimetres on exactly the same terms, and for
exactly the same reason.

Vectors carry the unit **by convention**; scalars carry it **in the type**. §5.1
governs scalars, and this is the whole of the carve-out — a bare `float64` scalar
quantity is still forbidden anywhere in the API. A *curve parameter* is not a
scalar quantity: a spline's knots and weights, a recorded segment's parameter
range (`TStart`/`TEnd`), and a conic's fullness `Rho` — from which a rational
quadratic's apex weight derives as `w = Rho/(1−Rho)` (all
`agent-docs/sketch-seam-design.md`) — are dimensionless indices into a
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
gate (`agent-docs/verification-design.md`) is stated over both, and needs no exponent
to be.

Every measurement returns one:

```go
vol, err := body.Volume()  // v1 after a boolean: {12.9997mm³, Approximate, 1e-3mm³}
                           // vN:                 {13.0000mm³, Exact,       0mm³}
```

A mesh's proof level is a property of the MESH, not of a scalar, so it is its
own vocabulary beside `Exactness`: `Verification` says how much proof
`Tessellate` was asked to run, and `Mesh.BoundaryVerified`/`Mesh.VolumeVerified`
say which proofs the returned mesh carries (`agent-docs/tessellation-design.md` §1).
`Mesh.Bound` remains a `units.Value` at every level and is never withheld, so
nothing here weakens invariant #2.

`Measurement`, `VecMeasurement` and `Box` (§6) are the **three and only three**
bounded results the API returns. Every one of them carries a `Bound`; how
verification judges each against the caller's tolerance is specified in
`agent-docs/verification-design.md`, and every one of them is judged — the gate is
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

func (d *Document) Bodies() []*Body            // live bodies
func (d *Document) OperationHistory() *OperationHistory // immutable successful-call snapshot
func (d *Document) Verify(ctx context.Context, opts ...VerifyOption) (*Report, error)

func Remove(b *Body) error                      // retire a live body by hand; the body names its document
```

`DocumentOption` is the tier of `WithGapTolerance` and `ExactOnly`, which set the
document's gap tolerance: how far the evaluator may move a held vertex off its
exact point to keep a faceted result embedded, charged to that vertex's bound
(`agent-docs/gap-tolerance-design.md`). With no option the tolerance is 0.01 mm.

Every document collects successful public operation history without an option.
Snapshots expose caller-visible body dependencies and outputs; they do not record
failed or read-only calls. The history API and its face coverage rules are owned
by `agent-docs/operation-history-design.md`.

`Body` is **immutable**; every operation returns a new one, and the input body is
retired from the document.

**A `*Body` carries its owning `*Document`.** This is what lets a boolean keep the
signature invariant #4 demands — no target-out parameter, no operand mutated —
while `Document.Bodies()` stays truthful: `Union(ctx, a, b)` reaches the document
*through* `a`, so retiring the operands and registering the result happen inside
the operation, with no `*Document` argument and no caller bookkeeping. Operands
themselves are untouched: retiring is a change of *document* membership, not of
the body, which stays immutable and readable. The rule is uniform — every
operation that consumes a body (the booleans of §8, `Fillet` /
`Chamfer` / `Shell`, the surface operations `Thicken` / `PatchEdges` /
`Stitch` / `Unstitch`, and `Transform`) retires its input body or
bodies from the document and registers every body it returns. A retired body
remains readable, but it is gone from `Document.Bodies()` and
`Document.Verify()` never reports on it. The copy operation of §8 —
`Copy` — is the deliberate exception: it
**depends on** the source without consuming it, exactly as an `Extrude` depends
on the body a `ToFace` names (§8.1), so the source stays live and the copy is a
new body beside it.

**`Remove` is the same retirement by the caller's own hand.** A caller
that added bodies it now rejects — say, support bodies `Verify` found
interfering — takes them back out. A removed body is a retired body in every
respect: it stays readable, its selectors and `Origin` still answer, and no
operation takes it, so there is no way back into the model. Remove produces
nothing, so no producer identity advances, and a body already built from the
removed one keeps its geometry and its provenance. A nil body, or a
body that belongs to no document, is `ErrDegenerate`, and a body
already retired — consumed or removed — is `ErrRetiredBody` (§12).

**A retired body is no longer part of the model, so no operation takes one.** It is
readable — its measurements still answer — but handing it to a boolean, to a modify
op, or to an extent that names a body (`ToFace`, §8.1) is `ErrRetiredBody` (§12).
Using such a body as a stop would resolve topology outside the live model.

It follows that **bodies from different documents cannot be combined**.
`Union(ctx, a, b)` where `a` and `b` have different owners has no defined result — which
document would own it? — and is `ErrForeignBody` (§12).

```go
type Body struct{ /* ... */ }

func (b *Body) Document() *Document // the document that owns this body

func (b *Body) Lumps() []*Lump    // disjoint pieces; len > 1 means disconnected
func (b *Body) Shells() []*FaceShell  // FaceShell.IsVoid() marks an internal cavity, IsOpen() a free boundary
func (b *Body) Faces() []*Face
func (b *Body) Edges() []*Edge
func (b *Body) Vertices() []*Vertex

func (b *Body) Kind() BodyKind  // BodySolid or BodySheet — what the body IS (agent-docs/surface-design.md §2.1)
func (b *Body) IsSolid() bool
func (b *Body) Bounds() (Box, error)
func (b *Body) Volume() (Measurement, error)   // error when not a solid — never 0
func (b *Body) Area() (Measurement, error)
func (b *Body) Centroid() (VecMeasurement, error)
func (b *Body) ExtremeFacesByCentroid(ctx context.Context, axis r3.Vec) (least, greatest *Face, err error)
func (b *Body) LocatePoint(ctx context.Context, p r3.Vec, tol units.Value) (PointLocation, error)
func (b *Body) DistanceToPoint(ctx context.Context, p r3.Vec, tol units.Value) (Measurement, error)
func (b *Body) ClosestPoint(ctx context.Context, p r3.Vec, tol units.Value) (ClosestPoint, error)
func (b *Body) Section(ctx context.Context, plane r3.Frame) (*sketch.Sketch, []*sketch.Profile, error)

func (b *Body) Origin() FeatureRef  // which feature created this body
```

`ExtremeFacesByCentroid` currently certifies only complete point-loft planar
Split slabs, their rigidly transformed copies, and positive uniform `ScaledAt` images.
It returns `ErrUnsupported` when
the cap centroid intervals do not strictly separate every side-face centroid.
See `agent-docs/loft-point-design.md` for the source proof.

`LocatePoint` returns `PointInside`, `PointOutside`, `PointOnBoundary` or
`PointUndecided`. It classifies an undisplaced straight prism, an undisplaced
stacked prism or stack-keeping BRep, and a full solid `Revolve` against their
exact records, a mesh Boolean result by combining its operands' exact answers
through the operation history, and every other body or point against the
body's verified mesh. It returns only classifications proved for the body, including
retired bodies. `agent-docs/point-containment-design.md` owns its proof and
refusal rules.

`DistanceToPoint` measures the minimum distance from `p` to the solid, so a
point inside or on its boundary has distance zero. Its `Measurement` encloses
the true distance using the verified mesh and its boundary bound. When the
mesh proof cannot prove membership, its lower bound is zero. The point
query design owns its proof and refusal rules.

`ClosestPoint` returns the mesh point that attains that distance, as a `ClosestPoint` holding the point, the same
`Distance` and an `OffSurface` bound on how far the point lies from the true boundary; a point inside the solid is its
own closest point. `Face.ClosestPoint` does the same for one trimmed face. `agent-docs/point-containment-design.md` §6
owns it.

`Section` returns Sketch profiles on the requested plane for admitted exact
prisms, exact cardinal cylinder chords, cardinal or central oblique source-sphere
cuts, exact or bounded cardinal right-cone cuts, and bounded rectangular-prism or
full-cylinder cuts. `agent-docs/section-design.md` owns the gates and reconstruction
proof. An exact profile uses the ordinary Sketch seam. A bounded profile
carries a private source certificate and currently enters only straight solid `Extrude`
with an independent `Distance` extent. Other Decad profile consumers refuse it.

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
  `Edge.IsConvex`, `Loop.IsOuter`, `FaceShell.IsVoid`, `FaceShell.IsOpen`,
  `Edge.IsFree`. These are answers, not approximations of answers, so they stay
  bare bools. `Body.Kind` joins them as a bare enum on the same terms: it is
  decided by the operation that built the body, and it measures nothing.

**`Body.Kind` says what the body IS; `IsSolid` says whether it is sound.** A
`BodySheet` encloses no material region — it is a boundary on its own — so
`Volume` and `Centroid` are `ErrNotSolid` for every sheet, sound or not, and
that refusal reports a CATEGORY. The same error on a `BodySolid` body reports a
DEFECT: a body built as a solid whose boundary did not prove closed. Without
`Kind`, `IsSolid() == false` would mean both, which is exactly the
"`volume == 0` means both empty and not-a-solid" sloppiness §4 rejects in
Fusion. `agent-docs/surface-design.md` §2.1 owns the kind, and §2 there owns what a
sheet holds; a sheet body uses every accessor above unchanged.

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
func (f *Face) Centroid() (VecMeasurement, error) // area centroid when the face has a proof
func (f *Face) DistanceToPoint(ctx context.Context, p r3.Vec, tol units.Value) (Measurement, error)
func (f *Face) ClosestPoint(ctx context.Context, p r3.Vec, tol units.Value) (ClosestPoint, error)
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
func (e *Edge) IsFree() bool       // exactly one adjacent face — a sheet's boundary
func (e *Edge) Start() *Vertex
func (e *Edge) End() *Vertex
func (e *Edge) Length() (Measurement, error)
func (e *Edge) IsConvex() bool
func (e *Edge) Wedge() EdgeWedge   // the material angle: WedgeSalient, WedgeReflex or WedgeUndecided

type EdgeWedge int // WedgeUndecided, WedgeSalient, WedgeReflex; String() "undecided" / "salient" / "reflex"
```

`Face.DistanceToPoint` measures the selected trimmed face patch, including
its edges. A point inside the owning solid can have a positive face distance.
It uses that face's verified mesh displacement bound; the point query design
owns the interval and refusal rules.

**What `len(Edge.Faces())` means is read per body kind.** On a `BodySolid`,
one adjacent face is non-manifold, as the gloss above says. On a `BodySheet`,
one adjacent face is a **free edge** — the boundary of the sheet, and the
expected shape — which `Edge.IsFree` reports and the `Free()` predicate (§9)
selects. Two is an interior edge on either kind; three or more is non-manifold
on either. `agent-docs/surface-design.md` §2.2 owns the reading.

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

**`Edge.Wedge` is the MATERIAL angle across the edge, read from the two faces'
outward normals.** `WedgeReflex` is a material angle over a half turn (a boss
root, an inside corner), `WedgeSalient` one under a half turn (a box's outer
edge, a boss's top rim, a hole's rim). The reading is the sign of
`(n1 × n2)·t` at each end of the edge, where `n1` is a planar face's outward
normal, `n2` the other face's, and `t` the direction the planar face walks the
edge with its material on the left seen from outside. That walk comes from the
signed area of the planar loop holding the edge and the loop's `IsOuter` role,
never from the stored loop direction, which builders do not agree on. The
answer is decided only where `|(n1 × n2)·t|` exceeds the bound composed from
`Face.NormalAt`'s own bounds, the normal's turn over the vertices' proven
displacement, and the tangent's displacement. That sign check can only withhold
an answer, never grant one on a small residual. The loop's area is decided the
same way, against the rounding of its terms and of their running sum, the band
the vertex and curve displacement sweeps, and the plane normal's bound; an
arc whose held sweep lies within its ends' displacement of zero gives no area.
Where both faces are planar, both readings must agree. Only edges whose angle
cannot change along them are read: a line between two planes, a line between a
plane and a cylinder parallel to it, and a circle or arc on a plane normal to
its axis meeting a coaxial cylinder, cone, sphere or torus, the circle carrying
a proven curve bound. That class gate does admit on a tolerance: "parallel" and
"coaxial" are `selectorquery.ParallelDirs`, a relative 1e-9. It chooses which
edges are read and decides no sign, and its tolerance cannot flip a decided
sign along the edge. On a line, the plane's normal and the tangent are
constant and a cylinder's normal turns monotonically by far under a half
turn, so the reading is a cosine of that turn and keeps the sign both ends
decide. On a circle or arc, the plane's and the round face's misalignment
from the circle's axis moves the reading by at most twice that misalignment,
which the bound charges. `internal/edgewedge`'s `admitted` states the argument.
Everything else is `WedgeUndecided`: a tangent junction, a `Faceted` face, a
`FacetedCurve` or `Ellipse3` edge, a line between two cylinders, a free or
non-manifold edge. `internal/edgewedge` owns the reading. The two readings
differ on purpose: a boss root on a `Union` result is `IsConvex` and
`WedgeReflex`, a hole's rim is concave and `WedgeSalient`.

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
// NURBSSurface is a free-form face's geometry. `agent-docs/spline-design.md` §7 owns
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
// Ellipse3 is an elliptical arc edge, swept counter-clockwise about Axis from
// the start vertex to the end vertex as Arc3 is: the point at angle φ from the
// unit semi-major direction Major is
// Center + SemiMajor·cos φ·Major + SemiMinor·sin φ·(Axis × Major)
// (agent-docs/loop-fillet-design.md §2). An edge whose start and end vertex are one
// vertex is the whole ellipse, one turn from that vertex, as a Circle3 edge is. A
// placement maps Center and Major as points and directions; a reflection also
// negates Axis, so the arc keeps its sense between its mapped vertices.
type Ellipse3 struct {
	Center, Axis, Major  r3.Vec
	SemiMajor, SemiMinor units.Value
}
// FilletMiter3 is the intersection edge of two adjacent analytic fillet
// patches at a sharp corner involving a circular wall. Its endpoints and
// adjacent faces identify the continuous intersection branch. The evaluator
// keeps the offset-foot parameterization private; the edge is not a NURBS or
// a chain of chords.
type FilletMiter3 struct{ /* private */ }
// NURBSCurve is a free-form edge's geometry, NURBSSurface's 1-D analog
// (agent-docs/spline-design.md §7).
type NURBSCurve struct{ /* private */ }
// FacetedCurve is a boolean-built edge's chord chain, Faceted's 1-D analog.
type FacetedCurve struct{ Bound units.Value }
```

**Surface parameters carry no `Exactness`, and this is not an exception to
invariant #2.** An analytic `Surface` variant's parameters are exact for the
surface it names. A face whose built geometry is that surface is exact; a tagged
analytic variant may instead be a bounded stand-in and carry its departure in
computed measurements (`agent-docs/modify-reach-design.md` §8.3). A face whose held
geometry is faceted is `Faceted`, and `Faceted` is the flag; the one exception
is a tagged analytic variant on a faceted face, a float-image stand-in whose
departure is carried in `NormalAt`, never in `Surface()`'s parameters.
`Transform` and `Copy` carry such a tag to the float image of the source tag,
and unless the placement is exactly the identity, `NormalAt` charges a placed
group that has no denoted patch the tag's own departure plus the float image's
(`agent-docs/faceted-source-retention-design.md` §3.4) where the tag is a retained
`Plane`, `Cylinder`, `Sphere` or `Torus`, and a direction bound of 2 otherwise
(`agent-docs/faceted-vertex-bounds-design.md` §4.3). So a `Cylinder.Axis`
or a `Sphere.Center` is an exact parameter of its named surface, while
`Face.NormalAt(p)` — a quantity the evaluator *computes*, on a face that may be
a bounded analytic stand-in — is a measurement and reports its `Exactness` like
every other. It answers for the five analytic variants, and `NormalAt` on a
`NURBSSurface` or on a `Faceted` face is `ErrUnsupported` (`agent-docs/spline-design.md`
§7); the faceted reading lands with the faceted certificate stage, whose internal
source certificate is what bounds the true patch normals
(`agent-docs/payload-verification-design.md` §5.4, §13). A positional `Faceted.Bound`
alone does not imply a normal bound (`agent-docs/payload-verification-design.md`
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

**Profile measurements and provenance.** `MeasureProfile` authenticates a live
sketch profile and returns a
`MeasuredProfile` snapshot. Its `Area`, `Centroid`, and `SecondMoments` methods
give bounded plane-local 2D readings. The evaluator's curve records stay in
`internal/`; they are geometry snapshots, not a serialized model or replay
format. Callers obtain the sketch plane through `s.Plane().Frame()` when they
need to place the centroid in world coordinates. Each method reports its own
unsupported or degenerate input independently of the other readings.

`FeatureRef` is an opaque provenance value returned by bodies and topology. Its `Role` field is public for inspection; the document-local producer identity is private. Callers pass returned references to `CreatedBy` or `FaceCreatedBy` instead of constructing or persisting producer numbers. `CapStart` and `CapEnd` provide the common cap references.

The public selector roots are `EdgeSelector` and `FaceSelector`. There is no exported common selector interface because feature signatures already state which topology family they accept.

**`Axis`** — sealed; what a revolve may spin about.

```go
type Axis interface{ axis() }

type SketchLine   struct{ /* ... */ } // a line in the source sketch
type ConstructionAxis struct{ /* ... */ } // an explicit axis in the document
// EdgeAxis is a linear edge, selected — never a pointer. Body is what Edge
// resolves against: a Revolve is handed no body, so the axis must name its own.
type EdgeAxis struct{ Body *Body; Edge EdgeSelector }
```

`EdgeAxis.Edge` MUST resolve to **exactly one** linear edge of `Body`
(`EdgeQuery.Exactly(1)`); zero or many is `ErrCardinality` (§12), and a non-linear
edge is `ErrDegenerate`. `Body` MUST be a live body of the same `Document` passed to
`Revolve` — another document's is `ErrForeignBody`, a retired one is
`ErrRetiredBody`. Resolving the axis does not consume its body.

**`Interference` / `Clearance`** — the pairwise results of §10. Both name the two
bodies and carry their quantity as a `Measurement`, so both report their own
exactness like everything else. `agent-docs/interference-design.md` owns the
read-only overlap proof; `agent-docs/clearance-design.md` owns the disjointness and
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

For a pair holding a sheet operand, `Clearance` carries a narrower claim
(`agent-docs/surface-design.md` §9.3): a sheet has no volume to be disjoint FROM, so
`Gap` states the proven distance between the sheet and the solid's boundary,
whether the sheet lies wholly inside or wholly outside it, WITHOUT asserting
which side. A solid-solid row's meaning — the two bodies do not overlap, and
how close they come — is unchanged.

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

**`BodyKind`** — what a `Body` is. §6 draws the line between it and `IsSolid`,
and `agent-docs/surface-design.md` §2.1 owns it in full. Its constants are
`Body`-prefixed, in the style of `BooleanErrorCode`'s `Boolean*`, because a bare
`Solid` or `Sheet` at package scope names no type it belongs to:

```go
type BodyKind int

const (
    BodySolid BodyKind = iota // encloses a material region
    BodySheet                 // encloses none — a boundary on its own
)
```

`Direction` — the enumerated sense a standalone extent carries — is *not* deferred:
it is specified in full in §8.1, and declared there.

The remaining topology of §2's chain, each deferred but for the one method that
carries its weight:

```go
// Lump is a connected piece of a body — solid or sheet. Body.Lumps() returning
// more than one means the body is disconnected. A sheet lump holds exactly one
// shell (agent-docs/surface-design.md §2.2).
type Lump struct{ /* ... */ }

// FaceShell is a connected set of faces.
type FaceShell struct{ /* ... */ }
func (s *FaceShell) IsVoid() bool // true when the shell bounds an internal cavity
func (s *FaceShell) IsOpen() bool // true when the shell has at least one free edge

// Vertex is a topological point.
type Vertex struct{ /* ... */ }
func (v *Vertex) Position() VecMeasurement // millimetres (§5.2); a computed coordinate, so it is bounded (§6)

// MeshBody is an imported triangle mesh (STL, OBJ). It NEVER claims to be a solid
// B-rep — invariant #5. It reports its bounds, area, closed-surface status and,
// when closed and consistently wound, its volume and exact point queries; no
// feature or Boolean takes it.
// agent-docs/mesh-import-design.md owns it.
type MeshBody struct{ /* ... */ }
```

The current solid builders publish a separate void shell for a full `Revolve`
of a profile with a hole, and for a Boolean result with an enclosed negative
boundary component, such as a `Cut` by a wholly embedded tool. `Extrude` and
`Loft` of holed profiles make passages between their end caps. An admitted
open-path `Sweep` does the same. A partial `Revolve` opens its groove at the
angular caps. Those bodies have one connected, non-void shell. The `Shell`
operation that removes a face leaves an opening, so its inner and outer skins
connect. `Shell` accepts the closed `WithNoOpenings()` form of
`agent-docs/modify-reach-design.md` §9.2 and returns `ErrUnsupported` for it on every
receiver until reach §14 rows C and D build it. Sheet shells
always report `IsVoid() == false`, even when closed
(`agent-docs/surface-design.md` §2.2). `Stitch` refuses a closed assembly whose
components are not proven separate, rather than publish an unrecorded cavity.

### 6.3 Operation form: free functions, and methods for queries

An operation is a package-level function. A query is a method. This section
owns the rule; a companion design that writes an operation as a method
(`func (d *Document) Sweep(...)`) means the
free function the table below names for it.

**An operation is any call that builds a body, retires a body, or registers a
body in a document.** It is a free function, so the document and every body it
consumes appear in the argument list. Calls read the same whether they take one
body (`Fillet(ctx, b, sel, r)`), two (`Cut(ctx, target, tool)`) or none
(`Extrude(ctx, doc, s, p, e)`).

**A query is a read that changes nothing.** It retires no body, registers no
body, and leaves `Document.Bodies()` as it found it. A query is a method on the
value it reads, so a retired body still answers it (§6). The query set is:

- topology and identity reads: `Faces`, `Edges`, `Vertices`, `Lumps`, `Shells`,
  `Kind`, `IsSolid`, `Origin`, `Document`, and every method on `Face`, `Edge`,
  `Loop`, `CoEdge`, `Vertex`, `Lump` and `FaceShell`;
- measurements: `Bounds`, `Volume`, `Area`, `Centroid`, `MassProperties`,
  `LocatePoint`, `DistanceToPoint`, `ExtremeFacesByCentroid`;
- `Tessellate`, which returns a `Mesh` and registers nothing;
- the document reads `Bodies`, `Verify`, `VerifyMotion`, `VerifyLinkage`,
  `VerifyJointBox`, `ContactPair`, `SweepPair` and `SweptBox`, each of which
  returns a report and leaves the document unchanged;
- the accessors and report methods on result types (`Report`, `MotionReport`,
  `SweepReport`, `Path`, `Mesh` and the like).

**The topology type is `FaceShell`.** The package-level name `Shell` belongs to
the shell operation, so the connected set of faces between a `Lump` and its
`Face`s is `FaceShell`, and `Body.Shells()` returns `[]*FaceShell`.

**Form of a free function.** Parameters appear in this order: `ctx` when the
operation takes one, then the owning `*Document` when no body argument
identifies it, then the body or bodies operated on, then the remaining inputs,
then options. An operation that takes a body reaches the document through that
body (§6) and takes no `*Document` argument. An operation that takes no body
(a feature built from a sketch) takes the `*Document` first.

| Shipped method form | Free function |
|---|---|
| `Document.Sweep` over a profile or a chain | `Sweep(ctx, doc, s, sec, path, opts...)`, one generic function over `Section` (§8) |

**Every operation that builds a body from a sketch takes a `ctx`, `Extrude`,
`Revolve` over a chain included.** Each checks `ctx` at
entry, between recording the section, resolving the extent and building the
body, and inside the build evaluators. A canceled `ctx` returns `ctx.Err()`
unchanged before commit and leaves `Document.Bodies()` and every other body as
it found them. A nil `ctx` is `ErrDegenerate`, on §8's rule for every operation
that takes one. The build can run long: a free-form wall proves its curvature
sign over a bounded work budget (`agent-docs/spline-design.md` §5.2), and a nonzero
`WithTaper` builds through the draft evaluator.

`Union`, `Cut`, `Intersect`, `CombineJoin`, `Stitch`, `Transform` and `Copy`
already have this form. `Transform(ctx, b, t)` places a body, and
`Copy(ctx, b, WithCopyPlacement(t))` copies it under a rigid motion (§8).

**A builder is not an operation.** `Linkage` is a mutable value that records
joints and loops, and its methods (`Close`, `DeclareJointContact`, `Ground`,
`Links`, `Loops`, `Configuration`, `PoseAt`, `Schedule`) edit or read that
value. They stay methods. A `Linkage` method that built or retired a body
would be an operation and would follow the rule above.

**New API follows the rule.** A new public entry point that builds, retires or
registers a body is a free function. A new read is a method on the value it
reads. The code in the root package carries the method form of every row in the
table; each migrates to its free function in the change that updates that operation's
companion design.

### 6.4 The report package

`report/` holds the result vocabulary that `Verify`, `ContactPair` and
`SweepPair` publish: `Measurement`, `VecMeasurement`, `Box` and `Exactness`;
the outcome, reading, survey and diagnostic enums; the contact and sweep
requests and enums; the pair path segments; the sweep instant, interval,
departure, event, sample and report records; and the contact, diagnostic,
interference, clearance and Verify report records. A record that names bodies, faces, edges or vertices is
generic over those reference types, and its name ends in `Of`
(`ContactReportOf[B, F, E, V]`, `DiagnosticOf[B, C]`, `ReportOf[B, F, C]`).

`decad` instantiates each generic record with its own handles and exports the
instantiation as an alias, so `decad.ContactReport` and
`report.ContactReportOf[*decad.Body, *decad.Face, *decad.Edge, *decad.Vertex]`
are one type, and the enums, constants and requests are the same types under
both spellings. Callers name the types through `decad`; the field and method
documentation lives in `report`. The package imports no `decad` type, so the
internal kernels instantiate the same records with topology nodes and
`report.Lift` maps a finished node record to its handle form.

`linkage/` holds the linkage model of `agent-docs/linkage-check-design.md` the same
way: `Linkage`, `Link`, `Drive`, `LinkageLoop`, `Schedule` and the linkage and
joint-box reports are generic over the body reference type, and `decad`
exports each instantiated with `*Body` as an alias. Its checks run over the
motion engine through a small set of hooks that map a body reference to its
topology node and back.

## 7. The sketch seam

`sketch` answers every 2D question; decad consumes the answer and NEVER
re-derives it. The recording contract at this seam — the trim contract
(`TStart`/`TEnd`/`TExact`), the recording IR a `Step` carries, and
`ErrUnrecordableProfile`'s full semantics — is specified in
`agent-docs/sketch-seam-design.md`; this section states the feature-call rules built
on it.

```go
w := sketch.NewWorld()
s, _ := w.CreateSketch(w.XY())
s.CreateRectangle(0, 0, 100, 60)
s.Solve(ctx)

// sketch has already proven this region closes: it is a valid, extrudable profile.
prof := s.Profiles()[0]

doc := decad.New()
body, err := decad.Extrude(ctx, doc, s, prof, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
```

**A feature takes the sketch and the profile, in that order.** A
`sketch.Profile`'s geometry is pure plane-local 2D — an outer loop and its holes
in `(u, v)` — so the profile alone does not place the region in space: the plane
is the sketch's. `s.Plane()` is the construction plane the sketch is drawn on,
and `s.Plane().Frame()` is the orthonormal `r3.Frame` that lifts the plane-local
profile into world space. A `Step` records that frame as an internal
`sectionrecord.PlaneRecord` —
origin, u, v — because an `r3.Frame` does not survive encoding (§6.2), and the
frame is what the profile normal — the sense `Direction.Along` means for a
linear extent (§8.1) — is read from.

**`p` MUST be an unaltered current profile of `s`, and decad records only a
fresh sketch-authenticated match.** `Profile.Sketch()` MUST be `s`.
Every boundary entity MUST be non-nil and present in `s.Entities()`; a foreign
boundary entity is `ErrForeignProfile` (§12). For an ordinary profile, decad
then calls `s.Profiles()` and requires every exported field of `p` to exactly
match one fresh current profile, including entity identities, boundary
order/ranges, polylines, holes, area and validity. For a profile returned by
`s.UnionProfiles(indices...)`, decad reads its private selected indices through
`UnionRegionIndices()`, recomputes `s.UnionProfiles(indices...)`, and requires
those indices and every exported field to match. It records the fresh match,
never caller-mutable `p`. No match is `ErrInvalidProfile` (§12).

A `*sketch.Profile` is freshly allocated by every `Profiles()` or
`UnionProfiles()` call, so pointer membership cannot authenticate it. Exact
snapshot matching preserves the profile value while obtaining a trusted copy.
`Profile.Sketch()` still proves
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
caller rebuilds with `s.Profiles()` or `s.UnionProfiles()` and passes a current
one.

`Extrude` REJECTS a `sketch.Profile` whose `Valid` is false — a
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
whole-edge rules are specified in `agent-docs/sketch-seam-design.md`.

**An OPEN sketch curve reaches the same internal recording path.** A
`sketch.Chain` is `Profile`'s open counterpart — an ordered run of the same
`BoundaryEdge` values, carrying the same `TStart`/`TEnd`/`TExact` trim contract
and the same `Sketch()`/`Revision()`/`IsStale()` handles — so every rule above
governs it unchanged: the foreign, stale, snapshot-match, validity and `TExact`
gates, and the four sentinels they return. Two things read differently, and
`agent-docs/surface-design.md` §13 owns both: the snapshot match runs against the
whole fresh `s.Chains()` result rather than at one index, because that slice's
ORDER consults entity names and `Sketch.Revision()` hashes none of them; and
`Chain.Length` is compared for equality and never read as a measurement,
because it is a sampling-convergent underestimate for every entity but a line,
arc or circle.

Whether the *sketch* is fully constrained is a separate, sketch-level question: a
profile can close while the sketch still has degrees of freedom. It is not decad's
to answer — an agent that wants that guarantee gates on `sketch.Sketch.Verify`
before extruding. decad never re-derives it.

## 8. Features

v1 vocabulary, deliberately small: **Extrude, Revolve, Union/Cut/Intersect,
Fillet, Chamfer, Shell, Transform, ScaledAt, Copy, Loft, LoftSections, LoftFromPoint, Sweep, Coil,
PatchProfile, PatchEdges, Stitch, Unstitch, Thicken**, plus the sweeps of an OPEN sketch curve —
**LoftChain**, and **Extrude**, **Sweep** and **Revolve** given a
`*sketch.Chain` — which take a `*sketch.Chain` where their siblings take a
`*sketch.Profile` and always build a sheet.
`agent-docs/loft-design.md` owns `Loft`'s signature, its two-profile correspondence
rule, and its increment-1 scope. `agent-docs/loft-sections-design.md` owns the
separate `LoftSections`: three quadratic homothetic line sections or
three to thirty-two independently varying axis-aligned rectangles,
exactly identical curved sections, linearly varying four-arc circles, and the
bounded varying fitted-flank class with fixed arcs; or four rounded-rectangle
sections with cubic width and fixed height and corner radius.
`agent-docs/loft-point-design.md` owns the separate
`LoftFromPoint(ctx, doc, apex, sketch, profile)` solid construction from
an exact world point to one recorded Sketch profile. `agent-docs/sweep-design.md` owns `Sweep`'s
signature, spatial `Path`, frame transport, refusals, and staged reach.
`agent-docs/helix-design.md` owns `Coil`'s signature, its axis and its extent in
turns.
`agent-docs/surface-design.md` owns `PatchProfile`, `PatchEdges`, `Stitch`, `Unstitch` and `Thicken`,
together with `WithSurfaceResult()`, the option that
makes `Extrude`, `Revolve`, `Sweep` and `Loft` return their wall set as a
sheet instead of closing it into a solid, and the chain-fed forms that
sweep an open curve (§13 there). `agent-docs/draft-design.md` owns `WithTaper`'s
build and `Draft`, the face draft of an existing prism, with its
`NeutralFace`/`NeutralFrame` planes and the `Walls(b)` selector.

```go
func Extrude[S Section](ctx context.Context, d *Document, s *sketch.Sketch, sec S, e Extent, opts ...ExtrudeOption) (*Body, error)
func Revolve[S Section](ctx context.Context, d *Document, s *sketch.Sketch, sec S, axis Axis, a AngularExtent, opts ...RevolveOption) (*Body, error)
func Sweep[S Section](ctx context.Context, d *Document, s *sketch.Sketch, sec S, path *Path, opts ...SweepOption) (*Body, error)
func Coil(ctx context.Context, d *Document, s *sketch.Sketch, p *sketch.Profile, axis Axis, pitch, turns units.Value, opts ...CoilOption) (*Body, error)
func LoftFromPoint(ctx context.Context, d *Document, apex r3.Vec, s *sketch.Sketch, p *sketch.Profile) (*Body, error)

// Section is the sketch geometry a section-fed operation consumes: a closed
// *sketch.Profile or an open *sketch.Chain.
type Section interface{ *sketch.Profile | *sketch.Chain }
```

`Extrude` infers `S` from `sec`, so no call site names the type argument. A
profile section builds a solid, or a sheet under `WithSurfaceResult()`, and a
nonzero `WithTaper` builds a draft body. A chain section builds a sheet with no
draft build, so `Extrude` returns `ErrUnsupported` for `WithSurfaceResult()` and
for `WithTaper` of any angle, zero included, on a chain. That refusal follows
the chain's seam gates and nil-option check and precedes extent resolution and
any commit (`agent-docs/surface-design.md` §13.2).

`Sweep` infers `S` from `sec`, so no call site names the type argument. A
profile section builds a solid, or a sheet under `WithSurfaceResult()`. A
chain section builds a sheet, so `Sweep` returns `ErrUnsupported` for
`WithSurfaceResult()` and `WithSweepTwist` on a chain, before any commit
(`agent-docs/surface-design.md` §13.2, `agent-docs/sweep-design.md` §15.3).

`Revolve` infers `S` the same way. A profile section builds a solid, or a
sheet under `WithSurfaceResult()`. A chain section builds a sheet, so `Revolve`
returns `ErrUnsupported` for `WithSurfaceResult()` on a chain, after the
chain's seam gates and before the axis gates and any commit
(`agent-docs/surface-design.md` §13.2).

`LoftSections(ctx, doc, sections ...LoftSection)` carries each `Sketch` and
`Profile` together. It admits the bounded classes in
`agent-docs/loft-sections-design.md`.

`LoftFaces(ctx, doc, sections ...*Face)` consumes two to thirty-two ordered planar caps from
authenticated point-loft Split slabs. It retains their original Sketch
profile and exact cut-plane maps through rigid placement and positive
uniform scale. Its bounded source class and proof are in
`agent-docs/loft-face-sections-design.md`.

```go
// ConeTangentEnd names the cone that the last, constructed section touches.
type ConeTangentEnd struct {
	Cone   *Body  // a live, unplaced, full straight-meridian Revolve cone about world X
	Radial r3.Vec // direction of the touching ruling's radius; X must be 0, (Y, Z) nonzero
}

// LoftFacesToConeTangent lofts through 1 to 31 selected caps, then a
// constructed last section tangent to end.Cone along one ruling.
func LoftFacesToConeTangent(ctx context.Context, end ConeTangentEnd, sections ...*Face) (*Body, error)
```

`LoftFacesToConeTangent` is a free function. Its `sections` follow `LoftFaces`
(the least slab's toe cap, then heel caps in source order), and its last
section is built from `end.Cone`'s exact apex and slope instead of being
selected. The result is an ordinary LoftFaces body: its end face carries the
cap-end role of the new loft and a rounded `Plane` tag, and `Cut` and
`Intersect` continue from it as they do from `LoftFaces`. A cone that is not an
exact point cone, a placed or retired cone, a `Radial` with a nonzero `X` or
without a finite nonzero `(Y, Z)`, and a section count outside 1 to 31 are
refused with `ErrUnsupported`, `ErrRetiredBody` or `ErrDegenerate`. The
construction is in `agent-docs/loft-face-sections-design.md`.

`ConeFromFace(ctx, body, face, far)` is a free function that returns the full
cone about the world X axis of a Revolve `body` whose meridian extends the
selected conical `face`, using the nearest representable strict offset from
that face's exact line (`far` is a `units.Value` length). This need not be one
float step: the rounded value may already be strictly sided, or the bound may
be MaxFloat64. The construction guarantees separation only from the selected
face line; later checks handle
other blank and tool relations. `face` resolves under the implicit exactly-one
rule. The function reads the exact recorded section, never the rounded `Cone`
tag. The result is an ordinary Revolve body, and `body` stays live. `far` must
be a finite negative axial coordinate (below X = 0) that is also below both
ends of the face. An
unsupported body or face, a `far` that is not below the face, or a `far` that
is not negative is `ErrUnsupported`, as is a cone Revolve would classify as
a cylinder. A `nil` ctx, a `nil` body or a body of no document is
`ErrDegenerate`. Every refusal precedes registration, so the document and
provenance numbering are unchanged. The rule is in
`agent-docs/loft-face-sections-design.md`.

Each takes the **sketch** as well as the profile, because a `sketch.Profile`'s
geometry is plane-local and the plane is the sketch's (§7). `p` MUST be a
current, unaltered profile of `s`: another sketch's profile or a foreign
boundary entity is `ErrForeignProfile`, a stale one is `ErrStaleProfile`, and a
snapshot that does not match a fresh `s.Profiles()` result is
`ErrInvalidProfile` (§7/§12). decad
reads the plane through `s.Plane()` and its frame through `s.Plane().Frame()`, and
records the plane and profile as internal `sectionrecord.PlaneRecord` and
`momentinput.Profile` values before
building the feature, so later evaluation does not retain live sketch state.

Booleans are **explicit** — not folded into every feature with an ambient,
implicitly-chosen target — and they never mutate an operand or take a target-out
parameter:

```go
func Union(ctx context.Context, first *Body, others ...*Body) (*Body, error)
func Cut(ctx context.Context, target, tool *Body, opts ...BooleanOption) (*Body, error)
func Intersect(ctx context.Context, a, b *Body, opts ...BooleanOption) (*Body, error)
```

`BooleanOption` is a sealed option tier for `Cut` and `Intersect`. A nil or zero
option is `ErrDegenerate`. `WithMatchingFaces()` declares that faces of the two
operands are meant to match where they meet: a cap drawn to touch the tool's
cone, or two faces meant to coincide. Faces drawn in floating point seldom
match exactly, and a design that matches on paper can miss by a few last-place
steps. With the option, a miss of at most 64 binary64 steps at 1 is taken as
the intended match and its effect is charged into the result's error bounds. A
larger miss disproves the declaration and refuses as it would without the
option. The declaration comes from the caller and decad's check can only reject
it, so the option never admits a shape for another reason. It is read only
where a pair's source records keep a cone limit (a point-Loft cut or
intersected by a cone about an axis through the world origin in a world
coordinate plane); elsewhere it has no effect. The
construction is in `agent-docs/revolve-loft-sector-design.md`.

`Union` requires at least two distinct live bodies from one document. A
two-body call uses the pair evaluator. For three or more bodies, one operation
builds one result and retires every input together. The exact prism class
accepts hole-free whole-line sections on the identical recorded plane and
sweep interval, with distinct source line carriers. It arranges every section
in one Sketch scene, audits the resulting boundary, and charges its cuts
once. Authenticated straight-source axial Lofts can enter that same scene as
private prisms when each Loft has a positive-area overlap with the first
prism and all operands meet the scene's stricter source requirements. Other
prism inputs enter the two-body evaluator in source order after the
specialized multi-body classes miss. Each intermediate result stays
transient; the final result commits only if every pair passes its proof.
A pair can still return `ErrUnsupported`.

The bounded mirrored-Loft class accepts one exact centered circular prism
and up to thirty-two copies of an authenticated three-station reflected Loft
pair. It joins all lower sources against one half of the prism, reflects the
result, and proves the shared cap, source boundary and occupied volume.
One-float-step axial placement differences receive explicit boundary and
swept-volume allowances.

The finite Revolve/point-Loft class accepts an authenticated full Revolve
section, one cone-trimmed point-Loft sector, and rigid copies of that sector,
all about one axis: world X, or an axis through the world origin in a world
coordinate plane. Serial and variadic `Union` rebuild one section ring and its
certified sector caps. A centered cylinder spanning both axial caps can `Cut`
a body joined about world X or about another admitted axis. The source,
clearance, boundary and volume proofs are in `agent-docs/revolve-loft-sector-design.md`, and the general axis
in `agent-docs/revolve-axis-design.md`.

The bounded spiral-bevel class accepts an exact finite revolve blank and
separate rigid copies of one authenticated, cone-trimmed nine-section tooth.
The tool sectors must be proven disjoint. The
evaluator cuts the blank and all spiral tools in one mesh Boolean, certifies
each root and cap contact against its original tooth source, and commits one
body only after the complete output passes the boundary, volume and topology
audits. Multi-body inputs outside these classes and the prism pair chain
return `ErrUnsupported`.
A call with no `others` returns `ErrDegenerate`; a
zero-body call does not compile. A failed or canceled call leaves every
operand live.

No `*Document` appears in those signatures, and none is needed: a `*Body` carries
its owning document (§6), so each call retires its operands and registers its
result inside the document that owns them. The operands are themselves unchanged
— invariant #4 — and `Document.Bodies()` and `Document.Verify()` stay truthful.
Operands owned by different documents are `ErrForeignBody`.

**Each cancels the complete Boolean before its atomic commit.** They pass `ctx`
through operand tessellation, exact-predicate classification, cutting,
stitching, mesh audit, and exact volume calculation. Cancellation returns
`ctx.Err()` unchanged and leaves all operands and the live-body set unchanged.

**A nil context is `ErrDegenerate`, for every operation that takes one.** The
context is polled rather than merely stored, so a nil one is a caller mistake
the operation cannot carry out, not a request to run uncancellably; it is
refused at the entry point, before any operand is examined and before the
document is touched.

**A boolean failure is typed, because its three failures are three different
caller actions.** A boolean that produces no body, reaches a valid-model limit
this evaluator cannot handle, or breaks an internal invariant returns a
`BooleanError` with a branchable `Code`, wrapping the sentinel `errors.Is`
already branches on:

```go
type BooleanError struct {
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
`DiagUnsupportedPairContact` diagnostic (`agent-docs/verification-design.md` §1.1), so it
reads the same way whether a caller ran the boolean directly or reached it
through interference. A pair `Verify` resolves EARLIER never reaches that
outcome, and two resolvers run before the mesh boolean does. The coplanar
`Plane`×`Plane` contact certificate (`agent-docs/clearance-design.md`) makes a
certified coplanar touch read as a touching/clearance result — an `Exact`-zero
gap, no interference. The read-only analytic `OpIntersect` dispatch
(`agent-docs/prism-boolean-design.md`; `agent-docs/interference-design.md` §3.4) measures an
admitted coplanar prism pair's overlap outright. Neither emits a
`DiagUnsupportedPairContact`.

**What each boolean refusal leaves the caller.**
"Choose a construction that does not lean on a tangent contact" names a move
without naming the move, so the two refusals a modelling caller actually meets
are stated here concretely.

**The coplanar contact.** The analytic reduction
(`agent-docs/prism-boolean-design.md`) admits co-directional straight prisms
sketched on one plane, or on a datum plane and its `CreateOffsetPlane`, to
`Union`, `Cut` and `Intersect`, so their overlapping caps build rather than
refuse. Every pair outside that admitted class stays on the mesh path. There, two bodies
extruded from ONE sketch plane to one end plane have coplanar caps by
construction, and where their footprints OVERLAP those caps share positive
area, which is the contact the exact predicates cannot classify. Coplanar caps
on their own are not a contact: two such bodies standing apart in the plane
share no cap area, and every boolean over them runs.

A document with a gap tolerance (`agent-docs/gap-tolerance-design.md` §5.2,
0.01 mm by default) runs such a boolean again with the second operand moved by
up to that tolerance, along a direction that opens every coplanar contact. The
same retry resolves an edge grazing along a facet and an isolated point touch,
so bodies meeting along an edge or at a corner join in a `Union`. A
`Union` of bodies touching face on face then builds one body, a `Cut` of such a
body leaves its target, an `Intersect` of them is empty, and a tool flush with
its target's wall cuts through it. The result reads `Approximate` and its
bounds cover the move. `ExactOnly` keeps the refusal below, and so does a
tolerance whose shift no direction can apply, such as a tool flush with both
walls of a slot.

Under `ExactOnly`, or when the retry cannot apply, the contact is
`BooleanUnsupportedContact`. What replaces it is an INTERIOR overlap: no face
pair coplanar, and each operand reaching into the other's interior, so every
contact is a transversal crossing the evaluator does classify. Both operands span the same interval
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

**The chain depth.** A boolean's result is a `Faceted` body whose held bound
is composed per vertex from the operation that made it
(`agent-docs/faceted-vertex-bounds-design.md` §3): a surviving operand vertex keeps
its operand's bound, a vertex the boolean creates carries its own facet pair's
trim amplification, and each adds its own final rounding displacement. §9's
chord tolerance for the next pair is a fixed fraction of that pair's diameter,
raised to a coil's, mitred sweep's, or certified `LoftFaces` result's held
bound when that is larger, since those operands hold one fixed mesh. The pair tolerance is the gate's
reference for those held operands only: an analytic partner is meshed at its
own diameter-derived tolerance, never coarser than the pair's
(`agent-docs/chained-cut-design.md` §4.1). A certified twisted `Sweep` also
restates a finer mesh than its own at a tenth of the stationed build's target
(§4.2), so a chain of twisted bore cuts through one tube touches no earlier
bore's rim.
The next boolean compares the two only where the new pair meets the operand
(`agent-docs/faceted-vertex-bounds-design.md` §5): every facet of an operand that
restates a held mesh (a boolean result or a mitred sweep) that the contact
classification reports as meeting the other operand, or as coming within the
tangency gate's slack of it, must carry a bound no coarser than the pair's
chord tolerance. A chorded analytic operand is meshed at that tolerance and
is not compared. Where one is coarser, the
boolean is refused before any facet is cut, with a plain `ErrUnsupported`,
not a `BooleanError`, since the limit is the bound the operand holds there.
A refused two-body `Union`, `Cut` or `Intersect` with a mesh Boolean result as an operand is then
rebuilt from the source bodies the operation history names, under a set identity
that leaves the result unchanged, when every pair but one takes an analytic route
(`agent-docs/boolean-chain-regroup-design.md`). The result is the same set, its history
is the single call, and a selector naming a source feature's origin may match fewer faces.
Any other refusal stands.
The refusal names the operand (`Cut`'s target or tool; the first or second
operand otherwise), quotes the bound of the facets the pair touches and the
pair's chord tolerance, and states that a boolean takes no chord tolerance:
there is nothing to retry with. The document's gap tolerance is not that
parameter and does not lift the refusal. The document's chain tolerance
(`decad.WithChainTolerance`) does: a touched bound within it is admitted, every
bound the result carries is still proven, and the result's `Bound` reads the
larger figure that admits. Geometry the new pair does not touch keeps the bound it
already published and is never the reason for a refusal, so a result whose
`Bound` exceeds the next pair's chord tolerance in untouched regions is still
an ordinary operand. What limits a chain is therefore where its contacts land:
a contact on an earlier rim whose bound has outgrown the pair tolerance is
refused, and a contact elsewhere continues. A restating operand therefore
contributes at most the pair's chord tolerance `tol` to any rim, as a freshly
chorded operand does, so a rim between two such facets carries at most
`2·tol/sin θ` of its own facet pair. Where the comparison does refuse, it is geometry rather than an
argument the caller got wrong. The booleans take no chord-tolerance parameter (§9),
by the same decision that puts the tolerance's whole effect on the result's
proven `Bound`. The bounds are readable rather than merely printed in the
refusal: each `Vertex`, edge and face of a `Faceted` body reports its own
bound, and `Body.Tessellate` at any tolerance the faceted body already meets
returns a `Mesh` whose `Bound` is the largest of them, so a caller can see
which regions carry coarse rims before planning the next contact. A
construction needing many booleans through one region has to be reshaped, not
retried.

Modify operations return a new body, retiring the receiver, on the same terms:

```go
func Fillet(ctx context.Context, b *Body, sel EdgeSelector, r units.Value, opts ...FilletOption) (*Body, error)
func Chamfer(ctx context.Context, b *Body, sel EdgeSelector, d units.Value, opts ...ChamferOption) (*Body, error)
func Shell(ctx context.Context, b *Body, sel FaceSelector, thickness units.Value, opts ...ShellOption) (*Body, error)
func ChamferAngle(ctx context.Context, b *Body, sel EdgeSelector, d units.Value, reference FaceSelector,
    angle units.Value) (*Body, error)
func FilletChord(ctx context.Context, b *Body, sel EdgeSelector, chord units.Value) (*Body, error)
```

`FilletChord(ctx, b, sel, chord)` rounds right-angled planar edges with the radius `chord/√2` and builds the result with
`Fillet`. `agent-docs/fillet-chord-design.md` owns its rules.

`ChamferAngle` sets back `d` on `reference` and `d·tan(angle)` on the other face of each right-angled planar edge, and
builds the result with `Chamfer`. `agent-docs/chamfer-angle-design.md` owns its rules.

Each bounds cancellation latency in its cancellable construction
and audit paths, including Shell's shared offset-construction, crossing, and
nesting audit. Cancellation returns `ctx.Err()` before commit, so the receiver
stays live and the document remains unchanged.

Modify reach options state modeling intent, not evaluator switches:

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
`agent-docs/modify-reach-design.md` owns exact receiver/target limits, refusal order,
payloads, and validation.

Surface operations build, close and thicken a **sheet body** — a body that
encloses no material region (§6, `BodyKind`). `Thicken` returns a solid;
`agent-docs/surface-design.md` is normative for all these operations:

```go
type SurfaceResultOption interface {
    ExtrudeOption
    RevolveOption
    SweepOption
    LoftOption
}

func WithSurfaceResult() SurfaceResultOption

func PatchProfile(ctx context.Context, d *Document, s *sketch.Sketch, p *sketch.Profile) (*Body, error)
func PatchEdges(ctx context.Context, b *Body, sel EdgeSelector) (*Body, error)

func Stitch(ctx context.Context, bodies ...*Body) (*Body, error)
func Unstitch(ctx context.Context, b *Body) ([]*Body, error)
func Thicken(ctx context.Context, b *Body, thickness units.Value, opts ...ThickenOption) (*Body, error)
func WithThickenSide(side ThickenSide) ThickenOption

// The chain-fed forms. Each sweeps an OPEN sketch curve and always returns a
// BodySheet, so WithSurfaceResult() is not among their options: ChainLoftOption
// is LoftChain's own sealed tier, which SurfaceResultOption does not implement.
// Extrude, Sweep and Revolve take a chain through Section instead (above).
func LoftChain(ctx context.Context, d *Document, s0 *sketch.Sketch, c0 *sketch.Chain, s1 *sketch.Sketch, c1 *sketch.Chain, opts ...ChainLoftOption) (*Body, error)
```

`ThickenSide` is `ThickenPositive` (default), `ThickenNegative` or
`ThickenCentered`; `agent-docs/surface-design.md` §16 owns the receiver and proof
limits. `WithSurfaceResult()` omits the faces that exist only to close the
solid and changes nothing else — a wall keeps its surface, its role and every bound.
`PatchProfile` builds a planar face from a recorded profile and consumes
nothing; `PatchEdges` fills a closed chain of the body's free edges and
retires it. `Stitch` retires every operand and `Unstitch` its operand, on §6's
uniform terms, and neither takes a `*Document` because a `*Body` carries its
own. `Stitch` closes an existing face assembly; `Thicken` builds a new skin
and rim from an admitted sheet. `Stitch` joins two free edges where
coincidence is proven and, under the document's gap tolerance
(`agent-docs/gap-tolerance-design.md` §5.3), where straight edges on planar
faces miss by no more than it. The joined vertex holds one position and the
miss is charged to the result's bounds, so no `isTolerant` topology §2.1
rejects is recorded.
An edge it cannot join stays free, which is a result rather than an error, so
the caller reads `Edges(Free())` to see what did not close.

**A chain-fed form takes the sketch beside the chain, on §7's terms, and
consumes nothing.** All four (`Extrude`, `Sweep` and `Revolve` over a chain included) build the part of
their reach `agent-docs/surface-design.md` §1.2 states: a chain `Sweep` sweeps one
straight path span and `LoftChain` rules a `LineSeg`-only correspondence between
two exactly-parallel walks, and each refuses the rest at the call with
`ErrUnsupported`. All four run §7's four gates over
the internal chain recording path and return the same four sentinels a profile
earns, and there is
no chain-fed `PatchProfile`: an open walk encloses no region to fill, and a
walk whose ends met is a `sketch.Profile`.

Transform is a body operation on the same terms: it retires the input and
registers the transformed body.

```go
func Transform(ctx context.Context, body *Body, t r3.Transform) (*Body, error)
func ScaledAt(ctx context.Context, b *Body, anchor r3.Vec, factor units.Value) (*Body, error)
```

`Arrange(ctx, bodies, bed)` places several bodies on a flat bed by one `Transform` each (two for a body that `WithArrangeTurns` turns), in shelves of their bounding
boxes, and `agent-docs/arrange-design.md` owns its rules.

This is the whole of the "explicit transforms" story: a body is positioned by an
argument the caller states — an `r3.Transform`, a rigid motion (§5.2) — never by
an ambient assembly context (§4). The zero `Transform{}` is invalid
(`Transform.IsValid`) and is `ErrDegenerate` (§12). `Transform` checks `ctx` through any
faceted-payload rebuild and returns its error without changing the document.

`ScaledAt` retires an authenticated point-loft planar Split slab, an exact
line/circular solid prism, a solid Revolve, complete or partial, whose section has no
displacement, an exact line-only stacked prism or stack-backed BRep, a
faceted result of the generic mesh Boolean, or any other faceted solid whose
held mesh passes its audit, after a positive, finite,
dimensionless uniform scale around `anchor` in millimetres. The other faceted
solids include `LoftSections` and `LoftFromPoint` results and retained
`Fillet` or `Chamfer` results of a mesh Boolean. It accepts
rigidly placed and previously scaled copies of these classes. A Split slab
keeps its original cap origins, face-centroid certificates and rational source
cut maps. A prism or Revolve keeps its source-face roles and planar tags, while
curved faces become `Faceted`; later operations that require an analytic source
can refuse it. A stacked prism or stack-backed BRep keeps its source-face roles
and planar tags but loses its structural section record after scale. A faceted
Boolean keeps its source-face roles, its planar tags and its retained
cylinder, sphere and torus tags, but loses structural Boolean source records
after scale. Any other faceted solid keeps its source-face roles, its planar
tags and its retained cylinder, sphere and torus tags, such as those of a
retained `Fillet` or `Chamfer`; its retained cones, denoted faces and
unretained curved faces become `Faceted`, and it loses its structural source
records, such as a point loft's source Sketch. These
classes rebuild the held mesh with scaled vertex, area and occupied-volume
bounds. A nonpositive factor is `ErrDegenerate`, and a length factor is
`ErrUnitKind`. A sheet, including a sheet Revolve, a faceted solid whose mesh
fails its shell or crossing audit, and any other unsupported body return
`ErrUnsupported`. Cancellation and proof refusal leave the receiver live.

**A body can be copied without consuming it.** `Transform` retires its input, so
modelling a part once and placing several instances — a bolt pattern, several
interference placements, one cutting tool reused for several holes — would mean
rebuilding the whole feature chain per instance. `Copy` closes that gap:

```go
func Copy(ctx context.Context, source *Body, opts ...CopyOption) (*Body, error)
func WithCopyPlacement(t r3.Transform) CopyOption
func WithCopyRotation(center, axis r3.Vec, angle units.Value) CopyOption
```

`Copy` returns a NEW live body and leaves its source **live**: the source is
depended on, never consumed. With no options, it re-registers the source's
immutable payload under a new producer identity at the same placement. With
`WithCopyPlacement(t)`, it re-evaluates the payload under the composed rigid
motion exactly as `Transform` does. Repeated placement options use the last value.
The zero `Transform{}` is `ErrDegenerate`; `r3.Identity()` is a valid no-op.
`WithCopyRotation(center, axis, angle)` places the copy by the rotation about the axis
through `center`. `WithCopyPlacement` and `WithCopyRotation` are both placement
options, so the last of either wins. A zero or non-finite axis is `ErrDegenerate`, a non-finite
center or angle is `ErrNotFinite`, an angle that is not an angle is
`ErrUnitKind`, and a zero angle is the identity. A faceted copy made with it
denotes an exactly orthogonal rational rotation about exactly that axis, chosen
near `angle`: it agrees with `angle` to within a few float steps and is not in
general the rotation by exactly that transcendental angle. The copy's held
vertices are the ones `WithCopyPlacement` of `r3.RotationAround(center, axis,
angle)` would hold, and their bounds charge the distance to the exact rotation.
Every other payload is placed by `r3.RotationAround(center, axis, angle)`,
exactly as `WithCopyPlacement` of that transform.
An exact faceted copy carries no analytic surface tags: every face reads as
`Faceted`, so `Face.NormalAt` refuses it, and `ToFace` stops and `MirrorFace`
refuse it. It also stops being a point-loft source and a loft mirror join copy,
so cone trims and the loft mirror cylinder joins treat it as any other faceted
body. A copy that needs those readings uses `WithCopyPlacement`.
`VerifyMotion` and `VerifyLinkage` refuse to re-place an exact faceted copy, or
any body that descends from one by `Copy`, `Transform` or `ScaledAt`, with
`ErrUnsupported`: the check has no pose proof for a re-placed exact rotation.
The refusal applies where a mover is placed, so a mover that is fixed, or that
has no evaluated pair, does not reach it. A copy made with `WithCopyPlacement`
is not affected when its source has no preceding exact placement; a source
that descends from an exact copy keeps the mark through every later copy.
A nil option is `ErrDegenerate`, and a body this evaluator did not build is
`ErrUnsupported`. Cancellation during a faceted rebuild leaves the source
and live-body set unchanged.

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
  upstream `FeatureRef` is a `FaceCreatedBy` provenance origin, not a reference
  to the copy: a boolean's faces already carry the operands' upstream origins, not
  a role keyed to the boolean itself (§9), so keeping it verbatim is outside
  the never-inherited rule — that rule governs the new references a copy DOES
  re-derive from its own record, `Body.Origin()` (the `body` role, below) and the
  analytic/modified feature roles. A certified Loft mirror `Union` is a bounded
  faceted exception: it mints two outer-cap roles, so a placed copy keeps the
  source origins and also mints those two roles under the copy's identity;
- an **analytic** or **modified** copy (a prism, revolve, cup, tube) carries no
  separate upstream provenance — a prism cap is created by the prism step itself
  — so re-evaluation reproduces the body's own fixed roles (`capStart`, `capEnd`,
  `side(i, j)`, …) from the same record, keyed to the copy's own private producer
  identity, and the copy's own
  `CapStart` / `CapEnd` / `FaceCreatedBy` resolve against it;
- the `body` role — `Body.Origin()` — uses the copy's own private producer
  identity in every case.

Copies do not consume their source. Each successful copy gets a fresh private
producer identity so callers can use its returned provenance values without
persisting or constructing internal identifiers.

**A hole is one call.** `Hole(ctx, b, HoleSpec)` cuts a round, rectangular or slot-shaped, through or blind hole, with an optional
counterbore or countersink head on a round hole, at a world point on a planar face of `b`, or radially at a point on a cylindrical one. It builds the tool bodies, cuts them from `b` in one
atomic commit, and retires `b`. `agent-docs/hole-design.md` owns the placement, size and refusal rules.

**A rib is a thickened line.** `Rib(ctx, b, s, line, thickness, e)` thickens one straight sketch line across its width,
extrudes the outline by `e`, and joins it to `b`; `Web` does the same for several lines in one commit.
`RibArc(ctx, b, s, arc, thickness, e)` does the same for one sketch arc, thickened into an annular band.
`WithRibOverrun` buries the ends of a rib in the walls it spans. `agent-docs/rib-design.md` owns the rules.

**A relief is a profile on a face.** `Emboss(ctx, b, s, profiles, face, mode, height)` extrudes closed profiles of a sketch
that lies in the plane of a planar face, outward for `EmbossRaised` and inward for `EmbossEngraved`, and joins them to
`b` with `Union` or `Cut` in one commit. `agent-docs/emboss-design.md` owns the rules.

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
type ToFace struct { Body *Body; Face FaceSelector; Offset units.Value }

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
type ToFaceAngular struct { Body *Body; Face FaceSelector }

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
`ToFaceAngular.Body`, and `EdgeAxis.Body`. At a feature call that body must be a
live `*Body`, and `Face` is
resolved as `Face.SelectFaces(Body)` and nothing else. The rules are the same
everywhere:

- the selector MUST resolve to **exactly one** face — that is what
  `FaceQuery.Exactly(1)` is for. Zero faces or more than one is `ErrCardinality`
  (§12); "extrude up to that face" is meaningless when "that face" is four faces;
- `Body` MUST be a live body of the same `Document` the feature is called on.
  Another document's body is `ErrForeignBody` (§12) — a face in another document's
  coordinates would stop the sweep somewhere the caller never named — and a body the
  document has retired is `ErrRetiredBody` (§6);
- the feature **depends on** that body. It is not consumed or retired.

Options carry the rest (`WithTaper(units.Degrees(3))`), via
`github.com/lestrrat-go/option/v3` — the house functional-options library, and an
approved dependency. Every option must have one clear, validated effect on the
feature call and must leave the document unchanged when validation fails.

## 9. Selectors — intent, not identity

The topological-naming problem, in Fusion's own words: `tempId` is valid only
"as long as the owning BRepBody is not modified in any way", and entity tokens
"should never be compared". Handles do not survive an edit, and index order is not
stable. Its `isTangentChain` flag is the workaround; we take the lesson properly.

**A feature is given a query, never a pointer.**

Features accept the **interfaces**; the constructors return the **concrete query
types** that implement them and that carry the cardinality assertions. Both
interfaces are sealed independently, so features accept only package-defined
selector values.

```go
type EdgeSelector interface {
    SelectEdges(*Body) ([]*Edge, error)
}

type FaceSelector interface {
    SelectFaces(*Body) ([]*Face, error)
}

func Edges(preds ...EdgePredicate) *EdgeQuery
func Faces(preds ...FacePredicate) *FaceQuery

type EdgeQuery struct{ /* ... */ }

func (q *EdgeQuery) SelectEdges(*Body) ([]*Edge, error)
func (q *EdgeQuery) Or(preds ...EdgePredicate) *EdgeQuery // adds a union branch
func (q *EdgeQuery) Exactly(n int) *EdgeQuery   // errors at resolve unless exactly n match
func (q *EdgeQuery) AtLeast(n int) *EdgeQuery

type FaceQuery struct{ /* ... */ }

func (q *FaceQuery) SelectFaces(*Body) ([]*Face, error)
func (q *FaceQuery) Or(preds ...FacePredicate) *FaceQuery
func (q *FaceQuery) Exactly(n int) *FaceQuery
func (q *FaceQuery) AtLeast(n int) *FaceQuery

// Predicates compose.
func Convex() EdgePredicate
func Concave() EdgePredicate
func Reflex() EdgePredicate                  // Edge.Wedge() == WedgeReflex
func Salient() EdgePredicate                 // Edge.Wedge() == WedgeSalient
func ParallelTo(v r3.Vec) EdgePredicate
func EndpointAt(p r3.Vec) EdgePredicate // either endpoint equals p in stored coordinates
func LongerThan(l units.Value) EdgePredicate
func CreatedBy(f FeatureRef) EdgePredicate   // provenance
func OuterLoopOf(f FeatureRef) EdgePredicate // outer boundary of a face with this provenance
func Circular() EdgePredicate
func Free() EdgePredicate                    // exactly one adjacent face — a sheet's boundary

func Planar() FacePredicate
func Cylindrical() FacePredicate
func NormalTo(v r3.Vec) FacePredicate          // planar face whose normal is parallel to v — EITHER sense
func Facing(v r3.Vec) FacePredicate            // planar face whose OUTWARD normal points ALONG v — ONE sense
func FaceCreatedBy(f FeatureRef) FacePredicate // provenance, the face analog of CreatedBy
```

`Convex()` and `Concave()` read `Edge.IsConvex` (§6.1): the walked-boundary
convexity, not the material angle across the edge. A hole's rim edges are
concave, so a fillet meant for them asks for `Concave()`.

`Reflex()` and `Salient()` read `Edge.Wedge` (§6.1), the material angle, so
`Fillet(Edges(Reflex()), r)` rounds a boss root or an inside corner of a
`Union` result, where `Concave()` matches nothing. An edge whose wedge is
`WedgeUndecided` matches neither, so `Reflex()` and `Salient()` together need
not cover a body's edges.

`OuterLoopOf(CapStart(body))` matches only edges on the start cap's outer
boundary. It excludes hole loops by `Loop.IsOuter`, so a cap chamfer can
leave a bore rim sharp. The predicate matches no edge when the referenced
face has no outer loop or is absent from the receiver.

`Free()` reads `Edge.IsFree` (§6.1) and composes with every other clause, so
`Edges(Free()).Exactly(8)` asserts a surface extrude's rim count and
`Edges(Free(), Circular())` picks the circular ones. On a solid, and on a
closed sheet, it matches nothing — an ordinary `ErrNoMatch`, or
`ErrCardinality` under an assertion, never an error in itself.

`EndpointAt(p)` matches an edge when either stored endpoint position equals
`p` component-wise, with no distance tolerance. Non-finite `p` fails at
resolve. A ribbon's two free sweep edges share the same direction, length,
convexity and face provenance; `EndpointAt` distinguishes them by their
recorded end positions. Combine it with `Free()` and `ParallelTo` to name
one end for `Extend`. A placed or rounded model needs the position its live
topology reports, not an assumed exact mathematical point.

**`Facing(v)` is the signed one-face predicate.** `NormalTo(v)` matches a planar
face on either normal sense, so a slab's two parallel caps both match `NormalTo(z)`
— the right answer when a caller wants both, the wrong one when they mean the top
only. `Facing(v)` matches only the face whose OUTWARD (material-leaving) normal
points along `v` — parallel to `v` **and** the same sense, a positive projection —
so `Faces(Planar(), Facing(z))` picks the single top cap that
`Faces(NormalTo(z))` returns as a pair. A zero or non-finite `v` is rejected at
resolve, as `NormalTo`'s is (§12).

**Typed role helpers keep the fixed roles out of string literals.** A provenance
selector names a `FeatureRef` (§6.1), and the roles a feature mints — `capStart`,
`capEnd`, `body` — are a public part of the contract, but hand-typing `"capStart"`
invites a typo the compiler cannot catch. The fixed roles get typed constructors:

```go
func CapStart(b *Body) FeatureRef // the start-cap role of b's producing operation
func CapEnd(b *Body) FeatureRef   // the end-cap role of b's producing operation
```

`FaceCreatedBy(CapStart(body))` selects the start cap without the caller writing
the string; each helper pairs the body's private producer identity with its
fixed role. The body-role reference is already `Body.Origin()`, so no helper
duplicates it.

**The cap roles exist only where the producing operation mints them.** `CapStart(b)` /
`CapEnd(b)` name `b`'s own producer, so they resolve to a face only when
that operation actually mints the fixed cap role: an extrude, a partial revolve, a
shell that built a tube — the analytic prisms and partial revolves whose
evaluator emits `capStart` / `capEnd` — an admitted analytic `Union`, which
rebuilds an analytic prism with fresh roles under its own identity, an
authenticated axial Loft/cylinder `Union` or its centered bore `Cut`, whose
audited planar caps also get fresh roles, an authenticated axial Loft mirror
`Union` with two retained outer caps, the modified full revolution made by
a positive-taper circular draft rim fillet (draft §10.4), whose two planar
disks get cap roles, and a `Transform` or
`Copy` of one, which re-mints those roles under the copy's
own identity (the copy-provenance rule above). On a body whose producer mints no such
role the helper still returns a well-formed `FeatureRef` — it just matches
nothing, an ordinary `ErrNoMatch` at resolve (or `ErrCardinality` under an
implicit exactly-one). That covers an ordinary mesh-path `Union` / `Cut` / `Intersect`
result (its faces carry the operands' UPSTREAM origins, not a cap role keyed to
the boolean's own identity), an ordinary full revolution (no caps at all), and `CapEnd` on a
cup (a cup mints `capStart` and a `shellCap` pocket floor, no `capEnd`). To name
a cap that an ordinary mesh boolean's operand contributed, select
`FaceCreatedBy(CapStart(originalBody))` against the upstream body whose
provenance the mesh boolean preserved (§9).

The **indexed** `side(i, j)` roles stay positional and get no helper: a wall
is named by geometry — `Faces(Planar(), Facing(v))` — never by a hand-counted
loop/segment index, so a fillet that re-segments a section can never silently
invalidate a hand-typed number, and the one selection style that survives a
rebuild is the geometric one selectors exist to provide.

```go
body, err = decad.Fillet(ctx, body,
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

**A query is a union of branches, and each branch is a conjunction.**
`Edges(preds...)` / `Faces(preds...)` start the query with one branch.
Each `Or(preds...)` call appends one more branch and returns the receiver, as
`Exactly` and `AtLeast` do. An entity matches the query when it satisfies every
clause of at least one branch. A branch with no clauses matches every entity, as
`Faces()` does. Resolution keeps the topology accessor's order, so an entity
that two branches match appears once and counts once. `Exactly(n)` and
`AtLeast(n)` assert the size of that final set, whether they are called before
or after `Or`. Every branch's predicates are validated at resolve before any
entity is examined.

A partial-turn revolve whose angular caps are not coplanar needs this: no
single conjunction matches both caps without also matching the planar end
faces of its meridian. Its shell names the two caps by provenance:

```go
q := decad.Faces(decad.FaceCreatedBy(decad.CapStart(body))).
    Or(decad.FaceCreatedBy(decad.CapEnd(body))).
    Exactly(2)
```

The union lives on the query, not on a predicate. The rejected alternative is a
predicate combinator, `AnyOf(preds ...FacePredicate) FacePredicate`. Go has no
overloading, so it would need a separately named edge twin. A branch that is
itself a conjunction would need a second combinator, `AllOf`, to nest inside it.
And a nested clause has no single place in the per-clause running count that
`SelectionError.Residuals` reports. `Or` adds one method per query type, and
every existing consumer that type-asserts `*EdgeQuery` / `*FaceQuery` (the
modify ops, `ToFace`, `EdgeAxis`) accepts a union unchanged.

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
    Branch    int    // the clause's branch: 0 for the Edges/Faces clauses, i for the i-th Or
    Predicate string // the clause's stable rendering — "convex", "parallel_to(0,0,1)"
    Remaining int    // candidates still matching after this clause AND every earlier clause of its branch
}

type SelectorKind int // EdgeSelectorKind / FaceSelectorKind; a stable String()
```

`Residuals` is each branch's conjunction evaluated cumulatively in query order:
`Remaining` starts at the body's whole edge or face count and can only fall, and
the clause whose `Remaining` reaches zero is the one that emptied that branch.
A union lists its branches one after another, in branch order, and each branch
starts again from the whole count; a branch with no clauses contributes no
entry. `Error()` names the clause that emptied the set. For a query with more
than one branch it names the emptying clause of each branch that emptied, with
the branch index: `the clause circular of branch 1 matched none`. It is diagnostic
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
and a verification `Diagnostic.Message` (`agent-docs/verification-design.md` §1.1)
reuse:

```go
func (q *EdgeQuery) String() string
func (q *FaceQuery) String() string
```

The rendering is a canonical, deterministic function of the query's content:
equal query values render identically. It exists for diagnostics, not as a
parseable or persistent format.

`q.String()` is `<kind>(<pred>, <pred>, …)<branches><cardinality>`:

- **kind** is `edges` for an `EdgeQuery`, `faces` for a `FaceQuery`.
  (`SelectionError.Kind`'s `SelectorKind.String()` is the
  singular `edge` / `face`, naming the entity; the query prefix is plural and the
  error's kind field singular, deliberately distinct.)
- **preds** are the first branch's clauses in query order, `, `-separated; no
  clause renders `edges()`.
- **branches** is one `.or(<pred>, <pred>, …)` per `Or` call, in call order,
  with the same clause list; an `Or()` with no clause renders `.or()`.
- **cardinality** is empty for no assertion, `.exactly(<n>)` for `Exactly(n)`,
  `.at_least(<n>)` for `AtLeast(n)` — the codec's `exactly` / `at_least` keys.
  It always renders last, after every branch, because it asserts the union's
  size. (`SelectionError.Expected` renders the same assertion in prose —
  `exactly <n>`, `at least <n>`, or `any` for none — a human line, not this
  suffix.)

Each predicate renders by its codec kind token and payload:

| Predicate | Rendering |
|---|---|
| `Convex()` / `Concave()` / `Circular()` | `convex` / `concave` / `circular` |
| `Reflex()` / `Salient()` | `reflex` / `salient` |
| `Free()` | `free` |
| `Planar()` / `Cylindrical()` | `planar` / `cylindrical` |
| `ParallelTo(v)` | `parallel_to(<vec>)` |
| `NormalTo(v)` / `Facing(v)` | `normal_to(<vec>)` / `facing(<vec>)` |
| `LongerThan(l)` | `longer_than(<value>)` |
| `CreatedBy(f)` / `FaceCreatedBy(f)` | `created_by(<ref>)` / `face_created_by(<ref>)` |
| `OuterLoopOf(f)` | `outer_loop_of(<ref>)` |

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
- **`<ref>`** is a `FeatureRef` as `<producer>:<role>` — an opaque document-local
  producer number and the role QUOTED with `strconv.Quote`, so a role that itself holds parentheses or
  commas stays unambiguous: `created_by(3:"capStart")`, `created_by(2:"side(0,1)")`.
- a zero-value, kind-less predicate — which the constructors never produce, but a
  half-decoded query might carry — renders `<invalid>` rather than panicking.

So `Faces(Planar(), Facing(z)).Exactly(1)` renders
`faces(planar, facing(0,0,1)).exactly(1)`, and
`Edges(Convex(), ParallelTo(z)).Exactly(4)` renders
`edges(convex, parallel_to(0,0,1)).exactly(4)`, and
`Faces(FaceCreatedBy(CapStart(b))).Or(FaceCreatedBy(CapEnd(b))).Exactly(2)`
renders `faces(face_created_by(3:"capStart")).or(face_created_by(3:"capEnd")).exactly(2)`
when `b`'s producer is 3.

This is also why the decad code and the eventual Fusion code stay structurally
parallel: a real Fusion script must pick edges by geometric predicate too.

## 10. Verification — the product

The 3D counterpart of `sketch.Verify`. One non-mutating call; a rich report; a
`Status` at both the body and the document level, aggregated by a fixed severity
precedence; and one bit an agent gates on. Deliberately mirrors
`sketch.VerificationReport` / `WorldVerificationReport`.

```go
func (d *Document) Verify(ctx context.Context, opts ...VerifyOption) (*Report, error)

func (r *Report) Passed() bool                            // the single bit to gate on
func (r *Report) ForBody(body *Body) (*BodyReport, error) // exact-identity lookup
```

The report vocabulary — `Report`, `BodyReport`, the per-survey result records
(`WallResult`, `UndercutResult`, `ConcaveRadiusResult`, `ValidityResult`,
`RegionReadings`), `Status` and its severity precedence, and the
`VerifyOption` set including `WithTolerance` — and the tolerance gate that
judges every `Exactness` and `Bound` the report carries are specified in
`agent-docs/verification-design.md`. Its §1.2 also owns `Verify`'s public cost
warning and caller-deadline guidance. The non-mutating proof and bounded
volume behind every `Interference` row are specified in
`agent-docs/interference-design.md`. `Status` reserves zero as `Unverified`, so a
zero-value `Report` fails `Passed`; every report and body report returned
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
  exempt — and `Report.Passed()` is true only when the whole report is
  `Sound`; it does not imply that every optional survey ran.
- **`Verify` returns structured diagnostics.** On a report returned by
  `Verify`, `Report.Diagnostics` is one
  branchable `Diagnostic` per reason the report is not `Sound` — a reading
  beyond tolerance, a reading with no usable tolerance reference, an undecided
  validity, an undecided or staged pair, an undecided or unsupported survey, a
  survey blocked on a body's own invalid or undecided validity, an undecided
  clearance, a proven wall or undercut violation, an interference. Every
  optional-body-survey reason carries `Diagnostic.Survey` (`SurveyWall` /
  `SurveyUndercut` / `SurveyConcaveRadius`), set even when the reason names no
  bounded reading, so a caller distinguishes which survey failed without
  parsing `Message` text; every core or pair reason carries `SurveyNone`. The
  slice is empty exactly when the report is `Sound`, and the same equivalence
  holds per body for `BodyReport.Diagnostics`, so an agent reads the reasons
  instead of reconstructing them.
  `agent-docs/verification-design.md` §1.1 owns its shape.

`agent-docs/collision-dynamics-design.md` specifies pair-contact and two-body sweep
queries and the separate dynamics subpackage. Their source-box and vertical
rebound slices are available; other cases remain staged.

Fusion answers **none** of a single validity verdict (with diagnostics), an
explicit wall-survey outcome and interval (B-rep), a confirmed-undercut list
with its own coverage state, or a concave-radius outcome and interval. That
gap is decad's mandate.


## 11. Export and translation

```go
func (b *Body) Tessellate(ctx context.Context, tol units.Value, opts ...TessellateOption) (*Mesh, error) // an OUTPUT, not the representation
// package export:
func STL(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, opts ...decad.TessellateOption) error
func OBJ(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, opts ...decad.TessellateOption) error
func ThreeMF(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, opts ...decad.TessellateOption) error
func STEP(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, opts ...STEPOption) error
func WithSTEPName(name string) STEPOption
func WithSTEPTimestamp(timestamp time.Time) STEPOption
func WithSTEPAuthor(author string) STEPOption
func WithSTEPOrganization(organization string) STEPOption
func WithSTEPHeader(header step.Header) STEPOption
```

`agent-docs/tessellation-design.md` is normative for the mesh these calls consume and
return: shared boundary samples, two-sided `Mesh.Bound`, source-face mapping,
area slack, the symmetric-difference proof a boolean requires, the
`Verification` level each proof is gated on, and explicit
per-payload staging. A payload with no complete boundary proof is never
exported. A mesh without the separate occupied-volume proof is never admitted
to a boolean by an unproved generic bound.

**`Tessellate` defaults to `VerifyAll`; `export.STL`, `export.OBJ`, and
`export.ThreeMF` default to `VerifyNone`.** The caller supplies a positive length
chord tolerance to every writer. These three writers consume vertices and
indices but no proof term. All accept `WithVerification` to demand a stronger
proof and accept its refusals. An STL file from a sheet is not a solid; at
`VerifyNone`, a solid mesh is closed but not proven free of self-intersection.
`ThreeMF` writes a sheet as a 3MF `surface` object and a solid as a `model`
object. See `agent-docs/3mf-export-design.md`. `export.GLTF` writes the same
mesh as a glTF 2.0 file with an embedded buffer, in metres, and defaults to
`VerifyNone` like the others. See `agent-docs/gltf-export-design.md`. `export.PNG` draws the mesh as an
orthographic flat-shaded image from a `View`. See `agent-docs/png-export-design.md`. `export.DXF` writes the mesh's feature edges as
a 2D DXF drawing with hidden edges on their own layer. See `agent-docs/dxf-export-design.md`.

`Tessellate` and every writer pass `ctx` through chording, loop-clearance
scans, cap triangulation, mesh audits, and faceted restatement. Cancellation
returns `ctx.Err()` unchanged.

`export.STEP` writes a boundary-verified AP214 B-rep. It uses analytic planes,
cylinders, lines, and circles when the body's whole boundary fits that set;
other supported solids use the boundary-verified faceted mesh.
Simple callers pass the file name, author, and organization as options and
need no STEP module type. The timestamp defaults to the current UTC time, or
the caller sets it with `WithSTEPTimestamp`. `WithSTEPHeader` supplies the
complete `step.Header` to the same `export.STEP` function. The writer rejects
calls that combine it with field options; it does not fill missing header fields.
`export.NewSTEPFile` returns the underlying `step.File`.
The root package does not import STEP. STEP export preserves supported analytic
surfaces but does not preserve the caller's feature program. See
`agent-docs/step-export-design.md`.

**Fusion codegen is out of scope for v1.** Callers model in ordinary Go and use
the resulting bodies, measurements, and verification reports directly.

## 12. Errors and concurrency

- `(T, error)`. NEVER a `bool` return, a `-1` sentinel, or a deferred health state.
- Sentinel/typed errors for the cases an agent must branch on: `ErrNoMatch`
  (a selector carrying no cardinality assertion matched nothing), `ErrCardinality`
  (a cardinality assertion failed), `ErrForeignBody` (an operation was handed bodies
  owned by different documents, an extent or axis named a body owned by another
  document, §8.1), `ErrForeignProfile` (a feature was handed a profile built from
  a different sketch than the one given, or a boundary entity the source sketch
  does not own, §7),
  `ErrStaleProfile` (a feature was handed a profile built before the sketch's
  current state — `Profile.IsStale`, §7), `ErrRetiredBody` (an
  operation, or an extent, was handed a body the document has retired, §6),
  `ErrBodyReportNotFound`
  (`Report.ForBody` found no entry for the requested body, including a body
  foreign to the report's own document; `agent-docs/verification-design.md`),
  `ErrNegativeMagnitude` (a magnitude was
  given as a negative value; magnitudes are non-negative and sense is enumerated,
  §8.1), `ErrUnrecordableProfile` (a feature was handed a profile whose boundary
  contains a `Partial` fragment `sketch` could not certify — `TExact == false`,
  from a sampled arrangement cut or whole-sketch withholding — or one whose
  certified range the seam's falsifier disproves.
  A `Partial` fragment `sketch` certifies exact records as its entity's own
  variant with the certified range, and a whole edge of every kind records;
  full semantics in `agent-docs/sketch-seam-design.md`),
  `ErrNotSolid`, `ErrDegenerate`, `ErrBooleanFailed`, `ErrInvalidProfile` (the
  profile is invalid, contains a nil boundary entity, or no longer exactly
  matches a fresh current profile from its source sketch),
  `ErrUnitKind`, `ErrNotFinite` (a non-finite `units.Value` magnitude or
  `r3.Vec` component handed as a parameter, a non-finite value derived while
  validating a Revolve axis whose input is otherwise finite, or a non-finite
  measurement or bound derived by an analytic evaluator — `units` construction
  admits a non-finite value and only its operations reject one, so the call
  must; option semantics in `agent-docs/verification-design.md`),
  `ErrUnsupported` (the current evaluator cannot yet build the requested intent;
  staging is explicit and rejected at the call, never silently approximated or
  narrowed; `agent-docs/evaluator-design.md` §2). `BooleanError` carries a `Code`
  (empty / unsupported-contact / evaluator-failure), wrapping `ErrBooleanFailed`
  (empty, evaluator-failure) or `ErrUnsupported` (unsupported-contact); a valid
  tangent or coplanar contact this evaluator cannot classify is
  unsupported-contact, **not** `ErrDegenerate` (§8). `SelectionError` carries the
  selector kind, the query's stable rendering, the body, the expected/actual
  cardinality and the per-clause residual counts, wrapping `ErrNoMatch` or
  `ErrCardinality` (§9).
- **The four `*Profile` sentinels cover an open chain too.** Each names a CAUSE
  — a foreign source, a stale snapshot, a failed authentication, a range this
  seam cannot record exactly — and the caller's repair is the same whether a
  `sketch.Profile` or a `sketch.Chain` carried it, so there is no parallel
  `*Chain` set to branch on (§7; `agent-docs/surface-design.md` §7, §13.3).
- **`ErrUnitKind` covers exactly the wrong-`Kind` values.** A `units.Value` whose
  `Kind` is not the one the parameter takes: an angle where a length is wanted, and
  a `WithTolerance` value that is not `Dimensionless`
  (`agent-docs/verification-design.md`). It is never a coercion (§5.1).
- **`ErrNegativeMagnitude` covers exactly the magnitudes.** Those are `Distance.D`,
  `DistanceSide.D`, `Symmetric.D`, `AngleExtent.A`, `AngleSide.A`,
  `SymmetricAngle.A`, every fillet and chamfer radius or distance, every shell
  thickness, the `LongerThan(l)` edge-predicate length (§9), the `WithTolerance`
  relative tolerance, the `WithMinWallThickness` tool size and its
  `WithDraftAllowance` draft allowance (§10, `agent-docs/verification-design.md`), and
  the `Tessellate` tolerance (§11). Three `units.Value` parameters are **signed
  displacements, not magnitudes**, and are outside it: `ToFace.Offset`, which
  displaces along the target face's normal and whose sign says which side of that
  face the sweep stops on (§8.1); `ExtrudeOpts.Taper` (`WithTaper`, §8.1),
  whose sign says which way the wall leans; and `Draft`'s angle
  (`agent-docs/draft-design.md` §10), read the same way about its neutral plane. Neither carries a `Direction` to reverse,
  so a negative value there is a legal intent, not an error.
- **`ErrCardinality` takes precedence at zero matches.** A failed cardinality
  assertion is `ErrCardinality` **even when the selector matched nothing** — and
  that covers both the explicit assertions, `Exactly(n)` / `AtLeast(n)` (§9), and
  the *implicit* exactly-one of `ToFace` / `ToFaceAngular` (§8.1) and of `EdgeAxis`
  (§8.1). `ErrNoMatch` is
  reserved for the one remaining case: a selector that asserts no cardinality at
  all and resolves to zero entities.
- `Body` is immutable → safe to read from many goroutines.
- `Document` owns mutable state and is NOT safe for concurrent mutation. `Verify`
  is non-mutating and safe. The live body set alone is guarded, so `Bodies`,
  `Remove` and `Verify` may run concurrently with one another; a `Verify`
  reports on the set as it stood when the call began.

## 13. Non-goals for v1

Assemblies (`Component`/`Occurrence` instancing and the DAG that comes with it; a
`Container` that only groups members of one document is not one,
`agent-docs/container-design.md`), a
feature tree / timeline / rollback, native analytic STEP export, sheet metal, STEP, IGES
and 3MF import, turning an imported mesh into a `Body`, GUI or view state of any kind, and
Fusion code generation. STL and OBJ import into the separate `MeshBody` type is in
(`agent-docs/mesh-import-design.md`).

Sheet bodies themselves are **not** a non-goal — `agent-docs/surface-design.md` owns
them. `Thicken` admits the sheet families §16.1 names and `Offset` the two
§17.1 names; every other sheet family refuses at its call with
`ErrUnsupported`. `Trim`, `Extend` and `Split` build over a pair whose two
sweeps share one generator and refuse every other pair
(`agent-docs/surface-design.md` §1.1, §1.2); a sheet operand in any boolean refuses
permanently, and `Split` carries that intent instead
(`agent-docs/surface-design.md` Table X).
Ruled and Boundary Fill refuse permanently, because the surface each
needs is a fit through a boundary nothing generated (`agent-docs/surface-design.md`
§1.3). Reverse Normal takes no increment either: no shipped operation's result
turns on a side its caller cannot already name (`agent-docs/surface-design.md` §1.4).

The assemblies non-goal rests on a capability in hand, not on an instancing
graph: interference and clearance (§10) are computed between
**explicitly-placed bodies** — `Transform(ctx context.Context, body *Body, t r3.Transform)` (§8) — which needs
no `Component`/`Occurrence` machinery.
