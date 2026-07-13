# Verification Design

How `Document.Verify` judges what decad measures: the report it returns (§1),
the caller's tolerance and the relative gate (§2), the reference every bounded
result is judged against (§3), the noise floor that keeps the gate meaningful
at zero (§4), the three rules that fall out (§5), and the statuses the verdicts
aggregate into (§6). Companion to `docs/api-design.md` — the core design, which
owns `Verify`'s place in the API, the three bounded result shapes
(`Measurement`, `VecMeasurement`, `Box`), and the error vocabulary. References
of the form "core §N" are to that document.

## 1. The report

`Verify` is one non-mutating call returning a rich report — the 3D counterpart
of `sketch.Verify`, deliberately mirroring `sketch.VerificationReport` /
`WorldVerificationReport` (core §10):

```go
func (d *Document) Verify(ctx context.Context, opts ...VerifyOption) (*Report, error)

type Report struct {
    Bodies        []*BodyReport
    Interferences []Interference // proven pairwise overlap, with the overlap VOLUME; always computed
    Clearances    []Clearance    // WithClearances(): minimum gap between disjoint pairs
    Status        Status
}

func (r *Report) Trustworthy() bool // the single bit to gate on

type BodyReport struct {
    Body              *Body
    Status            Status       // Sound / Suspect / Violating / Unsound — this body only

    // Validity readings — facts of the boundary the evaluator holds, which is
    // exact as data; what they prove about the PART is Status's to say,
    // decided against the boundary's own proven bound (§6):
    Solid             bool
    Watertight        bool
    Manifold          bool         // every edge bounds exactly 2 faces
    SelfIntersecting  bool
    Lumps             int          // > 1 == disconnected pieces
    Voids             int          // internal cavities (Shell.IsVoid)

    // Boundary quantities — properties of the boundary the evaluator holds,
    // which every body has; always present:
    Area              Measurement
    Bounds            Box

    // Region quantities — properties of the enclosed region, which only a
    // valid solid has; non-nil exactly when the body is a proven solid (§6):
    Volume            *Measurement
    Centroid          *VecMeasurement // a computed coordinate, so it is bounded (core §5.3)

    Exactness         Exactness      // the weakest link across the quantities this report carries

    // Opt-in, expensive; nil unless the option asks AND the body is a proven
    // solid. MinWallThickness and MinRadius add a third leg: the feature must
    // exist (below):
    MinWallThickness  *Measurement   // WithMinWallThickness(tool, ...) — the thinnest wall: material
                                     // between skins opposing within the draft allowance (§6); nil when
                                     // no wall exists (below); decided against the tool (§6)
    Undercuts         []*Face        // WithPullDirection(v) — every entry a proven undercut: non-empty
                                     // is Violating; empty claims none exist, held to proof (§6)
    MinRadius         *Measurement   // WithMinRadius() — the tightest concave radius; nil when no
                                     // concave feature exists (below); the caller compares (§2)
}
```

**A quantity a body does not have is absent — nil — never zero.** Core §4
rejects `volume == 0` meaning both "empty" and "not a solid", and core §6 makes
`Body.Volume()` `(Measurement, error)` for exactly that reason; a struct field
has no error to return, so the report says it with presence, in the shape the
opt-in fields already use. A zero `Measurement` could not say it: its
`Exactness` is `Exact` and its `Bound` zero, so it reads as an exactly-zero
volume — the sentinel core §4 rejects, reintroduced one level up. The line
between the two groups is what a quantity is a property of. `Area` and `Bounds`
are properties of the body's **boundary** — the faces and points the evaluator
holds — and every body in the document has one, broken skin or not, so both are
unconditional. `Volume` and `Centroid` are properties of the **enclosed
region**, and a body that is not a valid solid encloses none: there is no
honest `Measurement` to put there. Both are therefore non-nil exactly when the
body is a **proven** solid — validity proven the §6 way, on the held boundary
beyond its own proven bound. A body proven invalid is `Unsound`; one whose
validity is undecided is `Suspect`; and neither carries a region quantity,
because neither has a region the report can vouch for. The opt-in quantities
are region quantities too — a wall, an undercut and an endmill's reach are
features of a solid — so each is computed only when its option asks for it
**and** the body is a proven solid, and is nil otherwise.

Two legs — the option asked, the body a proven solid — are the whole rule
exactly when the quantity's existence is already the precondition's: a
boundary always has an area and a box, and a region always has a volume and a
centroid. `Undercuts` carries its
further question — do any exist? — in the slice itself: asked on a proven
solid it is non-nil, every face it lists is a **proven** undercut, and
**empty** is the answer *no face is an undercut* — an answer and not an
absence, and an answer §6 holds to proof like any other, marking the body
`Suspect` when the evaluator cannot give one.
`MinWallThickness` and `MinRadius` each measure a feature a valid solid may
simply not have. A wall is material between **opposing skins** — two boundary
patches facing each other across it, within the draft allowance (§6),
read by the ball that spans them, skin to skin — and a body that is all
edge — a tetrahedron, whose skins everywhere meet at the body's own 70.5°
dihedral, past any allowance short of it (§6), and nowhere oppose — has no
wall at all, not a thick one. A concave radius is a feature an all-convex body — a plain
block — does not have. For neither can a `Measurement` honestly stand for
*none*. Zero is core §4's sentinel reintroduced: a real wall pinches to a
genuine `Exact` zero where its skins meet at tangency — a knife edge, a
cusp — on a body that is still a proven solid (§6 decides it thin against any
real tool), and a real concave radius can be arbitrarily tight, so zero is a
value each quantity itself approaches. (That zero is always a wall's, never a
body's: a body flat through and through encloses nothing, is no proven solid,
and carries no reading at all — the second leg already said so.) And an
infinity is not a measurement of anything the body has, and turns the §2 gate
vacuous. So both carry the existence leg the way the pair results below carry
their preconditions — in existence: each is non-nil exactly when the option
asks **and** the body is a proven solid **and** the feature exists. Nil with
the option asked on a proven solid is not a question left unanswered; it is
the determination *this body has no wall* — nothing exists for the tool to be
thinner than — or *this body has no concave feature* — the best possible
answer to the endmill question — and §6 holds the
evaluator to proving it, marking the body `Suspect` when it cannot. Nor do the
causes of a nil ever blur where a nil is read as an answer: which options were
passed is the caller's own knowledge, `Status` carries validity — proven,
refuted or undecided (§6) — and carries the survey §6 lets no evaluator
silently fail, as `Suspect`; inside a `Sound` report what remains is the
determination, and a report that is not `Sound` is not one to read answers out
of (§6).

`Interference` and `Clearance` are the pairwise result types of core §6.2: each
names its two bodies and carries its quantity as a `Measurement`, so each
reports its own exactness like everything else. Their preconditions are carried
the same way a body's are — in **existence**, never in a fabricated value: an
`Interference` exists only for a pair **proven** to overlap, a `Clearance`
only for a pair proven disjoint, so `Interference.Volume` is always a real
overlap's volume and `Clearance.Gap` always a real gap. A touching pair's zero `Gap` is a
**measured** zero, not the sentinel core §4 rejects: the sentinel is a zero
standing in for *there is no such quantity*, and a `Clearance`'s existence has
already said there is one — two disjoint interiors have a minimum distance,
and for a touching pair it is genuinely zero, gated at the noise floor like
every near-zero answer (§5): an `Approximate` zero must earn trust with a
vanishingly tight bound, an `Exact` zero passes on its own terms. Pairs are
drawn from the document's **proven solids** only. The partition is a statement
about interiors — a pair either shares volume or has a gap between disjoint
interiors — and it is answered to proof like everything else: a pair the
evaluator can prove neither way joins neither list and makes the report
`Suspect` (§6). A body that is not a proven solid has no interior the report
can vouch for; it joins no pair, and nothing is lost by that: proven invalid,
it has already made the report `Unsound`, which outranks anything a pair could
add, and undecided, it has already made the report `Suspect` — so its missing
pairs are never a silence a `Sound` report rests on (§6). Of the two lists,
`Interferences` is always computed — the `Interfering` rung of §6 reads it, so
the report could not aggregate honestly without it — and `Clearances` is
computed when `WithClearances()` (§2) asks for it.

## 2. Tolerance — what "beyond the caller's tolerance" means

`Suspect` — and through it `Trustworthy()` — turns on an approximation being
*coarser than the caller will accept*, so the caller must be able to say what
they accept:

```go
func WithTolerance(rel units.Value) VerifyOption // Dimensionless; default units.Scalar(1e-3)
func WithMinWallThickness(tool units.Value, opts ...WallOption) VerifyOption
func WithDraftAllowance(a units.Value) WallOption // Angle; default units.Degrees(15)
func WithPullDirection(v r3.Vec) VerifyOption
func WithMinRadius() VerifyOption
func WithClearances() VerifyOption
```

`WithTolerance` sets the gate this section defines; each of the other four
`VerifyOption`s switches on one quantity, and each lands in a named place in
the report: `WithMinWallThickness` fills `BodyReport.MinWallThickness`,
`WithPullDirection` fills `Undercuts`, `WithMinRadius` fills `MinRadius`, and
`WithClearances` fills `Report.Clearances`. `WithDraftAllowance` is no fifth
switch: it is the wall question's second parameter (below), it travels inside
`WithMinWallThickness` — a `WallOption`, the nesting core §8.1's
functional-options house style gives an option's own parameters — and it
cannot be stated without the question it parameterizes, so no allowance ever
dangles with no wall question to mean anything by. The correspondence is
total in both directions —
every option feeds the report, and everything in the report is producible:
`Bodies`, `Interferences` and `Status` unconditionally, the rest each by its
option. What is opt-in is what is expensive and **required by no rung the caller is
owed unasked**: interference must always run — the `Interfering` rung of §6 is
unanswerable without it, and overlap is a failure whether or not anyone asked —
while no rung needs a gap the caller never asked for, and a caller who did not
ask is never gated on the trustworthiness of answers they do not want. The
`Violating` rung (§6) reads only opt-in answers and stays answerable for the
same reason it exists: it enforces specs, an option is where a spec is stated,
and an option left off states none. An option takes a parameter exactly when
the question the caller poses states one. Undercuts are relative to a pull
direction, so `WithPullDirection` takes it — a zero direction poses no
direction at all, and is `ErrDegenerate` (core §12). The wall question states
two, because it is one spec in two parts: *no wall thinner than the tool that
has to cut it* — core §1's own words — *a wall being skins that oppose within
the draft allowance*. `WithMinWallThickness` takes the tool, and the tool is
the **spec, never the probe**: the reading needs no probe, because a wall
carries its own — a wall is material between opposing skins, read by the ball
that spans it, skin to skin (§6) — so `MinWallThickness` is a fact of the
body alone, and the tool enters only where §6 decides the reading against it:
a wall proven thinner makes its body `Violating`. `WithDraftAllowance` takes
the allowance, and the allowance is spec for the reason §6 works through: no
line intrinsic to the geometry separates a drafted wall from a shallow
wedge — the caller's **process** draws it, drafting the walls it molds and
chamfering the edges it breaks — so where the wall ends and the edge begins
is the caller's to state, exactly as the tool is. Unlike the tool it carries
a **convention default**, `units.Degrees(15)`: no convention can guess a
tool size, but draft practice is narrow — single-digit degrees at its
heaviest — and deliberate wedges start far above it, so one generous line
covers real processes (§6 works both brackets), and a caller whose process
leans harder moves it. A zero tool states the one spec no wall can violate —
no thickness is thinner than zero — a comparison with a single outcome, no
question at all, and is `ErrDegenerate` too (core §12). A **zero allowance**
is legal, and is the strictest reading — exact opposition and its tangency
limits only, every tapered skin deliberate — but the range is capped where
opposition itself gives out: at 90° or beyond, skins meeting at a square
corner would count as facing each other and every block's every edge would
read as a zero-thickness wall (§6) — a question no longer about walls, and
`ErrDegenerate` on the zero tool's own grounds. The legal range is
`[0°, 90°)`. A minimum radius and a
minimum gap are well-posed bare, so `WithMinRadius` and `WithClearances` take
nothing — and the wall *reading* is well-posed with no tool at all, so the
tool is not what makes it posable; the reading's own parameter is the
allowance, defaulted above, and the tool is the spec core §1's question
states — only a stated spec earns a verdict (below).

**A parameter is a spec, and only a spec earns a verdict.** The line between an
option the report answers and an option the report merely fills runs exactly
where the parameters are, and it is drawn once, here. `WithTolerance` states
how many figures the caller will accept, and the `Suspect` rung enforces it;
`WithMinWallThickness` states the tool no wall may be thinner than — the
allowance inside it stating what counts as a wall at all —
`WithPullDirection` the direction the part must pull along, and the `Violating`
rung of §6 enforces both. `WithMinRadius` and `WithClearances` take nothing, so
they state nothing, and `MinRadius` and `Clearance.Gap` are **measurements, not
verdicts**: the tightest concave radius and the smallest gap, gated for
trustworthiness like every bounded result but compared against no threshold,
because the endmill and the clearance spec live with the caller, who was never
asked to name them. A nil `MinRadius` on a proven solid is the comparison's
best case, not a missing answer: §1 makes it the determination that no concave
feature exists — no radius for any endmill to be too large for. The report
never invents a spec the caller did not state, and never withholds a verdict
on one they did.

**The tolerance is relative, and it is one number for every kind.** `rel` is the
largest error the caller will accept **as a fraction of the quantity being
measured**:

> A bounded result is **within tolerance** when `Bound <= rel × Ref`, where `Ref` is
> the result's **reference magnitude**.

One comparison, one number, no exponentiation. It is scale-invariant — a 1mm part
and a 1m part are judged on the same footing — which mirrors how `sketch` makes its
conditioning gate scale-invariant. `rel` is `Dimensionless` (`units.Scalar`); any
other `Kind` is `ErrUnitKind` (core §12), never a coercion; a negative `rel` is
`ErrNegativeMagnitude`; and a non-finite `rel` is `ErrNotFinite` — the name is
`units`' own, and the check is `Verify`'s to make, because `units.Value`
**construction** checks nothing: `units.Scalar(math.Inf(1))` is a representable
value, and only `units` *operations* refuse a non-finite result. Unrejected, an
infinite `rel` turns the gate off — `Bound <= (+Inf) × Ref` holds for every
bound, so the ±62% off-cut of §5 would read within tolerance — and a `NaN`
turns it inside out: every comparison against a `NaN` is false, so every
answer reads `Suspect`, the `Exact` answers §6 promises can never trip the
gate included. Neither is a tolerance, so neither is allowed to act as one.
The same rule closes the class for every parameter the options above take: a
non-finite `WithMinWallThickness` tool, a non-finite draft allowance, or a
`WithPullDirection` vector with a non-finite component, is `ErrNotFinite` on
the same terms — a negative tool or a negative allowance is
`ErrNegativeMagnitude`, an allowance whose `Kind` is not an angle is
`ErrUnitKind` (core §12), and the zero tool, the zero direction and the
90°-or-beyond allowance are `ErrDegenerate`, above. All are returned from
`Verify` — never deferred into
the report — which is why it returns an `error` (core §10).

**The default is `units.Scalar(1e-3)` — three significant figures — and it is set
by what a tessellation-backed evaluator can actually prove.** A v1 boolean bounds a
volume by roughly `surface area × chord error`: a Ø20×10mm cylinder tessellated to a
1e-3 mm chord has an area of 1257 mm², so a bound of order 1.3 mm³ against a volume
of 3142 mm³ — a relative error of 4e-4. A default of `1e-6` would put every such
body — every *correct* such body — in `Suspect`, and a gate no honest evaluator can
pass has no content. `1e-3` passes it with room to spare, and is still an order of
magnitude tighter than the ±1% Fusion ships by default (core §1) — and unlike
Fusion's figure it is a **proven bound**, not a nominal one. A caller who wants six
figures asks for them, and gets the honest `Suspect` an approximate evaluator owes
them.

An **absolute** threshold cannot do this job. A `Bound` carries the `Kind` of the
quantity it bounds (core §5.3), so an absolute length `t` has nothing to say about a
volume bound: raising it — `t²` for an area, `t³` for a volume — makes the gate
meaningless, because a linear error `t` propagates to a volume error of order
`area × t`, not `t³`. A Ø20×10mm cylinder tessellated to a 1mm chord carries a
volume bound of order 10³ mm³ while `t³` would admit 1 mm³, so *every* body touched
by a boolean would read `Suspect` at *every* tolerance, and the gate would have no
content at all. The ratio has content: it is exactly "how many significant figures
of this answer are real".

## 3. The reference

`Ref` is fixed per shape, and the report carries bounded results in exactly three
shapes (core §5.3) — a `Measurement`, a `VecMeasurement`, a `Box` — so the table is
total:

| Bounded result | `Ref` |
|---|---|
| `Measurement` — a volume, an area, a length, a gap | `max(abs(Value), Quantum)` |
| `VecMeasurement`, a **direction** (`Bound` is `Dimensionless`, core §5.3) | `1` — the magnitude of a unit vector |
| `VecMeasurement`, a **position**, and `Box` (`Bound` is a Length) | `D` |

`D` is a **diameter**: the greatest distance between two points of the geometry
the result belongs to. **Every reference is anchored to the thing the result
belongs to**, and ownership, not convenience, decides whose geometry that is:

- a result on a `BodyReport` — `Volume`, `Area`, `Centroid`, `Bounds`,
  `MinWallThickness`, `MinRadius`, and every vertex position and face normal of that
  body — belongs to **one body**: `D` is **that body's own** diameter — the
  distance between its two farthest points — and the boundary measures below are
  that body's own surface and edges. A body's answers are judged against the
  body they are answers about;
- an `Interference.Volume` and a `Clearance.Gap` belong to a **pair**, so no single
  body's size is theirs to be judged against — but the pair's own is. For those,
  and only those, `D` is the **pair's**: the diameter of the union of the two
  operand bodies' points — the greatest distance between two points drawn from
  either body — the one reference both members share and the only geometry the
  result is about. The boundary measure of an interference is the pair's own as
  well: the operands' summed surface areas. A body that is not an operand has no
  say in a pair's verdict, however large it is or wherever it sits.

A diameter is a distance between two points of the geometry, and a surface area
or an edge length is intrinsic to the bodies measured, so every reference is
invariant under rigid motion — and translation invariance is exactly what a
position's reference must have.

## 4. The noise floor

`Quantum` is the quantity's **noise floor** — the magnitude below which a value
of that `Kind` is not distinguishable from zero by any evaluator. Vertex
positions are the primitive everything else is computed from, and they are
trusted to an absolute noise of `δ = ε × D` — `D` decided by the ownership rule
above: a body's own for a body's results, the pair's for a pair's — with
`ε = 1e-9` fixed. What that noise does to a quantity is set by the **boundary
the quantity depends on**: displace every boundary point by `δ` and a volume
moves by at most `δ ×` the area of the surface enclosing it, an area by at most
`δ ×` the length of the edges bounding it, a length by `δ` itself. `Quantum` is
that product:

| Quantity | `Quantum` |
|---|---|
| a body's volume | `δ ×` that body's surface area |
| an `Interference.Volume` | `δ × (AreaA + AreaB)` — the overlap's boundary lies on the operands' skins: `∂(A∩B) ⊆ ∂A ∪ ∂B` |
| an area | `δ ×` the total length of the edges bounding the measured surface — the body's edges for `BodyReport.Area`, the face's own loops for a face |
| a length, a gap | `δ` |
| a dimensionless quantity | `ε` |

This is the same shape as the bound a v1 boolean actually proves — `surface
area × chord error` (§2) — evaluated at the noise displacement instead of
the chord error: the floor is where that bound would land if the evaluator were
perfect to the last bit of its own coordinates.

Every input to the gate is **intrinsic** — a property of the owning geometry's
own point set, a body's or a pair's, never of its pose — and that is the
rule's half of making a verdict a property of the **part, not the pose**.
Surface area and edge length do not move under a rigid motion, and neither
does `D`: a diameter is
realised by two points of the geometry, and a rigid motion preserves their
distance. So every `δ`, every `Quantum` and every `Ref` is the same real
number in every pose — the **rule** reads nothing pose-dependent, and so
introduces no pose dependence of its own. The floating-point **evaluator**
owns the other half — what its arithmetic returns for those same inputs
after the coordinates move — and that half is a statement about coordinate
**magnitude**: applying a rigid `r3.Transform` rounds every coordinate, and
the rounding is an absolute noise of order
`ulp(|coordinate|) ≈ |coordinate| × 2⁻⁵²` — relative to the coordinate, not
to the feature computed from it — so a feature of size `L` built from
coordinates of magnitude `R` re-measures with a relative wobble of order
`(R / L) × 2⁻⁵²`. For a model whose coordinates are of the order of the
geometry they describe — parts kept near the origin at their own scale —
the two magnitudes coincide and the wobble is ulps: a 10 mm segment
re-measured after a rotation by π/7 reads `10.000000000000002`, a relative
error of order 1e-16, thirteen decades below what the default `rel = 1e-3`
resolves — a fact about the evaluator's arithmetic, not about the gate's
geometry, and one that can move a verdict only when a `Bound` already sits
within a few ulps of its gate, the unavoidable property of any threshold
computed in floating point. A body parked far from the origin is a different
matter, and it fails upstream of the gate: it has spent its mantissa on its
own position. At `1e17 mm` the ulp is `16 mm`, so every coordinate of a
10 mm body parked there quantizes to a multiple of 16 mm and the segment
re-measures as `0` or `16 mm` — value, `Bound`, `D`, area, every gate input
computed from those coordinates is gone together, and no rule reading them
can defend numbers that no longer carry the geometry. What the gate does own
is that the loss must be **visible**. A `Bound` is a *proven* bound (core
§5.3), so an honest evaluator folds the coordinate rounding it was handed into
the `Bound` it reports, and the escalation is a ladder the reader can check:
past `R / D ≈ ε × 2⁵² ≈ 4.5e6` — a 10 mm part beyond about 45 km — the
rounding exceeds the `δ = ε × D` the noise model trusts a vertex to; past
`R / L ≈ rel × 2⁵² ≈ 4.5e12` at the default, the honest `Bound` exceeds
`rel × Ref` and the body reads `Suspect`; by `R / L ≈ 2⁵² ≈ 4.5e15` the ulp
reaches the feature itself and nothing measured survives, as at `1e17 mm`
above. The gate cannot restore figures the coordinates no longer hold; what
it guarantees is that an honestly bounded loss reads `Suspect`, never a
confidently wrong `Sound`. No axis-aligned box measure appears anywhere in the
gate, because a box's pose dependence is **geometric**, not arithmetic. The
box's bulk is off by decades: the box volume of a slender body posed diagonally
is of the order of the cube of its length — eleven decades over the same body
laid on axis, for a metre-long micron wire. And even the box's *diagonal*
breathes with pose — it lies between `D` and `√3 × D` depending on
orientation — and a factor that moves geometrically moves verdicts: any `Bound`
that lands inside the band reads differently in two poses of the same part. A
bounded geometric error is not intrinsicness, so the gate admits no box
measure, bounded or not. The one axis-aligned box in the report is the
`Bounds` **result**, and pose-dependence is that quantity's nature — it
answers where the body sits in these axes — but the gate judging its `Bound`
reads `D`, not the box.

The floor is honest at every aspect ratio, and the condition is sharp:
`Quantum` reaches a body's real volume only when `Volume / Area` — the body's
mean thickness — is itself under `δ`, i.e. only when the body is thinner than
the coordinate noise. The same holds one dimension down: an area's floor
reaches the area only when `Area / edge length` — the face's mean width — is
under `δ`. A body an evaluator can resolve at all sits decades above its floor.
The floor's ingredients — `D`, an area, an edge length — are the evaluator's
own readings and may themselves be approximate: `D` is read off the evaluated
boundary, and for a polyhedral approximation the greatest vertex-to-vertex
distance is the polyhedron's exact diameter — a convex hull and rotating
calipers, or any exact max-pair pass over the hull's vertices, computes it —
understating a curved body's true diameter by at most the chord error. A floor
is a magnitude, not an answer, and a per-mille error in it moves no verdict. A
surface with no edges at all — a sphere — gives its area a `Quantum` of zero,
and that errs in the only safe direction: a floor too low can only demand more
of an answer, never admit one. `ε` is a constant of the gate, not the caller's
knob: it is **not** `rel`, and `rel` never multiplies a `Quantum`.

## 5. The three rules

Three things follow, and all three are rules:

- **The gate is genuinely relative.** For any quantity above its noise floor — every
  volume, area, length and gap a real model measures — `Ref` **is** `abs(Value)`, and
  the test is exactly `Bound <= rel × abs(Value)`: how many significant figures of
  this answer are real. `Quantum` is a floor, not a scale factor, and it never
  loosens the test for a quantity that has a magnitude of its own — at **any**
  aspect ratio, because the floor engages only under the body's own skin. A
  100×100×0.001mm sliver measures 10 mm³ against a `Quantum` of
  `δ × 20000 mm² ≈ 2.8e-3 mm³` — three and a half decades under it — so `Ref`
  is the volume itself, and a ±5 mm³ bound — a 50% error — is `Suspect` at the
  default, and it must be, and it is. Nor may `Ref` ever be a yardstick of the
  body rather than of the value: a body need not fill its box — a thin shell,
  an L-bracket — and judging a volume against the box's bulk, or against any
  measure the value need not reach, would loosen its gate by every unfilled
  decade. That would be an absolute threshold wearing a ratio's clothes,
  judging a volume against the body rather than against the volume itself.
- **At and below the noise floor the gate becomes absolute.** A zero clearance, a
  knife-edge zero wall, or the volume of a body thinner than the coordinate noise
  (the flat limit below), has `|Value|` at or under `Quantum`, and a ratio
  to it is undefined or explosive. `Ref` collapses to `Quantum` there — and that is
  the whole of the near-zero rule, because it is the same formula: `Bound <= rel ×
  Ref` reads `Bound <= rel × Quantum` — an **absolute** threshold, a thousandth
  of the noise floor at the default `rel`. It is a real number and the reader
  can check it: a knife-edge zero wall thickness on a proven solid whose `D`
  is 1 mm has `Quantum = δ = 1e-9 mm`, so the gate is `1e-12 mm`; a zero
  clearance between two bodies whose union spans 100 mm has `Quantum = 1e-7 mm`
  and a gate of `1e-10 mm`. So a near-zero answer passes only with a bound
  that is, in practice, vanishingly tight. A tessellation does not produce one — an
  `Approximate` near-zero answer will
  essentially always read `Suspect` — while an `Exact` answer has a zero `Bound` and
  passes at the floor as it does everywhere else. That is the intent: a zero
  clearance reported as `0 ± 5mm` is untrustworthy and must be `Suspect`; a zero
  clearance known to `1e-12 mm` is not. The flat limit keeps a real floor. A
  genuinely flat body — a 100×100 mm sheet of zero thickness — never brings a
  volume to this gate: it encloses no region, and an evaluator proves as much —
  a skin whose two faces coincide is decisively no solid's — so §1 carries the
  answer in presence: `Unsound`, `Volume` nil, the boundary quantities only,
  its `2×10⁴ mm²` of area gated relatively as everywhere. The floor's work at
  that limit is done just above it, on the thinnest body that is still a body.
  A 100×100 mm plate `1e-7 mm` thick is a real solid, and it is thinner than
  the coordinate noise — §4's sharp condition, the only place the floor
  reaches a real volume: its `D` is 141.4 mm, so `δ ≈ 1.4e-7 mm`, its skin
  carries `2×10⁴ mm²`, and `Quantum ≈ 2.8e-3 mm³` — the volume a `δ`-thick
  skin over the plate would hold, the finest anything reading coordinates at
  `δ` can tell from zero — while the plate's whole volume is `1e-3 mm³`,
  under its own floor. `Ref` collapses to `Quantum` and the default gate is
  `2.8e-6 mm³`. The plate carries a `Volume` on §1's terms — an `Exact`
  evaluator's bound is zero, any positive thickness clears it, and it proves
  the plate solid outright (§6) — and that `Exact` volume passes with its
  zero `Bound`. An `Approximate` answer passes only with a bound under
  `2.8e-6 mm³`, a thousandth of the skin — and only from an evaluator sharp
  enough to have proven the plate solid at all: one whose proven bound cannot
  resolve the plate's `1e-7 mm` of self-clearance carries no `Volume` here to
  gate, only an undecided validity and a `Suspect` body (§1, §6). A
  point-like body is the full limit in every direction at once: proven no
  solid, so no region quantities at all (§1), and `D`, surface area and edge
  length all zero, so every `Quantum` its boundary quantities keep is zero,
  and only an `Exact` answer passes — a point has nothing to be approximately
  right about.
- **A coordinate is judged against `D` alone**, never against its own magnitude:
  the magnitude of a position is origin-dependent, and translating the model must
  never change the verdict. Because that `D` is the **owning body's**, the verdict
  is also scale-free: a centroid is judged against the size of the body whose centroid
  it is, so a 100mm bracket sharing a document with a 1.5m enclosure is judged against
  its own hundred-odd millimetres, and does not inherit a slack tolerance from the
  biggest thing in the document. Pair results keep the same discipline through
  the pair's own `D`: a 1 µm gap bounded to ±5e-4 mm between two bodies spanning
  100 mm together sits decades above the `1e-7 mm` floor that pair `D` sets, so
  its gate is `rel ×` its own `1e-3 mm` magnitude — `1e-6 mm` at the default —
  and it reads `Suspect` whether those two bodies share the document with
  nothing or with a building. No result, a body's or a pair's, is ever judged
  against geometry it is not about.

Worked, at the default `rel = 1e-3`, on four bodies — the last two are the same
wire, posed twice:

| Body | Box | `D` | `Quantum` | `Volume` | `Bound` | `Ref` | `rel × Ref` | Verdict |
|---|---|---|---|---|---|---|---|---|
| a small boolean off-cut | 2×2×2 mm | 3.46 mm | 8.3e-8 mm³ | 8 mm³ | 5 mm³ (±62%) | 8 mm³ | 8e-3 mm³ | **`Suspect`** — 5 ≫ 8e-3 |
| a Ø20×10mm cylinder, 1e-3 mm chord | 20×20×10 mm | 22.4 mm | 2.8e-5 mm³ | 3142 mm³ | 1.3 mm³ (±0.04%) | 3142 mm³ | 3.14 mm³ | `Sound` — 1.3 ≤ 3.14 |
| a 1m wire, 1µm square section, on axis | 1000×0.001×0.001 mm | 1000 mm | 4e-6 mm³ | 1e-3 mm³ | 5e-4 mm³ (±50%) | 1e-3 mm³ | 1e-6 mm³ | **`Suspect`** — 5e-4 ≫ 1e-6 |
| the same wire, along a cube diagonal | 577×577×577 mm | 1000 mm | 4e-6 mm³ | 1e-3 mm³ | 5e-4 mm³ (±50%) | 1e-3 mm³ | 1e-6 mm³ | **`Suspect`** — the same row |

`Quantum` is decades under the value in every row, so `Ref` is the volume itself
in all four. The off-cut and the cylinder are separated by three orders of
magnitude in *relative* error, which is the only thing that distinguishes them,
and it is exactly what the gate reads. The cylinder's `D` is `√(20² + 10²) ≈
22.4 mm` — rim to opposite rim — not its box's 30 mm diagonal: the body does
not reach its box's corners, so the box overstates it, and the gate never asks
the box. The wire is the stress case in both directions at once. Aspect ratio:
a real, nondegenerate solid whose volume is a trillionth of `D³` gets its floor
from its own 4 mm² of skin — `Quantum = δ × Area = 4e-6 mm³`, two and a half
decades under the volume it measures — so a ±50% volume error reads `Suspect`
on a wire exactly as it does on a cube. Orientation: posing the same wire along
a diagonal balloons its box's bulk from 1e-3 mm³ to nearly 2e8 mm³, while
volume, area and `D` are the wire's own in either pose — its `D` the same
1000 mm between the same two end points. The two wire rows list the rule's
quantities, and those are the same real numbers column for column; an evaluator
re-measuring them after the move reproduces them to ulps (§4 — the wire's
coordinates stay at its own 1000 mm scale in either pose), decades inside
every margin in the row. The floor tracks the body's skin and its own
two farthest points, never its box, so neither aspect ratio nor orientation can
touch it: the verdict is the part's, in every pose whose coordinates still
carry the part (§4).

## 6. Status and aggregation

The gate has nothing to miss, because **every one of the three shapes carries a
`Bound`** (core §5.3):

- a `Measurement`, a `VecMeasurement` or a `Box` on a `BodyReport` that is beyond
  tolerance — `Volume`, `Area`, `Centroid`, `Bounds`, `MinWallThickness`,
  `MinRadius`, each when the report carries it (§1) — makes that `BodyReport`
  `Suspect`;
- an `Interference.Volume` or a `Clearance.Gap` beyond tolerance makes the `Report`
  `Suspect` directly — those two are properties of a *pair*, so there is no
  `BodyReport` for them to travel through, and a gap known to only one significant
  figure is not an answer the caller said they would accept.

Either path makes `Trustworthy()` false. `Exact` answers have a zero `Bound` and
can never trip it, at any tolerance. **Nothing in the report is exempt**, and the
`VecMeasurement` of core §5.3 is what makes that true: a centroid or a vertex
position carries a bound like everything else, so a boolean that puts the centroid
off by more than `rel` of the body's own size cannot hide inside a `Sound` body —
which is the confidently-wrong failure core §1 exists to prevent.

Absence is not an exemption, emptiness is not, and a predicate is not, because
one standard governs the whole report, stated once:

> **An absence is an answer, an empty list is an answer, a predicate is an
> answer, and an answer must be proven. An answer the evaluator cannot prove
> never reads as a pass: the asked question is undecided, and undecided reads
> `Suspect`.**

The standard draws its line by what a claim is **about**, never by the claim's
shape: a claim about the data the evaluator holds is exact — a `Faceted` face
is exactly the polygon it is (core §6.1) — and a claim about the **part** that
data stands for is proven or `Suspect`, quantities, absences and predicates
alike. There is no exempt class. How that decides validity itself is the rule
of the `BodyReport.Status` bullet below.

A quantity the report does not carry is absent only where §1 permits it: a
region quantity of a body that is not a proven solid, an opt-in quantity that
was not asked for, or an absence the standard itself governs. The first exists
only on a body whose validity has already spoken in `Status` — proven invalid,
`Unsound`, the worst verdict in the precedence below; undecided, `Suspect`
under this same standard — so the absence never outruns the verdict that
explains it. The second is a quantity the evaluator never computed, so there
is no answer, trustworthy or otherwise, for the report to be silent about —
and no verdict owed either: an option is where a spec is stated (§2), so an
option left off poses no question for the report to fail. Everything else the
report says by absence or emptiness is an **answer** — a decided answer, never
an approximation of one — and the report gives four answers this way:

- **A nil `MinWallThickness` on a proven solid** is the determination *no wall
  exists* — nowhere do two of the body's skins oppose within the allowance
  (the wall rule below),
  so nothing exists for the tool to be thinner than. On analytic faces the
  proof exists: which face pairs oppose within the allowance across material,
  and whether any meeting inside it pinches a wall to zero, are closed-form
  facts of the surfaces, and
  the spanning survey over them (below) is that proof. A survey that is itself
  approximate proves nothing of the kind — a feather pinched below the chord
  error lands inside one facet and leaves no opposing pair to find. A
  `Faceted` body whose survey turns up no spanning pair is therefore an asked
  question the evaluator cannot answer, and the body reads `Suspect` with
  `MinWallThickness` nil — nothing proven wrong, nothing proven right.
- **A nil `MinRadius` on a proven solid** is the determination *no concave
  feature exists*. On analytic faces the proof exists: convexity and curvature
  are exact facts there, and a survey over them is that proof. A survey that
  is itself approximate proves nothing of the kind — a tessellation cannot see
  below its own chord error, so a concave dimple shallower than the chord
  lands inside one flat facet and leaves no concave edge to find. A `Faceted`
  body whose survey turns up nothing concave is therefore an asked question
  the evaluator cannot answer, and the body reads `Suspect` with `MinRadius`
  nil — nothing proven wrong, nothing proven right, which is exactly the
  rung's meaning.
- **An empty `Undercuts`** is the claim *no face is an undercut* — the same
  rule for the same reason, because the claim quantifies over the part, not
  over the survey. On analytic faces every point's normal is an exact fact
  with a closed-form range over the face, and the survey of those ranges (the
  membership rule below) is the proof that no region of any face opposes the
  pull. A `Faceted` survey that finds no undercut has proven no undercut
  *among the facets it holds*, and the claim is about the part those facets
  stand for: a reverse-draft face smaller than the chord lands inside one
  facet and leaves no reverse facet to find. So the body reads `Suspect` with
  `Undercuts` empty — asked, and undecided.
- **A pair in neither list** is the answer *these two bodies do not overlap* —
  the claim the `Interfering` rung reads, made by omission for every pair of
  proven solids without an `Interference` row. It is held to the same proof: a
  pair is proven disjoint when the boundaries the evaluator holds clear each
  other by more than the proven bounds those boundaries carry — the true skins
  then cannot touch — and an exact evaluator decides it outright. A pair whose
  held boundaries come closer than their own bounds, or overlap by less, is
  proven neither overlapping nor disjoint: no `Interference` row is fabricated
  for it — every row is a proven overlap (§1) — and the undecided pair makes
  the `Report` `Suspect` directly, at the same level its bounded results
  travel (the table below).

What the standard buys is the only reading that matters: inside a
`Trustworthy()` report, a nil `MinWallThickness` is a **proven** *no wall*, a
nil `MinRadius` a **proven** absence, an empty
`Undercuts` a **proven** all-clear, a pair with no `Interference` row a
**proven** disjointness — and every body a **proven** solid (the validity rule
below) — each as good as any `Exact` answer, because an unprovable answer
never reaches the caller inside a `Sound` report. (On a
`Suspect` report any of the five could be either the proven answer or the
survey that could not decide, and nothing turns on which: `Suspect` already
says this report is not one to read answers out of.) The gate covers every
bounded result the report carries, and what the report does not carry is
outranked, never asked, or proven.

**That guarantee is relative, and it is stated relatively because that is what is
true.** The gate on a centroid is `Bound <= rel × D` with `D` the owning
body's diameter, so at the default `rel = 1e-3` a centroid bound passes only
when it is under one part in a thousand of that body's own size — for these
box-shaped bodies, `D` is the distance between opposite corners:

| Body | `D` | `rel × D` | a 1 mm centroid `Bound` reads |
|---|---|---|---|
| 100×100×100mm block | 173.2 mm | 0.173 mm | **`Suspect`** |
| 1200×800×600mm enclosure | 1562 mm | 1.562 mm | `Sound` |
| 1m cube | 1732 mm | 1.732 mm | `Sound` |

A millimetre is a coarse answer on a 100mm block and the gate says so. On a body a
metre across it is under one part in a thousand — three significant figures, which is
what the default tolerance *means* and what the caller asked for. A caller who needs
an absolute millimetre on a metre-scale body buys it with figures: `rel = 5e-4` puts
the 1m cube's gate at 0.87mm and the enclosure's at 0.78mm. What is ruled out
absolutely — at every tolerance, on every body — is the failure core §1 names: an
error that is large *relative to the part it is an error about* sitting inside a
`Sound` report. Judging the centroid against the owning body rather than the
document is what makes that reading hold at any scale.

`Status` is one type used at two levels:

```go
type Status int

const (
    Sound       Status = iota // every body a proven solid; every stated spec met; every asked absence proven; nothing approximate beyond tolerance
    Suspect                   // an answer is Approximate beyond the caller's tolerance, straddles a stated spec, or is undecided — a validity the boundary's bound cannot settle, an unproven absence, a pair in neither list
    Violating                 // a stated spec is proven to fail: a wall thinner than the tool, an undercut against the pull
    Interfering               // bodies overlap
    Unsound                   // some body is proven not a valid solid
)
```

- **`BodyReport.Status`** is per-body: `Sound` (a proven solid; every stated
  spec met; nothing approximate beyond tolerance; every asked absence proven),
  `Suspect` (nothing proven wrong and something not proven right: an answer
  `Approximate` with a `Bound` beyond the tolerance of §2, a stated spec
  straddled — the interval rule below — an asked absence left unproven, the
  standard above: `WithMinWallThickness` asked and the evaluator could
  neither decide the wall against the tool nor prove the body has no wall,
  `WithMinRadius` asked and the evaluator could neither
  measure a concave radius nor prove the body has none, or `WithPullDirection`
  asked and the evaluator could neither prove an undercut nor prove there is
  none — or the body's own validity left undecided, the rule just below),
  `Violating` (a proven solid, but a spec a §2 option stated is proven to
  fail: `MinWallThickness` decided below the tool, or `Undercuts` non-empty),
  or `Unsound` (**proven** not a valid solid). Validity is decided first,
  before any quantity is read, which is what lets §1 key a region quantity's
  presence on it with no circularity: only a proven solid's quantities exist
  to decide `Sound` against `Violating` and `Suspect`. A body is never
  `Interfering` — interference is a property of a *pair*, not of a body.

  **Validity is a claim about the part, so it is held to the standard like
  every other answer: proven, or `Suspect`.** The predicate fields report the
  boundary the evaluator holds, and about *that* they are exact — a `Faceted`
  face is exactly the polygon it is; what it approximates is which surface it
  stands for (core §6.1), never what it is — so whether the held skin closes,
  pinches or crosses itself is a fact of data the evaluator possesses in
  full, a decided answer (core §6). But *this body is a valid solid*
  quantifies over the part that boundary stands for, exactly as `MinRadius`'s
  nil quantifies over the part and not the survey: a sub-bound pinhole, pinch
  or graze is absent from the held boundary precisely the way a sub-chord
  dimple is absent from the `MinRadius` survey. The instrument that decides
  it is one the report already owns — the proof the undecided pair above
  reads, turned inward. The held boundary approximates the true one within a
  **proven** bound (the bound every vertex and facet reports, core §5.3: a
  `Faceted` evaluator's chord error, an `Exact` evaluator's zero), and a
  validity claim is proven exactly when the held geometry is **decisive
  beyond that bound** — when no true boundary the bound admits could flip
  the answer. Decisive against — a defect wider than the bound — is proven
  invalid, `Unsound`. Decisive for — the held skin clean, and its
  **self-clearance** — the nearest its skin comes to itself across space
  rather than along it — beyond the bound, leaving a sub-bound defect no
  room to hide — is proven valid. In between — a defect, gap or graze inside
  the bound — the question is undecided, and the body reads `Suspect`: not
  `Sound`, and not `Unsound` either, because nothing is proven wrong.
  Predicate by predicate:

  - **watertight** — a hole in the held skin wider than the bound is a proven
    hole: no true skin the bound admits closes it. A held skin that closes,
    with self-clearance beyond the bound, is proven watertight: none the
    bound admits opens it. A sub-bound gap — or a seam sealed at a sub-bound
    near-tangency, where the true surfaces may part exactly where the facets
    met — is undecided.
  - **manifold** — sheets crossing at a shared edge by more than the bound
    are a proven non-manifold junction; a clean held skin with self-clearance
    beyond the bound is proven pinch-free; a junction or pinch inside the
    bound is undecided.
  - **self-intersection** — the held skin through itself deeper than the
    bound is proven self-intersection; self-clearance beyond the bound proves
    there is none; a graze inside the bound is undecided.
  - **`Lumps` and `Voids`** — a count is proven when every gap that separates
    and every neck or wall that joins clears the bound: pieces further apart
    than the bound cannot be one piece, a throat thicker than the bound
    cannot be a split, a cavity walled off by more than the bound cannot
    open to the outside. A sub-bound gap, neck or wall leaves the count
    undecided, and an undecided count is `Suspect` like any undecided
    answer: the reported `int` is the held boundary's, and the part's is
    the claim.

  Every predicate and both counts read the same instrument — the held
  boundary's own feature scale against its own proven bound — so one survey
  decides them together, and `Solid` is decided as their conjunction. An
  `Exact` evaluator's bound is zero and its boundary is the truth: it proves
  validity outright, every time. A `Faceted` evaluator proves it whenever the
  geometry is decisive, and on real parts that is the ordinary case, not the
  exception: §2's Ø20×10mm cylinder at a 1e-3 mm chord has a self-clearance
  of its own 10 mm height — four decades beyond the bound its facets carry —
  and is proven valid outright. What a faceted evaluator cannot do is what
  it never honestly could: read `Sound` over a seam its chord cannot resolve.
  The sub-bound pinhole, pinch or neck of a near-tangent boolean is an asked
  question it cannot answer, and it reads `Suspect` — nothing proven wrong,
  nothing proven right, which is exactly the rung's meaning.
- **`Report.Status`** is the document-level aggregate — over the bodies *and* over
  the pairwise results, which belong to no body.

**A wall is material between opposing skins, the allowance says how much
draft opposition tolerates, and the reading needs no probe.**
`MinWallThickness` reads the body's maximal inscribed balls — every
ball that fits the material and can grow no further — and fit alone is not a
wall: no ball of positive radius fits a sharp convex edge, so a bare infimum
over fits reads zero on any body with one, and §6 would faithfully find a
plain 100 mm cube `Violating` against a 1 mm tool — a verdict about every
part's edges and about no part's walls. What discriminates is what a ball
touches. A ball **spans a wall** when two of its boundary contacts are
within the draft allowance α of diametrally opposite — `WithDraftAllowance`,
default 15° (§2), *within* inclusive as within tolerance is (§2): its
diameter then runs skin to skin — squarely at exact opposition, across the
lean when the contacts sit inside the allowance — and the material it fills
is a slab between two boundary patches that face each other. The
mid-plane ball of a 10×10×0.5 mm plate touches the two 10×10 skins at
opposite poles — it spans, and its 0.5 mm diameter is the plate's wall —
while a ball wedged near a cube's edge touches the two faces at points 90°
apart: those skins meet, they do not oppose, nothing lies *between* them, and
the ball spans nothing however small the edge starves it. In general the two
contacts of a ball inscribed in a dihedral of angle δ sit π − δ apart, so
the allowance is a line through the dihedrals: **a wedge of dihedral within
the allowance is a wall, beyond it an edge** — δ ≤ α spans, δ > α never
does, at every size the edge starves a ball to.

The allowance is the caller's spec (§2) because no line intrinsic to the
geometry separates a drafted wall from a shallow wedge — the **process**
draws it, and two cases bracket where it can sit. A 100 mm long,
20 mm tall wall, 1.0 mm thick at its base and drafted 1° — 1.35 mm at the
top, as every molded wall is drafted — is a wall by any process's lights,
and a 1.5 mm tool spec must find it thin: its skins' contacts sit 179°
apart, and a reading that demanded exact opposition would read the part
`Sound` against the tool that cannot cut it — the exact failure the
question exists to catch. A 60° chamfer is an edge
by the same lights, and the same tool must find nothing there: its
contacts sit 120° apart, and the balls its edge starves say how sharp the
edge is, not how thin any wall is. Between those brackets the geometry
offers no line, so the default is a convention the caller owns, with its
grounds stated (§2): 15° clears the heaviest real draft — single-digit
degrees, 0.5–3° as a rule, 5–7° on heavily textured mold faces — by a
factor of two, and sits a factor of four under the 60° chamfer. A process
whose deliberate wedges run finer — a knife ground to a 12° bevel — is
exactly a caller with a line of their own to state, and moving the line
moves only where wall ends and edge begins, never the reading's law. The
range cap is where opposition itself gives out: two
skins face each other only while their contacts sit past a right angle
apart, so α is legal on `[0°, 90°)` (§2) — and it is the cap, not the
default, that a square edge can never cross. A cube's edges are excluded
at every legal allowance; its only spanning ball is its center's, touching
two opposite faces 50 mm out each way: its reading is its own 100 mm slab,
and a 1 mm tool finds nothing to violate.

`MinWallThickness` is the infimum of the diameter over spanning balls,
closed under limits: a family of balls whose contacts approach within-α
opposition contributes the diameter it converges to. On the drafted wall
the infimum is the base-tangent spanning ball, pinned by the two skins and
the base at once — diameter 1.009 mm, the base thickness read across the
1° lean — and against the 1.5 mm tool the interval rule below decides it
`Violating`. The closure is the knife edge's zero (§1): where two skins
meet at **tangency** — a wall ground out to nothing, a face running out
tangent onto another — the balls between the skins thin to nothing with
their contacts inside every allowance, zero included, and the infimum is a
genuine 0 on a body that is still a proven solid, decided thin against any
real tool by the interval rule below. A taper within the allowance pinches
the same way — a drafted wall run out to a sharp edge is a wall ground to
nothing, and reads 0 — while an edge of dihedral beyond α contributes
nothing however acute: its contacts hold π − δ apart at every size, short
of the allowance, so a 20° chamfer and a cube's edge are the same answer
at the default — *an edge, not a wall*. A caller whose spec is about the
edges themselves — a minimum edge angle, say — is stating a spec
`WithMinWallThickness` does not pose, and no option states, so no verdict
enforces it (§2): the allowance says where walls end, and demands nothing
of the edges beyond. And a body may have no wall at all: a regular
tetrahedron's inscribed ball touches its four faces pairwise
arccos(−1/3) ≈ 109.5° apart — the body's own 70.5° dihedral, read in
contacts — so nothing spans at the default, or at any allowance short of
that dihedral, and no family closes a limit: every skin meets its
neighbours and opposes none. The infimum is over nothing, and §1 carries
that answer in presence: `MinWallThickness` nil, the determination *no
wall exists*, held to proof by the absence standard above. Bulk changes
nothing: an equilateral wedge prism 30 mm on a side and 80 mm long has its
side pairs at 60°, its caps at 90° to them, and its two parallel caps
beyond any inscribed ball's reach — no ball exceeds the cross-section's
8.66 mm inradius, so none touches caps 80 mm apart — no spanning ball
anywhere, nil, and no tool spec it can violate.

Worked, at the default α = 15° — spanning is contacts within 15° of
opposite, a dihedral of 15° or less:

| Body | nearest skins | contacts | reading | against the tool |
|---|---|---|---|---|
| 10×10×0.5 mm plate | the 10×10 skins, parallel | 180° | 0.5 mm | **`Violating`** vs 1 mm |
| 100 mm cube | opposite faces — its edges sit at 90°, past every legal α | 180° | 100 mm | `Sound` vs 1 mm |
| 100×20 mm wall, 1.0 mm base, 1° draft | its two skins | 179° | 1.009 mm | **`Violating`** vs 1.5 mm |
| 60° wedge prism, 30 mm side, 80 mm long | side pairs 120°; caps opposite but beyond any ball | — | nil — no wall | nothing to violate |
| regular tetrahedron | face pairs | 109.5° | nil — no wall | nothing to violate |
| knife edge | skins at tangency | → 180° | `Exact` 0 | **`Violating`** vs any tool |
| 100×100×0.001 mm sliver | the 100×100 skins, parallel | 180° | 0.001 mm | **`Violating`** vs any real tool |

The reading quantifies over inscribed balls, but everything it reads is the
boundary the evaluator holds, as with the undercut survey below. On analytic
faces the spanning survey is closed form: whether two of core §6.1's
variants oppose within the allowance across material, the ball between
them, and the meeting inside the allowance
that pinches a wall to zero are exact facts of the surfaces, so an analytic
evaluator decides the reading — and the absence — outright, the boundary
case included: a wall drafted at exactly α spans — within is inclusive,
above — and an exact evaluator reads its thinnest spanning ball like any
other wall's. A `Faceted`
evaluator surveys facet pairs whose normals carry tilt bounds (core §6.1),
and what its survey cannot pin, its proven `Bound` must cover: a
pair whose tilt bounds admit both sides of the allowance it can neither
count as a wall nor dismiss — the at-allowance wall is that pair every
time, as the exactly-vertical wall is the undercut survey's undecided face
(below) — and it widens the
reading's interval until it admits both answers, and the interval rule below
does the rest — an interval that straddles the tool reads `Suspect`, the
honest verdict on a tessellated feather — or a wall drafted at the
allowance itself — whose facets really could lean
either way. And where dismissing the undecidable pair would leave no wall
at all, the two answers are a reading and an absence, no interval spans
them, and the absence standard above already holds: *no wall* is a claim
the survey cannot prove, and the body reads `Suspect` with
`MinWallThickness` nil.

**A spec is decided on the proven interval, never on the bare `Value`.** A
`Measurement` proves its truth lies in `[Value − Bound, Value + Bound]` (core
§5.3), and the comparison reads that interval, so it is total in three cases:

- **`Value + Bound < tool` decides thin.** Every thickness the interval admits
  is thinner than the tool, so the body is `Violating` — whatever the gate
  would say about the bound's size, because the `Bound` is a *proven* bound and
  a proven interval below the tool is a proven violation at any coarseness.
- **`Value − Bound >= tool` decides the spec met.** No admissible thickness is
  thinner than the tool — exactly tool-thick is not thinner — and the trust
  gate then judges the bound on its own terms, as it judges every bounded
  result: a wall proven thick by a coarse measurement is still a coarse
  measurement.
- **An interval that straddles the tool decides nothing**, and an undecided
  stated question is exactly what `Suspect` means: the measurement cannot
  answer the question the caller posed. It reads `Suspect` even when the bound
  passes the gate — the gate demands figures of the answer, the spec demands
  resolution at its own threshold, and near the threshold the second is the
  stronger demand.

Worked, at the default `rel = 1e-3`, against a 1 mm tool, on a body whose
every other answer is sound and in tolerance:

| `MinWallThickness` | interval | the spec | the gate | body reads |
|---|---|---|---|---|
| 0.5 mm, `Exact` | [0.5, 0.5] | decided thin | passes — `Exact` | **`Violating`** |
| 0.2 ± 0.3 mm | [−0.1, 0.5] | decided thin | fails — 0.3 ≫ 2e-4 | **`Violating`** — a ±150% bound, and still a proven violation |
| 1.2 ± 1e-4 mm | [1.1999, 1.2001] | met | passes — 1e-4 ≤ 1.2e-3 | `Sound` |
| 1.2 ± 0.05 mm | [1.15, 1.25] | met | fails — 0.05 > 1.2e-3 | **`Suspect`** — thick, but not to the figures asked |
| 1.0005 ± 0.001 mm | [0.9995, 1.0015] | straddles | passes — 1e-3 ≤ 1.0005e-3 | **`Suspect`** — the straddle, not the gate |

The last two rows are the trust-versus-spec distinction the ladder keeps: an
untrustworthy measurement of a thick wall and a trustworthy measurement that
cannot resolve the threshold both read `Suspect`, while a proven-thin wall —
however coarsely proven — reads `Violating`, one rung worse.

The comparison and the gate read the same interval and cannot contradict each
other: neither reads the other's output, and both feed the same worst-wins
precedence below. The `Exact` zero-thickness wall — a knife edge on a body
that is still a proven solid, the only body that carries the reading (§1) —
is the sharp case: its zero `Bound` passes the gate at the noise floor as an
`Exact` answer passes everywhere (§5), and it is `Violating` — [0, 0] sits
below any real tool — so the gate's pass certifies the *measurement*, never
the part. An `Approximate`
zero at the floor lands the same way through §5's other arm: on a body whose
`D` is 1 mm the floor gate is `1e-12 mm` (§5), so a bound tight enough to pass
it puts the whole interval decades under any tool a caller would name —
decided thin, `Violating`. And a zero reported as `0 ± 5 mm` decides nothing
and fails the gate too: `Suspect` on both counts, because it is proven neither
thin nor thick. A trust-pass never implies a spec-pass, a spec-pass never
implies a trust-pass, and the rungs compose by precedence alone.

The other spec of §2 reads the same kind of interval, one dimension over —
and quantified over the face, because a face does not have *a* normal. What
core §6.1 defines is the normal **at a point**: `Face.NormalAt(p)`, a
`VecMeasurement` whose proven bound is zero on an analytic face, where every
normal is an exact fact of the surface, and the facet's honest tilt error on
a `Faceted` one, because a facet's normal is exact *for the facet* and
approximate *for the part the facet stands for*. A curved face sweeps a
**range** of normals — a single cylindrical, spherical or toroidal face
carries normals both with and against any pull — so membership is decided
pointwise first. The pointwise comparison reads the point's proven interval
exactly as the wall's does, and is total the same three ways: a point whose
whole interval **opposes** the pull — every direction the bound admits has a
component against it — provenly opposes; a point whose whole interval does
not oppose provenly clears — exactly perpendicular is not opposed, as
exactly tool-thick is not thinner; a point whose interval straddles decides
nothing. The face's membership quantifies those points — existentially for
the violation, universally for the all-clear, the proof always on the claim:

- **a face with a provenly opposing point is a proven undercut.** One point
  is enough, and by the surface's continuity it never comes alone: around it
  lies a region facing against the pull, material the pull cannot clear. The
  face is listed, and a non-empty `Undercuts` makes its body `Violating`
  exactly as a non-empty `Interferences` makes the report `Interfering` — at
  any coarseness, for the same reason a proven-thin wall does, because a
  proven interval on the wrong side of a spec is a proven violation. Listing
  is the honest unit: a spherical face that reaches past its equator against
  the pull provenly opposes there and is listed for that region — the claim
  is *this face has a proven undercut*, never *all of this face opposes*;
- **a face whose every point provenly clears is settled** — proven no
  undercut;
- **a face in between** — no point provenly opposes, some point straddles —
  **is undecided**: it is **not** listed (every listed face is a proven
  membership) and its body reads `Suspect`, an asked spec with a face the
  evaluator could not decide.

The quantifier costs nothing core does not expose: `Face.Surface()` (core
§6.1) is the face's own geometry, and on an analytic variant — a plane,
cylinder, cone, sphere or torus — the normal over the face's bounded region
is closed-form, so the survey of its range is exact and the three-way answer
is decided outright. A `Faceted` face is its facets, and each facet is one
point of the survey: its normal is a single `NormalAt` reading, constant
across the facet, whose tilt bound is the interval the pointwise rule reads.
The rule is the claim, not the algorithm — but everything it quantifies over
is a reading the evaluator holds. The boundary case is the vertical wall — a
face everywhere exactly perpendicular to the pull, draft angle zero. It
provenly clears: the pull slides along it, not into it, and the strictness
sits on the same side as the wall rule's — exactly perpendicular is not
opposed, exactly tool-thick is not thinner. An analytic vertical wall is
settled by the exact survey; a `Faceted` one never is — its facets' tilt
bounds straddle zero draft from both sides, so it is the undecided face
above and its body reads `Suspect`, the honest answer: inside its bound a
tessellated wall really could lean either way. A caller whose spec is a
positive minimum draft — not mere clearance — is stating a spec
`WithPullDirection` does not pose, and no option states, so no verdict
enforces it (§2): the draft *allowance* is no such spec — it says which
skins the wall reading counts as opposing (above), and demands nothing of
the pull. And an **empty** `Undercuts` claims more than every held
face settled — it is the absence answer of the standard above, quantifying
over the part and not the survey: provable by the exact range surveys over
analytic faces, unprovable from a tessellation, whose sub-chord features no
per-facet interval covers — the body reads `Suspect` with `Undercuts` empty.

Aggregation is by **severity precedence — worst wins**:

**`Unsound` > `Interfering` > `Violating` > `Suspect` > `Sound`**

Read down, each rung concedes the one above: `Unsound` — some body is proven
not even a solid, and nothing about its region is measurable; `Interfering` —
no body is proven invalid, but the document claims the same space twice;
`Violating` — the document is coherent, but a spec the caller stated is proven
to fail; `Suspect` — nothing is proven wrong, and something is not proven
right; `Sound` — everything asked is answered, met, and trusted.

Concretely, `Report.Status` is:

| Condition | `Report.Status` |
|---|---|
| any `BodyReport.Status == Unsound` | `Unsound` |
| else, `len(Interferences) > 0` | `Interfering` |
| else, any `BodyReport.Status == Violating` | `Violating` |
| else, any `BodyReport.Status == Suspect` | `Suspect` |
| else, any `Interference.Volume` or `Clearance.Gap` beyond tolerance (§2), or any pair left undecided (the absence standard above) | `Suspect` |
| else | `Sound` |

The last rung is what keeps the tolerance gate **total**: a bounded result that
hangs off the `Report` rather than off a `BodyReport` is gated exactly as every
other is, so a `Clearance.Gap` measured far coarser than the caller's tolerance can
never sit inside a `Sound` report. (Interference is caught by the rung above it as
well, and `Interfering` is the worse verdict; the rule is stated over both so that
nothing in the report is exempt.) The same rung carries the undecided pair, and
that keeps the pair partition total: a proven overlap is `Interfering`, a proven
disjointness is the one silence a `Sound` report may rest on, and a pair proven
neither is `Suspect`. Together with the `Suspect` rung above it, the
gate covers **every `Measurement`, every `VecMeasurement` and every `Box` the report
carries** — and per core §5.3 those are all of them.

`Report.Trustworthy()` is true **only** at `Report.Status == Sound`. A body
proven invalid, a validity left undecided, an unresolved interference, a
stated spec proven to fail or left undecided, an asked absence left unproven,
an undecided pair, or an approximation coarser than the caller's tolerance —
on a body or on a pair — each make it false, even when the geometry "looks"
fine.
