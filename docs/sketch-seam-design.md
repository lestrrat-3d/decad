# Sketch Seam Design

The recording contract at the seam between decad and `sketch`: what `sketch`
certifies about a profile's boundary (§1), the structural IR a `Recipe` `Step`
records a profile in (§2), and the one rejection the seam returns,
`ErrUnrecordableProfile` (§3). Companion to `docs/api-design.md` — the core
design, which owns the recipe/evaluator architecture, the completeness rule,
and the feature calls that consume a profile. References of the form "core §N"
are to that document.

Every capability this contract consumes exists in `sketch` today — there is no
open dependency gap. It is stated in full here because the mapping table (§2),
the core design's seam rules (core §7) and `ErrUnrecordableProfile` (§3) are
all built on it.

## 1. The trim contract

`sketch.BoundaryEdge` is `sketch`'s account of one directed edge of a profile
boundary — a whole sketch entity, or a fragment of one produced by splitting at
a crossing. A `BoundaryEdge` carries `Entity`, `Partial`, `Reversed` and
`Polyline` — and the trim itself:

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
is exact only when `sketch`'s closed-form kernel placed it, and that kernel
places a cut for exactly one kind of contact: a **crossing** whose source
curves are **both** a line, a circle or an arc, with a line on at least one
side. Among line/circle/arc sources a tangency is never a cut: the kernel
classifies it as a **non-splitting** contact — a shared-endpoint tangency is a
smooth join, an interior touch splits neither curve — so analytically tangent
entities arrive as whole edges (two externally tangent circles are two whole
one-edge loops). A grazing contact involving any other curve kind has no such
rule: it is resolved on the sampled polylines below, which can cut at the
touch. Every other cut is **sampled**: every
curve/curve crossing (circle × circle, circle × arc, arc × arc), and every cut
at a contact involving an ellipse, elliptical arc, conic, spline, closed
spline, fit spline or NURBS — **including a plain line** against one of those,
whether it crosses or merely grazes: such pairs are resolved on the sampled
polylines, and where the sampled arrangement cuts — it can cut at a grazing
touch — the parameter is sampled. A sampled cut yields `TExact == false` on
every fragment it bounds.

**No residual test on a fragment's endpoints could stand in for the flag, at any
tolerance.** A `Polyline` is a **sample of the curve**: its vertices are
evaluated from the curve (the one exception is an elliptical arc's two end
vertices, which are its pinned ends, §2). A sampled cut is the crossing of a
**chord** between two such vertices, so as the crossing approaches an evaluated
vertex the cut point approaches the curve, and its residual against the curve
goes to zero. A sampled
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
the flag nor re-derives it — re-deriving a 2D answer is what core §7 forbids.

- `TExact == true` → the fragment records: the entity's own variant, built
  from the entity's defining data and `TStart` / `TEnd` (the table in §2).
  A circle crossed by a rectangle records.
- `TExact == false` → `ErrUnrecordableProfile` (§3). An approximate parameter
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
answers a question decad is not asking. `sketch` decides the flag by
reproduction — the checkable meaning above — so on a whole edge it turns on how
the entity's domain ends arise. Every kind but one evaluates them from the
curve itself, so its whole edge reads `true`. The exception is the whole
`*EllipticalArc` edge, whose flag is **contingent**: the arc's endpoints are
pinned to sketch points rather than evaluated from the curve, so the flag reads
`false` whenever those pinned points miss the parametric ellipse — the typical
solver outcome, a miss on the order of the solver tolerance — and `true` when
they happen to land on it. Either reading is a fact about the pinning, **not
topology distrust**, and neither may affect whole-edge recording: an
implementer who "helpfully" gates whole edges on `TExact` would make an
elliptical-arc boundary's recordability turn on how the solver converged —
nondeterministic on data the entity itself records exactly.

**What records reaches exactly as far as `sketch`'s exact kernel does, and no further.**
For fragments, that is a line, circle or arc fragment whose bounding cuts were
placed by the closed-form crossing kernel — each cut a line-involved crossing
between line/circle/arc sources — plus whole edges of every recordable kind,
which record from entity data with no cut to certify. Tangencies add nothing
to that set: among lines, circles and arcs a tangency splits nothing (above),
so no fragment is ever bounded at one — a tangent entity arrives whole, or in
fragments whose every bound is a crossing. A fragment cut by anything else — an
ellipse, elliptical arc, conic, spline, closed spline, fit spline or NURBS on
either side of the contact, and every curve/curve crossing — carries
`TExact == false` and is `ErrUnrecordableProfile`.
That is not decad declining to record it; it is `sketch` reporting that the parameter is
sampled, and decad recording no fragment on a range it was told is approximate. Widening
that set is an upstream question about the arrangement, not an API question here.

## 2. The recording IR

A `Step` records the region an `Extrude` or `Revolve` sweeps in decad's own
types — a `ProfileRecord`, and the `PlaneRecord` that lifts it into world
space — because a `Recipe` is a value and a live profile is not (core §2,
core §6.2):

- A `*sketch.Profile` is a pointer into a live, mutable `*sketch.Sketch` — a
  *handle*, and core §2 says the recipe is a value. Its `Entities` and its
  `BoundaryEdge.Entity` are `sketch.Entity`, an interface with unexported methods,
  which no decoder can reconstruct. And its `BoundaryEdge.Polyline` is a **densified
  sample** — a tessellation, which core §2 says a `Recipe` never names. (Its first and
  last points are the edge's start and end. They are the one thing decad reads from
  it, and only to check — they are what §1's falsifier tests the recorded range
  against — never to record; see below.)
- `r3.Frame`'s fields are unexported: it marshals to `{}`, so a `Step` that stored
  one would silently drop the plane — the single field without which the step is
  incomplete.

So decad **converts, it does not reference**:

```go
// PlaneRecord is the sketch plane, as three vectors: it survives encoding, which
// an r3.Frame does not. Orthonormal, right-handed; the plane normal is U × V, and
// that normal is the sense Direction.Along means for a linear extent (core §8.1).
type PlaneRecord struct {
    Origin r3.Vec // millimetres (core §5.2)
    U, V   r3.Vec // the in-plane axes: the (u, v) a Point2 below is expressed in
}

// Point2 is a plane-local coordinate, a length in millimetres — core §5.2's
// carve-out in the plane's own (u, v).
type Point2 struct{ U, V float64 }

// ProfileRecord is the region a Step extrudes or revolves: one outer loop and its
// holes, structural and plane-local. Not a sample, not a pointer, not a sketch.
type ProfileRecord struct {
    Outer LoopRecord
    Holes []LoopRecord
}

// LoopRecord is a closed, directed boundary loop: each segment's walk — from
// its point at TStart to its point at TEnd — ends where the next segment's walk
// starts, and the last closes onto the first. A single closed segment — a
// circle, an ellipse, a closed spline — is a loop on its own. Outer loops run
// counter-clockwise in (u, v), holes clockwise.
type LoopRecord struct {
    Segments []CurveSegment
}

// CurveSegment is one curve of a loop, recorded structurally — never as a
// sample. Sealed, like Surface (core §6.1).
// A variant records exactly the defining data of the curve the edge IS — the
// fields of the source entity's own geom value, verbatim, in plane-local
// Point2 — plus the recorded range: TStart/TEnd, sketch's normalized t on the
// entity, the full domain for a whole edge and the certified range for a
// Partial fragment (TExact, §1). An evaluator reconstitutes the curve
// through sketch/geom from the entity's own fields and trims it at the range —
// it never re-derives either (core §7). What geom DERIVES from those fields —
// an arc's radius and angles, an elliptical arc's eccentric parameters — is
// never recorded in their place: the readings are geom's answers, computed on
// demand, and they do not determine the fields — an arc's readings fix End
// only up to its angle, an elliptical arc's eccentric angles name points on
// the parametric ellipse its pinned ends need not lie on (§1) — so a record
// of readings could not reconstitute the entity's geom value without
// synthesizing its points. One variant serves each entity kind, whole and
// trimmed alike: the entity picks the variant, and only the range differs.
type CurveSegment interface{ curveSegment() }

// The five analytic kinds. Every variant's TStart/TEnd — like a spline's
// knots and weights — is a curve parameter, not a quantity (core §5.2).
type LineSeg struct {
    Start, End   Point2  // geom.Line: the endpoints, verbatim
    TStart, TEnd float64 // the full domain for a whole edge
}

// CircleSeg and EllipseSeg are the closed analytic kinds — a whole edge of
// either is a LoopRecord on its own — so, like ClosedSplineSeg, they carry the
// walk's winding in (u, v) as CCW alongside the range.
type CircleSeg struct {
    Center       Point2      // geom.Circle: the center —
    Radius       units.Value // — and the radius
    CCW          bool
    TStart, TEnd float64 // the full period for a whole edge
}

// ArcSeg mirrors geom.Arc: three pinned points, the arc swept counter-clockwise
// from Start to End about Center. The sweep is the entity's own definition, so
// no field restates it. Radius and angles are geom's derived readings, never
// fields. geom's arc geometry is Center plus Start's radius, End contributing
// only its angle: the boundary geom emits runs on Start's radius and ends at
// End's angle, evaluated from the curve — never pinned to End — so it passes
// through a pinned End only when End lies on that radius (which is why a whole
// *Arc edge's flag reads true, §1). A center/radius/angle record re-evaluates
// to exactly that boundary; what it loses is the fields: the readings do not
// determine End's radial position, and an evaluator holding angles in place of
// points would have to synthesize the points from them — re-deriving what geom
// owns (core §7) — instead of reconstituting geom.Arc from its own fields and
// asking it.
type ArcSeg struct {
    Center, Start, End Point2  // geom.Arc: the pinned points, verbatim
    TStart, TEnd       float64 // the full domain for a whole edge
}

// EllipseSeg is sketch's ellipse. Rx and Ry are the semi-axes along the
// ellipse's own local x and y, and they are UNORDERED — geom.Ellipse does not
// enforce Rx >= Ry; the axes are simply the local x and y, and Rotation is the
// angle of that local frame. Naming them Major/Minor would oblige an
// implementer to normalise the pair, which is re-deriving 2D geometry (core §7).
type EllipseSeg struct {
    Center           Point2
    Rx, Ry, Rotation units.Value
    CCW              bool
    TStart, TEnd     float64 // the full period for a whole edge
}

// EllipticalArcSeg mirrors geom.EllipticalArc: the ellipse (Center, Rx, Ry,
// Rotation — unordered, as EllipseSeg) restricted to the counter-clockwise
// eccentric-angle sweep from Start to End. The sweep is the entity's own
// definition, so no field restates it. Start and End are the entity's PINNED
// points, verbatim — they lie on the parametric ellipse only within solver
// tolerance (§1) — and, unlike an arc's, the boundary geom emits interpolates
// them: its interior is evaluated from the ellipse, its ends are the pinned
// points themselves (the gap §1's contingent whole-edge flag turns on). So no
// eccentric-angle pair can stand in for them: angles re-evaluate to points ON
// the parametric ellipse, a different boundary than the one the entity
// defines.
type EllipticalArcSeg struct {
    Center, Start, End Point2      // geom.EllipticalArc: the pinned points, verbatim
    Rx, Ry, Rotation   units.Value // local-x / local-y semi-axes, unordered; frame angle
    TStart, TEnd       float64     // the full domain for a whole edge
}

// The five free-form kinds. Degree, knots and weights are curve parameters on
// the same terms as every range (core §5.2); a conic's fullness Rho — from
// which a rational quadratic's apex weight derives as w = Rho/(1-Rho) — is of
// exactly the same class as a NURBS weight.

// SplineSeg mirrors geom.Spline: an open cubic B-spline over its control
// points. Degree 3, the clamped uniform knot vector and unit weights are the
// entity's DEFINITION, not stored data — geom.Spline's one field is Control —
// so the record carries none of them: a Degree, Knots or Weights field here
// would hold values the entity does not, synthesized, which the verbatim rule
// above forbids.
type SplineSeg struct {
    Control      []Point2 // geom.Spline: the control points, verbatim
    TStart, TEnd float64
}

// NURBSSeg mirrors geom.NURBS: a clamped B-spline of arbitrary degree with a
// non-decreasing knot vector and a per-control weight — every field the entity
// holds, verbatim, and nothing it derives.
type NURBSSeg struct {
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
// the points and NEVER runs the interpolation solve itself (core §7).
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

Ten variants, one per `sketch` `Entity` implementation — line, circle, arc,
ellipse, elliptical arc, conic, spline, closed spline, fit spline, NURBS. That
is `sketch`'s entity vocabulary exactly and entirely, and `sketch` puts every
one of those kinds on a profile boundary: the four open free-form curves as
boundary edges, the closed spline as a closed curve bounding a region on its
own. `SplineSeg` and `NURBSSeg` are distinct variants because the entities are
distinct data: `geom.Spline` holds only its control points — degree 3 and the
clamped uniform knot vector are its *definition* — while `geom.NURBS` holds
`Degree`, `Knots` and `Weights` as fields. A spline recorded through a
NURBS-shaped variant would carry a degree, a knot vector and weights its entity
never held — synthesized fields, not verbatim ones. So **every entity `sketch`
can put on a boundary has a record** — whole, and trimmed: a `Partial`
fragment records through the same ten variants — its entity's own variant,
with the certified range — so the vocabulary needs no fragment-specific
kinds. What has no record is a `Partial` fragment whose cut
`sketch` samples — `TExact == false` (§1) — and such a profile is rejected
rather than recorded lossily. A
new entity kind upstream needs a new `CurveSegment` variant before decad accepts a
profile that uses it; there is no fallback to a sample.

**A whole edge records its entity. A `Partial` fragment records exactly when
`sketch` certifies its cut — `TExact == true` — and it records from
`TStart` / `TEnd`.** The range is `sketch`'s answer — normalized `t` in the
entity's natural direction, `TStart < TEnd`, never wrapping (§1) — and the
fragment's record is the entity's own defining data plus that range: the same
variant a whole edge records, narrowed. Admission never depends on the
entity's kind: the flag admits the fragment, and the kind only selects which
variant records it. Whole edges never consult the flag (§1).

| `BoundaryEdge.Entity` | whole edge | `Partial` fragment — records when `TExact`; else **`ErrUnrecordableProfile`** (§3) |
|---|---|---|
| `*Line` | `LineSeg` — the entity's endpoints; the full domain | `LineSeg` — the same endpoints; the certified range |
| `*Circle` | `CircleSeg` — the entity's center and radius; the full period | `CircleSeg` — the same; the certified range |
| `*Arc` | `ArcSeg` — the entity's pinned points; the full domain | `ArcSeg` — the same points; the certified range |
| `*Ellipse` | `EllipseSeg` — the entity's axes and frame; the full period | `EllipseSeg` — the same; the certified range |
| `*EllipticalArc` | `EllipticalArcSeg` — the entity's pinned points, axes and frame; the full domain | `EllipticalArcSeg` — the same; the certified range |
| the free-form five | the matching free-form variant, `TStart`/`TEnd` spanning the full domain | the matching free-form variant, `TStart`/`TEnd` the fragment's range |

**Every row records a certified fragment, and every row rejects a sampled one.**
Every row records the entity's fields verbatim, and the two columns differ only
in the range. There is no per-row parameter mapping to apply: every variant
records `sketch`'s normalized `t` itself — `geom.BoundaryEdge`'s published
contract — so the range is handed over, never converted; nothing is solved
for, no point is evaluated from a parameter, and no point is ever inverted to
one (core §7). Under `sketch`'s exact kernel the fragments that carry `TExact == true`
are those of a line, a circle or an arc cut within that family (§1), so those
are the rows the true column reaches today: a circle cut by a rectangle edge
records, and a circle cut by another circle — or an ellipse cut by anything,
including the line fragments that crossing leaves on the rectangle — is
`ErrUnrecordableProfile`, because its range is sampled.

Conversion is mechanical, and it happens once, in the feature call. decad walks
`p.Outer` and each loop of `p.Holes` and reads each `BoundaryEdge`'s source `Entity`
for its defining fields. `Partial` selects the column of the table above, and
on a fragment `TExact` decides admission (§1). `Reversed` is **baked into the
segment** as the order of its range — `TStart` and `TEnd` swapped, so
`TStart > TEnd` says the segment runs against the curve's natural sense, and a
closed kind's `CCW` flips with it; `sketch` hands the range over in natural
direction, so the swap is decad's record of the walk, composed with that range.
The entity's fields are never reordered: a walked-backwards line still records
its entity's `Start` and `End` as the entity states them, and the walk's own
endpoints are read off the range. A `LoopRecord` therefore carries no residual
flags and no back-reference.

**What decad reads of `BoundaryEdge.Polyline`, and nothing more: `Polyline[0]` and
`Polyline[len-1]`, on every `Partial` fragment whose `TExact` reads `true`, as
the observations §1's falsifier tests the certified range against — read on the
fragments that record and on the ones the falsifier rejects alike, to check and
never to record.** They never enter a `Step`: every recorded value is the
entity's own defining data and the certified range, so no sampled content
reaches a `Recipe` through them, which is why core §2's "a `Recipe` never names
a tessellation" holds without qualification. On a fragment the flag already
rejects — `TExact == false` — the `Polyline` is not read at all; on a whole edge
the entity's own data is the record and the `Polyline` is never read either. No
interior point of a `Polyline` is ever read, and no `Polyline` content ever
enters a `Step`.

`CurveSegment` is one of the closed variant sets decad owns, so decad ships its
codec under core §6.2's serializability rule: each variant encodes as a tagged
object and decoding dispatches on the tag — exactly what no codec can do for
`sketch.Entity`, which is why the entity itself never enters a `Step`. Every
value a segment carries encodes: a `units.Value` field — a radius, a semi-axis,
a frame rotation — round-trips through its own text form (core §6.2), and a
curve parameter — a range, a knot, a weight, a `Rho` — is a plain dimensionless
float (core §5.2).

### 2.1 Decoded records

`RecordProfile` admits a live profile only after consuming these `sketch`
answers:

- source `Profile.Valid` said the region was valid;
- every partial boundary fragment had `TExact == true`;
- the reject-only range falsifier found no contradiction;
- whole entities were recorded from their defining data.

Serialization preserves the admitted geometry, not those answers. The original
live `sketch.Profile`, `BoundaryEdge.TExact`, and source arrangement are absent,
so a decoded or caller-built `ProfileRecord` is untrusted input, not an
exactness certificate.

`Recipe.Validate` independently re-proves the stored region. It reconstructs
the recorded entity definitions in a private sketch, asks `sketch` to build the
arrangement, and accepts only one valid arranged profile that exactly matches
the stored outer/hole walks, entity fields, ranges, order, and sense. That match
proves closure, loop simplicity, hole nesting/disjointness, and winding. Every
matched partial fragment MUST report `TExact == true` and pass §1's reject-only
range falsifier. No match, an ambiguous match, or any unproved property rejects;
a small residual never admits a trim.

Format version 1 carries no duplicated validity flag, certificate, hash, or
sampled polyline. A flag supplied by the same untrusted JSON would add no
evidence; the private arrangement is derived independently from the stored
analytic entities. The evaluator receives only the validated record and never
reads the private or original sketch.

`DecodeRecipe` protects resource use and wire meaning; it is not an
authenticity check. Applications that need to know who produced a recipe sign
or authenticate the complete encoded artifact outside decad. Full loading and
evaluation rules are in `docs/recipe-replay-design.md`.

## 3. `ErrUnrecordableProfile`

`ErrUnrecordableProfile` is the seam's one rejection of a *valid* profile: a
region `sketch` has proven closed, whose boundary decad nonetheless cannot
record exactly. It is not a validity judgement — `Profile.Valid` is `sketch`'s
answer and a separate gate (core §7) — and it is a sentinel error the caller
can branch on (core §12). It is returned in exactly two cases:

- **a `Partial` fragment whose cut `sketch` reports sampled** — `TExact ==
  false` (§1). An approximate parameter range is never recorded as an exact
  analytic fragment, never recorded as the whole entity — a different region
  than the caller drew — and never repaired;
- **a certified range the falsifier disproves** — `TExact == true`, but
  evaluating the source entity at the reported range does not reproduce the
  fragment's `Polyline` endpoints, the flag's own stated meaning (§1). The
  flag is disproven, decad rejects, and the discrepancy is reported upstream
  as a `sketch` bug.

A `Step` that recorded the whole curve where the caller drew a piece of it, or
an approximate range as an exact trim, would be the lossy record the
completeness rule forbids (core §6.2). So decad rejects: it never repairs,
projects or fits a point `sketch` handed over, and it never solves for one —
admission is decided by what `sketch` says, and a decad-side check may only
falsify an upstream claim, never bless one (§1). Widening the set of
recordable profiles is an upstream question about `sketch`'s arrangement, not
an API question in decad.
