# Spline Design

How the evaluator supports the free-form recorded segment kinds — `SplineSeg`,
`ClosedSplineSeg`, `NURBSSeg`, `ConicSeg`, `EllipseSeg`, `EllipticalArcSeg`,
`FitSplineSeg`.

`docs/sketch-seam-design.md` owns the recording contract and already records
every one of them. `docs/api-design.md` owns the public surface. This document
owns what happens BETWEEN them: which free-form questions the evaluator can
answer, at what exactness, by what construction, and which stay refused for
cause.

Read before writing any free-form geometry, any `NURBSSurface`/`NURBSCurve` code,
or any per-segment-kind dispatch.

Three tables are normative:

| Table | Owns | Section |
|---|---|---|
| **F** | per-kind exactness tier + admission | §3 |
| **R** | refusals + their sentinels | §4 |
| **C** | per-capability reach + construction | §8 |

## 1. Router

| Question | Section |
|---|---|
| which kinds work at all | §2 scope, Table F §3 |
| why a crossed spline is rejected | §2.1 |
| why an elliptical arc cannot build | §2.2 |
| how a spline's area is EXACT | §5 |
| what a repeated knot or a repeated control net does | §5.1 |
| what a knot multiplicity above the degree does | §5.1.1 |
| how a fit spline's area is EXACT, without re-running sketch's solve | §5.1.2 |
| how a length/extreme is bounded | §6 |
| what a revolve's area needs beyond length | §6.1.1 |
| why an undercut or radius reading is `Suspect` | §6.3 |
| why a build refuses on a bracket it cannot decide | §6.4 |
| how a free-form wall edge's convexity is decided, or refused | §6.5 |
| what surface an extruded spline gets | §7 |
| what each capability answers | Table C §8 |
| what stays refused forever | Table R §4 |
| landing order | §10 |

## 2. Scope — whole entities, joined end to end

The evaluator accepts a free-form segment whose recorded range spans the
entity's FULL domain, in walk order. It accepts no other free-form range,
because no other free-form range is recordable (§2.1).

A recorded free-form range is therefore always `[0, 1]` or `[1, 0]`. Every
construction here may assume it, and MUST NOT carry a partial-domain path that
cannot be reached — dead exactness machinery is worse than none, because a later
reader trusts it.

### 2.1 A trimmed free-form fragment is unrecordable

`geom.BoundaryEdge.TExact` is true for a CUT bound only when sketch's
closed-form kernel placed it, and that kernel runs only when BOTH curves of the
pair are a `Line`, `Circle` or `Arc`. Every contact involving a free-form curve
is sampled — a line crossing a spline included, a line TANGENT to one included.
So a free-form fragment always reports `TExact = false`, and seam §1 rejects it
as `ErrUnrecordableProfile`.

A WHOLE edge is bounded by the curve's own domain ends, not by a contact, so
`recordEdge` never consults `TExact` for it (seam §1). A closed spline, and a
chain of free-form curves joined at shared endpoints, record cleanly.

**Public consequence, and it MUST be documented on `Extrude`/`Revolve`:** a
free-form curve must meet other curves at shared endpoints, never by crossing.
The error names a cause the caller cannot guess, so the doc comment states the
remedy — join the endpoints in the sketch — beside the sentinel.

Lifting this restriction is an upstream ask (§9), never a decad-side repair. A
decad-side check may only falsify an upstream claim, never bless one (core hard
rules), and admitting a sampled cut because its residual is small is exactly the
unsound admission gate that rule forbids.

### 2.2 An elliptical arc's record is self-inconsistent

`EllipticalArcSeg` carries the parametric ellipse (`Center`/`Rx`/`Ry`/`Rotation`)
AND the entity's pinned `Start`/`End`. sketch pins those endpoints to solver
tolerance, so they lie on the parametric ellipse only approximately — `geom`
documents the miss as ~5e-3. seam §1 records the whole edge anyway, because a
whole edge's admission never consults `TExact`.

<!-- The ~5e-3 figure is a checkable claim about the pinned sketch module: it is
written in package geom, in `geom/region.go`, in the doc comment on the
`BoundaryEdge.TExact` field, which states that an EllipticalArc's ends are pinned
to the sketch Start/End points and that eval(t=0/t=1) misses the pinned Polyline
end by that tolerance. That is production code carrying the magnitude, not a test
tolerance. Package `sketch`'s own `profiles.go` states the same pinning rule
without a number, so `geom` is the site the figure comes from. -->

The two datasets disagree, and decad has no exact reconciliation:

- trusting the parametric curve moves the segment's ends off the neighbour's
  join point, so the loop no longer closes and the topology builder has nothing
  to build;
- trusting the pinned endpoints leaves no curve between them.

`ArcSeg` has the same shape of problem and decad already resolves it by
REFUSING an inconsistent record: `validateMomentSegment` computes both pinned
radii and rejects when they disagree. The elliptical analogue — do the pinned
ends lie on the parametric ellipse — fails on realistic input.

So `EllipticalArcSeg` is `ErrUnsupported` at the evaluator, permanently from
decad's side, pending the upstream fix (§9). A whole `EllipseSeg` is closed and
carries no pinned endpoints, so it is unaffected. `ConicSeg` is unaffected — a
rational quadratic Bézier interpolates its endpoints by construction.

## 3. Table F — per-kind exactness tier

The tier decides what a measurement over that kind may claim, never whether the
kind is admitted. Admission is §2 and Table R.

| Kind | Tier | Area / centroid / second moments | Construction |
|---|---|---|---|
| `SplineSeg` | **A** | exactly rational, rounded ONCE | §5 |
| `ClosedSplineSeg` | **A** | exactly rational, rounded ONCE | §5 |
| `NURBSSeg`, all weights equal | **A** | exactly rational, rounded ONCE | §5 |
| `ConicSeg` | **B** | `Approximate`, proven interval | §5.3 |
| `EllipseSeg` (whole) | **B** | `Approximate`, proven interval | §5.3 |
| `NURBSSeg`, weights unequal | **C** | `Approximate`, proven interval | §5.4 |
| `EllipticalArcSeg` | — | refused, §2.2 | — |
| `FitSplineSeg` | **A** | exactly rational, rounded ONCE | §5.1.2 |

**Tier A means the integral is exact, NOT that the reported measurement is
`Exact`.** The integral's value is an exact rational, and the `units.Value` the
measurement returns carries a single `float64` magnitude — `Mag()` in the
value's own unit, `Base()` in the kind's base unit. So the reported bound is a
SINGLE rounding of that rational into that magnitude, and it is zero — hence
`Exact` — exactly when the rational is representable in the magnitude the value
ACTUALLY CARRIES, never in an abstract float64: the same rational can be
representable in one unit and not in another.
A 5-control closed spline whose area is 293/18 reports `Approximate` with a
one-ulp bound; the same section scaled by 3 has area 293/2, is representable, and
reports `Exact`.

Equal weights are equal at any MAGNITUDE. They cancel in the homogeneous
quotient, so a `NURBSSeg` whose weights are all 1e300 names the very same curve
as one whose weights are all 1, and it is Tier A on the same terms. The
reconstruction decad asks sketch for region topology through is where the
magnitude bites — differentiating a rational curve squares its homogeneous
denominator, so weights past about the square root of the largest float64
overflow it and the entity reconstructs as no valid profile at all. The weights
are normalized to 1 for THAT reconstruction only: the question sketch is asked
is about the same curve, and the exact integration reads the recorded weights
untouched. Refusing instead would deny a measurement to a record that is exactly
representable and owes one.

That is precisely the standing the LINE path already has, and it is the point:
Tier A's bound is one rounding rather than a quadrature estimate, so free-form
support costs decad none of its exactness discipline. NEVER describe a Tier A
reading as unconditionally `Exact`.

A tier is a CEILING, never a promise about a specific reading. Arc length is
never exact in ANY tier (§6.1), so a Tier A body's `Area` always carries a
positive bound even where its `Volume` does not.

## 4. Table R — refusals

Every refusal appears once, with the sentinel picked by the existence test of
modify §1: no such body exists → `ErrDegenerate`; the body exists and this
evaluator cannot build it → `ErrUnsupported`; the INPUT cannot be recorded
exactly → `ErrUnrecordableProfile`.

| # | Condition | Sentinel | Permanent from decad's side |
|---|---|---|---|
| **R1** | free-form fragment (`TExact` false) | `ErrUnrecordableProfile` | yes, §2.1 |
| **R2** | `EllipticalArcSeg` reaches a build or an integral | `ErrUnsupported` | yes, §2.2 |
| **R3** | free-form walk in a section a `Shell` offsets | `ErrUnsupported` | yes for a curved walk, §4.1 |
| **R4** | `Fillet` corner with a free-form carrier | `ErrUnsupported` | yes for a curved walk, §4.1 |
| **R5** | `Chamfer` corner with a free-form carrier | `ErrUnsupported` | yes for a curved walk, §4.1 |
| **R6** | a Tier A free-form walk, `FitSplineSeg` among them, reaches a BUILD | `ErrUnsupported` | no, §10 P4 |
| **R7** | exact-rational conversion, length bracketing, integration or topology reconstruction exceeds its work budget | `ErrUnsupported` | no, §5.2, §6.1 |
| **R8** | chording a free-form walk needs more than the chord cap | `ErrUnsupported` | no, reuses `errTooManyChords` |
| **R9** | a `Verify` reading's proof does not close — its bracket cannot separate it from its threshold, or a §6.3 certificate fails | not an error — `Suspect` | no, §8 |
| **R10** | a Tier B or Tier C walk reaches a BUILD before its moments land | `ErrUnsupported` | no, §8 |
| **R11** | a free-form bracket cannot decide a BUILD-time comparison | `ErrUnsupported` | no, §6.4 |
| **R12** | an interior knot at multiplicity above the degree whose two one-sided limits are DIFFERENT recorded coordinates | `ErrDegenerate` | yes, §5.1.1 |
| **R13** | an admitted record whose Bézier extraction this evaluator cannot slice — a C0 join's stride, an over-clamped end knot's dead control | `ErrUnsupported` | no, §5.1.1 |
| **R14** | a free-form curve whose control points all coincide reaches a length bracket or an integral | `ErrDegenerate` | yes, §6.1 |
| **R15** | a free-form arc-length enclosure whose upper bound runs past `MaxFloat64` | `ErrUnsupported` | yes, §6.1 |
| **R16** | a `FitSplineSeg`'s fit points are finite but the interpolant sketch builds from them — its cumulative chord parameter or a span coefficient — is not | `ErrUnsupported` | no, §5.1.2 — a float64 range limit of sketch's own exported interpolant, not decad's to lift |
| **R17** | a `FitSplineSeg`'s converted chain does not reach its own record's natural-end fit point — sketch's dedup (`fitChordEps`) collapsed it into its predecessor | `ErrDegenerate` | yes, §5.1.2 |
| **R18** | a free-form directional-extreme enclosure no float64 interval holds — an end past `MaxFloat64`, or a width past it | `ErrUnsupported` | yes, §6.2 |
| **R19** | a free-form wall edge whose chain proves no single curvature sign — a span whose speed is not proven nonzero, a span's Bernstein certificate still mixed at the depth cap, two spans or a joint in conflict, or a joint the walk reverses at | `ErrUnsupported` | no, §6.5 |

<!-- R16 was challenged as narrower than the evaluator's actual refusal
behaviour, on the theory that a stalled (non-increasing) cumulative chord
parameter could reach ErrUnsupported through a clause R16 does not name.
Checked against sketch's geom/fitspline.go size(): at i=1 the loop tests
Params[1] first (finite for the 2-point prefix {0,0},{1e300,0}, which
succeeds with params=[0 1e+300]) and Points[1] (finite), so the only clause
that can return prefix 1 is !spanFinite(0) — a non-finite SPAN COEFFICIENT,
exactly what R16 names. The non-increasing-parameter check further down the
function is never reached at that prefix. This is structurally forced: a
stalled span has h=0 between distinct post-dedup points, so the natural-cubic
solve divides by zero and poisons the second derivatives the published span
coefficients are built from, which is what makes spanFinite(0) fail first.
Four stall shapes were tried, including ones with a healthy leading chain
({0,0},{1,1},{1e300,1},{1e300,1+1e-6}; {0,0},{1,1},{2,0},{1e300,0},
{1e300,1e-6}; {0,0},{1e300,0},{1e300,1e-6},{1e300,2e-6}), and every one
reported prefix 1 (the span-coefficient clause). A decad-side probe building
a ProfileRecord from a FitSplineSeg over {0,0},{1e300,0},{1e300,1e-6} plus a
closing LineSeg and calling Area() returned "a fit splines interpolant runs
off the float64 range and cannot be described, though the fit points are
finite" — errors.Is ErrUnsupported true, ErrNotFinite false, ErrDegenerate
false, the same sentinel and message R16 already names. -->

R9 is the one row that is not a refusal. An intent the evaluator cannot BUILD is
`ErrUnsupported` at the call; a `Verify` question it cannot ANSWER is accepted
and reads `Suspect` (evaluator §11). A free-form reading whose proven interval
straddles its threshold is the second case, and so is a direction cone §6.3
cannot certify. R11 is the first case reached from the same brackets: a build
gate has no `Suspect` to fall back on (§6.4).

**What a caller does while a row stands.** A curve refused here has one
substitute in the caller's own hands, and it is a different curve rather than a
workaround for the same one: build the sketch entity as a POLYLINE through the
points the free-form curve was defined by, which is analytic and which every
current build accepts. Nothing about that is decad's inference — the caller
chooses the chords, so the caller knows the sagitta, and the resulting body is
the exact prism over the polygon rather than an approximation of the intended
one. What the caller owes is to say so: a proof over a chorded section proves a
chord approximation of the intended part, and any volume or clearance assertion
it makes has to carry the chording error the caller introduced, on top of the
`Bound` decad reports.

The caller substitute above answers the BUILD path only. §5.1.2 states the
fit-spline reduction that makes a `FitSplineSeg` section Tier A for the
moments path: a caller with a `FitSplineSeg` section gets an exact `Area`,
`Centroid` and `SecondMoments`, with no chording of their own to account for.
What they do NOT get is a body — `Extrude` of that same section still
refuses, staged the same way every other Tier A free-form kind's build is,
under §10's P4.

### 4.1 Why the modify refusals stand

Modify §2 reduces every modify op to an EXACT rewrite of the recorded 2D
section. Each reason below follows from that reduction, not from missing effort
— and each is stated for a CURVED free-form walk, which is the whole of what the
exactness barrier covers. The STRAIGHT slice is refused by the same rows on a
different ground, stated after them.

- **R3.** `Shell` needs the section's exact offset. The exact offset of a curved
  polynomial spline is not polynomial, so no recordable section represents it.
- **R4.** A fillet's blend centre is the intersection of the two carriers'
  material-side offsets. A curved carrier's offset is not representable, so
  there is no exact centre to record.
- **R5.** A chamfer's foot sits a setback distance along the boundary curve,
  measured as ARC LENGTH. On a curved span that length is a bracket and never
  exact (§6.1), so the foot is not exact, so the chord's recorded endpoints
  would be approximate coordinates — which core §6.2 forbids outright.

**The straight slice is refused too, and NOT for those reasons.** A degree-1
`NURBSSeg` is a polyline: a rational degree-1 Bézier is a convex combination of
its two control points, so the point set is the polyline through the control
points whatever the positive weights. Every barrier above dissolves on it — the
exact offset of a straight walk is a straight walk with miter or arc corner
joins, all recordable; a corner between two of its segments has the exact blend
centre the line/line case already computes; and its length enclosure is a POINT,
chord and control polygon coinciding (§6.1), so its setback foot is the same
closed form the analytic `LineSeg` walk already records. R3–R5 refuse it anyway,
on the recorded segment KIND: no modify construction reads a free-form walk at
all, so admitting the straight slice means building offset, blend-centre and
setback cases for a kind that has none. That refusal is staging, so
`ErrUnsupported` is the right sentinel for it as well — the same sentinel, a
different reason, and the only part of R3–R5 decad could lift on its own. No §10
increment schedules it.

**The admissible slice, and it is not a refusal.** A corner whose BOTH carriers
are analytic stays buildable in a section that holds free-form walks elsewhere:
the rewrite is local to the corner, and every untouched walk re-emits verbatim.
What it needs beyond that is the modify §5 audit running over free-form
elements — a crossing test and a boundary-contact test no other capability here
calls for. §6.4 owns both constructions and the R11 refusal for a contact they
cannot decide.

## 5. Exact rational moments — Tier A

### 5.1 The reduction

Take the control coordinates exactly. They are floats, so they are exact
rationals — the same take-the-floats-exactly discipline `clearance_poly.go`
already uses.

**Take the KNOT VECTOR exactly too, from sketch, and NEVER re-derive it.** Every
number the conversion reads is upstream's own float lifted into a rational: a
`NURBSSeg`'s knots are recorded verbatim, and a `SplineSeg`'s are not recorded at
all, so the evaluator reads `geom.ClampedKnots` and lifts what it returns.
`geom` builds each interior knot as `float64(j)/float64(n−3)`, so sketch stores
the ROUNDING of `j/(n−3)` and not that rational; rebuilding the vector from the
closed form is the no-re-derivation rule broken, and the cost is not confined to
a bound. The exact area over the rational knots is often representable where the
area over sketch's own knots is not, so the conversion publishes a zero bound and
an `Exact` claim about a curve nobody recorded, and on a near-cancelling section
the published magnitude moves too. The two vectors agree only where `n−3` is a
power of two, which is why four- and five-control fixtures cannot see the
difference. Whichever kind is converted, the knots come from sketch and the
conversion is over them.

Convert the recorded curve to piecewise Bézier form. Knot insertion is a
rational convex combination, so the conversion is exact:

| Recorded kind | Conversion | Knot vector |
|---|---|---|
| `SplineSeg` | clamped uniform cubic → one Bézier per span | `geom.ClampedKnots(n)`'s floats, lifted |
| `ClosedSplineSeg` | periodic uniform cubic → one Bézier per span, `n` spans for `n` control points | none — the periodic uniform basis is per span |
| `NURBSSeg` | clamped arbitrary degree → one Bézier per NONEMPTY knot span | the recorded `Knots`, lifted |
| `FitSplineSeg` | natural cubic interpolant → one Bézier per interval between consecutive ACTIVE fit points | none — the natural cubic is per span, §5.1.2 |

**Repeated knots and repeated control points are admitted, so every construction
in §5 and §6 runs per NONEMPTY span and must hold on a COLLAPSED one.**
`validateNURBSSegment` checks the knots finite, non-decreasing, clamped at both
ends and the whole domain nonempty (`Knots[Degree] < Knots[n]`), the control
points finite and at least `Degree + 1` of them, every weight positive, and R12's
one continuity rule — an interior knot past `degree` whose two one-sided limits
are different recorded control points; the `SplineSeg`/`ClosedSplineSeg` arms
check control count, point finiteness and parameter range. Neither arm tests
distinctness anywhere, and a recording gate rightly does not. Three consequences
the constructions carry:

- **an EMPTY knot span — `t_i = t_{i+1}`, a repeated interior knot — carries no
  Bézier segment.** It enters no sum here and is not a span any §6 row runs on.
  The nonempty-domain check leaves at least one span to run on.
- **an interior knot whose multiplicity EXCEEDS the degree carries the two
  one-sided limits onto two RECORDED control points**, and whether the record
  describes one walk is decided by whether those two coordinates are identical —
  §5.1.1, R12 and R13.
- **a COLLAPSED span — every control point of that span the same point — is a
  span of zero length**, on which `C(t)` is constant and `C′ ≡ 0`. A walk whose
  spans ALL collapse has zero length, and the free-form walk owes it the refusal
  the analytic walk already makes: `validateMomentWalk`'s `ErrDegenerate`, a
  zero-length segment contributes no boundary — Table R R14, which §6.1's length
  bracket reaches on its own terms. That refusal does NOT reach one
  collapsed span inside a longer walk — four coincident controls in the middle of
  a clamped cubic net collapse a span while the walk's own length stays positive
  — so every §6 row must bound, bracket or refuse a collapsed span on its own
  terms. §6.3's speed floor is where that bites.

The conversion raises every interior knot to multiplicity `degree` and then cuts
consecutive spans that SHARE their boundary control point. A recorded interior
knot repeating MORE than `degree` times admits no such cut, whatever the curve
does there, so the conversion refuses it — narrowing the accepted input rather
than widening the converter, which would push a new closure question into the
moments path for no gain.

WHICH refusal is a fact about the recorded curve, not about the converter, and
Table R's R12/R13 split states it: a record whose two one-sided limits at the
knot differ is broken and rejected in NURBS validation, and one whose limits are
identical is a continuous curve this converter cannot slice and is admitted as a
record. A boundary knot over-clamped past `degree+1` is the same reading with no
discontinuity available at all — the extra repeat leaves a dead control point —
so it is R13 too.

The slicing must PROVE its own shape on the knot vector before it cuts a single
span. A divisibility test on the control count does not: a degree-3 curve with
four multiplicity-4 interior knots holds 16 control points, and 15 divides by 3,
so the stride-degree cut silently straddles the breaks.

Integrate the Green's-theorem boundary forms. Each integrand is a POLYNOMIAL, so
each integral is an exact rational — which the reported measurement then rounds
once, per §3:

| Reading | Boundary form | Integrand degree |
|---|---|---|
| area | `½∮(u dv − v du)` | `2p−1` |
| `∫u dA` | `½∮u² dv` | `3p−1` |
| `∫v dA` | `−½∮v² du` | `3p−1` |
| `∫u² dA` | `⅓∮u³ dv` | `4p−1` |
| `∫uv dA` | `½∮u²v dv` | `4p−1` |

Walk direction carries the sign through the recorded range order, exactly as
§2's `[1, 0]` case states. Nothing is reordered.

#### 5.1.1 A knot above the degree, and the record it still describes

**At an interior knot of multiplicity `m ≥ p + 1` the two one-sided limits are
two RECORDED control points, and the record describes one walk exactly when
those two coordinates are identical.** For the knot occupying knot indices
`j+1 … j+m` they are `P_j`, which ends the left piece, and `P_{j+m−p}`, which
starts the right piece — adjacent exactly when `m = p + 1`, with `m − p − 1` DEAD
control points between them otherwise. Weights do not enter: a single control
point projects to itself under any positive weight. Degree 1 is representative
rather than a special case; the same two limits stand at degree 2 with `m = 3`
and at degree 3 with `m = 4` and `m = 5`.

Equality is EXACT float identity, NEVER a tolerance. Both directions are exactly
decidable on the recorded floats, so no admission here rests on a residual —
which is what the core falsify-never-bless rule requires.

- **the two limits at DIFFERENT coordinates — R12, `ErrDegenerate`,
  permanent.** The record describes two disconnected walks, which close no loop
  and bound no region, so no such body exists.
- **the two limits at the IDENTICAL coordinate — a C0 join, ADMITTED.** The
  curve is bit-for-bit the concatenation of its Bézier pieces, which is the walk
  §2 already admits when the same shape is spelled as a chain of free-form
  curves joined at shared endpoints; a closed one closes with a gap of exactly
  `0` and encloses positive area. `record.go` admits it today —
  `validateNURBSSegment` tests distinctness nowhere — so the evaluator owes it a
  build or a staged refusal. Where the Bézier extraction cannot slice a stride
  whose spans share no boundary control point, that is R13 `ErrUnsupported`, and
  it is not permanent.

**An END knot above multiplicity `p + 1` is the same record with no piece on the
far side, and it is a valid body.** `validateNURBSSegment` requires the first
`p + 1` knots equal and the last `p + 1` equal, and rejects no repeat beyond
them: degree 2 with knots `[0, 0, 0, 0, 1, 1, 1]` and 4 control points is
admitted, and it is one quadratic Bézier over `P_1, P_2, P_3` with `P_0` DEAD. A
dead control point enters no nonempty span, so it enters no §5 sum, no §6 hull
and no bracket, and the walk is exactly the concatenation of its nonempty spans.
Admit it; where the extraction cannot slice it, that is R13 `ErrUnsupported`,
never `ErrDegenerate`.

#### 5.1.2 The fit-spline reduction

`FitSplineSeg` records the points sketch's natural-cubic interpolant passes
through, never the interpolant itself (`record.go`'s own doc comment); decad
never runs the interpolation solve. Sketch EXPORTS the solved result —
`geom.FitInterpolant`, via `geom.NewFitInterpolant`/
`(*sketch.FitSpline).Interpolant` — in both a defining `Params`/`Points`/
`SecondDerivs` triple and a `Spans()` monomial restatement, so decad consumes
the former for the moments path below, still without running the solve
itself.

**Consume `Params`/`Points`/`SecondDerivs`. NEVER consume `Spans()`.**
`FitInterpolant`'s own doc comment states the per-span formula the exported
triple defines the curve by — with `h = Params[i+1]−Params[i]`,
`a = (Params[i+1]−p)/h`, `b = (p−Params[i])/h`:

```
v(p) = a·v[i] + b·v[i+1] + ((a³−a)·m[i] + (b³−b)·m[i+1])·h²/6
```

and states that this is the arrangement `FitInterpolant.Eval` itself computes,
so a reconstruction using it reproduces `FitSpline.Eval` bit for bit.
`FitInterpolant.Spans()` restates the SAME curve in monomial form, but its own
doc comment says the two agree only "to rounding, not bit for bit" — its
`cubicSpanCoeffs` commits two or three float roundings per coefficient that
`Params`/`Points`/`SecondDerivs` never do. Integrating `Spans()`'s floats
exactly would publish a possibly-zero bound, hence a possible `Exact` claim,
for a curve displaced from the recorded one by an amount nothing bounds —
exactly the false-`Exact` failure `clampedUniformKnots`'s own doc comment
(§5.1) forbids for a `SplineSeg`'s re-derived knot vector, and the same
argument settles it here. `Params`/`Points`/`SecondDerivs` carry the standing
every other Tier A kind's defining data has: sketch's own computed floats,
taken exactly, over which the stated arithmetic IS the curve's definition.
`h` is the one term neither `Eval` nor `Spans` publishes, and both recompute
it in float64 (not exact in general); decad forms it as an exact rational
difference of the published `Params`, integrating the definition rather than a
float re-rounding of it.

**The closed form.** With `A = m_i·h²/6` and `B = m_{i+1}·h²/6`, the span's
four Bézier control values per coordinate reduce to

```
b0 = v_i
b1 = (2·v_i + v_{i+1})/3  −  h²·(2·m_i + m_{i+1})/18
b2 = (v_i + 2·v_{i+1})/3  −  h²·(m_i + 2·m_{i+1})/18
b3 = v_{i+1}
```

Every operand is `Params`/`Points`/`SecondDerivs` taken as an exact `big.Rat`,
so nothing rounds. `b3` of span `i` and `b0` of span `i+1` are both the
rational lifted from the same float `Points[i+1][coord]`, so consecutive spans
share their boundary control point EXACTLY — by identity, not by proximity —
which is what `bezierSpan`'s own contract requires.

**Deduplication.** `geom.NewFitInterpolant` collapses consecutive fit points
closer than an absolute `1e-12` (`geom`'s `fitChordEps`), keeping the FIRST of
each run. So the chain's endpoints are `Points[0]` and `Points[len-1]`, and the
LAST one is not always `Fit[len(Fit)-1]`: a record whose last two fit points
coincide within that threshold has a curve whose true end is the
second-to-last recorded point. decad integrates exactly the curve
`FitSpline.Eval` walks — `Points`, never the raw `Fit` — which is the correct
answer for that curve; the consequence is that the curve decad integrates need
not be the one the record's neighbouring segment still joins to. §2's
whole-loop join is normally sketch's own claim, never re-derived — but here
sketch's reconstruction rebuilds the SAME entity from the SAME `Fit` points
and reports the SAME dedup-collapsed curve every time, so it can never falsify
this particular mismatch. decad's own moments path therefore runs the one
self-consistency check that CAN: `requireFitSplineTerminalJoins`
(`moments_validate.go`) compares the converted chain's own natural-end
coordinate against `Fit[len(Fit)-1]` by exact identity — not a tolerance,
since the two floats are bit-identical whenever nothing was collapsed — and
refuses `ErrDegenerate` on any difference (R17): the record's own boundary
does not close, so no such body exists. It is a check of the SEGMENT against
its own record, not a re-derivation of the loop's join — the SplineSeg/
ClosedSplineSeg/NURBSSeg kinds need no such check, since none of their
conversions can drop a recorded endpoint this way (Table F). An all-coincident
fit set collapses to one active point and zero spans, which reaches R14 with
no special-case code: the identical shape the length bracket already refuses
on its own terms.

Every recorded free-form range is `[0, 1]` or `[1, 0]` (§2); a `FitSplineSeg`
trimmed to any other range refuses through `requireFullFreeformRange`, the
same "full domain" cause every other Tier A kind reports.

**R16.** `geom.NewFitInterpolant` returns `ErrNonFiniteFitInterpolant` when
finite fit coordinates give a cumulative chord parameter or a span coefficient
that leaves float64 range, or a parameter that stalls. The fit points
themselves are finite — checked by `spline_fit.go`'s own scan immediately
before the call, since `record.go`'s validation runs only at JSON decode and a
caller-built `ProfileRecord` reaches this reduction without ever passing
through it — so this is `ErrUnsupported` — the curve exists, described by
finite fit points, and this evaluator cannot state it — never `ErrNotFinite`,
whose subject is a non-finite INPUT and is refused by that same scan ahead of
this row.
`geom.ErrTooFewFitPoints` is unreachable (record validation floors `Fit` at 2,
re-checked defensively) and maps to `ErrDegenerate` with no table row of its
own.

**The exactness tier is A, in full**, on the same terms as every other Tier A
kind: the span's four Bézier control values are exact rational functions of
sketch's own published floats, so the Green's-theorem integrals are exact
rationals and §3's single-rounding rule applies verbatim. Nothing about a fit
spline may be described as unconditionally `Exact`; the reported bound is the
single rounding of the region's exact rational into the magnitude the
returned `units.Value` actually carries, zero exactly when that rational is
representable there.

### 5.2 Discipline

The exact rational is the only result. NEVER fall back to quadrature on a Tier A
kind — a float sum of Gauss nodes has no exact value to round from, so it can
never reach the zero bound a representable rational does, and `exactnessOf`'s
zero bound is a CLAIM that the value is exactly representable (`bounds.go`).

The held float MUST be the exact rational rounded once, never a separate float
evaluation of the same formula: a second evaluation would add its own error to a
bound that already speaks for the rounding.

Rational coefficient size grows with degree and span count. Charge every span,
every coefficient product and every integral term against a `freeformWork`
counter (`spline_bezier.go`), and refuse as R7 when it runs out. NEVER widen to a
float path to stay inside the budget.

The exact-rational counter is the RECORD's, not each segment's. One
`ProfileRecord` pass opens one counter and every segment in it charges that same
counter, because the work that actually runs is the aggregate: a counter opened
per segment reads a record of individually cheap curves as cheap however many of
them it holds, and bounds nothing.

That exact-rational counter spans the whole OPERATION, not one pass through the
record. A feature call reads the same record several times — the moments
preflight behind its area falsifier, the preflight the build runs again, and the
walk resolution that reads every segment after it — and each of those phases
charges work over the same curves. So the counter is opened where the operation
first touches the record, and every later phase spends what is left of it. A
counter minted per phase hands one record a fresh full ceiling each time, and a
later phase then runs work an earlier one already proved unaffordable; the
arc-length bracket (§6.1) is where that bites hardest, being the most expensive
of the three passes. A pass that legitimately holds no counter — a
re-evaluation under a rigid placement, a modify op's rewritten section, a
survey, an extent reading or a tessellation reading a body already built — opens
exactly one exact-rational counter for the record walk it is about to make. Never
one per segment, never one per loop, and never one inside the walk resolution
itself: a resolution handed no counter has no ceiling at all and refuses.

Charge EARLY as well as conservatively. The ceiling is fixed because the public
`ProfileRecord` methods take no context and so cannot be cancelled, so every
pass whose cost grows with the record must sit BEHIND a charge already levied —
the knot-multiplicity probes the conversion runs, including the probes that
insert nothing, and the sketch reconstruction validation samples the curve
through before any integral is taken. A ceiling consulted after such a pass
bounds nothing it was added to bound.

That applies to MEMORY as much as to time, and the conversion's own charge is
where it bites: a record's insertion demand is readable from SIZES and from data
already in hand as floats — a `SplineSeg`'s degree is fixed and the shape of the
vector `geom.ClampedKnots` returns follows from its control count, a
`ClosedSplineSeg` owes no insertion at all, and a `NURBSSeg`'s runs read off its
RECORDED knots — so the whole conversion is charged before one `big.Rat` exists.
Charging after the lift refuses a record that was hopeless from its control
count alone only once it has allocated two rationals per control point and a
whole rational knot vector. The open-spline charge is quadratic, so a refused
record allocates orders of magnitude more than any accepted one ever could.

A `FitSplineSeg`'s charge (`fitInterpolantCost`, §5.1.2) sits beside these,
levied from its `Fit` slice's own length before `geom.NewFitInterpolant`
allocates or solves anything. It is LINEAR, carrying no quadratic term at
all: a natural cubic interpolant gives one span per interval directly, with no
knot vector and so no insertion pass to charge for — unlike an open spline's
quadratic `clampedConversionCost`, which pays for repeatedly scanning and
copying growing control and knot vectors as each of possibly many knots is
inserted one at a time.

The RECONSTRUCTION carries its own counter. It is not covered by the conversion
and integration counter: that counter bounds decad's rational arithmetic, and
the reconstruction is sketch's — it chords each recorded source and ARRANGES the
result. Both counters span the record's whole operation, but their ceilings are
separate because their cost models are independent. Without the reconstruction
counter a kind whose conversion is linear clears the exact-rational ceiling at a
control count whose reconstruction runs for seconds, uncancellable, inside a
public measurement method.

That charge is the RECORD's, not any source's, because the arrangement is
GLOBAL: sketch chords every source it was given and then tests every PAIR of
chords in one loop over the whole scene. So the charge is a saturated quadratic
in the record-wide chord TOTAL. A sum of per-source squares is not a bound on
it — it drops every cross-source pair, which is nearly all of them once a record
holds more than one curve.

Every source counts toward that total, analytic ones included. A chord total
that skips the lines, arcs and circles beside a spline says nothing about the
pass they are arranged in, and a record naming NO free-form source at all is
charged on the same terms — its arrangement is the same global quadratic, and
the public methods that reach it take no context, so nothing else can stop it.

A source SEVERAL segments name is one source. The reconstruction interns the
entities it builds, so a circle one crossing cut into two fragments is chorded
once, and a chord total counting the fragments separately squares a total the
arrangement never holds. The interning is by the entity's own defining data.
Only the analytic kinds, whose key is a fixed-size record of that data, are
interned in the charge: a free-form key must read every control point, which is
a per-element pass this charge exists to precede, so free-form fragments stay
counted one by one — an over-count of the scene, never an under-count of it.

The chord counts are sketch's own, restated: a free-form source is chorded 16
times per control point (an OPEN spline per span, so 16 per control point less
its degree) with a FLOOR of 64, a circle 256 times, an arc 256 times its share
of a turn, a line once. The floor is what makes a record of many tiny curves
expensive, and a charge reading the control count alone misses it entirely.

The pass runs the arrangement more than ONCE, so the charge is one
arrangement's quadratic times the number of arrangements — which is what makes
an uncharged record cubic in its source count rather than quadratic. Validation
arranges the scene to list its candidate profiles, `RecordProfile` arranges it
again for each candidate it authenticates (`Sketch.Profiles` rebuilds the
arrangement on every call), and the whole pass repeats on the rescaled record.
The two whole-scene arrangements validation always runs are levied ONCE, at the
record-level preflight, before sketch is asked anything; each candidate's own
re-arrangement is charged on the same record's reconstruction counter immediately
before it runs.
Every arrangement is therefore paid for before it happens, and a record may
clear the preflight and still be refused inside the pass.

A record naming no free-form source is levied that same once at the
reconstruction's own entry rather than at the preflight, on the same
reconstruction counter and for the same amount. The reason is the exact
whole-circle certificate, which answers from disk containment alone and runs no
arrangement whatsoever: it is
reached only by an analytic record, and a charge levied ahead of it would refuse
a record no arrangement ever reads. The free-form charge stays at the preflight,
which no such certificate can be reached from, so it also covers an evaluator
preflight that never runs the reconstruction at all.

The exact-rational ceiling stands at 2^20 charged units. The reconstruction
ceiling stands at 2^26 charged units, admitting 5792 chords for validation's
first two whole-scene arrangements; candidate authentications spend that same
reconstruction counter. The larger reconstruction ceiling admits ordinary
analytic plates with several circular holes without widening the conversion and
integration ceiling. The conversion charges beneath the exact-rational ceiling
must still move ahead of the chains they precede, because a closed spline converts
linearly while integrating its chain costs some 270 times more per span, so a
record whose integration is over budget allocates every span before that ceiling
can see it.

One record-level preflight therefore owns every free-form cost before its first
expensive step. It owns FOUR things. Every cheap structural refusal is evaluated
on SIZES — knot count, degree, slice lengths, the recorded range — before any
array is scanned, so a record that cannot be well formed at any content refuses
in constant time however large a caller made it. The preflight levies the
conversion, re-anchoring, integration and, for a free-form record, the first two
whole-scene reconstruction arrangements. An analytic record levies those same
reconstruction arrangements at reconstruction entry, after the exact
whole-circle certificate. An analytic record whose known arrangement charge
already exceeds its ceiling refuses before per-segment interval validation; that
validation cannot make the reconstruction reachable. The chains the preflight
converted are what the moments pass integrates, so the conversion the record
paid for happens once. And a segment's TIER is decided before its CONVERSION
charge.

That last one is a rule about which ANSWER a caller gets, not about cost. Whether
a `NURBSSeg` is Tier A at all is a property of its recorded weights, so a
rational one owes its own Tier C reason (§5.4) rather than the R7 ceiling message:
the two are different answers — one says this evaluator has no certified
quadrature for a rational curve, the other says a record this size will not fit
the budget.

**The scope of that rule is the conversion charge, not every charge, and the
BOUND is why.** Deciding the tier means reading all `n` weights, so it is
inherently linear and no O(1) charge can follow it. A scan placed ahead of every
charge is therefore unbounded and uncancellable — exactly what the ceiling exists
to stop — so the SIZE-DERIVED rational-lift charge is levied first, from slice
lengths alone, and it bounds every element scan behind it. The order is the
recorded range, the slice sizes, the size-derived lift charge, the content checks
and the tier test, the conversion charge, and only then the rational lift itself.

What that charge bounds those scans BY is a constant factor, not an equality.
The property to state and to keep is: **every element-touching pass between the
size-derived charge and the conversion charge is a single walk over one array
whose own length is a term of that charge.** The charge therefore counts every
such array — the controls, the knots AND the weights — and a record's element
visits are at most `K` times the units it levies, so a whole record's are at most
`K·2^20` however the record is split into segments. Adding a validator of that
shape can only raise `K`; it can never unbound the work. A pass of any other
shape — one over an array no term counts, or more than a constant number of walks
over a counted one — is outside the invariant and owes a charge of its own,
exactly as the conversion's quadratic does.

Counting every walked array is what sets the admission boundary: a degree-1
`NURBSSeg` holds `n` controls, `n+2` knots and `n` weights, so it charges `4n+2`
units and the ceiling admits a few hundred thousand control points on size alone.
That is three orders of magnitude above anything that can produce a measurement,
because the conversion and integration charges cap a lone free-form source first.

What that costs is stated exactly: a record whose SIZE alone cannot fit the
ceiling reports R7 rather than its kind's reason. Every record within the
budget — which is every record that could ever yield a measurement — still reads
its own Table R reason, and a record over it is refused either way, so R7 is
equally true of it.

The per-element diagnostic is part of that bound. A description formatted for
every knot and every weight rather than only for the element that fails made the
content scan roughly seventy times the cost of the slice walk it is, so a "single
linear pass over slices the caller already holds" is a claim about the WALK and
the walk alone.

The independent-implementation rule stands. sketch computes its own free-form
area internally and reports it as `Profile.Area`; decad integrates its OWN
records (evaluator §4). The two agreeing is the §1 falsifier working. decad must
NOT consume sketch's moment internals even if they are exported later — a single
implementation cannot falsify itself.

### 5.3 Tier B — closed forms carrying transcendental terms

`ConicSeg` and whole `EllipseSeg` have closed-form moments carrying π, trig or
`atanh` terms. That is the standing `CircleSeg`/`ArcSeg` already have: the closed
form is exact in the reals, and the reported result is a `ratInterval` bracket
around the float evaluation (`moments.go`). Same machinery, new formulas.

### 5.4 Tier C — certified quadrature for a rational NURBS

For a rational curve with `u = U/W`, `v = V/W`, the moment integrand reduces:

```
u v′ − v u′ = (U V′ − V U′) / W²
```

A polynomial over a squared polynomial. The exact antiderivative needs `W`'s
roots and yields log/arctan terms, so exact integration is out of reach in
general.

`W > 0` is GUARANTEED — `record.go` validates every NURBS weight positive — which
is what makes a rigorous remainder bound reachable: bound the integrand's
derivatives on each span from `W`'s proven positive range, then bisect
adaptively until the remainder bound closes, under §6.1's iteration cap — a
measured target is not on its own a termination proof. The result is a proven
`[lo, hi]`,
never an adaptive estimate compared against itself. An estimate that measures
its own convergence is not a bound.

## 6. Bounds for the readings no tier makes exact

### 6.1 Arc length — a proven two-sided bracket

Arc length integrates `√(u′² + v′²)`, which on a curved span has no polynomial
antiderivative in any tier. Bracket it instead:

- the CHORD is a lower bound;
- the CONTROL POLYGON is an upper bound (variation diminishing);
- de Casteljau subdivision shrinks the gap toward zero.

**The reported bound is the MEASURED post-subdivision enclosure, never an
assumed per-level rate.** Recompute both bounds on the child spans, sum them, and
report the half width of the gap that sum actually shows. The familiar 4× per
level is ASYMPTOTIC only: the first levels of a span with a far control point
shrink the gap by substantially less, so a bound read off that rate claims a
precision the actual enclosure does not support. NEVER size a depth from a rate,
and NEVER state a width the enclosure has not been measured to reach.

**This bracket subdivides to a FIXED depth, because it has no target of its
own.** No reading downstream compares an arc length against a caller tolerance,
so there is no threshold for a loop to stop on; the depth is sized to bound the
work and the rational denominators instead. A fixed depth therefore promises no
relative width at all — what a span's enclosure comes to varies with the span,
and the reported bound is whatever that enclosure measured.

Where a target DOES exist, the loop belongs there: §6.4's build-time gate and
§6.1.1's product enclosure. Such a loop subdivides until the MEASURED enclosure
meets the target, under a HARD ITERATION CAP. The cap is not a safety net but a
termination proof: every leaf sum is rounded outward, so the measured relative
gap cannot fall below a small multiple of `2⁻⁵³` — a few tens of ulps on the
spans measured — and a target below that floor never arrives however long the
loop runs. `clearance_poly.go`'s `rpRefineRootContext` is the shape that already
does this correctly: a measured stopping predicate, a fixed iteration cap, and
the honest wide interval standing when the cap is reached.

Both bounds are proven, not sampled. Report the interval midpoint as the value
and its half width as the bound, `Approximate` always — a zero bound here would
be a false Exact.

A COLLAPSED span (§5.1) has a POINT enclosure: its chord and its control polygon
are both `0`, which is that span's true length, so its enclosure is already a
point at depth zero and the span contributes an honest `0` to the walk's sum. It
never reaches a reading alone — a walk of nothing but collapsed spans is the
zero-length walk §5.1 refuses.

A DEGREE-1 span's enclosure is a point too, its chord BEING its control polygon,
and the point it encloses is that span's exact length — which is what §4.1's
straight slice reads. It does reach a reading alone, and the `Approximate` rule
above still holds for it: the enclosed length is a float square root, so the
reported value carries that rounding and never a zero bound. So the rule speaks
for every length decad reports.

**A REPORTED half width of zero is therefore a REFUSAL, never a reading.** One
shape reaches it: the walk whose every span is collapsed, so the whole control
net is a single point and the control-polygon upper bound is `0` because the
curve is that point. No such curve bounds an arc, so it is R14 —
`ErrDegenerate`, the zero-length walk §5.1 already refuses, and the same answer
the moments path gives the identical record. Nothing else reaches it: a degree-1
span reports its float square root carrying that root's own rounding (above),
and on a curved span every subdivision level rounds the lower sum down and the
upper sum up, so a curve of positive length can never close its own interval.

**SCALE IS NOT A REFUSAL.** Every bound here is a float square root of an exact
rational squared distance, decided by exact comparison and seeded by a float
`sqrt`. Seeding it from that rational ROUNDED TO A FLOAT is what turns scale
into a refusal: a leg short enough to make the squared distance subnormal keeps
only a few significand bits, a leg long enough to make it overflow keeps none,
and either seed lands far outside the few-ulp adjustment walk, so the walk
exhausts and the bound escapes to its outward extreme. Seed by SCALING instead —
split the rational into mantissa and binary exponent, force the exponent even,
root a mantissa that always sits in `[0.5, 2)`, and scale back by half the
exponent, which moves no significand bit. Then every valid record measures at
every scale a finite coordinate can reach, and one near-duplicate control pair
cannot poison an otherwise ordinary curve. The seed is never the proof: the
exact comparisons still decide each bound, so a better seed can only remove
false refusals and can never widen or invert the interval.

**The one length that has no interval to report is R15.** A curve long enough
that its proven UPPER bound runs past `MaxFloat64` has no float64 interval,
so there is nothing to publish and it refuses — `ErrUnsupported`, because the
curve EXISTS and this evaluator cannot state its length. It is never
`ErrNotFinite`: that sentinel's subject is a non-finite INPUT, and every
coordinate reaching a bracket is finite. The test is on the ENCLOSURE, so a
length just under the top of the range whose upper bound is not representable
refuses too — the enclosure is the only length in hand, and refusing an answer
decad cannot state is right.

The subdivision is CHARGED like every other free-form pass (§5.2), and the
charge must read the span's DEGREE, not only the depth. Subdividing to depth `d`
makes `2ᵈ−1` exact splits and `2ᵈ` leaves, and one split blends all `n(n−1)/2`
de Casteljau pairs, so cost grows with depth and degree TOGETHER; a charge
counting leaves alone admits a single high-degree span whose splits run for
hours. Over budget is R7.

Consumers: a prism's side-face `Area` (`length × height`), `Edge.Length()`, and
the setback R5 refuses. A revolve's lateral area is NOT one of them — length
alone cannot determine it, and §6.1.1 supplies what it needs.

### 6.1.1 The radial first moment `∫ r ds` — what a revolve's area needs

Pappus's first theorem gives a revolved curve's lateral area as `θ · ∫ r ds`:
the sweep angle times the curve's FIRST MOMENT about the axis, never `θ ×
length`. Two meridian walks of equal length at different radii sweep different
areas, so §6.1's bracket cannot decide the reading on its own. Bracket the first
moment itself. This is the same quantity the analytic path already supplies per
segment kind (evaluator §6's swept-arc-length × sweep angle × centroidal radius),
so the free-form path owes its own bracket for it rather than a length.

`r` is AFFINE in the point. The axis is coplanar with the profile plane
(evaluator §6's axis gate), so in plane coordinates it is a line, and the region
lies in one closed half-plane of it — so with the sign chosen toward the material,
`r(u, v) = a·u + b·v + c` with `a² + b² = 1` and `r ≥ 0` over every walk.
Evaluating that affine form at the span's control points gives the span's radial
control values `r(P_i)`, and the convex-hull property bounds `r` over the span
with no root-find at all:

```
r_lo = min_i r(P_i)  ≤  r(t)  ≤  max_i r(P_i) = r_hi
```

A rational span uses the same control points — a positive-weight rational Bézier
lies in their hull too, and `W > 0` is proven (§5.4) — so this construction needs
no separate rational form.

**`r_lo` carries no sign, and the axis gate does not give it one.** The gate
proves `r ≥ 0` over the REGION and so over every walk on its boundary; a control
point is not a point of either, and a hull that dips across the axis holds a
negative radial control value while the curve stays strictly on the material
side. A span whose four radial control values are `1, −1, 3, 1` has the radial
polynomial `1 − 6t + 18t² − 12t³`, whose stationary points `t = (3 ∓ √3)/6` give
values `0.4226` and `1.5774` against endpoint values of `1`: the walk's minimum
radius is `0.4226 > 0` with `r_lo = −1`. So the hull bound is one-sided in sign —
a genuine lower bound on `r`, and a negative one proves only that the hull
crossed the axis. Clamp it with what the gate DID prove:

```
r_lo⁺ = max(0, r_lo)  ≤  r(t)   for every t on the walk
```

Per span, with `[L_lo, L_hi]` the §6.1 length bracket of that SAME span:

```
r_lo⁺ · L_lo  ≤  ∫ r ds  ≤  r_hi · L_hi
```

Both factors of each product are non-negative — `r_lo⁺` by the clamp, `r_hi`
because it dominates the walk's own values and the gate proves those
non-negative — and that is exactly what licenses multiplying the two lower
bounds together and the two upper bounds together. **NEVER write the lower bound as `r_lo · L_lo`.** Where `r_lo < 0` that
is the LARGER of `r_lo·L_lo` and `r_lo·L_hi`, so it is not the product interval's
lower end; it survives only on the separate fact that `∫ r ds ≥ 0`, which is a
different argument and a weaker bound than the clamp gives.

Sum the per-span enclosures. de Casteljau subdivision shrinks `[L_lo, L_hi]` and
`[r_lo, r_hi]` together — the hull converges onto the curve, so `r_lo` rises
toward the sub-span's own minimum radius and the clamp goes inert — and **the
stopping certificate is again the MEASURED enclosure, never an assumed rate**
(§6.1). A span still holding `r_lo < 0` contributes a lower bound of `0`:
honest, slack, and no reason to stop. A COLLAPSED span (§5.1) contributes
`[0, 0]` — `L_lo = L_hi = 0`, and its hull is the single walk point whose radius
the axis gate already proved non-negative, so the clamp needs no exception
there. Subdivide until the measured product enclosure meets its target, under
§6.1's iteration cap. Report
the interval midpoint as the value and its half width as the bound,
`Approximate` always.

This bracket serves the lateral area alone. A revolve's `Volume` takes Pappus's
SECOND theorem over the region's area first moment `∫u dA`, which §5 integrates
exactly as a rational — a different integral, and the exact one. A partial
sweep's cap areas are the §5 region area, likewise exact.

### 6.2 Extremes, sagitta, normals and curvature reduce to one existing engine

`clearance_poly.go` already owns a certified polynomial root engine — `ratPoly`
over `math/big.Rat`, Sturm chains, square-free reduction, Cauchy root bounds,
root isolation into intervals that cannot lie, and deterministic bisection under
a fixed depth budget, all context-aware. In Bézier form every free-form question
decad needs is one of its root problems. Reuse it. Do NOT fork a second root
finder.

**Every row splits on the span's form, and the middle column's identities hold
ONLY on a POLYNOMIAL Bézier span** — Tier A (Table F). Applying one to a span
that is not a polynomial Bézier understates the extreme and unsounds the proof
that reads it:

- a RATIONAL span — Tier C's unequal-weight `NURBSSeg`, and `ConicSeg`, which is
  a rational quadratic — takes the RIGHT-hand column. Write it `u = U/W`,
  `v = V/W`, with `W > 0` and its positive range proven (§5.4), and clear the
  positive denominator before isolating anything;
- a whole `EllipseSeg` is neither. Its record is the parametric ellipse, so
  §5.3's closed forms answer these questions directly, exactly as they do for
  `CircleSeg`/`ArcSeg`.

The chord-sagitta row is the one exception, and it is not an identity: both its
columns MEASURE the same control-point distance, which §6.2.1 justifies for a
rational span unchanged.

| Question | Polynomial Bézier span (Tier A) | Rational span | Consumer |
|---|---|---|---|
| directional extreme | `d/dt(g·C(t)) = 0`, degree `≤ p−1` per span | `(g·U)′W − (g·U)W′ = 0`, degree `≤ 2p−1` per span — the positive `W²` denominator cleared | `extentAlong`, `Box`, through-all stops |
| chord sagitta | the control points' distance to the chord SEGMENT `P_0 P_p`, MEASURED per subdivision level (§6.1) | the same distance on the span's own control points, MEASURED per level | `chordCount`, tessellation |
| tangent/normal DIRECTION cone | hodograph control hull — a degree `p−1` Bézier with control points `p·ΔP_i` — a CONE only where that hull excludes the origin (§6.3) | the control hull of the numerator hodograph `(U′W − UW′, V′W − VW′)`, degree `≤ 2p−1` — the positive `W²` scales `C′` and never rotates it, so the direction cone is the numerator's, under the same origin-exclusion test (§6.3) | undercut survey |
| speed, for a Lipschitz bound | that same hodograph hull's maximum norm | the numerator hodograph hull's maximum norm divided by the square of `W`'s proven positive LOWER bound | extreme-VALUE brackets |
| curvature extreme | `2K′S − 3KS′ = 0` with `K = u′v″ − v′u″` and `S = u′² + v′²`, degree `≤ 4p−6` — PLUS both span endpoints; the bracket needs `S`'s proven positive floor, so a span without one is `Suspect` (§6.3) rather than a candidate list | the same stationarity over the rational derivative forms, the positive powers of `W` cleared before isolation — plus the same endpoint and speed-floor cases | `MinRadius` |

`K` is the curvature NUMERATOR, so **`K`'s own roots are the inflections**
(`κ = 0`, infinite radius) — the opposite end of the range `MinRadius` reports.
They are not the candidate set and NEVER stand in for one: a span can hold its
tightest radius at a parameter where `K` is far from zero.

An extreme VALUE bracket follows from the isolated parameter interval plus the
row's own Lipschitz bound, exactly the pattern `clearance_poly.go` already uses
for its critical values. Bracket EVERY isolated root and both span endpoints
before reporting a `Box`, a through-all stop or a `MinRadius`: a candidate set
that misses an interior root understates the reading, which is the direction that
breaks the proof rather than merely widening it.

**A stationarity polynomial that is identically ZERO makes its objective
constant, and a zero root count is never on its own the proof of anything.**
Isolation returns an empty list for every polynomial below degree 1 —
`clearance_poly.go` trims trailing zero coefficients and returns early for
`rpDeg < 1` — so a nonzero constant, which genuinely has no root, and the zero
polynomial, which is a root everywhere, come back identical. The rows above
survive that because each candidate set carries BOTH span endpoints and a
constant attains its extreme there: a collapsed span (§5.1) has a constant
directional extreme and a zero sagitta, and the endpoints report each exactly.
The curvature row reads no candidate list on such a span at all — §6.3's speed
floor gates it first, and there `S` is the zero polynomial. That floor is the one
certificate reading the COUNT itself as its proof, so it owes the extra test §6.3
states.

**Contract consequence.** A `Box`'s bound is the directional-extreme bracket's
own, so the split is between a bracket with a nonzero width and one without —
never between a free-form section and an analytic one. A free-form interior
extreme is an irrational root evaluation, so **a `Box` whose extreme is held by
such a candidate is `Approximate`** with the bracket's bound. A section whose
extremes are all held by exactly representable candidate values reports a zero
bound and stays `Exact`, and a free-form section reaches that reading too: a
Bézier interpolates its endpoints exactly, so a span monotone in the direction
being read (`P′` with no root in `[0, 1]`) contributes its two endpoint values
and nothing else, and where every one of them is representable under the
functional there is no width to report. An all-analytic prism's `Box` keeps its
zero bound and `Exact` unchanged, as before. `prismBoundsContext` reports
exactly that split (P4a) — reachable only from internal tests while R6 stands,
since no free-form prism can exist through the public surface yet.

**The one extreme that has no interval to report is R18**, §6.1's R15 one row
over. The enclosure is exact and rational, so it is proven whatever its
magnitude, but the reading published from it is a float64 midpoint and a float64
half width: an end past `MaxFloat64`, and a width past it between two ends that
are not, each leave nothing to publish. Both refuse `ErrUnsupported` — the curve
EXISTS and this evaluator cannot state this extreme of it — and never
`ErrNotFinite`, whose subject is a non-finite INPUT while every coordinate
reaching the bracket is finite, nor `ErrDegenerate`, which claims no such body
exists at all. The refusal is on the ENCLOSURE and belongs to the CONVERSION
that publishes it, so it must fire before the enclosure joins a fold: an
infinity folded into a running extreme is indistinguishable from a candidate
nothing contributed, and the reading then reports the empty region's own
`ErrDegenerate` — the opposite existence claim — or an infinite `Box` bound.

### 6.2.1 Which distance the sagitta row measures

The chording error a chord commits is the curve's distance to that chord as a
SEGMENT, so the sagitta row measures the control points' distance to the segment
`P_0 P_p` and to nothing else. Three quantities are within a careless reading of
each other here, and only one of them is the bound:

- **distance to the chord's carrier LINE understates it.** A span may overshoot
  its own endpoints along that line. The control net `(0, 0)`, `(−3, 0.01)`,
  `(4, 0.01)`, `(1, 0)` has every control point within `0.01` of the line through
  its ends, while the curve reaches `u ≈ −0.76` — `0.76` beyond the chord, and
  `0.76` from it. Perpendicular distance to an infinite line is not a chord
  bound.
- **the parametric deviation `|C(t) − L(t)|` is a different quantity**, larger
  than the sagitta and not bounded by the control points' deviations on a
  RATIONAL span. The weights reparameterise: the collinear net `(0, 0)`,
  `(1, 0)`, `(2, 0)` with weights `1, 1, 100` has every control point ON its
  linear interpolant, yet `|C(1/2) − L(1/2)| ≈ 0.96`. Its sagitta is `0` — which
  the segment distance reports correctly.
- **distance to the SEGMENT is the bound, and it holds unchanged on a rational
  span.** Distance to a convex set is a convex function, so its maximum over the
  control hull is attained at a control point; every curve point is a convex
  combination of the same control points, positive weights included (`W > 0`,
  §5.4). That argument reads no parameterisation at all, which is why the row's
  two columns measure the same thing and why §11 asks for no rational fixture
  here. It reads no NONDEGENERACY either: a collapsed span's chord is a single
  POINT (§5.1), distance to a one-point convex set is convex like any other, and
  the same maximum-at-a-control-point argument reports the sagitta `0` — that
  span's true deviation, and one chord covers it.

### 6.3 The speed floor, and when a direction cone is a cone

**A hodograph control hull that CONTAINS the origin encloses every direction, so
it proves nothing.** The cone is what supplies a free-form face's proven normal
interval, and verification §6 decides undercut membership from that interval
pointwise. A whole-plane interval straddles at every point, so no point provenly
opposes and none provenly clears. Read carelessly that face is simply not listed
and its body passes — the silent pass §8.1 forbids. Read honestly it is
undecided, which is `Suspect`. Two mechanisms put the origin inside the hull:

- **zero speed.** `C′(t) = 0` at some `t` puts the origin ON the hodograph, so
  every hull covering that `t` contains it and no subdivision escapes. A recorded
  net can do exactly this: the 4-control `SplineSeg` `(−1/8, 1/4)`,
  `(1/8, −1/12)`, `(−1/8, −1/12)`, `(1/8, 1/4)` — clamped, so §5.1's conversion
  gives the one Bézier span over that same net — has `C′(1/2) = (0, 0)` exactly,
  with `C″(1/2) ≠ 0`: an ordinary cusp, where the tangent reverses and the curve
  has no direction at all. `record.go` admits it, and rightly so — the `SplineSeg`
  arm checks control count, point finiteness and parameter range, and a speed test
  is not a recording question. That same arm admits an ALL-COINCIDENT net, where
  `C′ ≡ 0` across the whole span rather than at one `t`: the hodograph hull is
  the origin itself and `S` below is the zero polynomial (§5.1).
- **a wide turn.** A tangent turning through half a revolution across one span
  sweeps the hull across the origin even where the speed never vanishes.

Both mechanisms are decided EXACTLY, never at a tolerance — the control points
are floats taken exactly as rationals (§5.1):

- **the speed floor, on a POLYNOMIAL span.** `S(t) = u′² + v′²` is a polynomial
  with exact rational coefficients, degree `≤ 2(p−1)`, and `S ≥ 0` everywhere by
  construction. Test its COEFFICIENTS before its roots: an identically zero `S`
  is a collapsed span (§5.1) — a curve with no speed anywhere — and it FAILS this
  certificate outright. Its root count is not evidence either way, because
  isolation returns the same empty list for the zero polynomial as for a positive
  constant `S` (§6.2). For a NONZERO `S`, count its real roots on the closed span
  with `ratPoly`'s Sturm chain (§6.2): `S ≥ 0` with no root there proves `S > 0`,
  and bracketing `S`'s minimum over its own isolated stationary points and the
  span endpoints gives the floor `s_min` that the Lipschitz and curvature
  brackets need. The certificate closes only where that bracket's LOWER end is
  strictly positive; a bracket reaching `0` is a failed certificate, never a
  floor.
- **the speed floor on a RATIONAL span — divide by `W_max⁴`, never by nothing.**
  `C′ = H/W²` with `H = (U′W − UW′, V′W − VW′)` the numerator hodograph, so the
  squared speed is `S = S_num/W⁴` with `S_num = |H|²` the numerator's own
  polynomial. Run BOTH tests above on `S_num` — the positive `W⁴` moves no zero
  and no sign — for the numerator floor `h_min`. Then divide:

  ```
  s_min = h_min / W_max⁴
  ```

  `W_max` is the upper end of `W`'s proven positive range (§5.4), read off the
  span's largest Bézier weight because `W`'s Bernstein coefficients ARE those
  weights. **NEVER hand `h_min` itself to the Lipschitz or curvature brackets.**
  It overstates the floor by as much as `W_max⁴`, and §5 normalizes no weight
  anywhere — a net carrying weights `1, 1, 100` is admitted as recorded — so
  nothing bounds `W_max` to `1`. The controls `(0, 0)` and `(1, 0)` at weights
  `1, 2` show the whole gap exactly: `H ≡ (2, 0)` gives `h_min = 4`, while
  `C(t) = 2t/(1+t)` has `|C′|² = 4/(1+t)⁴` and a true minimum of `1/4` at
  `t = 1` — `W_max⁴ = 16` times smaller. An inflated floor understates curvature,
  which OVERSTATES the `MinRadius` Table C advertises as proven, so the error
  runs in the unsafe direction. The mirror bound divides by its own power of `W`
  the same way — §6.2's speed row.
- **origin exclusion.** Whether the origin lies in the convex hull of the
  hodograph's control points is an exact rational sign test — no root-find. Where
  a hull fails it, subdivide the hodograph and retest the children, and run that
  subdivision ONLY on a span whose speed floor closed: the termination argument
  below rests on that floor, so subdividing a span without one can only spend the
  budget on a reading already `Suspect`. The child hulls shrink toward a curve
  that stays a proven distance from the origin, and the separation is the SQUARE
  ROOT of the floor of whatever quantity the SUBDIVIDED hull bounds — `√s_min`
  on a polynomial span, whose hull is `C′`'s own and whose floor `s_min` floors
  the squared speed `S`, and `√h_min` on a rational span, whose hull is the
  numerator's (§6.2). The two are not interchangeable: `h_min = s_min·W_max⁴`
  sits BELOW `s_min` wherever `W_max < 1`, so reading `s_min` in the numerator
  plane would claim a separation that hull does not have and exclude the origin
  too early. A child hull whose own diameter has fallen below its span's own
  separation cannot contain the origin. So the subdivision terminates.

What each failure costs, per R9 — a `Verify` question the evaluator cannot answer
is accepted and reads `Suspect`:

| Certificate | Fails when | Cost |
|---|---|---|
| speed floor `s_min > 0` | the tested polynomial — `S`, or a rational span's `S_num` — is identically zero (the collapsed span), or has a root on the span, or its bracketed minimum reaches `0` | `Undercuts` AND `MinRadius` read `Suspect` for that body |
| origin exclusion on every subdivided hull | the turn is too wide to separate within the subdivision budget | `Undercuts` reads `Suspect` |

**Neither failure refuses a BUILD.** Chording, volume, area, export and the
boolean path read no direction cone, so nothing in this section withholds a
body — the readings that need a direction are the only ones it leaves
undecided, and a body it leaves undecided still takes part in an interference
proof. A collapsed span shows the whole cost: it fails the speed floor, so it
costs those two readings, on a body that still builds and reports its
`Volume`. What refuses an ALL-collapsed walk at build time is §5.1's
zero-length walk rule, never this certificate.

**A zero-speed span does cost a body, under a DIFFERENT certificate.** §6.5
proves a wall edge's `convex` bool from the curvature numerator's sign, and a
sign of the SIGNED curvature exists only where the speed is nonzero, so §6.5
demands the same `S > 0` this section's floor starts from — as a
precondition, without the floor's bracketing — and refuses R19 where it is
unproven. That refusal is a build refusal, since evaluator §3 decides `convex`
at build and a bool has no `Suspect`. So the cusp net above, and equally an
ordinary spline whose first two control points coincide, reaches a build
refusal through §6.5. It never reaches one through this section, whose own
failures still cost `Undercuts` and `MinRadius` and nothing else. The
collapsed span keeps building because §6.5 skips it entirely: it carries no
verdict and no joint of its own.

### 6.4 A build-time comparison a bracket cannot decide

`Suspect` is a `Verify` answer. A BUILD has no such fallback — it produces a body
or refuses, exactly as the modify audit's undecidable nesting does (modify §5
S9). So a gate that compares a free-form bracket against a threshold at build
time refuses as R11 when the bracket straddles it. Two gates do:

- **a through-all stop's in-path test.** `stops.go` decides, on each body's own
  `extentAlong` interval and the displacement that reading publishes, whether
  the sweep meets that body. A free-form body's extent is a bracket (§6.2), and
  the decision is made OUTSIDE it — beyond the sketch plane by more than the
  displacement, or short of it by more — so a bracket that straddles the sketch
  plane in the travel sense is R11, never a guessed dependency. The far side the
  stop then records charges the same displacement to the level's own axial
  displacement (evaluator §5), so a met body's bracket refuses nothing.
- **the §4.1 analytic-corner slice's audit.** The modify §5 crossing and
  boundary-contact audit must run over the section's free-form walks. A crossing
  is a root problem for §6.2's engine, exactly; the contact floor `δ = ε·D` needs
  a certified minimum-distance bracket between two spans — a control-hull
  branch-and-bound, structurally §8.1's. A contact that bracket cannot decide is
  R11.

R11 is not permanent: refining the bracket decides every case but an exact
tangency, and a tangency is a contact the §5 audit refuses anyway. That
refinement is a measured-target loop, so it carries §6.1's iteration cap.

**Current state.** The through-all stop's gate IS the straddle test above:
`stops.go` reads each payload's `extentAlong` interval beside its displacement,
charges that displacement to the level it resolves, and refuses only where it
straddles the sketch plane in the travel sense. The wider refusal — ANY nonzero
bracket bound — survives in `prismPayload.extentAlongWork` and its revolve twin,
which serve the one consumer that has no bound to widen and falls back rather
than fails: `clearance.go`'s separating-plane short-circuit. The §4.1
analytic-corner slice's audit gate is not implemented yet (P10).

### 6.5 A wall edge's convexity — proven from the curvature numerator's Bernstein coefficients, or refused

Evaluator §3 decides a wall edge's `convex` bool without ever measuring a 3D
dihedral: a circular wall reads its own turn (counter-clockwise convex,
clockwise concave), and a straight wall — having no turn of its own — reads
its loop's role instead. Neither test reaches a free-form wall. Its curve's
signed curvature is not fixed the way a circle's or a line's is — it can
change sign INSIDE one recorded span — so neither `true` nor `false` is a
fact about the curve in general, and Table F's per-kind tier says nothing
about it either: a tier decides what a MOMENT may claim, never what a
boundary sign is.

**This section is TOTAL over what §5.1's conversion produces.** Its subject is
that conversion's own output and nothing wider: a CHAIN of polynomial Bézier
spans of degree `p ≥ 1`, each holding `p + 1` exact rational control points,
beside the reversal flag `freeformBezierSpans` reports. Every span and every
joint such a chain can hold ends below in a stated verdict or a stated
refusal. Table K enumerates them, and a shape reaching this section that Table
K does not name is a gap in this section rather than a licence to read a
coefficient set no clause defines.

**Table K — every shape §5.1's conversion can hand this section**

| Shape | Where it lands |
|---|---|
| a span of degree `p = 1` | `C″ ≡ 0`, so `K ≡ 0` exactly: a degree-`0` Bernstein form holding one coefficient, that coefficient `0` → verdict `0` |
| a span of degree `p = 2` | `K` is the constant `4·cross(ΔP₀, ΔP₁)`, carried at the STATED degree `2p − 3 = 1` as two equal coefficients |
| a span of degree `p ≥ 3` | `K` at the stated degree `2p − 3` |
| a COLLAPSED span | no verdict and no joint of its own; the chain skips it |
| two or more CONSECUTIVE collapsed spans | skipped as one RUN, whose neighbours pair across the whole run |
| a collapsed run at an open chain's FIRST or LAST position | no pair to make, so no joint is formed and the run contributes nothing |
| a chain of exactly ONE span | the joint set is EMPTY and the fold over it is the identity `0`, so the chain's verdict is that span's own |
| a chain whose every span is collapsed | never reaches this section — the zero-length walk is R14, `ErrDegenerate` (§5.1) |
| a zero-length control edge at a span's START | `S(t_lo) = 0`: the endpoint value paired with the root count refuses, R19 |
| a zero-length control edge at a span's END | `S(t_hi) = 0`: the half-open count itself refuses, R19 |
| a zero-length control edge INTERIOR to a span | admitted: the speed need not vanish anywhere, and only a span's FIRST and LAST control edges are ever read |
| the joint a SUBDIVISION creates | verdict `0`, known from the split rather than folded |
| a chain that CLOSES on itself (`ClosedSplineSeg`) | the closing joint is interior to that one wall edge and folds like every other |
| a walk recorded in the REVERSE sense | the chain folds unreversed and the verdict is negated once at the end |
| a `FitSplineSeg`'s converted chain | always cubic spans (§5.1.2), so no degenerate degree arrives from that kind |
| a Tier B or Tier C span | never reaches this section — R10 refuses the build first |

**The certificate is Bernstein positivity on the curvature numerator.** The
sign of a curve's signed curvature is the sign of `K = u′v″ − v′u″`, the
curvature NUMERATOR §6.2 already names, and on a polynomial Bézier span of
degree `p ≥ 2` (Tier A, Table F) `K` is itself a polynomial of degree
`≤ 2p − 3` with exact rational coefficients. Both factors of each product are
Bernstein forms of the span's own control points — the hodograph §6.2 names,
control points `p·ΔP_i` of degree `p−1`, and its own hodograph in turn,
control points `p(p−1)·Δ²P_i` of degree `p−2` — and the Bernstein
coefficients of a PRODUCT of two Bernstein forms are a binomial convolution
of the factors' own, so `K`'s Bernstein form over the span is exact and
rational with no conversion to the monomial basis and no rounding anywhere.

**The degree the coefficients are carried at is the STATED one, and at `p = 1`
that degree is `0`.** A degree-1 span holds a 2-point net, which has no second
difference at all, so the `p(p−1)·Δ²P_i` factor above does not exist and
`2p − 3` names no degree: `C″ ≡ 0` there, `K` is identically zero, and the
span IS a straight segment. Carry it as a degree-`0` Bernstein form holding
one coefficient, that coefficient exactly `0`, which the all-zero rule below
reads as verdict `0`. That is an ordinary admitted input rather than an edge
case: `record.go`'s `validateNURBSSegmentSizes` refuses only `Degree < 1`, and
Table F puts a unit-weight `NURBSSeg` in Tier A whatever its degree, so a
3-control degree-1 record converts to two degree-1 spans and reaches this
section. Above `p = 1` the coefficient array's length is fixed by the stated
degree `2p − 3` regardless of a vanishing leading coefficient, and NO verdict
depends on `K`'s true degree: a `p = 2` span's `K` is the constant
`4·cross(ΔP₀, ΔP₁)` — one degree below the stated bound, and exactly `4` times
that span's lone control-polygon turn — carried as the two equal Bernstein
coefficients of a degree-1 form, whose signs are that constant's own
(`spline_convexity_internal_test.go` pins both cases).

**The span's speed must be proven nonzero BEFORE a single coefficient is
read.** `sign(K)` is the signed curvature's own sign only where the speed is
nonzero: signed curvature is `K/|C′|³`, so a parameter with `C′(t) = 0`
carries no curvature sign at all, and the wall extruded through that point
carries no normal either. `K` stays perfectly well behaved there, so the
coefficient test cannot see the cusp on its own — it reads a strict sign off
a span that has none and publishes `convex` for a wall edge the curve doubles
back on. So run §6.3's own speed polynomial on the span first, and read no
coefficient until it closes:

- `S(t) = u′² + v′²` is a polynomial with exact rational coefficients (§6.3)
  and `S ≥ 0` by construction, so `S` with no root on the CLOSED span is
  `S > 0` on it.
- `ratPoly`'s Sturm chain (`clearance_poly.go`) counts roots on the HALF-OPEN
  `(t_lo, t_hi]`, so pair a count of `0` with `S(t_lo) ≠ 0` and the closed
  span is covered. That pairing is not a formality: a net whose first two
  control points coincide has its only root exactly at `t_lo`, which a
  half-open count alone reports as no root at all.
- a nonzero count, or a zero at `t_lo`, leaves the span's speed unproven.
  Refuse, R19.
- `t_hi` needs no clause of its own. The count's interval is half-open at the
  LOW end and CLOSED at `t_hi`, so a span whose LAST control edge has zero
  length — `S(t_hi) = p²·|ΔP_{p−1}|²` — is a root the count already sees, while
  the same edge at the span's START is what the endpoint value catches. Each
  end is covered by a different half of the same test, which is why both halves
  are stated.
- a zero-length control edge INTERIOR to a span makes `S` vanish nowhere in
  general, and this precondition admits such a span. It is right to: the tangent
  is defined across the whole closed span. `S(t_lo) = p²·|ΔP₀|²` and
  `S(t_hi) = p²·|ΔP_{p−1}|²`, so a span that reaches a verdict at all has its
  FIRST and LAST control edges already proven nonzero, and those are the only
  two edges any rule below reads.

This is the same machinery §6.3's speed floor runs and a WEAKER demand on it:
the floor brackets `S`'s minimum to publish a numeric `s_min`, while this
certificate needs only that `S` never vanishes, so it stops at the root count
and brackets nothing. It also runs ONCE per span, before any subdivision: a
dyadic child's curve is a sub-arc of its parent's, so it inherits the
parent's proven nonzero speed and never retests it.

A COLLAPSED span is the one span the precondition does not refuse, because it
is the one span whose coefficients are never read: it has no verdict and no
joint of its own, and the joint rule below is what pairs across it. A chain
whose spans ALL collapse never arrives here at all — that walk has zero length
and is refused R14, `ErrDegenerate`, by §5.1's own rule before any wall edge is
decided — so the fold below is never asked for a verdict over an empty span
set.

Now read `K`'s coefficients:

- every coefficient `≥ 0` and at least one strictly `> 0` → by the convex-hull
  property a Bernstein form's values lie inside the convex hull of its own
  coefficients, so `K ≥ 0` across the WHOLE span and `K` is not the zero
  polynomial. The span never turns the other way. Verdict `+`.
- the mirror — every coefficient `≤ 0`, at least one strictly `< 0` →
  verdict `−`.
- every coefficient exactly `0` → `K ≡ 0`, which makes `C′` and `C″` parallel
  across the span — the precondition above has already proven the speed
  nonzero there — and so confines the span to a single straight LINE.
  Verdict `0`.
- MIXED signs → INCONCLUSIVE, and never a disproof: the hull over-estimates
  the range, so a mixed coefficient set is consistent with a `K` that keeps
  one sign throughout. Subdivide the span at its midpoint by exact dyadic de
  Casteljau — the same split `spline_length.go` already runs — and fold the
  children's verdicts. The cap is a FIXED depth rather than a measured target,
  and it is `freeformLengthDepth`, the same constant §6.1's own bracket
  subdivides to (`spline_length.go` owns its value). A child still mixed at
  that depth refuses: Table R row **R19**, `ErrUnsupported`.

**The joint a SUBDIVISION creates is known `0`, not folded.** A midpoint de
Casteljau split leaves the left child's last control edge and the right
child's first control edge the IDENTICAL vector — both are half of
`b₁^{p−1} − b₀^{p−1}`, the last blend the split makes — so the cross is
exactly zero and the two tangents point the same way, which is verdict `0`
under the joint rule below. Splitting the SPAN and splitting `K`'s own
Bernstein coefficients agree on every sign too: a child's `K` is its parent's
restricted `K` scaled by `1/8` per level, a POSITIVE factor, since halving the
parameter halves `C′` and quarters `C″`. So a fold over the children needs no
joint term of its own, and either route to the children's coefficients reads
the same verdict (`spline_convexity_internal_test.go` pins both).

This proves a sign; it never measures one. Nothing here samples `C(t)`, and
no residual is available to fall back on — a sign is not a quantity to bound
— so an inconclusive certificate refuses rather than publishing a `convex`
bool nothing proved, exactly as the core falsify-never-bless rule requires.

**Read `≈` literally: a COMPUTED figure this section quotes without it is
exact.** Two things round. §5.1 lifts every recorded coordinate into the
rational its float already is, so a control written below as `0.9` or `−1/12`
— the literal a record is built from, not the rational the record holds —
enters as the binary float64 nearest that number, and a quantity computed from
such a control carries the lift rather than the round value. A figure with
more digits than are worth writing is quoted to the digits shown. Both are
marked `≈`. `spline_convexity_internal_test.go` pins each marked figure at the
precision its own text states and holds the exact rational there; it pins each
unmarked one exactly.

**NEVER read the control polygon's own turns instead.** A chain whose turns
all share one sign proves nothing about the curve's curvature: the classical
theorem needs the CLOSED control polygon convex, and an open single-sign
chain does not give that. The cubic control chain `(0, 0)`, `(1, 0)`,
`(−4, 1)`, `(0.9, 0)` has both its turns strictly positive — `1`, and
`≈ 1/10` — while its own `K` is `18` at `t = 0`, negative at `t = 5/7` and
positive again at `t = 1`: two curvature sign changes under zero polygon-turn
sign changes. `record.go` admits that net as a degree-3 `NURBSSeg` at unit
weights and §5.1 converts it to exactly one span, so a polygon-turn rule would
publish `convex` for a wall whose curvature changes sign twice. The Bernstein
certificate above refuses it instead, and subsumes the polygon test wherever
the polygon test would have been right
(`spline_convexity_internal_test.go` pins both facts).

**The verdicts fold, and `0` is the identity.** The proof runs PER SPAN, so
fold a subdivided span's children together, then fold every span of the chain
together with every JOINT between consecutive spans: `0` folds into anything
and leaves it unchanged, `+` folds with `+` and `−` with `−`, and `+` meeting
`−` is a curvature sign change the chain genuinely has — refuse, R19, exactly
as an undecided child does. Folding the joints in is not optional: a chain of
individually convex spans can still turn the other way where two spans meet,
so a per-span proof alone would publish a bool the chain does not have.

**What the fold runs over, at both ends of the chain.** A chain of exactly ONE
span has an EMPTY joint set, and the fold over an empty set is its identity
`0`, so that chain's verdict is its own span's — no clause treats a single span
specially. A chain that CLOSES on itself folds one joint MORE than an open one:
a `ClosedSplineSeg` converts to `n` spans for `n` control points (§5.1) and is
one wall edge whose curve has no free ends, so the closing joint between the
last span and the first is INTERIOR to that edge and folds exactly like every
other. Skipping it would publish a bool the closed walk does not have, the same
way skipping an interior joint would. An open chain's joint count is one below
its span count; a closed chain's equals it.

**A joint's own verdict.** At each joint form the cross product of the
incoming span's last NONZERO control edge with the outgoing span's first
NONZERO control edge. A Bézier's one-sided tangent DIRECTION is its first
nonzero control edge, §5.1 makes every control edge exactly rational, and
consecutive spans share the joint point exactly (§5.1), so the cross product
is the joint's own turn and its sign is never a tolerance call. A positive or
negative cross is verdict `+` or `−`. A ZERO cross with the two tangents
pointing the same way is verdict `0` — parallel tangents through a shared
point turn off no line. A zero cross with the two tangents pointing OPPOSITE
ways is a reversal, where the walk doubles back on itself and no curvature
sign covers the joint at all: refuse, R19. On a span the precondition admitted
the first and last control edges are already nonzero (above), so the word
NONZERO here carries the collapsed-span skip below and never a search inside an
admitted span.

A span with no nonzero control edge is a COLLAPSED span (§5.1) — one point, no
direction — and supplies no joint of its own: skip it and pair its neighbours
across it. Two or more CONSECUTIVE collapsed spans are skipped as one RUN, and
the pairing crosses the WHOLE run: a rule skipping a single span would pair a
neighbour with the next collapsed span, which has no direction to cross with,
and a degree-1 net carrying three coincident controls produces exactly that pair
of adjacent collapsed spans. A run at an open chain's FIRST or LAST position has
no pair to make, and none is made: the chain's own ends are where this wall edge
meets a DIFFERENT one, which evaluator §3 decides on its own terms, so such a
run contributes nothing and refuses nothing. On a chain that closes on itself
the same run pairs around the closing joint instead.

**Three outcomes, and they are exhaustive.**

- chain verdict `+` or `−` → the wall edge's `convex` bool is set from that
  sign under the orientation convention below. A STRICT sign is what sets the
  bool, and only that.
- chain verdict `0` → every span lies on one line and no joint turns off it,
  so the chain IS a straight walk and evaluator §3's straight-wall rule
  decides it by its loop's role (outer convex, hole concave). A net visiting
  only two distinct positions lands here by the same rule rather than by a
  case of its own, and so does a collinear net of any length: the degree-2
  `NURBSSeg` on `(0, 0)`, `(1, 0)`, `(2, 0)` at unit weights — three DISTINCT
  positions, and `record.go` gates a net's shape nowhere — has `K ≡ 0` and
  takes the loop-role rule (`spline_convexity_internal_test.go` pins it). A
  chain of degree-1 spans lands here whenever no joint of it turns, since every
  such span's `K` is identically zero by the degree rule above; one whose joints
  DO turn takes its sign from those joints exactly as any other chain does, and
  is not a straight walk.
- anything else — a span whose speed the root count does not prove nonzero, a
  span still mixed at the depth cap, two verdicts in conflict, a reversing
  joint → R19, `ErrUnsupported`.

**What the precondition costs, plainly.** A section carrying a cusp does not
build. Two coincident adjacent control points are the cheapest way to record
one — the cusp sits at a span END — and §6.3's own interior-cusp net
`(−1/8, 1/4)`, `(1/8, −1/12)`, `(−1/8, −1/12)`, `(1/8, 1/4)` is the other
shape, with the cusp inside the span. Both are records `record.go` admits
(§6.3), both reach this certificate, and both refuse R19 where the
coefficient test alone publishes a bool: the interior-cusp net's
`K ≈ −3/2·(2t − 1)²` is MIXED at the top level and one-signed with strict
entries on every dyadic child, so a fold reading the coefficients alone calls
it strictly `−` and publishes `convex` for an edge that doubles back on itself
(`spline_convexity_internal_test.go` pins both nets). The refusal is a BUILD
refusal because evaluator §3 decides `convex` at build and a bool has no
`Suspect` to fall back on — this is the one place a zero-speed span costs a
body, and §6.3's own two certificates still cost only their two `Verify`
readings. Table R marks R19 non-permanent for exactly this kind of case: a
later increment that splits a wall edge at its cusp gives each piece a
curvature sign, and nothing here forecloses it.

**The orientation convention, stated explicitly.** The sign that reaches the
bool is the turn in the LOOP'S OWN WALK direction, never the recorded
segment's parameter direction. The profile walk carries the material on its
left — outer loop counter-clockwise, every hole clockwise (evaluator §3, §4)
— and a counter-clockwise turn is convex, a clockwise one concave, the
identical convention the circular wall's own-turn test already fixes. Where
the recorded walk runs against the curve's natural sense — the reversal
`freeformBezierSpans` already reports beside its spans — NEGATE the chain's
verdict before reading it: reversing a span's parameter negates `C′` and
leaves `C″` unchanged, so it negates `K`.

**The negation is ONE operation at the end, over the UNREVERSED chain's own
fold.** `freeformBezierSpans` returns the same spans in the same order whatever
the recorded range order is, and reports the reversal beside them, so every span
verdict and every joint cross product is computed on that natural chain and the
reversal moves nothing but the sign of the answer
(`spline_convexity_internal_test.go` pins the identical chain under both range
orders). Negating `0` is `0`, so a straight walk recorded in either sense takes
the loop-role rule; and a refusal is not a sign to negate — R19 stands under
either order.

**Every Tier A kind's chain is the CONVERTED one, `FitSplineSeg` included.**
The certificate reads §5.1's own exact-rational Bézier control chain and
never a recorded net, and the difference is not cosmetic: `FitSplineSeg`
records no control points at all. Its record holds the points sketch's
interpolant passes THROUGH (§5.1.2), and §5.1.2's closed form is what turns
them into control points, so a rule phrased over recorded control points does
not reach that kind. The recorded fit points are not that chain and do not
even contain the curve — the `h²·m/18` terms subtract, which is exactly what
pushes a converted control outside the recorded points' own hull. Fit points
`(0, 0)`, `(1, 0)`, `(2, 1)`, `(3, 0)` convert to a first span whose `v`
controls are `0`, `≈ −0.0790`, `≈ −0.1580`, `0`, and that span's curve dips to
`v ≈ −0.0912`, below every recorded point's own `v`
(`spline_convexity_internal_test.go` pins it; the parameters are sketch's own
cumulative chord lengths, never a uniform `h`). That closed form emits one
CUBIC Bézier per interval between consecutive ACTIVE fit points, four control
points every time, so every span it produces has `p = 3` and no degenerate
degree reaches this section from that kind.

**Only Tier A spans reach this section.** §5.1's conversion emits polynomial
Bézier spans and nothing else: a `NURBSSeg` with unequal weights is Tier C
and a `ConicSeg` or whole `EllipseSeg` is Tier B, and each of those refuses a
BUILD at R10 before any wall edge is decided. So every chain §6.5 reads is
polynomial, `K` is a polynomial, and this section needs no rational column of
its own the way §6.2's rows do.

## 7. `NURBSSurface` and `NURBSCurve`

Core §6.1 reserved both names. This section fixes their shape.

Both the extrusion and the revolution of a B-spline are EXACTLY NURBS surfaces:

- extruded — the control net is the curve's control points against the two sweep
  ends, degree `(p, 1)`, weights carried through;
- revolved — the standard rational quadratic circle representation, degree
  `(p, 2)`, weights multiplied.

So core §6.1's promise holds unstrained: an analytic `Surface` variant is
`Exact` by construction. A `NURBSSurface` built from a recorded control net IS
the surface, not an approximation of it. `Faceted` remains the only inexact
variant, and no new exactness machinery enters the public API.

**The control net stays private in v1.** The variant is a tagged, opaque marker
carrying no exported geometry. Widening an opaque variant later is compatible;
narrowing an exposed one is not, and exposing the net would commit decad to a
wire format and a validation surface it does not yet need.

```go
// NURBSSurface is a free-form face's geometry — the exact extruded or revolved
// surface of a recorded free-form curve. Its control net is private in v1.
type NURBSSurface struct{ /* private */ }

// NURBSCurve is a free-form edge's geometry, NURBSSurface's 1-D analog.
type NURBSCurve struct{ /* private */ }
```

`NURBSSurface.Kind()` reports the existing `KindNURBS`: `Surface` declares
`Kind() SurfaceKind` and `KindNURBS` is one of that set's constants.
`NURBSCurve` reports no kind at all — `Curve` is sealed by its marker method
alone and declares no `Kind`, so the variant seals in with that method and
exports nothing else. A `switch` on `Surface` or `Curve` MUST already carry a
`default` (core §6.1), so adding these variants breaks no conforming caller.

**`Face.NormalAt(p)` on a `NURBSSurface` is `ErrUnsupported`.** Recovering the
`(u, v)` of a given point is a root-find, not a closed form, so no bound covers
the answer at all. The arm answers for the five analytic variants — `Plane`,
`Cylinder`, `Cone`, `Sphere` and `Torus` — each of which evaluates its normal in
closed form and publishes the bound that evaluation earns, zero only where the
arithmetic came out exactly right. `Faceted` is `ErrUnsupported` beside
`NURBSSurface` and for its own reason: its answer is a union of the held facets'
certificates rather than a closed form, and it lands with the faceted
certificate stage (`docs/payload-verification-design.md` §5.4, §13). Nothing
internal needs any of them: the undercut survey reads normals off the payload
walk, never off `NormalAt`.

## 8. Table C — per-capability reach

**Every build reads its section's moments, so a body's tier reach is its
section's.** A section whose free-form walks are all Tier A is the one §10's
P4 builds — Table R R6 refuses every free-form walk at the build today,
regardless of tier; a section carrying a Tier B or Tier C walk is
`ErrUnsupported` at EVERY build until §10's P9 supplies that tier's moments
(§5.3, §5.4) — Table R R10. "Tier A section"
below names exactly that condition, and now includes a section holding a
`FitSplineSeg` walk (§5.1.2) — its moments are Tier A today, so once §10's P4
lands a build it is one of the kinds P4 widens to reach, with no change to P4
itself. A `ProfileRecord` moment reading is not a build and is unaffected —
`FitSplineSeg` already has that reach for the moments path (§5.1.2).

**A Tier A section's exactly-rational reach is its free-form walks' alone.**
Analytic walks join them in the same section freely (§4.1), and a circular one
contributes `moments.go`'s own proven interval, so every row below reads "the
Tier A rational" as the section's COMPOSED moments — one rounding only where
every walk of the section is itself exactly rational (§3).

| Capability | Free-form reach | Construction |
|---|---|---|
| `ProfileRecord.Area`/`Centroid`/`SecondMoments` | Tier A exactly rational, rounded once; B/C proven interval | §5 |
| `Extrude` | Tier A section; `Volume` from the Tier A rational, `Area`/`Box` bounded | §6.1 length, §6.2 extremes, §7 surfaces; a through-all stop reading the bracket is §6.4; a wall edge's convexity is §6.5 |
| `Tessellate`, `STL`, `OBJ` | every section `Extrude` builds | §6.2 sagitta; rides the existing prism path, NOT tessellation T5 |
| `Union`/`Cut`/`Intersect` | every body `Extrude` builds, `Faceted` output as always | free once chording lands — the mesh boolean reads triangles, not kinds |
| interference proof | every body `Extrude` builds | free once chording lands — read-only mesh intersection already serves faceted pairs |
| `Undercuts` | proven where §6.3's certificates close, else `Suspect` | §6.2 normal cones; an enclosure decides a face only while it is a proper cone (§6.3) |
| `MinRadius` | proven interval under §6.3's speed floor, else `Suspect` | §6.2 curvature extremes; a measurement, never a verdict |
| `MinWallThickness` | proven interval, else `Suspect` | §8.1 |
| `Clearance` rows | `Suspect` until a free-form cell lands | box-disjoint pairs still read `Sound` |
| `Revolve` | Tier A section; surfaces of revolution per §7 | lateral `Area` by Pappus over §6.1.1's radial first moment, `Volume` and cap areas from §5's exact rational; meshing waits on tessellation T2–T5 |
| `Fillet`/`Chamfer`/`Shell` | refused per R3–R5, except the §4.1 analytic-corner slice | §4.1, with the free-form audit and its R11 refusal in §6.4 |

The sequencing that falls out: **chording an extruded free-form section rides the
existing prism tessellation path.** Only the boundary chording is per-segment.
`triangulate.go` and the loop-clearance audit consume chorded 2D samples and need
no change. So export, booleans and interference proof — the north-star oracle
capabilities — land early, before revolve and before the surveys.

### 8.1 Wall thickness is the one capability with no complete candidate set

`survey2d.go` answers `MinWallThickness` from a CLOSED-FORM candidate set of
critical inscribed disks, and the set is COMPLETE for the attained infimum over
line/arc boundaries. Completeness is what makes the answer exact.

No free-form analogue exists. A disk tangent to three cubic Bézier pieces is a
high-degree polynomial system, and isolating it exactly would still not prove
completeness for the infimum.

The sound construction is a certified branch-and-bound over pairs of Bézier
pieces with control-hull distance enclosures, returning an interval on the
minimum spanning diameter — structurally what `clearance_cells.go` already does
for its cone-involved and spindle-torus cells. The verdict rule already consumes
intervals (verification §6): proven thin → `Violating` at any coarseness, met →
the gate judges the bound, straddle → `Suspect`.

**Until that kernel lands, a free-form section's wall reading is `Suspect`** — an
asked-but-undecided answer, never a silent pass, and never a wrong number. What
the caller must be told plainly: for a free-form part decad proves closure,
volume and interference before it can prove wall thickness.

## 9. Upstream asks to sketch

Ordered by reach per unit of upstream work. None is a prerequisite for §10's
first four increments.

2. **Pin `EllipticalArc` endpoints onto the parametric ellipse.** Retires R2
   (§2.2): sketch's own coincidence constraint carries each moved endpoint to the
   neighbouring segment, so the loop still closes on a shared join point.
   Exporting exact eccentric parameters does NOT retire R2 on its own — it makes
   the parametric evaluation exact while the segment's ends stay at
   `ellipse(θ)`, which is precisely §2.2's rejected trust-the-parametric-curve
   branch: the neighbour's pinned join point is elsewhere and the loop stays
   open. Exported parameters retire R2 only together with an exact endpoint
   representation that preserves the shared join.
3. **Closed-form free-form intersection**, so a cut free-form fragment can report
   `TExact = true`. Retires R1 and lifts §2's whole-entities-only scope, letting
   free-form curves cross other curves in a sketch. Large upstream effort.

## 10. Increments

Each increment is a PR series behind the same public contract; nothing ships
half-silent. These stages do not consume a global evaluator increment number.

| # | Lands | Public effect |
|---|---|---|
| **P1** | this document + the core/evaluator table updates it resolves | none |
| **P2** | Bézier conversion, exact Tier A moments, the §5.2 budget | `ProfileRecord.Area`/`Centroid`/`SecondMoments` answer for Tier A, bounded by one rounding. No new types |
| **P3** | walk-kind discriminant across every `segmentWalk` consumer | none — behaviour preserved |
| **P4a** | §6.2 row 1's directional-extreme bracket, wired into the prism bounds reading and into §6.4's straddle-narrowed through-all stop gate | none — R6 still stands, so a free-form prism is reachable only from internal tests; the stop charges a met body's bracket to the level it resolves and R11 refuses only a straddling one, `extentAlongWork`'s wider refusal serving the clearance short-circuit alone, and R18 is live on the enclosure-to-float64 conversion the bracket publishes through |
| **P4b** | `NURBSSurface`/`NURBSCurve`, free-form extrude side faces, `NormalAt` refusal, §6.5's wall-edge convexity proof and its R19 refusal | Tier A free-form prisms build, `FitSplineSeg` walks among them since P4b is where R6's build refusal lifts (§5.1.2); `Volume` from the Tier A rational, `Area`/`Box` bounded. A Tier B or C section is R10; an undecidable through-all stop is R11; a wall edge whose curvature sign the chain does not prove is R19 |
| **P5** | free-form chording with proven sagitta + area slack | `Tessellate`/`STL`/`OBJ`, booleans, interference proof. Wall reading explicitly `Suspect` |
| **P6** | §6.3's speed floor and origin-exclusion certificates, hodograph normal cones, bracketed curvature extremes | `Undercuts` and `MinRadius` each answer where the certificates that reading needs close, and read `Suspect` per §6.3's cost table where they do not |
| **P7** | certified branch-and-bound inscribed-disk interval | `MinWallThickness` answered, with its own convergence evidence |
| **P8** | free-form surfaces of revolution, §6.1.1's radial first-moment bracket | `Revolve` builds for a Tier A section |
| **P9** | Tier B formulas; Tier C certified quadrature | Tier B/C moment readings answer, and the builds Table C stages on them follow — R10 retires |
| **P10** | the §4.1 analytic-corner modify slice over §6.4's free-form crossing and contact tests | fillet/chamfer on analytic corners of a mixed section |

## 11. Test obligations

Correctness must be observable — computed geometry, never "it ran" (core hard
rules).

- Assert the exact RATIONAL area, centroid and second moments of a Tier A
  section against a densely sampled reference AND against sketch's own
  `Profile.Area`. Two independent implementations agreeing is the §5.2 falsifier.
- Assert BOTH sides of §3's rounding rule, with representability read in the
  magnitude the reported `units.Value` actually carries: a Tier A section whose
  exact area is not representable there reports `Approximate` with a bound of one
  rounding, and a section whose exact area IS representable there reports `Exact`
  with a zero bound. A test that only covers one side cannot tell the rule from a
  constant.
- Assert BOTH sides of §6.2's contract consequence, since the split is the
  bracket's width and not the section's kind: a Tier A prism whose directional
  extreme is held by an interior root reports `Approximate` with a positive
  bound, and one whose extremes along the axis read are all held by exactly
  representable candidate values — a span monotone in that direction, so the
  candidate set is its two exactly interpolated endpoints — reports `Exact`
  with a zero bound. A test that only covers one side cannot tell the rule from
  a constant.
- Assert an arc-length bracket strictly narrows with subdivision depth and
  encloses a dense-sample reference at every depth, and that the reported bound
  is the enclosure MEASURED at the fixed depth rather than a width read off a
  rate (§6.1). Pin the relative half width of an ordinary span AND of the widest
  span the bracket's own preflight admits: they differ by two orders of
  magnitude, so a single figure stated for the depth is false for one of them,
  and a case that narrows by well under 4× at one of its levels must be among
  them. Scope any assertion of a particular width to the fixture it measures.
- Assert every §6.2 row whose rational identity differs from its polynomial one,
  each on a RATIONAL span whose true reading the polynomial-span identity would
  miss, and falsify each against a dense sample: a directional extreme attained
  at an interior parameter that is not a root of `d/dt(g·U)`, so the `Box` is not
  understated; a true maximum speed above the `p·ΔP_i` hull's norm, so the
  Lipschitz bound still holds; a true tangent direction outside the cone that
  same hull reports, so the undercut survey's enclosure still holds every normal;
  and a curvature extreme at a parameter that is not a root of the polynomial
  stationarity `2K′S − 3KS′`, so `MinRadius` is not overstated. The chord-sagitta
  row is EXCLUDED and needs no rational fixture: both its columns measure the
  same control-point distance to the chord per subdivision level, so it carries
  no rational-specific identity a fixture could falsify.
- Assert the §6.2.1 sagitta bound on a span that overshoots its own chord ends —
  the `(0, 0)`, `(−3, 0.01)`, `(4, 0.01)`, `(1, 0)` net, whose curve runs `0.76`
  past the chord: the reported bound must enclose the dense-sample deviation, so
  a bound measured to the chord's carrier LINE fails the test. Assert it again on
  a rational span whose control points all lie on their linear interpolant while
  the curve does not, so a bound built from the parametric deviation `|C − L|` is
  distinguished from the sagitta the chord actually commits.
- Assert `MinRadius` on a span carrying an INFLECTION: the reported interval
  encloses the tightest radius, which is attained where `K ≠ 0`, so a candidate
  set built from `K`'s roots alone fails the test (§6.2).
- Assert the §6.1.1 radial bracket on the reading a length bracket cannot make:
  two meridian walks of EQUAL length at different radii, whose §6.1 brackets
  coincide, must produce different `∫ r ds` enclosures, and each enclosure must
  contain a dense-sample reference. Then assert the P8 revolve's lateral `Area`
  falls inside `θ ·` that enclosure, and that a walk revolved twice at different
  radii reports areas in the ratio of those radii — a length-only construction
  reports them equal and fails.
- Assert the §6.1.1 bracket on a span whose radial control values include a
  NEGATIVE one while the walk stays strictly on the material side — radial
  controls `1, −1, 3, 1`, minimum radius `0.4226`: the reported enclosure must
  contain a dense-sample reference at every subdivision depth, starting at the
  depth where `r_lo` is still negative. A lower bound built as `r_lo · L_lo`
  passes only by sign accident, so assert the clamp directly: the first level's
  lower bound is `0`, and it rises once subdivision lifts every sub-span's
  `r_lo` to non-negative.
- Assert both §6.3 certificates by the readings they gate, and take the
  body-level half of that on the ONE-COLLAPSED-SPAN walk below, which is the
  fixture §6.5 leaves buildable — §6.5 skips a collapsed span rather than
  refusing it, so that walk builds, while a section carrying a cusp does not
  build at all. On the cusp net `(−1/8, 1/4)`, `(1/8, −1/12)`,
  `(−1/8, −1/12)`, `(1/8, 1/4)`: the hodograph hull contains the origin and
  subdivision never separates it, so both
  certificates fail. That net has no body to read either reading off — §6.5
  refuses its wall edge R19 — so assert the certificates themselves there, and
  assert the same for a coincident first control pair (zero speed at the span
  END). On a cusp-free span whose FIRST hodograph hull still contains the
  origin (a wide turn), subdivision must reach origin exclusion and report a
  proper cone. On a walk carrying ONE collapsed span — four coincident controls
  inside a longer clamped net, so `S` is the zero polynomial there while the
  walk's own length stays positive, and §6.5 skips the span rather than
  refusing the body — the speed floor must FAIL, `Undercuts` and `MinRadius`
  must read `Suspect`, and the body must still build, report its `Volume` and
  tessellate. A survey that instead returns an empty `Undercuts` list on that
  body is the silent pass §8.1 forbids, and must fail the test, and a
  certificate that reads the isolated root count alone reports a floor on that
  collapsed span and passes silently, so it must fail this test too.
- Assert the §6.3 speed floor's `W_max⁴` division on a RATIONAL span whose
  weights are not all equal, since the numerator floor alone errs in the UNSAFE
  direction. The degree-1 controls `(0, 0)` and `(1, 0)` at weights `1, 2` are
  the minimal falsifier: the numerator floor is `4` while the true squared-speed
  minimum is `1/4`, so a reported floor above a dense-sampled minimum of `|C′|²`
  fails the test. Assert the consequence on a CURVED rational span too — the
  reported `MinRadius` interval must enclose the dense-sample tightest radius,
  which a floor inflated by `W_max⁴` overstates.
- Assert R9's OTHER branch — a reading whose proven bracket straddles its
  threshold, not a §6.3 certificate failure. A free-form section whose §8.1 wall
  interval straddles the tool diameter reads `Suspect`; the same section against
  a tool the interval provenly clears reads its verdict from the gate, and
  against one the interval provenly undercuts reads `Violating` at any
  coarseness (§8.1). A test covering only the certificate branch cannot tell the
  straddle rule from a certificate check.
- Assert §6.5's wall-edge convexity certificate by the bool it publishes and by
  the refusal it withholds one with, on records `record.go` admits. A chain
  whose every span closes on one strict sign publishes `convex` from that sign
  under the loop's own walk orientation, and the same section recorded in the
  reverse sense publishes the opposite bool. A chain whose control-polygon
  turns all share one sign while its own curvature does not — the cubic net
  `(0, 0)`, `(1, 0)`, `(−4, 1)`, `(0.9, 0)` — must refuse R19, so a
  certificate reading the polygon's turns fails the test. A collinear net —
  degree 2 on `(0, 0)`, `(1, 0)`, `(2, 0)`, whose `K` is the zero polynomial —
  must take evaluator §3's loop-role rule and publish a bool rather than
  refuse. Two individually one-signed spans meeting at a joint that turns the
  other way must refuse R19, so a per-span proof that skips the joints fails
  the test. A test covering only the refusal cannot tell the certificate from
  a blanket free-form refusal.
- Assert §6.5's Table K on the shapes §5.1's conversion actually produces, since
  a rule written for the cases its author thought of reads as total until the
  next admitted record arrives. The DEGREE-1 row is the one a coefficient rule
  stated for `p ≥ 2` alone cannot serve: the unit-weight `NURBSSeg` on
  `(0, 0)`, `(1, 0)`, `(1, 1)`, degree 1, knots `[0, 0, 1, 2, 2]`, converts to
  TWO degree-1 spans, each with `K` the zero polynomial and `S` the nonzero
  constant `1` — so each span is verdict `0` — while the joint between them
  crosses `+1` and carries the chain's whole verdict. Assert the span count, the
  zero `K`, the root count beside its endpoint value, and that cross. Assert the
  DEGREE-2 constant beside it — `4·cross(ΔP₀, ΔP₁)`, which is `4` on `(0, 0)`,
  `(1, 0)`, `(1, 1)` — at the STATED degree `2p − 3 = 1` and at the degree its
  true `K` has, so an array sized from the true degree cannot pass. Assert the
  CONSECUTIVE collapsed run on the degree-1 net whose three coincident controls
  produce two adjacent collapsed spans: the surviving neighbours pair across the
  whole run and cross `+1`, where a rule skipping one span at a time pairs a
  neighbour with a span that has no direction at all. Assert the SUBDIVISION
  joint — a midpoint split leaves the left child's last control edge and the
  right child's first the IDENTICAL vector, and the children's coefficients
  under a span split and under a Bernstein split differ only by the positive
  factor `1/8`. Assert the REVERSAL is one negation over an unreversed chain:
  the same record with `TStart` and `TEnd` exchanged converts to the identical
  spans in the identical order, with the reversal reported beside them.
- Assert §6.5's regularity precondition on both cusp shapes, since a
  coefficient test alone publishes a bool for each. §6.3's own interior-cusp
  net `(−1/8, 1/4)`, `(1/8, −1/12)`, `(−1/8, −1/12)`, `(1/8, 1/4)` is the
  sharper case and must refuse R19: its `K` is `≈ −3/2·(2t − 1)²` under §6.5's
  `≈` convention, whose top-level Bernstein coefficients are MIXED while every
  dyadic child is one-signed with strict entries, so the fold alone reaches a
  strict `−` and publishes `convex` for an edge that doubles back — assert
  those coefficients at the precision that figure states and pin the exact
  lifted ones beside them, and assert that the child coefficients really are
  one-signed, or the fixture cannot tell the precondition from the depth cap.
  The unit-weight cubic `(0, 0)`, `(0, 0)`, `(1/3, 0)`, `(1, 1)` must refuse
  R19 too: its `K` is one-signed with strict entries at the top level, so the
  coefficient test publishes `+` outright, and its `S` has its ONLY root at
  `t = 0`, so a precondition that counts roots on the half-open span alone
  finds none and admits it — assert that root count and the endpoint value
  separately.
- Assert the §5.1 span rules on records `record.go` admits: a `NURBSSeg` with a
  repeated interior knot builds and its area matches a dense-sample reference,
  its empty span carrying no Bézier segment and no division by a zero span
  width; a free-form walk whose every span is collapsed is refused as a
  zero-length walk (R14, `ErrDegenerate`); and the §6.1 length, §6.1.1 radial and
  §6.2.1 sagitta enclosures of a walk holding one collapsed span each contain a
  dense-sample reference, with that span contributing `0`.
- Assert §6.1's scale rule at BOTH ends of the float64 range, not at one: the
  ordinary curve rescaled so its legs make the squared distance subnormal, and
  rescaled so they make it overflow, must each measure, enclose a dense-sample
  reference and keep the relative bracket width the unscaled curve gets. Add the
  near-duplicate case — one control pair a few hundred decades below the rest of
  an ordinary curve — since it is what turns the rule from a curiosity about
  extreme parts into a defect on an ordinary one. A test at one end only cannot
  tell a rescaled seed from a seed patched for underflow.
- Assert both §5.1.1 rules on records `record.go` admits today, each against a
  dense-sample reference. A `NURBSSeg` whose interior knot sits at multiplicity
  above its degree with the two one-sided limits at DIFFERENT coordinates is
  refused `ErrDegenerate` (R12). The SAME record with those two limits at the
  IDENTICAL coordinate either builds — its area matching the chain-of-curves
  section that spells the same walk as separate segments joined at that shared
  endpoint — or refuses `ErrUnsupported` while R13 stands, NEVER `ErrDegenerate`;
  a test that asserts only the different-coordinate direction cannot tell the
  rule from a multiplicity check. Cover an adjacent limit pair (degree 1,
  `m = 2`) and a non-adjacent one (degree 3, `m = 5`, whose limits `P_j` and
  `P_{j+m−p}` have a dead control between them), so neither direction can pass by
  assuming adjacency.
- Assert the §5.1.1 END-knot rule on the over-clamped record `record.go` admits —
  degree 2, knots `[0, 0, 0, 0, 1, 1, 1]`, 4 control points, `P_0` dead: the body
  either builds with the area of the single quadratic Bézier over `P_1, P_2, P_3`
  against a dense-sample reference, or refuses `ErrUnsupported` (R13), never
  `ErrDegenerate`. Assert the dead control point moves no reading: displacing
  `P_0` alone changes no area, no length enclosure and no `Box`.
- Assert `Undercuts` on a free-form face whose certified cone is proper: a face
  whose cone puts every point provenly opposing the pull is listed, and a face
  whose cone clears at every point is not.
- Assert directed-edge closure, positive triangle area, outward winding, and
  `len(SourceFaces) == len(Triangles)` on a free-form prism mesh.
- Assert byte-identical repeated STL/OBJ output.
- Assert a boolean of a free-form prism against a box carries a composed bound
  and breaks no invariant.
- Sample the true surface densely only as a FALSIFIER: any observed distance
  above `Mesh.Bound` fails, and passing samples never replace the bound's
  derivation.
- Assert every Table R row by behaviour, each with its own sentinel: a crossed
  spline, a `FitSplineSeg` whose interpolant runs off float64 range (R16), an
  `EllipticalArcSeg`, a free-form `Shell`, a
  free-form fillet carrier, a free-form chamfer carrier, a control net collapsed
  to a single point (R14), an `Extrude` whose through-all stop reads a free-form
  extent bracket straddling the sketch plane (R11), a `NURBSSeg` whose interior
  knot at multiplicity above its degree has DIFFERENT one-sided limits (R12), a
  record R13 stages, a Tier A section whose exact-rational integration exhausts
  the §5.2 work budget (R7 `ErrUnsupported`, and the same section under a budget
  that admits it integrates exactly — so the refusal cannot be a float fallback
  in disguise), a valid single-span record whose DEGREE alone exhausts that
  budget through §6.1's subdivision charge rather than through the integration
  one (R7 again), a free-form walk whose chording needs more than the chord cap
  (R8 `ErrUnsupported` through `errTooManyChords`), a curve whose arc-length
  enclosure runs past `MaxFloat64` (R15 `ErrUnsupported`, and NEVER
  `ErrNotFinite`), a section whose directional-extreme enclosure along one
  direction runs past that same range while another direction still reads
  exactly (R18 `ErrUnsupported`, and NEVER `ErrDegenerate` — the sentinel a
  saturated enclosure folded into the extreme accumulators produces instead),
  and — while R10 stands — an
  `Extrude` of a section carrying a Tier B or Tier C walk, whose Tier A
  counterpart builds in the same test. Run each of R3–R5 on a DEGREE-1
  `NURBSSeg` walk as well as a curved one and require the same `ErrUnsupported`,
  so the refusal stays keyed on the recorded kind rather than on the degree
  (§4.1).
- Assert R12 and R13 on the SAME knot vector, moving only the one control point
  that decides continuity: the record whose two one-sided limits differ is
  `ErrDegenerate`, the record whose limits are identical is `ErrUnsupported`, and
  a boundary knot over-clamped past `degree+1` — admitted by record validation,
  a curve with one dead control point — is `ErrUnsupported` too. A test that
  asserts only one of the two cannot tell the rule from a multiplicity test.
- Assert every free-form kind's recorded-range refusals as a table over the kinds
  crossed with {full, trimmed, non-finite}, each cell by its sentinel, on the
  caller-built record and on the same record decoded from its own wire form: a
  non-finite range is `ErrNotFinite` on EVERY kind, and a trimmed range never
  displaces the cause a kind is already refused for. A test that covers Tier A
  alone cannot see a kind reading the two refusals in the other order.
- Assert the §5.2 charge fires BEFORE the pass it bounds, by MEASURING the
  refusal rather than reading the cost formula: a degree-1 record with thousands
  of distinct interior knots — needing no insertion, so an insertion-only charge
  sees nothing to charge — and a high-degree record whose over-budget
  integration would otherwise be reached only after validation had sampled the
  curve in sketch, must each refuse promptly.
- Measure the ALLOCATION, not only the time: a record refused by the conversion
  charge must allocate on the order of the record itself, which is what tells a
  charge levied before the rational lift from one levied after it. Both refuse,
  and only the measurement distinguishes them.
- Measure the reconstruction charge by its own boundary: the largest record the
  ceiling admits still measures, and the next control point past it refuses in
  milliseconds. State the admitted worst case the ceiling guarantees.
- Assert the reconstruction charge on the whole RECORD, not a source at a time:
  the chord total of a two-source record is the sum, and its charge exceeds twice
  a single source's, which is the cross-source pairs a per-source charge drops. A
  record of many tiny curves must refuse on the chord FLOOR, and one of many arcs
  or circles beside a single spline must refuse on the analytic chords a
  free-form-only count cannot see.
- Assert the per-kind chord counts against sketch's own sampler as a table over
  the kinds, the floor and the open spline's per-span count among them. A test
  that checks one kind cannot see a row that reads the wrong field.
- Assert a rational `NURBSSeg` reports its Tier C reason with the counter left
  holding exactly its rational lift and nothing more, which is what distinguishes
  a tier decided before the CONVERSION charge from one decided after. Assert the
  stated cost beside it: with the counter exhausted the same record reports R7.
- Assert the size-derived lift charge precedes the per-element content scan
  MECHANICALLY rather than by timing, on two records one control point apart
  across the ceiling, each carrying a non-finite element at the end of its
  vector. The record inside the ceiling reports that element's own refusal, so
  the scan ran; the record past it reports R7, which it can only do by never
  reading the element.
- Back the constant-factor invariant with a BOUNDARY REGRESSION on measured cost
  rather than a per-pass accounting identity: the worst record that charge
  admits — well formed, so every pass it bounds runs before the refusal — must
  stay within a stated wall-clock and allocation budget, and the first record past
  the boundary must refuse without any of them running. An identity has to be
  restated whenever a validator is added and goes stale silently; a cost boundary
  fails when a pass is uncharged or allocates per element, whatever the accounting
  says.
- Assert that ONE public operation over one record spends ONE ceiling, and assert
  it by MEASUREMENT. A second counter reads well under the limit exactly as the
  first does, so no unit count can see it; only the cost of the work it admits
  can. Take a record whose preflight alone spends most of the ceiling, and require
  the phase after it — the walk resolution, whose arc-length bracket is the
  expensive one — to refuse within a stated wall-clock and allocation budget that
  a fresh ceiling's subdivision blows through. Assert beside it that a walk charges
  the counter it was handed rather than one of its own, so successive resolutions
  accumulate.
- Assert an OPEN spline's converted spans against an exact Cox–de Boor evaluation
  over `geom.ClampedKnots`'s own FLOAT knots, at a control count where `n−3` is
  not a power of two, comparing rationals rather than a tolerance. A fixture whose
  `n−3` is a power of two cannot see a re-derived knot vector, and a tolerance
  cannot see it either — the divergence is smaller than an ulp of the reading and
  still turns an `Approximate` into a false `Exact`.
- Assert recipe replay of every free-form step reproduces body order, provenance
  roles, and measurements within the evaluator's own exactness.
- **§5.1.2's fit-spline obligations.** A two-point `FitSplineSeg` reports the
  identical `Area`/`Centroid`/`SecondMoments` a `LineSeg`-recorded triangle
  does, bit for bit, both `Exact`
  with a zero bound on a fixture whose exact area is representable — the
  strongest single assertion, since `naturalSecondDerivs` is exactly zero
  below three points. The same identity holds for equally spaced COLLINEAR fit
  points at three and five points, pinning a multi-span chain's own join. A
  genuinely curved fixture's exact rational area matches a dense sample over
  sketch's own `FitSpline` evaluator AND sketch's independent `Profile.Area`
  (the §5.2 falsifier), and reports `Approximate` with a positive bound; a
  representable-magnitude fixture reports `Exact` with a zero bound — covering
  both sides of §3's rounding rule, since a straight-line-equivalent fit
  spline's area is always representable over integer coordinates and can
  never exercise the `Approximate` side on its own.
- Assert decad's exact rational Bézier control values, converted to monomial
  form and rounded to float64, agree with `FitInterpolant.Spans()`'s `X`/`Y`
  to a few ulps — the test that would have caught consuming the wrong
  exported form (§5.1.2), using sketch's own independent conversion as the
  oracle, internal to the package.
- Assert deduplication in both directions: a record whose `Fit` holds a
  consecutive coincident INTERIOR point integrates the identical active curve
  as the same section recorded without it (pinning that decad reads
  `Points`, never its own `Fit`), and a record whose fit-spline chain's
  converted endpoint is `FitInterpolant.Points[len-1]` rather than the raw
  `Fit[len(Fit)-1]` whenever the last two recorded fit points coincide within
  the absolute `1e-12` threshold.
- Assert R14 on an all-coincident `FitSplineSeg` fit set (`ErrDegenerate`, no
  boundary contributed) and R16 on fit coordinates whose interpolant runs off
  float64 range (`ErrUnsupported`, never `ErrNotFinite` — every fit coordinate
  in the fixture is finite).
- Assert R17 on a record whose `FitSplineSeg` terminal fit point collapses into
  its predecessor while a following segment's own recorded `Start` still names
  the dropped point: `Area`/`Centroid`/`SecondMoments` refuse `ErrDegenerate`
  rather than publish a bounded measurement for the region the boundary
  actually fails to close. Assert the pass case beside it — the identical shape
  with the terminal point moved past the dedup threshold — builds and measures,
  so the test cannot pass by refusing every fit spline outright; and the
  reversed-range (`TStart=1, TEnd=0`) case, where the same collapse sits at the
  walk's own START rather than its end.
- Assert the `fitInterpolantCost` charge fires, and is measured rather than
  merely read off the formula: a fit-point count whose linear charge alone
  exceeds the ceiling must refuse promptly, allocating on the order of its own
  `Fit` slice rather than on the order of the interpolant
  `geom.NewFitInterpolant` would have solved.
- Assert that `Extrude` of a `FitSplineSeg` section still refuses
  `ErrUnsupported` at the side-face build — R6 states the build refusal, and
  P4 (build support) is unimplemented and unaffected.
