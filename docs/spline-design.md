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
| how a length/extreme is bounded | §6 |
| what a revolve's area needs beyond length | §6.1.1 |
| why an undercut or radius reading is `Suspect` | §6.3 |
| why a build refuses on a bracket it cannot decide | §6.4 |
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
| `FitSplineSeg` | — | refused, §4 R6 | — |

**Tier A means the integral is exact, NOT that the reported measurement is
`Exact`.** The integral's value is an exact rational; `Measurement.Value` is a
float64. So the reported bound is a SINGLE rounding of that rational, and it is
zero — hence `Exact` — exactly when the rational is representable in float64.
A 5-control closed spline whose area is 293/18 reports `Approximate` with a
one-ulp bound; the same section scaled by 3 has area 293/2, is representable, and
reports `Exact`.

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
| **R3** | free-form walk in a section a `Shell` offsets | `ErrUnsupported` | yes, §4.1 |
| **R4** | `Fillet` corner with a free-form carrier | `ErrUnsupported` | yes, §4.1 |
| **R5** | `Chamfer` corner with a free-form carrier | `ErrUnsupported` | yes, §4.1 |
| **R6** | `FitSplineSeg` reaches a build or an integral | `ErrUnsupported` | pending §9 ask 1 |
| **R7** | exact-rational integration exceeds its work budget | `ErrUnsupported` | no, §5.2 |
| **R8** | chording a free-form walk needs more than the chord cap | `ErrUnsupported` | no, reuses `errTooManyChords` |
| **R9** | a `Verify` reading's proof does not close — its bracket cannot separate it from its threshold, or a §6.3 certificate fails | not an error — `Suspect` | no, §8 |
| **R10** | a Tier B or Tier C walk reaches a BUILD before its moments land | `ErrUnsupported` | no, §8 |
| **R11** | a free-form bracket cannot decide a BUILD-time comparison | `ErrUnsupported` | no, §6.4 |

R9 is the one row that is not a refusal. An intent the evaluator cannot BUILD is
`ErrUnsupported` at the call; a `Verify` question it cannot ANSWER is accepted
and reads `Suspect` (evaluator §11). A free-form reading whose proven interval
straddles its threshold is the second case, and so is a direction cone §6.3
cannot certify. R11 is the first case reached from the same brackets: a build
gate has no `Suspect` to fall back on (§6.4).

### 4.1 Why the modify refusals are permanent

Modify §2 reduces every modify op to an EXACT rewrite of the recorded 2D
section. Each free-form refusal follows from that reduction, not from missing
effort:

- **R3.** `Shell` needs the section's exact offset. The exact offset of a
  polynomial spline is not polynomial, so no recordable section represents it.
- **R4.** A fillet's blend centre is the intersection of the two carriers'
  material-side offsets. One offset is not representable, so there is no exact
  centre to record.
- **R5.** A chamfer's foot sits a setback distance along the boundary curve,
  measured as ARC LENGTH. Arc length is never exact (§6.1), so the foot is not
  exact, so the chord's recorded endpoints would be approximate coordinates —
  which core §6.2 forbids outright.

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

Convert the recorded curve to piecewise Bézier form. Knot insertion is a
rational convex combination, so the conversion is exact:

| Recorded kind | Conversion |
|---|---|
| `SplineSeg` | clamped uniform cubic → one Bézier per span |
| `ClosedSplineSeg` | periodic uniform cubic → one Bézier per span, `n` spans for `n` control points |
| `NURBSSeg` | clamped arbitrary degree → one Bézier per knot span |

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

### 5.2 Discipline

The exact rational is the only result. NEVER fall back to quadrature on a Tier A
kind — a float sum of Gauss nodes has no exact value to round from, so it can
never reach the zero bound a representable rational does, and `exactnessOf`'s
zero bound is a CLAIM that the value is exactly representable (`bounds.go`).

The held float MUST be the exact rational rounded once, never a separate float
evaluation of the same formula: a second evaluation would add its own error to a
bound that already speaks for the rounding.

Rational coefficient size grows with degree and span count. Charge every span,
every coefficient product and every integral term against a `workBudget`
(`budget.go`), and refuse as R7 when it runs out. NEVER widen to a float path to
stay inside the budget.

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
adaptively until the remainder bound closes. The result is a proven `[lo, hi]`,
never an adaptive estimate compared against itself. An estimate that measures
its own convergence is not a bound.

## 6. Bounds for the readings no tier makes exact

### 6.1 Arc length — a proven two-sided bracket

Arc length integrates `√(u′² + v′²)`, which has no polynomial antiderivative in
any tier. Bracket it instead:

- the CHORD is a lower bound;
- the CONTROL POLYGON is an upper bound (variation diminishing);
- de Casteljau subdivision shrinks the gap toward zero.

**The stopping certificate is the MEASURED post-subdivision enclosure, never an
assumed per-level rate.** Recompute both bounds on the child spans, sum them, and
stop when that measured gap meets the target. The familiar 4× per level is
ASYMPTOTIC only: the first levels of a span with a far control point shrink the
gap by substantially less, so a depth sized from that rate reports a bound the
actual enclosure does not support. NEVER size a depth from a rate.

Both bounds are proven, not sampled. Report the interval midpoint as the value
and its half width as the bound, `Approximate` always — a zero bound here would
be a false Exact.

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

Per span, with `[L_lo, L_hi]` the §6.1 length bracket of that SAME span and
`r_lo ≥ 0`:

```
r_lo · L_lo  ≤  ∫ r ds  ≤  r_hi · L_hi
```

Sum the per-span enclosures. de Casteljau subdivision shrinks `[L_lo, L_hi]` and
`[r_lo, r_hi]` together, so the product enclosure closes, and **the stopping
certificate is again the MEASURED enclosure, never an assumed rate** (§6.1).
Report the interval midpoint as the value and its half width as the bound,
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

| Question | Polynomial Bézier span (Tier A) | Rational span | Consumer |
|---|---|---|---|
| directional extreme | `d/dt(g·C(t)) = 0`, degree `p−1` per span | `(g·U)′W − (g·U)W′ = 0`, degree `≤ 2p−1` per span — the positive `W²` denominator cleared | `extentAlong`, `Box`, through-all stops |
| chord sagitta | control-point deviation from the linear interpolant, MEASURED per subdivision level (§6.1) | the same deviation on the span's own control points, MEASURED per level | `chordCount`, tessellation |
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

**Contract consequence.** `prismBoundsContext` reports `Exactness: Exact` with a
zero bound today. A free-form interior extreme is an irrational root evaluation,
so **a free-form prism's `Box` is `Approximate`** with the bracket's bound. State
it; never paper over it.

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
  is not a recording question.
- **a wide turn.** A tangent turning through half a revolution across one span
  sweeps the hull across the origin even where the speed never vanishes.

Both are decided EXACTLY, never at a tolerance — the control points are floats
taken exactly as rationals (§5.1):

- **the speed floor.** `S(t) = u′² + v′²` is a polynomial with exact rational
  coefficients. Count its real roots on the closed span with `ratPoly`'s Sturm
  chain (§6.2): zero roots proves `S > 0` there, and bracketing `S`'s minimum over
  its own isolated stationary points and the span endpoints gives the positive
  floor `s_min` that the Lipschitz and curvature brackets need. A rational span
  clears the positive `W²` first and tests the numerator hodograph.
- **origin exclusion.** Whether the origin lies in the convex hull of the
  hodograph's control points is an exact rational sign test — no root-find. Where
  a hull fails it, subdivide the hodograph and retest the children: under a proven
  `s_min > 0` the child hulls shrink toward a curve that stays `s_min` away from
  the origin, so the subdivision terminates.

What each failure costs, per R9 — a `Verify` question the evaluator cannot answer
is accepted and reads `Suspect`:

| Certificate | Fails when | Cost |
|---|---|---|
| speed floor `s_min > 0` | `S` has a root on the span | `Undercuts` AND `MinRadius` read `Suspect` for that body |
| origin exclusion on every subdivided hull | the turn is too wide to separate within the subdivision budget | `Undercuts` reads `Suspect` |

**Neither failure refuses a BUILD.** Chording, volume, area, export and the
boolean path read no direction cone, so a cusped section still extrudes and still
takes part in an interference proof — the readings that need a direction are the
only ones that go undecided. A build refusal would also refuse an ordinary spline
whose first two control points coincide: that is zero speed at a span END, which
costs the same two readings and nothing else.

### 6.4 A build-time comparison a bracket cannot decide

`Suspect` is a `Verify` answer. A BUILD has no such fallback — it produces a body
or refuses, exactly as the modify audit's undecidable nesting does (modify §5
S9). So a gate that compares a free-form bracket against a threshold at build
time refuses as R11 when the bracket straddles it. Two gates do:

- **a through-all stop's in-path test.** `stops.go` decides EXACTLY, on each
  body's closed-form `extentAlong`, whether the sweep meets that body. A
  free-form body's extent is a bracket (§6.2), so a stop whose bracket straddles
  the sketch plane in the travel sense is R11 — never a guessed dependency.
- **the §4.1 analytic-corner slice's audit.** The modify §5 crossing and
  boundary-contact audit must run over the section's free-form walks. A crossing
  is a root problem for §6.2's engine, exactly; the contact floor `δ = ε·D` needs
  a certified minimum-distance bracket between two spans — a control-hull
  branch-and-bound, structurally §8.1's. A contact that bracket cannot decide is
  R11.

R11 is not permanent: refining the bracket decides every case but an exact
tangency, and a tangency is a contact the §5 audit refuses anyway.

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

`Kind()` reports the existing `KindNURBS`. A `switch` on `Surface` or `Curve`
MUST already carry a `default` (core §6.1), so adding these variants breaks no
conforming caller.

**`Face.NormalAt(p)` on a `NURBSSurface` is `ErrUnsupported`.** Recovering the
`(u, v)` of a given point is a root-find, not a closed form, so an `Exact`
zero-bound answer is unavailable and the other variants all promise one. Nothing
internal needs it: the undercut survey reads normals off the payload walk, never
off `NormalAt`.

## 8. Table C — per-capability reach

**Every build reads its section's moments, so a body's tier reach is its
section's.** A section whose free-form walks are all Tier A builds; a section
carrying a Tier B or Tier C walk is `ErrUnsupported` at EVERY build until §10's
P9 supplies that tier's moments (§5.3, §5.4) — Table R R10. "Tier A section"
below names exactly that condition. A `ProfileRecord` moment reading is not a
build and is unaffected.

| Capability | Free-form reach | Construction |
|---|---|---|
| `ProfileRecord.Area`/`Centroid`/`SecondMoments` | Tier A exactly rational, rounded once; B/C proven interval | §5 |
| `Extrude` | Tier A section; `Volume` from the Tier A rational, `Area`/`Box` bounded | §6.1 length, §6.2 extremes, §7 surfaces; a through-all stop reading the bracket is §6.4 |
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

1. **Export the fit-spline interpolant's B-spline or piecewise-polynomial
   coefficients.** `geom/fitspline.go` already computes them internally. This is
   a signature, not an algorithm. It retires R6, and it is the only way to:
   decad must NEVER re-run the interpolation solve (seam §2), and consuming
   `geom.EvalFitSpline` would build geometry from samples.
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
| **P4** | `NURBSSurface`/`NURBSCurve`, free-form extrude side faces, §6.1 length brackets, §6.2 extremes, `NormalAt` refusal, §6.4's stop gate | Tier A free-form prisms build; `Volume` from the Tier A rational, `Area`/`Box` bounded. A Tier B or C section is R10; an undecidable through-all stop is R11 |
| **P5** | free-form chording with proven sagitta + area slack | `Tessellate`/`STL`/`OBJ`, booleans, interference proof. Wall reading explicitly `Suspect` |
| **P6** | §6.3's speed floor and origin-exclusion certificates, hodograph normal cones, bracketed curvature extremes | `Undercuts` and `MinRadius` answer where those certificates close, `Suspect` where they do not |
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
- Assert BOTH sides of §3's rounding rule: a Tier A section whose exact area is
  not representable reports `Approximate` with a bound of one rounding, and a
  section whose exact area IS representable reports `Exact` with a zero bound. A
  test that only covers one side cannot tell the rule from a constant.
- Assert `Box` reports `Approximate` with a positive bound for a Tier A prism
  (§6.2).
- Assert an arc-length bracket strictly narrows with subdivision depth and
  encloses a dense-sample reference at every depth, and that the depth is chosen
  from the MEASURED enclosure — a case whose first level narrows by well under 4×
  must still reach its target rather than stop at a rate-sized depth (§6.1).
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
  same control-point deviation per subdivision level, so it carries no
  rational-specific identity a fixture could falsify.
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
- Assert both §6.3 certificates by the readings they gate. On the cusp net
  `(−1/8, 1/4)`, `(1/8, −1/12)`, `(−1/8, −1/12)`, `(1/8, 1/4)`: the hodograph
  hull contains the origin, subdivision never separates it, `Undercuts` and
  `MinRadius` read `Suspect`, and the same body still reports its `Volume` and
  tessellates — a survey that instead returns an empty `Undercuts` list on that
  body is the silent pass §8.1 forbids, and must fail the test. On a cusp-free
  span whose FIRST hodograph hull still contains the origin (a wide turn),
  subdivision must reach origin exclusion and report a proper cone. On a span
  with a coincident first control pair — zero speed at the span END — the same
  two readings are `Suspect` while the build succeeds.
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
  spline, a `FitSplineSeg`, an `EllipticalArcSeg`, a free-form `Shell`, a
  free-form fillet carrier, a free-form chamfer carrier, an `Extrude` whose
  through-all stop reads a free-form extent bracket straddling the sketch plane
  (R11), and — while R10 stands — an `Extrude` of a section carrying a Tier B or
  Tier C walk, whose Tier A counterpart builds in the same test.
- Assert recipe replay of every free-form step reproduces body order, provenance
  roles, and measurements within the evaluator's own exactness.
