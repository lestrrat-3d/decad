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
| `SplineSeg` | **A** | `Exact`, zero bound | §5 |
| `ClosedSplineSeg` | **A** | `Exact`, zero bound | §5 |
| `NURBSSeg`, all weights equal | **A** | `Exact`, zero bound | §5 |
| `ConicSeg` | **B** | `Approximate`, proven interval | §5.3 |
| `EllipseSeg` (whole) | **B** | `Approximate`, proven interval | §5.3 |
| `NURBSSeg`, weights unequal | **C** | `Approximate`, proven interval | §5.4 |
| `EllipticalArcSeg` | — | refused, §2.2 | — |
| `FitSplineSeg` | — | refused, §4 R6 | — |

Tier A is not a convenience. It is the reason free-form support does not cost
decad its exactness discipline: a spline prism's `Volume` is `Exact` with a zero
bound, exactly as a line-only prism's is.

A tier is a CEILING, never a promise about a specific reading. Arc length is
never exact in ANY tier (§6.1), so a Tier A body's `Area` is `Approximate` while
its `Volume` is `Exact`.

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
| **R9** | a bracket cannot separate a `Verify` reading from its threshold | not an error — `Suspect` | no, §8 |

R9 is the one row that is not a refusal. An intent the evaluator cannot BUILD is
`ErrUnsupported` at the call; a `Verify` question it cannot ANSWER is accepted
and reads `Suspect` (evaluator §11). A free-form reading whose proven interval
straddles its threshold is the second case.

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
It needs only the §5 audit to run over free-form elements, which §8 requires
anyway.

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
each integral is an exact rational:

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

The exact result is the only result. NEVER fall back to quadrature on a Tier A
kind — a float sum of Gauss nodes cannot claim the zero bound the tier grants,
and `exactnessOf`'s zero bound is a CLAIM that the value is exactly
representable (`bounds.go`).

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
- de Casteljau subdivision shrinks the gap by 4× per level.

Both bounds are proven, not sampled. Report the interval midpoint as the value
and its half width as the bound, `Approximate` always — a zero bound here would
be a false Exact.

Consumers: a prism's side-face `Area` (`length × height`), a revolve's Pappus
area, `Edge.Length()`, and the setback R5 refuses.

### 6.2 Extremes, sagitta, normals and curvature reduce to one existing engine

`clearance_poly.go` already owns a certified polynomial root engine — `ratPoly`
over `math/big.Rat`, Sturm chains, square-free reduction, Cauchy root bounds,
root isolation into intervals that cannot lie, and deterministic bisection under
a fixed depth budget, all context-aware. In Bézier form every free-form question
decad needs is one of its root problems. Reuse it. Do NOT fork a second root
finder.

| Question | Reduces to | Consumer |
|---|---|---|
| directional extreme | `d/dt(g·C(t)) = 0`, degree `p−1` per span | `extentAlong`, `Box`, through-all stops |
| chord sagitta | control-point deviation from the linear interpolant, quartered per subdivision level | `chordCount`, tessellation |
| tangent/normal cone | hodograph control hull — a degree `p−1` Bézier with control points `p·ΔP_i` | undercut survey |
| curvature extreme | `(u′v″ − v′u″)` numerator roots | `MinRadius` |

An extreme VALUE bracket follows from the isolated parameter interval plus a
Lipschitz bound off the hodograph hull, exactly the pattern
`clearance_poly.go` already uses for its critical values.

**Contract consequence.** `prismBoundsContext` reports `Exactness: Exact` with a
zero bound today. A free-form interior extreme is an irrational root evaluation,
so **a free-form prism's `Box` is `Approximate`** with the bracket's bound. State
it; never paper over it.

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

| Capability | Free-form reach | Construction |
|---|---|---|
| `ProfileRecord.Area`/`Centroid`/`SecondMoments` | Tier A `Exact`; B/C proven interval | §5 |
| `Extrude` | full; `Volume` `Exact` on Tier A, `Area`/`Box` bounded | §6, §7 |
| `Tessellate`, `STL`, `OBJ` | full for an extruded free-form section | §6.2 sagitta; rides the existing prism path, NOT tessellation T5 |
| `Union`/`Cut`/`Intersect` | full, `Faceted` output as always | free once chording lands — the mesh boolean reads triangles, not kinds |
| interference proof | full | free once chording lands — read-only mesh intersection already serves faceted pairs |
| `Undercuts` | proven | §6.2 normal cones; reject-only use makes an enclosure sufficient |
| `MinRadius` | proven interval | §6.2 curvature extremes; a measurement, never a verdict |
| `MinWallThickness` | proven interval, else `Suspect` | §8.1 |
| `Clearance` rows | `Suspect` until a free-form cell lands | box-disjoint pairs still read `Sound` |
| `Revolve` | surfaces of revolution per §7 | Pappus over §6.1 brackets; meshing waits on tessellation T2–T5 |
| `Fillet`/`Chamfer`/`Shell` | refused per R3–R5, except the §4.1 analytic-corner slice | §4.1 |

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
2. **Pin `EllipticalArc` endpoints onto the parametric ellipse**, or export exact
   eccentric parameters for them. Retires R2 (§2.2).
3. **Closed-form free-form intersection**, so a cut free-form fragment can report
   `TExact = true`. Retires R1 and lifts §2's whole-entities-only scope, letting
   free-form curves cross other curves in a sketch. Large upstream effort.

## 10. Increments

Each increment is a PR series behind the same public contract; nothing ships
half-silent. These stages do not consume a global evaluator increment number.

| # | Lands | Public effect |
|---|---|---|
| **P1** | this document + the core/evaluator table updates it resolves | none |
| **P2** | Bézier conversion, exact Tier A moments, the §5.2 budget | `ProfileRecord.Area`/`Centroid`/`SecondMoments` answer `Exact` for Tier A. No new types |
| **P3** | walk-kind discriminant across every `segmentWalk` consumer | none — behaviour preserved |
| **P4** | `NURBSSurface`/`NURBSCurve`, free-form extrude side faces, §6.1 length brackets, §6.2 extremes, `NormalAt` refusal | free-form prisms build; `Volume` `Exact`, `Area`/`Box` bounded |
| **P5** | free-form chording with proven sagitta + area slack | `Tessellate`/`STL`/`OBJ`, booleans, interference proof. Wall reading explicitly `Suspect` |
| **P6** | hodograph normal cones, bracketed curvature extremes | `Undercuts` proven, `MinRadius` bounded |
| **P7** | certified branch-and-bound inscribed-disk interval | `MinWallThickness` answered, with its own convergence evidence |
| **P8** | free-form surfaces of revolution | `Revolve` builds |
| **P9** | Tier B formulas; Tier C certified quadrature | breadth per Table F |
| **P10** | the §4.1 analytic-corner modify slice | fillet/chamfer on analytic corners of a mixed section |

## 11. Test obligations

Correctness must be observable — computed geometry, never "it ran" (core hard
rules).

- Assert the exact RATIONAL area, centroid and second moments of a Tier A
  section against a densely sampled reference AND against sketch's own
  `Profile.Area`. Two independent implementations agreeing is the §5.2 falsifier.
- Assert `Volume` reports `Exact` with a zero bound for a Tier A prism, and that
  `Box` reports `Approximate` with a positive bound (§6.2).
- Assert an arc-length bracket strictly narrows with subdivision depth and
  encloses a dense-sample reference at every depth.
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
  free-form fillet carrier, a free-form chamfer carrier.
- Assert recipe replay of every free-form step reproduces body order, provenance
  roles, and measurements within the evaluator's own exactness.
