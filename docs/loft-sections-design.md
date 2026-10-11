# Smooth Three-Section Loft Design

`Document.LoftSections` builds one smooth solid through exactly three recorded
Sketch profiles. `docs/loft-design.md` continues to own the two-profile ruled
`Document.Loft`. This document owns the separate quadratic construction, its
admission gates, and the proof attached to its held mesh.

## 1. Public operation and admitted class

```go
type LoftSection struct {
    Sketch  *sketch.Sketch
    Profile *sketch.Profile
}

func (d *Document) LoftSections(ctx context.Context, sections ...LoftSection) (*Body, error)
```

The call requires exactly three sections. Each passes the ordinary profile
seam and area falsifier in argument order. The quadratic-scale route admits
one outer loop with the same number and order of whole `LineSeg` records on
all three profiles. Every line may walk forward or backward according to its
recorded full-domain range; the walked vertices must meet exactly. Holes,
trimmed lines, curved segments, and differing segment counts return
`ErrUnsupported` on that route. Section 6 owns a separate constant-section
route for exactly matching curved records.

All three planes have world-XY `U` and `V` axes, the same XY origin, and
strictly increasing Z origins with the middle Z exactly halfway between the
ends. A different pose or spacing returns `ErrUnsupported`. The returned body
is a solid; this increment has no surface-result or alignment option.

Let `P_j` be walked vertex `j` of the first section. Every corresponding
vertex of section 1 must equal `q_1 P_j`, and every vertex of section 2 must
equal `q_2 P_j`, in exact rational arithmetic over the recorded float values.
The scales must be positive. Thus each section is an exact positive homothetic
copy of the first, about the common plane origin. A mismatch returns
`ErrUnsupported`; no tolerance or fitted transform can admit it. Sketch's
profile validity establishes the first loop's simple, positive-area region.
On the quadratic-scale route, the first region must also be star-shaped about
the common origin. For every
counterclockwise edge from `P` to `Q`, exact rational arithmetic checks
`cross(Q-P, -P) >= 0`. This puts the origin in the polygon's kernel and
proves every radial contraction stays inside the region. A profile whose
scale origin lies outside the kernel returns `ErrUnsupported`.

## 2. Denoted solid and face identity

Use `t = (z-z_0)/(z_2-z_0)` and the unique quadratic through the three
scales at `t = 0, 1/2, 1`:

```text
q(t) = a t² + b t + 1
a = 2(q_2 - 2q_1 + 1)
b = q_2 - 1 - a
```

The middle Bernstein coefficient is `B = 2q_1 - (1+q_2)/2`. Require
`B >= 0`. With the positive endpoint coefficients, `q(t) > 0` everywhere
on `[0,1]`. An input failing this sufficient positivity gate returns
`ErrUnsupported` even if another proof could show its quadratic positive.

At height `t`, the exact section is `q(t)` times the first profile. The
strictly increasing Z coordinate separates different sections, and positive
scaling preserves the first profile's embedding at every height. For a base
line from `P_j` to `P_{j+1}`, the wall is
`(q(t)((1-u)P_j+uP_{j+1}), z_0+Ht)`. It is a tensor-product Bézier patch of
degrees `(1,2)`, so each wall reports `NURBSSurface`. The two end caps report
`Plane`. The middle profile is interpolated exactly and creates no seam face.

## 3. Held mesh and boundary certificate

Choose a power-of-two band count `N`, beginning at 16 and doubling through
128, until the rational displacement bound below is at most 0.05 mm. More
than 64 base edges or failure within 128 bands returns `ErrUnsupported`.
Every ring vertex and Z level is evaluated as an exact rational and must be
exactly representable as a finite float64; otherwise this increment returns
`ErrUnsupported`. This keeps the held vertex table equal to its stated
piecewise-linear interpolation, with no uncharged coordinate rounding.

At each `t_i=i/N`, the held ring uses the exact `q(t_i)` section. Adjacent
rings connect by a linear scale `l_i(t)`. Each edge's four corners lie in one
plane, so two triangles tile its trapezoid exactly. The first and last rings
share their vertices with the triangulated end caps. The held mesh runs the
closed, oriented faceted audit before construction commits.

Write `R_1=max_j (|P_j.U|+|P_j.V|)`. Quadratic interpolation on a band gives
`|q-l_i| <= |a|/(4N²)`. The same point on the denoted wall and held trapezoid
is separated by at most

```text
D = |a| R_1 / (4N²).
```

The parameter map covers both surfaces in both directions. Every wall facet
gets `D` as its source-face and vertex certificate. Cap facets have no
interpolation error, but may carry the same conservative bound. A request to
`Tessellate` below the held bound refuses; a request at or above it restates
the audited facets with their live source faces and `VerifyAll` proof.

## 4. Occupied volume and measurements

Let `A_0` be the exact rational shoelace area of the first loop and
`H=z_2-z_0`. The denoted body's volume is `A_0 H ∫_0^1 q(t)² dt`.
The held mesh's volume is

```text
A_0 H /(3N) · Σ_i (q_i² + q_i q_{i+1} + q_{i+1}²).
```

Inside every band, `q-l_i = a(t-t_i)(t-t_{i+1})` has one sign. Since both
scales are positive and the base region is star-shaped about their common
origin, the true and held cross-sections are nested with that
same sign throughout. The exact absolute difference of the two formulas is
therefore the occupied-volume symmetric difference, not merely a signed
volume residual. Round this rational difference upward into `volSymDiff`.
The faceted payload integrates the held mesh in exact rational arithmetic;
`Volume` includes this difference and final value rounding in its bound.
`Centroid` uses the same symmetric-difference term and a diameter upper bound
`2 q_max R_1 + H`. `Bounds` uses the boundary certificate `D`.

For area, each base edge `E=P_{j+1}-P_j` has determinant
`K=det(E,P_j)`. Its true wall area density is
`q sqrt(H²|E|²+K² q'²)`. The held density replaces `q,q'` with `l_i,l_i'`.
Set `d=|a|/(4N²)`, `d'=|a|/N`,
`M=max(|b|,|2a+b|)`, and `q_max=max(1,B,q_2)`. Integration over all bands
and the reverse triangle inequality give this non-cancelling error per edge:

```text
d (|E|_1 H + |K| M) + q_max |K| d'.
```

Sum those rational terms over every edge and round upward into `areaSlack`.
The same nonnegative allowance covers any face subset a later boolean keeps.
The faceted area and face readings add their own held arithmetic charges.

## 5. Consumers and refusals

`Tessellate(VerifyAll)`, mesh booleans, placement, and export use the standard
faceted proof payload. Placement carries the certified mesh and source-face
groups through the existing faceted placement proof; a placed planar tag gets
a conservative unit-normal departure bound. `Face.NormalAt` on a quadratic
wall remains `ErrUnsupported`, as for every `NURBSSurface`.

The admitted classes are increments, not a claim that general multi-section
interpolation is determined by profile boundaries. Nonhomothetic profiles,
varying curved profiles, arbitrary planes, guides, centerlines, and hole
loops remain `ErrUnsupported`. Nil context or document, or a section count
other than three, returns `ErrDegenerate`. A seam failure returns its own
sentinel and leaves the document unchanged.

## 6. Exact constant curved sections

Three profiles with curved segments have a second, narrower construction.
Each must have one outer loop, no holes, and 3 to 64 recorded segments.
At least one segment must be curved. `momentinput.ExactProfileEqual` must
match the middle and last structural records to the first in stored order,
including every curve parameter and range. This is a sufficient exact
identity test, not a fitted or sampled equivalence test. A mismatch returns
`ErrUnsupported` and leaves the document unchanged.

The three planes keep §1's world-XY axes, common XY origin, ascending levels,
and exact midpoint. Their total Z separation must itself be exactly
representable as a finite float64. These checks make each recorded profile
the same plane-local region at its own Z level. This route defines the
interpolation as the straight prism of the first profile over that total
height; it meets the middle profile exactly and creates no middle seam face.

`evalPrismContext` builds the prism from the first authenticated record with
one free-form work counter continued from its area falsifier. Its existing
segment gates remain in force: line and circular arcs and Tier A free-form
curves can build; unsupported tiers, curvature changes, unproven endpoint
joins, and exhausted work budgets refuse with their own sentinel. Side faces
retain the prism kernel's `Plane`, `Cylinder`, or `NURBSSurface` tags and
normal certificates. The two caps remain `Plane` faces. The same kernel
provides bounded `Volume`, `Area`, `Centroid`, and `Bounds` readings. Its
tessellator keeps live source faces and publishes the existing boundary and
occupied-volume proofs for `VerifyAll`. Placement and replay use its prism
payload, including the same curved source records.

This route covers sections whose curve records are exactly identical.
Changing a fitted-spline flank, circular radius, or arc range between
sections still lacks a bounded interpolation and returns `ErrUnsupported`.
