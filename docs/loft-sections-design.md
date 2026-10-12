# Smooth Section Loft Design

`Document.LoftSections` builds one smooth solid through three or four recorded
Sketch profiles. `docs/loft-design.md` continues to own the two-profile ruled
`Document.Loft`. This document owns the separate smooth constructions, their
admission gates, and the proofs attached to their held meshes.

## 1. Public operation and admitted class

```go
type LoftSection struct {
    Sketch  *sketch.Sketch
    Profile *sketch.Profile
}

func (d *Document) LoftSections(ctx context.Context, sections ...LoftSection) (*Body, error)
```

The call requires three or four sections. Each passes the ordinary profile
seam and area falsifier in argument order. The three-section quadratic-scale route admits
one outer loop with the same number and order of whole `LineSeg` records on
all three profiles. Every line may walk forward or backward according to its
recorded full-domain range; the walked vertices must meet exactly. Holes,
trimmed lines, curved segments, and differing segment counts return
`ErrUnsupported` on that route. Section 6 owns a separate constant-section
route for exactly matching curved records. Section 7 admits linearly varying
four-arc circles. Section 8 admits varying fitted flanks with fixed arcs.
Section 9 admits four non-affine rounded-rectangle sections.

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
varying curved profiles outside §§7–9, arbitrary planes, guides, centerlines,
and hole loops remain `ErrUnsupported`. Nil context or document, or a section count
other than three or four, returns `ErrDegenerate`. A seam failure returns its own
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
Section 8 owns its bounded fitted-flank exception. Changing another curved
record still returns `ErrUnsupported`. Section 7 handles one bounded circular
radius change.

## 7. Linearly varying circular sections

Each real Sketch producer must supply one valid, hole-free profile of exactly
four whole `ArcSeg` records. The records run counterclockwise through the
cardinal points `(r,0)`, `(0,r)`, `(-r,0)`, `(0,-r)` with center `(0,0)` and
full parameter range `[0,1]`. The radius of every section lies in `[1,32]`
mm. The planes have §1's matching world-XY axes and equally spaced Z levels;
total height is at most 32 mm. The three exact radii satisfy
`2r_1=r_0+r_2`, `r_0 != r_2`, and `r_2/r_0` in `[1/2,2]`. These exact
record comparisons identify circles and the linear law. Sketch remains the
authority for profile validity and area. A failed gate returns
`ErrUnsupported` and leaves the document unchanged. Other curved records
still reach §6's identical-record route.

At height `t`, the denoted section is the disk of radius
`r(t)=r_0+(r_2-r_0)t`. This is a ruled conical frustum that passes through
all three supplied profiles. Four side faces retain their arc source roles
and report `NURBSSurface`; the start and end caps report `Plane`. The middle
profile creates no seam face. The faceted payload handles placement with its
ordinary transformed-mesh proof.

The held boundary uses 96 polygon vertices per ring at `t=0,1/2,1`.
Trigonometry only proposes those vertices. Exact rational checks require
every proposed vertex to lie inside the unit disk, every turn to be strictly
convex, each point to stay in its named quadrant, each cardinal seam to lie
on its axis, and each supporting edge to lie at least `1023/1024` from the
origin. The polygon therefore lies inside the circle and contains the inner
circle of radius `r_0(1023/1024)`. Its coordinates, scaled ring coordinates,
and Z levels must be exactly representable as float64. The two bands tile
ruled planar polygon faces, while each cap is triangulated from the same
ring vertices. The faceted audit must confirm a closed outward mesh before
the body commits.

The corresponding circle and polygon boundary points on any radial ray
differ by at most `max(r_0,r_2)/1024`; both directions have this bound. It
must be at most 0.05 mm. `Tessellate(VerifyAll)` restates the audited mesh
only at a tolerance at least this held bound. Each wall facet names one of
the four original arc source roles.

Let `A_P` be the exact polygon shoelace area, `H=z_2-z_0`, and
`q=r_2/r_0`. The polygon sections stay inside the circular sections, so
their occupied-volume symmetric difference equals

```text
(pi*r_0² - A_P) * H * (1+q+q²)/3.
```

The proven upper rational endpoint for pi gives `volSymDiff` directly.
The faceted integrator then adds its held arithmetic charge to `Volume`,
`Centroid`, and other readings. The diameter allowance is
`2*max(r_0,r_2)+H`.

For area, both cap deficits total `(pi*r_0²-A_P)*(1+q²)`. The polygon is
convex, inscribed in the outer circle, and contains the inner circle, so
its perimeter lies between their circumferences. Each ruled polygon wall
has a slant between the inner and outer circle's slants. With
`δ=r_0/1024` and `Δr=|r_2-r_0|`, a non-cancelling lateral allowance is

```text
pi*(1+q)*δ*(H+Δr+(r_0-δ)*|q-1|).
```

Both terms are rounded upward into `areaSlack`, beside the faceted body's
own held arithmetic and vertex-bound charges. For the 8/10/12 mm public
case, the 0.1 mm `VerifyAll` boundary and volume proofs pass; default
`Document.Verify` currently reports `Suspect` because its conservative
measurement allowances exceed the default tolerance.

## 8. Varying fitted flanks with fixed circular arcs

### 8.1 Source class and denoted wall

Admit three authenticated, hole-free profiles whose outer loop has four
whole records in the same cyclic order: two `FitSplineSeg` records alternating
with two `ArcSeg` records. The three planes satisfy §1's common world-XY
frame and exact midpoint rule. Both arc records must be structurally identical
in all three sections, have center at the common XY origin, positive exact
squared radius, equal exact start/end squared radii, and a counterclockwise
walk. Their recorded joins to the fitted flanks must equal the fitted
interpolants' own endpoint coordinates exactly. A failed shape gate returns
`ErrUnsupported`; a seam failure retains the seam's sentinel.

For each corresponding fitted record, consume Sketch's exported natural-cubic
`FitInterpolant` through the existing Tier A exact Bézier conversion. Keep
the record's walk direction. Refine all three converted chains by exact
de Casteljau subdivision to their least common multiple of span counts.
The shared parameter `u` is the normalized span-index coordinate described
in `docs/loft-design.md` §5.1, extended from two to three sources: each
refined span occupies one equal slot, and its local parameter runs from zero
to one in recorded walk order. No nearest-point or sampled correspondence is
allowed. A common count that exceeds the resource ceiling refuses before
allocation.

Let `C_0(u)`, `C_1(u)`, and `C_2(u)` be the three exact converted curves in
one corresponding slot. With `t=(z-z_0)/(z_2-z_0)`, the denoted fitted wall
is the tensor-product polynomial patch

```text
C(u,t) = (1-t)^2 C_0(u)
       + 2t(1-t) [2 C_1(u) - (C_0(u)+C_2(u))/2]
       + t^2 C_2(u).
```

Its three Bernstein control rows in height are `C_0`,
`2 C_1-(C_0+C_2)/2`, and `C_2`, all exact rationals over Sketch's exported
interpolants. It equals each recorded curve at `t=0,1/2,1` exactly. The
unchanged arcs extrude as circular walls. Their endpoint agreement makes the
four wall patches continuous at every height. The two end caps use the
recorded end profiles; the middle profile makes no seam face. Fitted walls
report `NURBSSurface`, arc walls `Cylinder`, and end caps `Plane`.

### 8.2 Continuous-section admission

The first source profile has Sketch's valid positive-area region. Prove the
fitted wall's polar-angle derivative
`cross(C(u,t), ∂C(u,t)/∂u) > 0` throughout every refined span and the full
height. Form its exact rational tensor-product Bernstein coefficients;
subdivide a cell in `u` or `t` and recheck when their signs differ. Accept a
cell only when every coefficient is strictly positive. Exhausted subdivision
or work budget returns `ErrUnsupported`. Strict positivity also excludes the
common origin. A fixed origin-centered CCW arc of positive radius has the
same derivative sign. Exact endpoint joins give a continuous boundary for
each `t`; absence of origin crossings keeps its winding equal to the valid
first section's winding. Strictly increasing polar angle and winding one
make each intermediate boundary a simple radial graph. This proves the
interpolated sections Decad defines, not Sketch's source profiles anew.

### 8.3 Held mesh and boundary bound

Choose shared dyadic parameter cells across all three fitted chains. Measure
each candidate cell's exact control-net departure from its endpoint chord
and split until the bound fits its assigned tolerance or the station/work
ceiling refuses. Use one station table at every Z level. The fixed arcs use
one certified circular chord chain at every level. Select dyadic Z bands and
evaluate the exact quadratic at their endpoints; every Z level must be
representable exactly in float64. Round each XY station once and charge its
exact rational displacement. Join adjacent rings with the fixed diagonal
split from `docs/loft-design.md` §5. Cap triangles share the endpoint rings.

On each fitted parameter cell, the u-chord error is bounded by the largest
control-point difference between each of the three height-Bernstein curves
and its endpoint line. The height interpolation error within one Z band is
bounded by `max_u |C_2-2C_1+C_0|_1/(4N^2)`, where these are the height
Bernstein controls. The bilinear patch between
four ring corners differs from either diagonal triangle by at most one
quarter of the corner twist's L1 norm. Compose those terms and the station
rounding allowance outward to get a parameter-matched wall bound. The fixed
arcs use their measured circle-to-chord and station-rounding bounds. Publish
the maximum as `meshBound` and per-vertex bounds; refuse when it exceeds
0.05 mm. Run `internal/loftmesh`'s exact crossing audit on the complete
closed triangle set before constructing the faceted body. Each wall triangle
spans only adjacent Z levels, so every interior horizontal slice of the
audited mesh has one simple polygonal boundary.

### 8.4 Occupied-volume and area proof

At the same height, parameterize the true boundary and the held mesh's
horizontal slice by the same source segment and parameter cell. The linear
homotopy between them is continuous and closed. A point whose occupied state
differs must lie in the homotopy image: winding number cannot change elsewhere.
For one parameter cell, let `d` bound the XY distance between matched true
and held points, and let `L` bound the greater of their parameter speeds.
Its homotopy sweeps at most `d*L*Δu` area at a fixed height. Multiply by the
cell's exact Z-band height and sum all cells and bands with outward rounding.
This non-cancelling sum bounds the occupied-volume symmetric difference;
the signed difference of total volumes does not replace it. Pass the sum to
`facetedPayload.volSymDiff` for `VerifyAll` and downstream booleans.

For wall area, the fitted source patch has
`|S_u × S_t| <= |C_u|_1 (H+|C_t|_1)`. Its rational Bézier control
differences bound both derivatives. Each fixed arc wall has area at most its
certified arc-length upper bound times `H`. Every source cap lies inside a
disk whose radius is bounded by the fitted control nets and fixed arcs, so
the two cap areas together are at most `2*pi*R²`. Sum these source-area
uppers with the exact held triangle-area uppers into a conservative
`areaSlack`: the absolute difference cannot exceed their sum. The occupied
volume proof above remains local and much tighter. A finite XY/Z extent
bounds the diameter used by faceted centroid and bounds readings. A
non-finite proof term or a resource ceiling returns `ErrUnsupported`; no
zero is inferred from successful mesh construction alone.

## 9. Four rounded-rectangle sections with cubic width

Four real Sketch profiles may define a non-affine rounded-rectangle loft.
Each authenticated, hole-free outer loop contains four whole `LineSeg` and
four whole `ArcSeg` records in counterclockwise alternating order. The
records must exactly describe an axis-aligned rounded rectangle centered on
the common plane origin, with half-width `w_i`, half-height `h`, and corner
radius `r`. All sections have the same positive `h` and `r`, while their
half-widths may differ. Each line/arc join is a shared recorded endpoint;
the four arcs have exact cardinal quarter turns. A cyclic change in the
first recorded edge does not change the shape. A reversed walk, trimmed
record, hole, or mismatch returns `ErrUnsupported`.

The four world-XY planes share one XY origin and have strictly increasing,
equally spaced Z levels. The total height is at most 32 mm. The exact
recorded values satisfy `1 <= r < h <= 32` mm and `r < w_i <= 32` mm.
The corresponding cubic Bernstein width controls `B_0,...,B_3` interpolate
the four exact widths at normalized heights `0,1/3,2/3,1`. Require either
`B_0 <= B_1 <= B_2 <= B_3` or the reverse, and require every control to be
strictly greater than `r`. At least one interior width differs from the
straight interpolation of the end widths. These exact rational tests prove
`w(t)>r`, monotonic width, and a convex rounded-rectangle section for every
height. The outer loop stays simple because the width never closes the
horizontal straight edges. Source profiles outside this class refuse.

The denoted solid has a rounded-rectangle section with width
`w(t)=sum_j B_j binomial(3,j)t^j(1-t)^(3-j)` at each normalized height `t`.
Every source line and arc is met at its supplied height. The eight side
patches inherit the first profile's segment roles and report
`NURBSSurface`; the two end caps report `Plane`. Intermediate profiles do
not create seam faces. A first-to-last ruled Loft differs whenever an
interior width differs from its straight interpolation.

The held mesh uses a certified convex 96-gon inside the unit disk. Its four
cardinal vertices equal the source arc endpoints. The other dyadic
vertices are proposed numerically, then exact rational checks require them
inside the disk, in their named quadrants, with positive turns and every
supporting edge at radius at least `1023/1024`. Each rounded corner uses
one quarter of this polygon; exact straight segments connect the corners.
Every ring uses the exact cubic width at a dyadic height and shares its
vertices with the next band and the end caps. A station or level that cannot
be held exactly as float64 returns `ErrUnsupported`. A fixed diagonal splits
each wall quad. Both its vertices on one circular or vertical side move by
the same horizontal amount; a horizontal side has constant Y. Therefore
every held quad is planar and its two triangles tile the linear-width
polygon wall exactly. A closed oriented audit and a full facet-contact
audit run before the faceted body commits.

Let `M=6 max(|B_2-2B_1+B_0|, |B_3-2B_2+B_1|)`. Linear interpolation
of the cubic width over `N` equal bands differs by at most `M/(8N^2)`.
The two-sided boundary bound is at most this width term plus `r/1024`.
Choose a power-of-two `N` from 16 through 128 with a bound at most
0.05 mm; otherwise return `ErrUnsupported`. The faceted payload publishes
that bound, each live side/cap source face, and the completed mesh audits.

At one height, two rounded rectangles with the same `h,r` and different
widths are nested. Their symmetric-difference area is exactly
`4h |w_a-w_b|`. The held quarter-arc polygon lies inside its corresponding
round corner. If `A_P` is the certified unit 96-gon's exact area, its area
deficit is at most `r^2 (pi_upper-A_P)`. Thus the occupied-volume
symmetric difference between the denoted body and the held mesh is at most

```text
H * (4h M/(8N^2) + r^2 (pi_upper-A_P)).
```

This non-cancelling bound goes to `facetedPayload.volSymDiff` for
`VerifyAll`, placement, and booleans. A source-area upper bound uses the
maximum rounded-rectangle perimeter times `H+max|w'|`, plus both cap
areas. The exact held-triangle area uppers join it as conservative
`areaSlack`. A finite box enclosing the source and held mesh supplies the
diameter bound. General four-section free-form profiles, changing corner
radius or half-height, and non-monotone width controls remain
`ErrUnsupported`.
