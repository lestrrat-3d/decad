# Smooth Section Loft Design

`LoftSections` builds one smooth solid through three to 1024
recorded Sketch profiles. `agent-docs/loft-design.md` continues to own the two-profile ruled
`Loft`. This document owns the separate smooth constructions, their
admission gates, and the proofs attached to their held meshes.

## 1. Public operation and admitted class

```go
type LoftSection struct {
    Sketch  *sketch.Sketch
    Profile *sketch.Profile
}

func LoftSections(ctx context.Context, d *Document, sections ...LoftSection) (*Body, error)
```

The call requires at least three sections. Each passes the ordinary profile
seam and area falsifier in argument order. The three-section quadratic-scale
route admits one outer loop with the same number and order of whole `LineSeg` records on
all three profiles. Every line may walk forward or backward according to its
recorded full-domain range; the walked vertices must meet exactly. Holes,
trimmed lines, curved segments, and differing segment counts return
`ErrUnsupported` on that route. Section 6 owns a separate constant-section
route for exactly matching curved records. Section 7 admits linearly varying
four-arc circles. Section 8 admits varying fitted flanks with fixed arcs.
Section 9 admits four non-affine rounded-rectangle sections.
Sections 10–12 admit three to thirty-two axis-aligned rectangular sections
whose half-width and half-height vary independently. Section 13 admits three
to 1024 sections that turn about the origin and carry one fitted edge.
Any other five or more sections must be rectangles and stop at thirty-two.

For the three-section homothetic route, all three planes have world-XY `U` and
`V` axes, the same XY origin, and strictly increasing Z origins with the middle
Z exactly halfway between the ends. A different pose or spacing returns `ErrUnsupported`. The returned body
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
volume residual. Round this rational difference upward into `VolSymDiff`.
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

Sum those rational terms over every edge and round upward into `AreaSlack`.
The same nonnegative allowance covers any face subset a later boolean keeps.
The faceted area and face readings add their own held arithmetic charges.

## 5. Consumers and refusals

`Tessellate(VerifyAll)`, mesh booleans, placement, and export use the standard
faceted proof payload. Placement carries the certified mesh and source-face
groups through the existing faceted placement proof. Unless the placement is exactly the
identity, a placed analytic tag moves to its float image and gets a conservative unit-normal
departure bound. `Face.NormalAt` on a quadratic wall remains `ErrUnsupported`, as for every
`NURBSSurface`.

The admitted classes are increments, not a claim that general multi-section
interpolation is determined by profile boundaries. Nonhomothetic profiles
outside §§10–12,
varying curved profiles outside §§7–9, arbitrary planes, guides, centerlines,
and hole loops remain `ErrUnsupported`. Nil context or document, or fewer than
three sections, returns `ErrDegenerate`; more than thirty-two sections returns
`ErrUnsupported` unless §13 admits them (at most 1024). A seam failure returns its own sentinel and leaves the
document unchanged.

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

`prismrecord.Build` builds the prism from the first authenticated record with
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

The proven upper rational endpoint for pi gives `VolSymDiff` directly.
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

Both terms are rounded upward into `AreaSlack`, beside the faceted body's
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
in `agent-docs/loft-design.md` §5.1, extended from two to three sources: each
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
split from `agent-docs/loft-design.md` §5. Cap triangles share the endpoint rings.

On each fitted parameter cell, the u-chord error is bounded by the largest
control-point difference between each of the three height-Bernstein curves
and its endpoint line. The height interpolation error within one Z band is
bounded by `max_u |C_2-2C_1+C_0|_1/(4N^2)`, where these are the height
Bernstein controls. The bilinear patch between
four ring corners differs from either diagonal triangle by at most one
quarter of the corner twist's L1 norm. Compose those terms and the station
rounding allowance outward to get a parameter-matched wall bound. The fixed
arcs use their measured circle-to-chord and station-rounding bounds. Publish
the maximum as `MeshBound` and per-vertex bounds; refuse when it exceeds
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
`facetedPayload.VolSymDiff` for `VerifyAll` and downstream booleans.

For wall area, the fitted source patch has
`|S_u × S_t| <= |C_u|_1 (H+|C_t|_1)`. Its rational Bézier control
differences bound both derivatives. Each fixed arc wall has area at most its
certified arc-length upper bound times `H`. Every source cap lies inside a
disk whose radius is bounded by the fitted control nets and fixed arcs, so
the two cap areas together are at most `2*pi*R²`. Sum these source-area
uppers with the exact held triangle-area uppers into a conservative
`AreaSlack`: the absolute difference cannot exceed their sum. The occupied
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

This non-cancelling bound goes to `facetedPayload.VolSymDiff` for
`VerifyAll`, placement, and booleans. A source-area upper bound uses the
maximum rounded-rectangle perimeter times `H+max|w'|`, plus both cap
areas. The exact held-triangle area uppers join it as conservative
`AreaSlack`. A finite box enclosing the source and held mesh supplies the
diameter bound. General four-section free-form profiles, changing corner
radius or half-height, and non-monotone width controls remain
`ErrUnsupported`.

## 10. Three independently varying rectangular sections

This route consumes three authenticated, hole-free Sketch profiles. Each has
four whole `LineSeg` records whose walked vertices form an axis-aligned,
counterclockwise rectangle centered on the common plane origin. The three
rectangles may start their recorded walks at different corners. Their planes
have the world-XY axes, the same XY origin, and equally spaced ascending Z
levels. This route refuses a profile outside the rectangular class; the
other `LoftSections` routes keep their own admission. Dispatch chooses
this route when the dimensions have different exact ratios or the walks
start at different corners. Homothetic rectangles with matching recorded
walks keep §1's existing builder.

Let `w_i,h_i > 0` be exact half-width and half-height from the recorded
corners at normalized heights `0,1/2,1`. For `x = w,h`, define
`a_x = 2(x_2-2x_1+x_0)`, `b_x = x_2-x_0-a_x`, and
`x(t)=x_0+b_x t+a_x t²`. Require the middle Bernstein control
`B_x=2x_1-(x_0+x_2)/2` to be nonnegative. Positive endpoints and this
control prove `x(t)>0` for every `0<=t<=1`. Each exact section is the
rectangle `[-w(t),w(t)] × [-h(t),h(t)]` at `z=z_0+Ht`.
Distinct heights cannot intersect, and positive dimensions keep each section
embedded. Each source edge sweeps one quadratic wall and names one live
`NURBSSurface` face; the two end caps are planar.

The held shell uses `N` equal-height bands, with `N=16,32,64,128` chosen as
the first for which `D=(|a_w|+|a_h|)/(4N²) <= 0.05 mm`. Every exact rational
ring corner and height must be representable as a finite float64. At a band,
linear interpolation of the two endpoint widths and heights makes every
horizontal or vertical wall quad planar; two triangles tile it exactly.
The first and last rings also tile the end caps. The faceted shell audit
must pass before commit. The map with normalized rectangle coordinates
`(u,v)` sends `(u w(t),v h(t),z)` to `(u l_w(t),v l_h(t),z)` and is continuous,
one-to-one and orientation preserving in both directions. Its distance is
at most `D`, since `|x(t)-l_x(t)| <= |a_x|/(4N²)`. Adjacent maps use the same
held ring and the caps use their exact source corners.

Write `W=max(w_0,B_w,w_2)` and `K=max(h_0,B_h,h_2)`. The Bernstein hull
bounds both true and held dimensions by `W,K`. The true volume is the exact
rational `4H ∫_0^1 w(t)h(t) dt`; the held mesh integrates the piecewise-linear
dimensions. At each height, insert an intermediate rectangle with held width
and true height. The two resulting symmetric-difference strip areas sum to
at most `4(K|w-l_w|+W|h-l_h|)`. For either quadratic,
`∫_0^1 |x-l_x| dt = |a_x|/(6N²)`. Therefore the occupied-volume allowance is

```text
4H (K|a_w| + W|a_h|)/(6N²).
```

This is non-cancelling even when width and height curve in opposite
directions. A signed volume difference cannot replace it. The allowance
feeds `facetedPayload.VolSymDiff` and bounds the true volume reading against
the held tetrahedron sum.

For area, each pair of horizontal walls has density
`4w sqrt(H²+h'²)` and each pair of vertical walls has density
`4h sqrt(H²+w'²)` in normalized height. Let
`P_x=max(|b_x|,|2a_x+b_x|)`. The square-root factor is one-Lipschitz in its
derivative argument and is at most `H+P_x`. Piecewise-linear interpolation
also gives `∫|x'-l_x'|=|a_x|/(2N)`. The non-cancelling wall-area allowance is

```text
4[(H+P_h)|a_w|/(6N²) + W|a_h|/(2N)
  + (H+P_w)|a_h|/(6N²) + K|a_w|/(2N)].
```

The held and true end caps coincide; faceted measurement arithmetic adds
its own rounding charge. Round every rational proof upward. The diameter
upper bound `2(W+K)+H` covers the source and held shells. The existing
faceted placement, `Tessellate(VerifyAll)`, mesh-boolean, and export paths
consume the resulting payload. A nonpositive dimension, failed Bernstein
gate, unrepresentable station, or unmet 128-band bound returns
`ErrUnsupported` without committing a body.

## 11. Four independently varying rectangular sections

Four authenticated, hole-free profiles with the four whole-line rectangle
records of §10 may vary both half-width and half-height. The planes share
world-XY axes and origin at four ascending, equally spaced Z levels. The
recorded walk may start at any corner in each section. This route is tried
before §9's rounded-rectangle route; a profile outside the rectangular class
continues to §9.

For each dimension `x=w,h`, interpolate the four exact recorded values at
`t=0,1/3,2/3,1` with one cubic Bernstein curve `x(t)` with controls
`B_0,B_1,B_2,B_3`. Require `B_0,B_3>0` and `B_1,B_2>=0`. These exact
coefficient gates prove `x(t)>0` throughout `[0,1]`. No monotonicity gate
applies. The denoted solid has the section
`[-w(t),w(t)] × [-h(t),h(t)]` at `z=z_0+Ht`. Its four walls are cubic
tensor-product surfaces, and its two caps are planar. Strictly increasing
height and positive dimensions prove that distinct sections do not meet.

Let `M_x=max_[0,1]|x''(t)|`, evaluated from the two exact Bernstein second
differences. Let `X=max(B_0,B_1,B_2,B_3)` and
`P_x=3 max_i|B_(i+1)-B_i|`; the Bernstein hull bounds the dimension and
its derivative. Choose the first `N=16,32,64,128` for which
`D=(M_w+M_h)/(8N²) <= 0.05 mm`. Every ring coordinate at `t=i/N`
must be exactly representable by finite float64. The held wall joins
adjacent exact rings by linear interpolation. Each side quad is planar
because one coordinate is fixed on that side of a rectangular ring; its
two triangles tile the quad. The two caps use the first and last rings.
The closed faceted mesh audit runs before the body commits.

The linear interpolation remainder on each band satisfies
`|x-l_x| <= M_x/(8N²)`. Mapping each true rectangle to the held rectangle
at the same normalized `(u,v,t)` moves every point by at most `D` in both
directions. `MeshBound` and each vertex certificate carry `D`.
The true volume is `4H ∫_0^1 w(t)h(t) dt`. Both true and held dimensions
are bounded by their Bernstein maxima `W,K`. Since
`∫_0^1 |x-l_x| dt <= M_x/(12N²)`, the non-cancelling occupied-volume
allowance is

```text
4H (K M_w + W M_h)/(12N²).
```

For the two horizontal and two vertical walls, use §10's area densities.
The derivative's deviation from its bandwise average obeys
`∫_0^1 |x'-l_x'| dt <= M_x/(2N)`. Combining that bound with the
integrated position bound and the one-Lipschitz square root yields the
non-cancelling area allowance

```text
4[(H+P_h) M_w/(12N²) + W M_h/(2N)
  + (H+P_w) M_h/(12N²) + K M_w/(2N)].
```

The exact rational terms round upward before entering the faceted payload.
The diameter bound is `2(W+K)+H`. Placement, `Tessellate(VerifyAll)`,
booleans, and export consume that payload. A failed positive-control gate,
plane/spacing gate, exact-station gate, or 128-band bound returns
`ErrUnsupported` without committing a body.

## 12. Five to thirty-two independently varying rectangular sections

Five to thirty-two authenticated, hole-free profiles use §10's exact centered
four-line rectangle record gate. Their world-XY planes have one XY origin and
ascending, equally spaced Z levels. A different first corner in each recorded
walk does not change its rectangle. A non-rectangle, unequal spacing, or a
thirty-third section returns `ErrUnsupported` before body commit.

Let `x_i` be the exact recorded half-widths or half-heights. For each
dimension, set `m_0=x_1-x_0`, `m_(n-1)=x_(n-1)-x_(n-2)`, and
`m_i=(x_(i+1)-x_(i-1))/2` at interior sections. Span `i`, parameterized by
`u∈[0,1]`, has cubic Bernstein controls

```text
[x_i, x_i + m_i/3, x_(i+1) - m_(i+1)/3, x_(i+1)].
```

The same tangent at a shared section makes adjacent cubics C1 in height.
Require positive endpoint controls and nonnegative interior controls on
every span for both dimensions. Their Bernstein hulls then prove strictly
positive width and height throughout every span. Strictly ascending Z levels
and positive dimensions make distinct sections disjoint.

For each span, use §11's exact maximum second derivative `M_x`, Bernstein
maximum `X`, and derivative maximum `P_x`. Choose that span's first
`N_i=16,32,64,128` with `(M_w+M_h)/(8N_i²) <= 0.05 mm`. Reject if no
count qualifies or any ring coordinate cannot be held exactly by finite
float64. One ring at each shared section is used by both neighboring spans.
Each wall quad is planar and split into two triangles. The first and last
rings form planar caps; the closed faceted shell audit precedes body commit.

The source-to-held displacement bound is the maximum span bound. For each
span of height `H_i`, use §11's non-cancelling occupied-volume and area
allowances with that span's own `W_i,K_i,P_w,i,P_h,i,M_w,i,M_h,i,N_i`.
Sum those allowances across spans before upward rounding. The global
diameter bound is `2(max_i W_i + max_i K_i) + Σ_i H_i`. The resulting
faceted payload supplies `Volume`, `Area`, and `Tessellate(VerifyAll)`.

The independent rectangular builders in §§10–12 also retain their admitted
exact height-curve controls in the original body's payload. `Body.Section`
uses those controls for an exact interior world-XY rectangle when its
half-dimensions are representable. See `agent-docs/section-design.md`. Placement,
scaling and shape-changing operations drop this source certificate.

The original three-section whole-line builder also retains its base walk and
exact quadratic scale. `Body.Section` evaluates them for an exact interior
world-XY polygon only when every scaled coordinate is representable. See
`agent-docs/section-design.md`. Placement and scaling drop this certificate.

## 13. Twisted sections with one fitted edge

### 13.1 Source class and denoted wall

Three to 1024 authenticated, hole-free profiles whose outer loop has
one whole `FitSplineSeg` record and two to eight whole `LineSeg` records may
turn about the plane origin from section to section. Every section has the
same number of lines, the same number of fit points, and the fit spline in the
same cyclic position relative to its lines. The recorded walk may start at any
segment: the fit spline anchors the correspondence, so segment `k` of section
`i` is the one `k` steps after the fit spline, and edge `k` in the first
profile's order names the face `side(0,k)`. A loop with two fit splines, an
arc, a trimmed record or a different fit-point count returns `ErrUnsupported`.
This route is tried for three sections only after §§1–8 refuse, and for four
or more sections it replaces §§9–12, which cannot admit a fit spline.

The planes keep world-XY axes and a common XY origin. Z levels only need to
ascend strictly: float stations such as `i·0.26` are never exactly equally
spaced, so the blend below uses the exact rational spacing of the recorded
levels. Plane-local points differ freely between sections; a rotation about
the origin is not a parameter of the route. Every join between consecutive
records must be exactly the recorded join: a line's walked end equals the next
walked start, and the first and last control points of the converted fit chain
equal the neighbouring lines' recorded endpoints. The walk must run
counterclockwise about the plane origin.

Sketch's exported natural-cubic `FitInterpolant` converts each fit record to
exact rational cubic Bézier spans in walk order (the §8.1 conversion). A line
is the degree-elevated cubic on thirds. All sections must give the fit the
same span count, so the span-index coordinate of §8.1 is the shared curve
parameter `u` and no span is refined. Edge `e` of section `i` is then a chain
of cubic Bézier spans with exact rational controls.

Along height, use a C1 piecewise cubic Hermite blend through the section
curves, the Catmull-Rom blend of §12 with exact spacing. For a control point
`x_i` of section `i` at level `z_i`, the slope per unit Z is
`d_i = (x_{i+1} - x_{i-1})/(z_{i+1} - z_{i-1})` inside and the one-sided
quotient at either end. Height span `i`, `t∈[0,1]`, `h_i = z_{i+1} - z_i`,
has Bernstein controls `[x_i, x_i + h_i d_i/3, x_{i+1} - h_i d_{i+1}/3,
x_{i+1}]`. The blend is linear in the data, so the span-`k` patch of edge `e`
between sections `i` and `i+1` is the exact bicubic
`S(u,t) = (C(u,t), z_i + h_i t)` with 4×4 rational control points. It equals
every recorded section at `t=0,1`, and neighbouring spans share a first
derivative in `t`. Fitted and line side faces report `NURBSSurface`; the
caps report `Plane`. No interior section creates a seam face.

### 13.2 Continuous-section admission

The polar angle of the boundary at height `t` increases strictly when
`cross(C, C_u) > 0`. Take this function on each patch in exact Bernstein form,
degree `(5,6)` in `(u,t)`; accept a cell only when every coefficient is
strictly positive, otherwise bisect `u` and `t` alternately, to depth 12 and
under the shared work budget, as §8.2. A cell with a non-positive corner
coefficient is a counterexample and refuses at once. Exhaustion refuses with
`ErrUnsupported`. Strict positivity keeps `C` off the origin at every height.
A closed curve whose polar angle strictly increases turns an integer number of
times, at least once. The first section is a valid Sketch loop and so simple;
a simple closed curve turning with strictly increasing angle turns once. The
turn count is constant in `t` because `C` never reaches the origin, so every
intermediate boundary is a simple radial graph. Strictly increasing Z
separates different heights, so the wall is embedded. The gate does not need
the sections to be rotations of the first: it proves only what the solid
needs. A section whose boundary the origin does not see radially refuses.

### 13.3 Held mesh and certified distance

Call a patch `P` one edge span `k` between sections `i` and `i+1`. Over its
control net, in the L1 norm and with `Δ` the control differences in the named
direction, write

```text
B_u = 3 max |Δ_u C|            B_t = 3 max |Δ_t C|
M_uu = 6 max |Δ_u² C|          M_tt = 6 max |Δ_t² C|
M_ut = 9 max |Δ_u Δ_t C|
```

These bound `|C_u|`, `|C_t|` and the second derivatives of `C`; `S` adds only
the constant `h_i` to `S_t` (`S` is linear in `z`). Each patch gets integer
counts `n_u(k)`, the same at every height, and `n_t(i)`, the same for every
edge span, so one station table serves every ring and the mesh conforms.
For a target `τ`, set `n_t(i)` to the least integer with
`M_tt/(8 n_t²) <= τ/3` over all `k`, and `n_u(k)` to the least integer with
`M_uu/(8 n_u²) <= τ/3` and `M_ut/(4 n_u n_t) <= τ/3` over all `i`. The targets
`τ = 0.02, 0.03, 0.049 mm` are tried in turn and the first whose table holds
at most 500000 triangles is used, so a short loft gets the tightest bound.
More than 64 `n_u` or 32 `n_t` at a target skips it; no target left, or a
published bound above 0.05 mm, refuses.

The held vertex at `(u=a/n_u, t=b/n_t)` is the exact rational patch point
rounded to float64 once per coordinate. The L1 rounding distance, including
the rounded intermediate Z, is charged as `ρ`. The first and last ring are the
recorded coordinates and Z levels. A cell has two triangles on a fixed
diagonal. The caps triangulate the first and last ring.

For a cell, let `A,B,C,D` be its corners at `(0,0),(1,0),(1,1),(0,1)` of the
cell and `tw = A - B + C - D`. Separable linear interpolation differs from `S`
by at most `M_uu/(8 n_u²) + M_tt/(8 n_t²)` (vector valued). Either triangle
differs from the bilinear interpolant by `|tw| r(1-s)` with `r<=s` in the
triangle's parameters, at most `|tw|/4`, and `|tw| <= M_ut/(n_u n_t)`. So

```text
d_P = M_uu/(8 n_u²) + M_tt/(8 n_t²) + M_ut/(4 n_u n_t) + ρ
```

bounds the distance from any wall point to the held triangle with the same
parameters, and the same map covers the held wall in the other direction.
`MeshBound` and each vertex certificate carry the largest `d_P`. The proof
bounds the distance between the held wall and the denoted bicubic patches. It
does not claim anything about Fusion's own loft.

### 13.4 Occupied volume, area and consumers

The held mesh passes `loftmesh.LoftCrossingAuditBounded` (ceiling 4096 pair-enumeration
steps per triangle, fixed by the 500000-triangle limit) and the faceted
shell audit before commit, so it is a closed embedded oriented surface. The
denoted boundary is closed and embedded by §13.2. Join each wall point to its
parameter-matched held point by a straight segment. The caps lie in the planes
of the first and last section and sweep zero volume. Where the two closed
surfaces have different winding number, the point lies on one of the segments,
so the occupied symmetric difference lies in the sweep. The sweep's volume is
at most `∫∫∫ |E| |F_u| |F_t|` with `E = H - S` and `F = (1-λ)S + λH`. The
derivatives of `H` are means of `S_u`, `S_t` along a cell edge, so
`|F_u| <= B_u + 2ρ n_u` and `|F_t| <= B_t + h_i + 2ρ n_t`. Integrating the
pointwise distance, whose mean over a cell is two thirds of its supremum for
each linear-interpolation term and one third for the twist term,

```text
VolSymDiff <= Σ_P (2/3 (uu + tt) + ut/3 + ρ)(B_u + 2ρ n_u)(B_t + h_i + 2ρ n_t)
```

where `uu, tt, ut` are `d_P`'s three terms. A signed difference of volumes
cannot replace this non-cancelling sum.

The area allowance bounds `|∫|S_u × S_t| - Σ triangle areas|` by
`∫ |S_u - H_u||S_t| + |H_u||S_t - H_t|`. On a cell, the mean deviations are
`|H_u - S_u| <= M_uu/(3 n_u) + M_ut/(2 n_t) + 2ρ n_u` and
`|H_t - S_t| <= M_ut/(2 n_u) + M_tt/(3 n_t) + 2ρ n_t`. A cap differs from its
ring polygon by the area swept between ring and polygon, at most
`(M_uu/(8 n_u²) + ρ)(B_u + 2ρ n_u)` per edge span, for each cap. Every term
rounds upward into `AreaSlack`. This bound is first order in the cell size,
unlike the second-order distance and volume bounds, and is loose at this
resolution (about 40% of the area for the 41-section ribbon of
`TestLoftSectionsTwistedRibbonKeepsVolumeAndMeshProof`). The diameter
bound is `2 max |C|_1 + (z_{n-1} - z_0)`, over every control point.

Placement, `Tessellate(VerifyAll)`, mesh booleans and export use the standard
faceted payload. `Body.Section` has no source certificate for this route and
reports its usual refusal. Refusals, all `ErrUnsupported` with the document
unchanged: wrong record kinds, count or fit-point mismatch, differing plane
axes or origin, Z levels that do not strictly ascend, an inexact join, a
clockwise walk, a failed polar proof, a station or triangle ceiling (500000 triangles),
`MeshBound` above 0.05 mm, a non-finite proof term, or more than 1024
sections.

### 13.5 Long lofts and cost

Proof, gathering and station work are per patch, per section or per ring, so
cost is linear in the section count, and they run on all CPUs. The crossing
audit is the only global step, bounded by the triangle and scan ceilings.
Each patch's exact work runs on integers: its 32 control coordinates are
written as numerators over one positive denominator `d`, the least common
multiple of theirs (`twistCommon`). A positive scale keeps every sign and
every order, so the polar coefficients of §13.2 decide the same cells and
verdicts from their numerators, and the §13.3 maxima select the same control
differences; each bound is reduced once, as numerator times its factor over
`d`. A held station is a numerator over `d·n_u³·n_t³` (the Bernstein weights
at `a/n_u` and `b/n_t` are integers over those cubes, and the float origin
is a dyadic). It rounds from that numerator and denominator to the float
that `big.Rat` rounding of the same value gives (`proof.FracFloat64`), and
its rounding distance is the exact difference from that float, compared
with the ring's largest by cross multiplication. No float decides anything,
and every vertex and published number equals the reduced-rational
evaluation bit for bit. The mesh needs rings about every 0.04 mm of height for a moving tooth, whatever
the section spacing, so the triangle count follows the height and the
tolerance and not the section count. `twistTargets` are tried finest first
(0.02, 0.03, then 0.049 mm), and the first table at or under 500000 triangles
is used (`TestTwistedSelectCountsFallsBackPastTheCeiling`); a loft none of them
fits is refused. The bound is certified for any spacing; a coarser spacing
can also meet 0.05 mm but denotes a different, smoother solid, because
Catmull-Rom through undersampled tooth phases is not the helical tooth.

A mesh boolean tessellates the pair at a chord tolerance proportional to the
pair's diameter. `Intersect` and `Cut` refuse a long twisted loft whose held
bound is coarser than that tolerance, because a table that fine would exceed
the triangle ceiling.

### 13.6 Size and provability

The route is about 1000 lines of Go in `internal/smoothloft/` (patch algebra
and admission, gathering, mesh and bounds) plus 60 in the root glue. Every
bound above is a closed-form rational expression over exact control nets, so
the error bound is provable as stated. No numeric search or sampled residual
admits input. For the 41-section ribbon of
`TestLoftSectionsTwistedRibbonKeepsVolumeAndMeshProof` the mesh has about
39000 triangles with `MeshBound` 0.014 mm.
