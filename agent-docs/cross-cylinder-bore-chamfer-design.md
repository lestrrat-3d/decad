# Perpendicular cylinder bore-mouth chamfer

This route admits one equal-setback chamfer on the positive-X mouth of a
cross-drilled circular cylinder. The public producer is a `Cut` of two full
Sketch-circle extrusions. The `Cut` still uses the ordinary mesh Boolean and
keeps its existing contact proof. It adds a one-use source record only when
both prisms have the exact cardinal pose below. `Chamfer` rebuilds from those
source circles; its selected faceted edge identifies the mouth and supplies
no geometric samples to the new surface.

## Admission

The target cylinder is `x²+y²≤R², 0≤z≤H`, with an XY circle centered at the
origin. The tool removes `y²+(z-c)²≤r²`, with a YZ circle centered at `(0,c)`
and an X sweep extending more than `R+1 mm` either side. Both sections are
whole CCW circles with zero section and sweep-level displacement. The
authenticated circular source edges must carry finite curve bounds below
one-thousandth of their radii. Placement clears the source record.

The bounded class has `2≤R≤20 mm`, `0.25≤r≤R/4`, `2≤H≤40 mm`, and
`0<c-r<c+r<H`. One selected closed faceted edge must join the target and
tool wall origins at the positive-X mouth. Its held start lies at `x>R/2`.
The equal setback `d` is positive, at most `r/2`, and leaves both target caps
clear: `c-r-d>0` and `c+r+d<H`. The negative mouth, asymmetric setback,
changed pose, and other source or selection classes refuse.

## Source surface and band

For `θ` around the bore, set `C=cos θ`, `S=sin θ`,
`X=sqrt(R²-r²C²)`, and `Q=sqrt(R²-r²C⁴)`. The source intersection on the
positive side is `(X,rC,c+rS)`; the negative side uses `−X`.

Unroll the outer cylinder into `(R atan2(y,x),z)` and the bore into
`(x,rθ)`. Moving `d` along each developed surface's unit normal to the
intersection gives these feet:

- Outer foot: rotate `(X,rC)` around the Z axis by
  `δo=dCX/(RQ)`, then set `z=c+rS+dRS/Q`.
- Bore foot: set `x=X(1-d/Q)`, rotate `(rC,rS)` about the bore axis by
  `δb=dCS/Q`, then restore the Z offset `c`.

The chamfer face is the straight ruled band between corresponding feet.
Each foot is exactly `d` from the original seam on its own developed
cylinder. The complete remaining outer and bore walls join those feet; the
negative mouth and target caps stay on their source surfaces.

The outer foot's cylindrical angle increases with `C`. Its derivative has
the positive terms `r/X` and
`(d/R)[(R²−2r²C²)/(XQ)+2r²C⁴X/Q³]`. The class bound `r≤R/4` makes each
denominator positive and `R²−2r²C²>0`. The positive opening therefore stays
a single graph over the target cylinder's angle.

## Boundary and occupied volume

The positive mouth uses 4,096 whole-turn cells. Each cell evaluates the
source feet over outward rational sine, cosine, square-root and normal-angle
intervals. It requires the outer foot outside the bore, the bore foot
inside the target, and an outward support dot product at the bore foot.
The first, mixed and last polar-cross-product coefficients are positive,
which makes every band cross-section advance around the bore. The two
Jacobian endpoint coefficients have the same strict sign, excluding a band
fold. Failed interval signs refuse the operation.

Only the 1,025 cells of the first quarter turn, `0≤θ≤π/2`, are proved;
every other cell is the exact mirror image of one of them. The admitted
pose makes both walls, `x²+y²=R²` and `y²+(z-c)²=r²`, exactly symmetric
under `y→−y` and `z−c→−(z−c)`, and the feet depend on `θ` only through
`C` and `S`. With `θ_j=2πj/4096`, cell `j` has a base cell `k` with
`θ_j=θ_k`, `π−θ_k`, `π+θ_k` or `2π−θ_k`, an identity of exact turns, so
`θ→π−θ` (`C→−C`) negates every Y coordinate, `θ→−θ` (`S→−S`) negates
every Z−c coordinate, and X keeps its value. The feet of cell `j` over its
whole angular range are the feet of cell `k` with those coordinates negated,
so negating base `k`'s enclosures, an exact rational step, encloses cell
`j`. Interval width and absolute bounds do not change under negation. Each
of the eight interval signs pairs coordinates or derivatives of equal
parity, since a derivative gains one more sign under each map, so it takes
the same value at both angles. Before any cell is proved, the record is
rebuilt from its two source bodies by admission's exact comparisons and must
equal itself; a pose that is symmetric only to rounding refuses. The
normal-angle sine and cosine enclosures widen from the lower end of their
angle interval, so the enclosures computed directly in the other quarters are
valid but not the mirror images of the quarter-turn ones; the mirrored ones
are the ones used.

The held mesh has the two target caps, outer wall with two openings, bore
wall, and one chamfer band. A full exact facet-contact and closed-shell
audit checks it. The audit proposes each cap fan as a projected XY patch. It
omits only the within-cap pairs of a cap whose patch proof holds, and
classifies every other pair exactly, through the row-batched route. A cap
whose proof fails keeps all its pairs. The row preflight charges the scan
ceiling differently from the pairwise route, so a record near that ceiling
can refuse on one route and not the other; every such refusal is still a refusal. At each angular cell, paired source and held wall
points use the same height or axial fraction. The band uses the same transverse
fraction. Three cell coordinate diagonals enclose the source-to-facet
departure, with separate charges for held sample rounding, source-circle
displacement, and outer/bore cylinder chord sagitta. The held sample charge
compares every cell's own held outer foot, bore foot and negative-mouth
vertex with that cell's enclosures, mirrored or not, reading Z as an exact
shift of the Z−c enclosure by `c`. Every outer-ring vertex
is compared with an outward trigonometric interval at its recorded angle;
this charges radius and angle-conversion rounding. Exact rational differences
between consecutive recorded angles, including the full-turn wrap, bound
each outer chord. The bore sagitta uses
the angular change *across* the bore wall as well as the station gap.
The resulting bound must be below 0.1 mm.

The source and held solids can differ freely within a box around the one
modified positive mouth. The box width is bounded by
`R−sqrt(R²−r²)+2d`, and its Y and Z spans by `2(r+d)`, enlarged for source
displacement. Each foot lies within Euclidean distance `d` of its seam
point because its developed-cylinder travel has length `d`; this contains
the entire band in the box. Outside that box, differences fit the complete outer
and bore radial chord annuli. Their volumes and the box volume give the
occupied-volume allowance. The held mesh's exact rational volume must
exceed this allowance. A source-area upper bound plus the held mesh area
bounds the surface-area difference. The held diameter plus boundary
departure enters the usual volume-based centroid allowance.

## Shared cell proof

`CellExactProof` reads only the radii `R`, `r` and the setback `d` as plain
numbers, with the bore axis at `z − c = 0`; `Feet` reads the record through
`FeetAt`, which takes the same numbers and `c`. The retained saddle chamfer
(`agent-docs/retained-edge-blend-design.md` §4.9) maps a shaft and a
perpendicular bore read from exact retained tags onto this pose and proves its
tool's band with the same cell tests at its own station count
(`crossbore.ProveToolBand`); its admission is that document's, not this one's.

## Public case

For the public `R=5, r=1, H=10, c=5, d=0.5 mm` case, the result has
47,104 facets, a 0.046353 mm boundary bound, and a
`753.228711±10.088197 mm³` volume reading. `Tessellate(0.1 mm, VerifyAll)`
reports both boundary and volume verified.
