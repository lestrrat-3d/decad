# Source-cap nine-section loft

`Document.LoftFaces(ctx, sections ...*Face)` builds one solid through nine
ordered planar caps. The admitted producer is eight live, complete Split
slabs from one authenticated `LoftFromPoint` source. Each section is either
an original Split cap, its rigid image, or its positive uniform image. The
first section is the toe cap of the least slab; the remaining eight are the
heel caps in source order. This is a separate operation from the three-Sketch
`LoftSections` route. No public faceted cap edge is promoted to a Sketch
curve.

## Source and correspondence

Every admitted cap retains the same immutable point-loft Sketch/profile,
source station order, exact rational ray-plane cut map, and ordered rigid
and positive-scale steps. Each source record is current when the loft is
built. A side face, mixed source, duplicate cap, stale source, reordered
cap, missing proof record, or any section count other than nine refuses
without retiring an operand. The record, rather than a face's rounded
`Plane` tag or `FacetedCurve` edges, defines the cap curve.

Write `D` for the original authenticated planar Sketch region and `P` for
its certified station polygon. An exact cut cap maps each source point `p`
from the original far plane by `a + d(p-a)/(n·(p-a))`, where `a` is the
original apex and `n,d` come from the Split sheet's rational plane. The
far cap is the identity map. Subsequent similarities act in their recorded
order. Call the resulting map of section `i` `C_i(p)`. The denoted solid is
the image of `D × [0,8]` under the piecewise ruled map
`F_i(p,t)=(1-t)C_i(p)+t C_{i+1}(p)` for each unit band. The source
parameter `p` gives the correspondence; a fitted-flank sample or a
reconstructed Sketch profile does not.

## Embedding admission

The source station polygon and its certified curve displacement give a
closed rectangle containing `D` and `P`. Every cut-map denominator must
remain strictly positive over that rectangle. For each band, an exact
rational Jacobian interval gate proves one fixed invertible matrix `M_i`
satisfies `||M_i^-1 DF_i-I||∞ < 1` everywhere on the rectangle times that
band. Integrating the derivative along a source-domain segment then proves
`F_i` is injective. Subdividing the rectangle may tighten interval bounds,
but it cannot change the map or turn a sampled test into an admission gate.

For each interior section, its exact supporting plane is derived from the
cut map and ordered similarities. The entire preceding cap must lie
strictly on one side of that plane and the entire following cap strictly
on the other. A positive-denominator linear-fractional cap map has its
plane-functional extrema at rectangle corners. The two adjacent ruled
bands can therefore meet only on their common cap. The same plane test
places every earlier and later cap on opposite sides, so nonneighboring
bands are disjoint. These gates establish one
embedded solid before a faceted body is published.

## Held mesh and proof

The held mesh uses the selected caps' original station-indexed rings. Two
triangles span each pair of corresponding ring chords. The first and last
caps reuse the original point loft's triangulation, reversing the first.
The exact held mesh must pass the whole-mesh crossing and closed-solid
audits. Each wall face names its source segment and band. The end faces
carry fresh `capStart` and `capEnd` roles and their selected source origins.

The source region difference `area(D △ P)` is bounded by the original
station displacement area tube. An upper bound `J_i` on the absolute
Jacobian determinant of `F_i` gives a volume allowance
`area(D △ P) × Σ J_i` between the fitted and polygonal ruled solids.
Projective cap maps send each edge of `P` to a straight segment, but its
source parameter generally walks that segment nonlinearly. Its exact
denominator ratios bound this reparameterization. The mapped source
per-cell arc-speed and matched-departure bounds, each similarity stretch,
held-vertex rounding, and the existing cell twist allowance then bound
the polygonal ruled wall against its two held triangles. The source-region
allowance also covers the fitted end caps; a held-vertex sweep closes the
remaining occupied-volume proof. The final
allowance must be finite and below the positive held-volume floor.

The successful result is one certified held primitive for mesh booleans.
Its proven `meshBound` raises the pair's internal chord tolerance when
necessary; the analytic partner then chords at that raised tolerance and
publishes its own bounds. Rigid placement keeps this classification and
charges its new mesh bound. A Boolean result does not keep it: later pairs
must pass the local chain-depth gate on the facets they touch. The hidden
contact, rim, embedding, and occupied-volume gates still run for every pair.

The same per-cell displacement plus twist offset bounds the denoted wall
against its held mesh for `Tessellate(VerifyAll)`. Area bounds cover the
true ruled wall, both fitted end caps, and held arithmetic. A requested
tolerance below the certified mesh bound refuses. Cancellation leaves the
document and all source bodies unchanged.

## Two-cone continuation

A Cut by an unplaced, full, straight world-X cone can carry one continuation
record from an original, unplaced LoftFaces body. The final cap map must place
the entire authenticated source rectangle strictly outside the toe cone. The
Cut must leave every final-cap held triangle unchanged, with one surviving cap
face. A partial or altered cap receives no record.

The next Intersect accepts that record only with a second exact cone sharing
the toe cone's far plane and strictly ordered outside it over the original
LoftFaces mesh. The final cap map must have a positive denominator over the
source rectangle. With the heel cone's actual rational apex and slope, the
denominator-cleared cone pullback must be `A*u² + C*v²`, with `A,C > 0` and
the rectangle strictly on one side of `u = 0`. This proves every true cap
point is outside the heel cone. The cap face's held facets must also be
disjoint from the heel cone wall's held facets. Only that exact face pair
skips the proximity refusal. The held Intersect result must contain no facets
from the cap; a retained held cap would disagree with the true separation.

This continuation tries four, eight, sixteen and at most thirty-two times
the ordinary pair chord. It retries only a first-operand staging refusal.
The local chain-depth gate still checks the held bound at every other contact.
The same mesh Boolean still checks all other face pairs, contacts, rims,
occupied-volume bounds, and the final mesh. Placement drops the record.
The Intersect result has no record and cannot continue the special chain.
