# Source-cap face-fed loft

`Document.LoftFaces(ctx, sections ...*Face)` builds one solid through two to
thirty-two ordered planar caps. The admitted producer is one to thirty-one
live, complete Split slabs from one authenticated `LoftFromPoint` source.
Each section is an original Split cap, its rigid image, or its positive uniform image. The
first section is the toe cap of the least slab; the remaining sections are the
heel caps in source order. This is a separate operation from the three-Sketch
`LoftSections` route. No public faceted cap edge is promoted to a Sketch
curve.

## Source and correspondence

Every admitted cap retains the same immutable point-loft Sketch/profile,
source station order, exact rational ray-plane cut map, and ordered rigid
and positive-scale steps. Each source record is current when the loft is
built. A side face, mixed source, duplicate cap, stale source, reordered
cap, missing proof record, or a section count outside two to thirty-two refuses
without retiring an operand. The record, rather than a face's rounded
`Plane` tag or `FacetedCurve` edges, defines the cap curve.
The thirty-two-cap ceiling bounds pairwise section-plane and mesh-contact work;
every admitted count still passes the same exact embedding and volume proofs.

Write `D` for the original authenticated planar Sketch region and `P` for
its certified station polygon. An exact cut cap maps each source point `p`
from the original far plane by `a + d(p-a)/(n·(p-a))`, where `a` is the
original apex and `n,d` come from the Split sheet's rational plane. The
far cap is the identity map. Subsequent similarities act in their recorded
order. Call the resulting map of section `i` `C_i(p)`. For `N` sections, the
denoted solid is the image of `D × [0,N−1]` under the piecewise ruled map
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
When the original LoftFaces result carries two to thirty-two exact maps,
matching source-cell bounds, and a complete source-station group, a successful
toe Cut and heel Intersect retain an immutable source record on the unplaced
faceted result. The record identifies the two
surviving tool-cap face groups. A later source-specific Union rechecks the
original Sketch revision, map denominators, cone order, and both cap groups.
The curved source gate rederives both exact cone meridians from the live tool
bodies and rejects changed cached apex, far-plane, or slope values.
It also requires each tool-cap group to own one live result face without triangles
from another source group.
Every other Intersect result has no continuation record. Placement drops it.
The current spiral source-specific Union still admits only its nine-cap class.

For the first curved tooth joined to its blank, the retained source maps prove
one transverse blank-root crossing on each fitted flank through all eight
bands. The other profile edges stay strictly on one side of the root, and the
crossings advance from toe to heel. The complete original toe cap lies before
its tool/root circle in X. The complete original heel cap lies in an exact
plane that misses its entire tool/root circle. These whole-cap checks rule out
contacts through an end-cap interior. A caller may also request an exact box
around either moving flank/root crossing over a narrow X interval; the local
rim proof uses that box to decide which source surface remains exposed.
For the first band, the source proof also encloses each complete flank/root
seam from the finite toe root ring through cap two. It divides the moving
branch into exact time slabs and encloses each slab and the matching X strip
of an authenticated held switch chain. An exact maximum box distance below
the chain's smallest curve bound covers both boundary directions. The cap-two
source endpoint has a separate box for the held chain's final X tail. The
caller must first prove that the held chain is complete, strictly ordered in
X, and belongs to the matching blank-root and fitted-flank face pair.

The first-join proximity check binds each exceptional face pair to its live
source origin. The finite blank toe and heel outer meridians lie strictly
outside the toe Cut cone and inside the heel Intersect cone, respectively;
the opposite tool also stays on the retained side over each complete cap.
The blank root lies beyond the toe tool's root intersection and before the
heel tool's root intersection. These exact affine signs keep the certified
source crossings on the trimmed tooth. The near-pair table admits the six
flank contacts, two toe connector contacts, and one toe root-arc contact only
after their separate source crossing certificates pass. Five other near
pairs have strict source separation: each matching blank/tool cap pair, the
blank toe cap against the heel tool cap, and the finite blank root against
both tool caps. Their held facets must also be disjoint. Every unlisted
near pair keeps the ordinary refusal.

The root arc also crosses each finite blank end cap. The toe and heel source
certificates bound the crossing of the authenticated Sketch arc with the
blank cap's exact straight meridian. They check all source station cells,
bound the recorded arc's departure from a rational circle, and prove one
transverse crossing through the relevant LoftFaces band. Exact polynomial
signs bound the crossing's X coordinate and angular slope; isolated endpoint
boxes include the source construction error. A later Union may use those
bounds only while the original unplaced source record remains current.

The same source record can certify four short contact curves at each finite
blank end cap: two fitted flanks and two straight connectors. Each source
cell has one transverse cap crossing. The fitted flanks meet the finite cap
start once because that meridian equals the already certified root meridian
there; later fitted spans stay outside the root and cannot meet the cap.
Exact rational boxes enclose each clipped crossing for a later held-edge
comparison. A missing source segment, multiple clip, or unbounded box
refuses this source-specific certificate.

For the held comparison, divide each authenticated source crossing into
sixteen parameter cells. Each cell has a nonempty finite source arc. The one
cell that contains a fitted flank's isolated cap-start clip may enclose a
small excluded part, but its larger box is still safe. Divide each held
switch edge into four exact straight parts. Every source cell box must be
within the published bound of at least one held part, and every held part
box must be within that bound of at least one source cell. Compare the most
distant corners of each paired box with exact rationals and round the final
distance upward. These two checks cover both complete curves and their
endpoints without assuming that source X or angle is monotone.

## Generic finite Revolve source gate

The ordinary public `Union` also checks a bounded LoftFaces result after
its authenticated toe Cut and heel Intersect against a finite, unplaced,
full world-X Revolve. The Revolve section must have one axial span and
whole straight source edges. Its candidate conical roots keep their source
faces and exact rational meridians. Every non-axis face must retain the
surface tag and orientation reconstructed from its straight source run.
A placed or off-axis Revolve refuses this gate. The LoftFaces record must
still name the live whole-entity Sketch profile, the same exact maps and
source bounds, and two surviving tool caps. A stale, partial or foreign
profile refuses.
The copied station groups and every cap-bound term must still equal the
original LoftFaces cap record. Every crossing source group that has retained
triangles must map to one live result face carrying its origin, and each
crossing must retain at least one such face.

For each candidate root, the gate evaluates the exact signed cone implicit
on every mapped source station cell. Its recorded curve departure enlarges
each cell box before a strict side is accepted. A fitted curve crossing uses
the exact denominator-cleared Bernstein polynomial of the source spline
under both maps of every band. Strict endpoint signs and derivative signs
prove one transverse root in the same source span throughout the bands.
The root's axial derivative has a strict positive Bernstein bound. Exact
rational brackets locate the root on each cap; a cap's axial map must be
monotone or constant over that span. The complete profile walk must have
exactly two fitted crossings and matching strict sides between them. The
two finite tool cones must bracket the root span, while the original end
caps avoid their root circles. Every surviving source face origin must name
the authenticated LoftFaces producer. Any undecidable cell or extra crossing
refuses the gate.

For a live whole Sketch arc, its certified parameter-slice box encloses the
ideal circular carrier. Its recorded center and endpoints must match the
live entity, and the source box must contain the carrier. Exact cap-map interval
arithmetic proves a strict side or finite axial miss on each source and band
time cell; an undecidable box refuses. When both cap maps have constant
denominators and their source-plane vectors remain centered, orthogonal and
equally scaled throughout the band, a separate exact identity proves that
every point of the arc has the same axial coordinate and cone implicit at a
given band time. Three exact quadratic Bernstein controls describe that time
implicit. Its roots describe whole-arc contacts; they do not establish one
angular crossing. A fitted flank may share such a time root only when the
live Sketch endpoint is the same point handle and all three exact controls
match. This arc result alone does not admit `Union`.

For each nonvertical finite Revolve side, a source manifest classifies every
mapped fitted, straight and arc source span. It emits cells only for source
origins found on a live trimmed face, and binds each emitted cell to its
Sketch entity, LoftFaces band, source face origin and Revolve face.
An interior fitted Bezier knot
and a fitted-to-radial-arc endpoint have distinct live point handles and
contact kinds. When both have time roots in one fitted span and band, their
rational brackets must be disjoint. The manifest sorts them and proves each
open time gap with the complete source polynomial. A nonnegative Bernstein
box admits a strict gap with an exact zero on an excluded time endpoint only
when each source column has a positive control and closed ends are strict.
When a moving source crossing reaches an excluded exact time endpoint, the
ordinary closed-slab proof can refuse that open gap. Such a pair stays
unsupported until an open-end-aware moving-crossing proof is available.
Closed root brackets
use the corresponding one-sided source cell. A source-current check rejects
stale LoftFaces, Sketch and Revolve records before collecting these cells.
Every live Arc band must have a strict finite side, an axial miss, or an
exact whole-arc radial time contact. An undecidable Arc band refuses.
The manifest does not assert which source cells survive both cone trims.

For a vertical Revolve side, the source gate first binds the complete section
to the live Revolve and obtains exact toe and heel radial limits. A strict
gap between the retained radial range and the whole blank face proves a
face miss. Otherwise, one exact constant-X time within the LoftFaces maps
cuts every live line, fitted span and arc against those limits. Each retained
source interval keeps its open clip endpoints, mapped box, source origin and
unique live face. The all-side record gives every live Revolve boundary face
either a source candidate or a strict miss; it refuses an undecidable side.
The live blank face set must equal the section walk's face set exactly.
An axis span has no boundary face. These records do not prove a solid seam.

The two retained trim-cap faces also identify their original cone sources.
For each finite Revolve side, exact affine meridian residuals either separate
the entire side from a trim cone or locate their one possible contact circle
by exact axial coordinate and radius. An endpoint or coincident-side contact
is undecidable. A possible circle does not prove that the trimmed LoftFaces
source occupies any of its angles; that requires a separate source-region
check before the Boolean can admit the face pair.
For a possible circle, the complete authenticated source rectangle bounds
each mapped cap. The hull of neighboring caps bounds every band point.
If the circle's exact axial coordinate misses each band's hull, or its
radius lies strictly outside each overlapping band's radial range, the
original LoftFaces source misses the entire circle. A surviving circle
still needs an angular occupancy proof from the live Sketch region.
The original point loft stores its computed source rectangle by value when
published. A changed source rectangle refuses before a whole-circle miss is used.

An exact simple-loop gate uses the current Sketch profile and its newly
recorded whole boundary. Every current entity must occur once on that walk,
so another contour cannot change occupancy. Lines and fitted Bezier spans
need a strictly monotone coordinate. Whole arcs need equal exact endpoint
radii and a certified sweep shorter than a half-turn. Exact control hulls
and arc boxes must separate nonadjacent cells. Adjacent cells must meet at
one exact endpoint and lie on opposite strict sides of one coordinate.
One arc that alone passes an axis extremum determines the loop direction.
The real Sketch profile produces eight cells for the focused neutral case.
This gate does not decide which tool-cap angles the source occupies.

For a heel-tool circle on a vertical blank side, a positive-determinant
section map carries that simple source loop into the circle plane. The
constant-X source classifier must give exactly two transverse crossings
at the upper retained radius; every other source cell has a strict side.
The crossing boxes must lie at positive Y on opposite strict Z sides.
Their radial sign changes in Sketch walk order select the occupied arc.
The source-parameter brackets enclose its endpoints without treating them
as rational points. A stale circle, cap origin, or source record refuses.

This source certificate does not change the held mesh or grant a contact
exception. A certified pair currently returns `ErrUnsupported` at public
`Union` because the source-aligned held seam and its boundary, area, volume
and complete-contact audits have not been built. The same producer remains
unchanged after that refusal.
