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
faces and exact rational meridians. A placed or off-axis Revolve refuses
this gate. The LoftFaces record must still name the live whole-entity Sketch
profile, the same exact maps and source bounds, and two surviving tool
caps. A stale, partial or foreign profile refuses.
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
An interior fitted Bezier knot, a fitted-to-radial-arc endpoint, and a
line-to-fitted endpoint have distinct contact kinds. Each adjacency requires
the same live Sketch point handle on both whole entities, exact recorded
endpoint coordinates, and identical mapped endpoint Bernstein controls.
Its finite time roots have one-sided source cells on both incident curves.
For a line-to-radial-arc endpoint, the live Sketch point, exact source record,
mapped axial coordinate and normalized implicit Bernstein controls must
agree with the whole-arc certificate. Both incident line ends may reach the
arc at the same time root. A line that also reaches a fitted source at its
other end orders both endpoint roots before certifying the intervening open
time gap. The radial Arc keeps its separate whole-arc contact cell.
The root bracket owns both contact cells; adjacent time gaps exclude its
endpoints. Disjoint line-to-fit and interior-knot roots on one fitted span
are ordered by exact rational brackets after their live joint proofs are
recomputed. Overlapping brackets or a line-to-fit and radial-arc combination
refuse.
A strict axial hull gap across a whole source span and band certifies a finite
side miss before its infinite-meridian endpoint roots need combining.
When a fitted knot and radial arc both have time roots in one span and band, their
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

A sloped blank side uses the same angular source proof after an exact
section slice proves its retained radial interval. The circle X must lie
strictly inside the axis span and the selected sloped span, away from every
section vertex. Every other non-axis span must miss that X. The axis walk
heads toward positive X and the selected outer side toward negative X, so
the two slice crossings enclose the filled interval up to the circle.
The heel cone sets the upper radius; the toe cone sets the lower trim radius.
An extra crossing or vertex touch refuses.

Each occupied heel arc records its exact LoftFaces band and rational time.
Before a later consumer uses its endpoints, it rederives the contact circle
and the complete constant-X source cells from the live Sketch, LoftFaces,
trim cones and Revolve. A changed band, time, root bracket, source face or
cell sign refuses the arc. The comparison includes retained source intervals
and mapped boxes, not only the two selected circle crossings.

The source pair census rechecks every live Revolve face and retained LoftFaces
face against the current source-side and tool-cap records. A strict gap is a
miss; a mapped source crossing or retained vertical cell is a source candidate;
an occupied heel circle carries its ordered source-bound arc. Missing face
roles, duplicate pairs and undecidable side values refuse the whole census.
Later mesh consumers must rederive the census before using its mutable cells.
These source verdicts do not prove a shared seam, retained patch or volume.

Each occupied heel cap arc endpoint also binds to one live source-side cell.
For a sloped Revolve side, the cap radial and side meridian polynomials
coincide at the authenticated LoftFaces band time; the selected transverse
cell has one root. For a vertical side, the cap and source cell share the
same exact root bracket and mapped XYZ box. The check rederives the arc and
all face-pair verdicts from current producers, requires one live face and
source reference for each endpoint, and refuses ambiguous ownership or an
excluded time boundary. It does not place a mesh vertex or admit `Union`.

A whole radial Sketch arc contact can place one held seam row inside its
certified rational time bracket. Each point is rounded once and reused by
the open Revolve and LoftFaces source patches. Separate vertex bounds charge
the Revolve meridian residual and LoftFaces time, station and rounding error.
The patches retain their source origins. Their vertex bounds do not include
curve and twist error over whole faces, so they are not a closed shell or a
Boolean result.
The checked patch constructor rederives the live source certificate and
face-pair census. It matches one whole-arc contact and its rational root
bracket, then requires every issued station of that arc through its shared
endpoint. The point-loft record compares station coordinates and their
rounding bound against the values issued at construction. Changed or missing
stations and partial arc runs refuse the patch.

The blank-shell constructor selects a whole radial-arc source cell from the
current face-pair census. It compares every supplied side cell with a fresh
source census and requires every live blank and retained LoftFaces face to
have a source role. It inserts the complete arc station run into one angular
grid shared by all Revolve section sides. The contacted straight meridian
side splits at the certified root-time level; both halves reuse each seam
vertex index. All other walls and the planar end caps reuse the same rays.
The rational quarter-circle grid has angular step at most `2/n`, so its
chord sagitta is at most `maxRadius/(2*n*n)`. Straight ideal meridians have
zero twist. The face bound adds this sagitta and the largest exact radial
and rounding displacement of any vertex. The constructor requires a closed,
oriented, connected genus-zero mesh with positive exact volume and no exact
facet crossings. The neutral producer yields 494 vertices and 984 triangles.
The shell is an internal construction piece; it does not admit `Union`.

The cap-ring constructor rederives the face-pair census and splits that shell
at each occupied heel-cap circle. It rechecks each arc against the live source,
requires its exact circle to lie strictly inside one straight section side,
and orders multiple cuts by their exact meridian fractions. Both bands next
to a cut reuse the same ring vertex indices. The complete shell then repeats
the crossing, Euler, volume, and face-bound audits. A changed arc refuses.
The neutral producer yields two cap rings, 658 vertices, and 1312 triangles.
The shell still does not admit `Union`.

For each occupied heel arc, the endpoint builder rederives its live source
record. It projects each source-box center onto the exact circle and bounds
the full movement to every point in that box, including float rounding.
The focused neutral test requires all four endpoint bounds below 0.1 mm.
The builder inserts those four angular rays into every blank ring and uses
the same bounded held point on each owning heel circle. Each cap source box
must stay strictly between its neighboring rays, including another cap ray
or a seam ray with its source movement bound. A collision with a grid, seam,
or cap ray refuses. The two occupied ring walks follow the source arc's
positive-Y selection and retain live side-cell ties. Adding rays cannot
increase the blank circle chord gap; endpoint movement enters the shell's
face bound. The cap-ring constructor requires that bound below 0.1 mm and
repeats the exact crossing, Euler, and volume audits. LoftFaces wall and cap
facets need separate source-curve, chord,
and rounding bounds before they can be welded to these vertices.

The held heel-cap X-section builder cuts the actual cap facets at each live
occupied circle. It requires one simple path, with each end owned by the
matching live wall face. Its interior rays must advance through positive Y
and lie strictly between the certified source endpoint boxes. Each ray is
placed on the exact circle, while the original facet bound, cut-to-circle
movement and source-point rounding remain separate. This section is a
topology template; it does not select retained cap facets or admit Union.

Between two occupied heel circles, a source-only cap-region certificate
rederives the pair census, live Sketch loop and exact cone meridian. It uses
only float-representable points on the authenticated cone as a finite witness;
an unrepresentable upper circle refuses this bounded class. Every mapped cap
in the interval has source-independent, increasing X, and the exact quadratic
Bernstein controls of its Y/Z affine determinant stay positive. Each exact
time point and open interval between source-cell boundaries has a complete
source-parameter partition and exactly two transverse circle crossings with
opposite signs in the counter-clockwise loop walk. Adjacent intervals share
the same crossing span or one authenticated source vertex. A fitted-to-line
vertex contributes two source cells but owns one crossing; exact open time
ends assign the transfer to that vertex. Its two cells have the same exact
endpoint residual polynomial, strict equal derivative direction, and
opposite far-end signs throughout the root bracket. These conditions prove
one crossing exists across the two cells at every time in that bracket.
The first and last crossing pairs
must match the two live occupied-arc endpoints and their outward signs.
The simple-loop proof and these continuous, nonmerging crossings keep the
occupied angular arc connected between the two cap circles.

The same certificate partitions the live Revolve section at every vertex X
between those circles. On each open slice, the axis and exactly one outer
side are the only section crossings, with opposite walk directions. The
outer radius exceeds the heel-cone radius at every slice end except the
lower authenticated contact circle, where they are exactly equal. The upper
contact belongs to the live vertical blank face. Exact toe and heel cone
limits leave a strict retained radial interval throughout. Thus the whole
occupied heel cap region between the circles lies inside the blank; the
certificate still does not choose held facets or admit public Union. It
refuses interior radial steps, extra section crossings, ambiguous source
events, or a cone witness outside its exact finite bounds.

For each linked crossing, a separate source helper rederives the live event
graph and restricts its exact Bernstein residual to a caller's closed rational
time slab. Strict opposite signs and one derivative direction enclose its
unique moving root. At a fitted-to-line or fitted-knot vertex, both adjacent
source cells share the exact endpoint polynomial, so one root can transfer
between them. Rational source brackets map to an XYZ box with live face
origins. The box's L1 corner radius must fit the caller's positive millimeter
budget. A broad time slab or unresolved sign refuses; the caller may split
time and retry. These boxes do not select held facets or admit Union.

A reject-only heel-track consumer asks for bounded boxes on both crossing
curves at each exact source event. It bisects broad open time intervals and
requires every adjacent box to overlap at the shared source point. Each
rounded station has the same increasing X on both curves, lies on the
correct positive-Y side, and preserves their strict angular order. Stations
that round to the same X merge only when the combined exact box still fits
the caller's source budget. The per-station bound adds the box's L1 radius
and float rounding. Later cap triangles must add their own chord, held-facet
and volume bounds before this track can support a Union result.

After the upper heel circle, a separate source certificate chooses an exact
float witness on the live cone within one LoftFaces band. Two adjacent
connector lines reach opposite live endpoints of one radial Arc at the same
isolated time root. The Arc's whole-source time polynomial and both line
endpoint polynomials agree exactly. The open interval from the upper circle
to the root bracket has two linked transverse crossings. Inside the bracket,
those two line endpoint cells and the whole Arc are the only contacts; every
other source cell is strictly outside the cone circle. After the bracket and
through the witness, every source cell is strictly outside. The terminal
record stores the certified open pre-root slice so a cap builder can derive
both moving connector root boxes and their live wall-face owners. The complete
source box maps to positive world Y, so a negative-Y point on each cone circle
is outside the exact simple profile. With no boundary crossing after the
root, the entire circle is outside the profile. The current Revolve section
has no X beyond the upper circle, so the retained cap region is also outside
the blank there. The certificate claims only its bounded witness interval;
it does not select held facets or admit Union.

For every remaining LoftFaces band after that witness, a source-wide exact
rectangle bounds world Y below. Each cap map has a positive denominator and
constant X across the source. Both the Y lower bound and the heel cone radius
are affine in band time, so a strict Y-minus-radius margin at each band end
proves every source point lies outside the cone throughout that band. The
certificate stops at the cone apex or the final source cap, whichever comes
first. It records the live heel-cap face but selects no held facets.

Before the lower heel contact circle, a separate source certificate follows the
live heel cone from its finite far end. Every source span is strictly inside
the cone circle until one isolated whole tip-Arc time root. Both adjacent
fitted flanks share that Arc's live endpoint handles and exact root bracket.
The bracket has those two fitted endpoint contacts and the whole Arc contact;
every other source span is inside. Rational endpoint ownership checks cover
the strict cells on either side of the bracket. After the root, two fitted
crossings remain linked through their live knot transfers to the lower
occupied circle. The exact Revolve section lies strictly below the heel cone
before that circle and touches it only on the recorded blank side. The source
box maps to positive world Y throughout the occupied interval. Exact mapped
source boxes keep the two crossings on opposite Z sides in every point and
open time slice, so the positive-Y ray stays in the occupied arc. This source
certificate does not select held facets or admit Union.

This source certificate does not change the held mesh or grant a contact
exception. A certified pair currently returns `ErrUnsupported` at public
`Union` because the source-aligned held seam and its boundary, area, volume
and complete-contact audits have not been built. The same producer remains
unchanged after that refusal.


## Open retained heel-cone patches

The retained-cap constructor rederives the live tip-Arc birth, root-Arc
terminal, and strict right-tail certificates. It retains each raw moving-root
box and its source parameter, face, and origin even when rounded stations
merge. It places both terminal Arc station sets from their certified time
brackets. The lower and upper contact-circle rows reuse vertex IDs from the
closed Revolve shell. Every added triangle carries the live heel-cap origin.
Each Arc bracket also rederives the adjacent fitted or connector joint. The
joint's one-sided source sliver is narrowed only when exact Bernstein signs
prove every excluded source point stays strictly on one side over the entire
root-time bracket. The mapped XYZ box of the remaining sliver must fit the
caller budget. The first and last cap strips charge those boxes as well as
the open-slice moving-root boxes; a shallow source derivative that permits a
wide root motion refuses even when its time bracket is narrow.

The neutral nine-cap producer yields two open cone patches with 399 triangles.
Their source-only crossing audit passes. Exact rational Z/Y intervals bound
each triangle's angular span; the cone chord charge uses the largest cone
radius times one eighth of that span squared. The root-curve charge uses
twice the largest exact moving-root box radius. Point displacement and final
rounding are charged separately, and every triangle must stay below 0.1 mm.
These are construction bounds only. Adjacent LoftFaces walls, full closure,
occupied volume, and the public `VerifyAll` proof remain to be built.
