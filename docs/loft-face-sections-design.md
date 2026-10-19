# Source-cap face-fed loft

`LoftFaces(ctx, doc, sections ...*Face)` builds one solid through two to
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
The gate's norm is the exact maximum over the tiles; a float upper bound only
skips the exact evaluation of a tile that cannot reach that maximum.

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
`revolveloftfaces.BuildLoftFaces` assembles this mesh and runs the embedding,
audit and bound proofs below; root checks the selected faces against their
Split slabs and commits the result.

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
Its proven `MeshBound` raises the pair's internal chord tolerance when
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

A cone-tangent end (see "Cone-tangent end section") has `A = 0` and
`C > 0`: the denominator-cleared pullback is exactly `C*v²`, so the cap plane
touches the heel cone along one ruling and nowhere else. The Intersect takes
this branch only when the cap record carries a tangent end built for the heel
cone's exact apex and slope, that end reproduces the recorded final map, and
the rectangle lies strictly on one side of `u = 0`. The Intersect tries the
strict `A, C > 0` branch first and falls through to the tangent branch only
when the strict branch does not apply. The tangent branch exempts no face
pair from the proximity gate. Two exact side gates guard it, and a failure of
either refuses the Intersect with `ErrUnsupported`. Let `Pi` be the cap plane,
`phi` its exact functional, and `s` the strict common sign of `phi` over the
corners of the preceding cap map (the loft side).

- G1, cone on the loft side. The cone's axis midpoint `m = ((A+far)/2, 0, 0)`
  is in the open interior of the finite cone. Because the pullback is
  `C*v² >= 0`, the plane meets no interior point, so the whole interior lies on
  `m`'s side. The gate requires `s*phi(m) > 0`. A loft that approaches the plane
  from the far side of the cone is refused.
- G2, every earlier cap on the loft side. Each earlier cap map has a positive
  denominator over the source rectangle, and `s*phi` is positive at its four
  corners. The numerator of `phi` is affine in `(u, v)`, so every earlier cap
  lies strictly in `{s*phi > 0}`. On a band, `phi` is affine in the band
  parameter, so the loft lies in `{s*phi >= 0}` and meets `Pi` only in the end
  cap.

With G1 and G2 the Intersect runs on the tooth extended by an exact prism `E`
glued to the end cap on the side away from the loft. The prism is the end cap
translated by `-h*s*N/U`, where `N` is the cap normal, `h = 1` mm and
`U = RatSqrtUp(N.N)` is a rational upper bound of `|N|`. The displacement has
length `h*|N|/U <= h`, strictly positive, and points along `-s*N`. It lies in
`{s*phi <= 0}`, so it meets the tooth only in the end cap. The cone lies in
`{s*phi >= 0}` by G1. On the open half-space `{s*phi > 0}` the extended tooth
and the tooth coincide, so the regularized intersection of the extended tooth
with the cone is the intersection of the tooth with the cone. The argument uses
no ruling, ruling interval or profile corner. It holds wherever the prism
reaches, including the toe tool's removed material and the cone's far plane,
because the exact cap stays in the tooth and the cone stays on the loft side.
Correctness needs only the direction and the positive length of the
displacement, not its exact value.

The held proxy replaces only the end-cap facets of the held tooth. Each ring
vertex must equal, as floats, a station vertex of the last LoftFaces ring, and
each ring edge must be a distinct station chord, or the extension refuses. Each
ring vertex gets the extension vertex nearest to its exact translate, with a
vertex bound of the ring vertex bound plus the 3D radius of the exact rounding
gap. The ring edges become extension walls carrying the face id of the wall
they continue, and the cap facets move to the extension vertices with their
face id and orientation. An extension wall over chord `j` stands for the
translate of the true cap-boundary piece of chord `j`. The certified distance
between that piece and the held chord is
`chord_j = AbsSumUpper(SourceDeparture[j], ProjectiveDeparture[j], StationRound)`
of the final cap's bounds, and it includes the curved profile's departure. The
bounds of the wall's two extension vertices are raised to at least
`AbsSumUpper(max(beta(a), beta(b), chord_j), max(gap(a_e), gap(b_e)))`. A face
bound is recomputed as the maximum facet bound over the face's final facets.
The mesh bound is the maximum of the face bounds. The ring must be one closed
cycle with exactly one wall neighbour per edge. The proxy must pass the exact
closed, oriented, embedded mesh audit and have positive exact volume, and the
prism alone (old cap reversed, extension walls, extension cap) must pass the
same audit.

Gate E5 proves the proxy changes nothing the Boolean decides. Let `psi =
s*phi`, and let `S` be the vertices of the old held cap and of the held prism.
Let `eps_S` be the largest `psi` over `S` and `B_S` its exact axis box. Let
`eps_K` be the largest `-psi` over the held cone vertices. After both meshes
exist, E5 requires:

1. every extension-cap vertex has `psi < -eps_K`;
2. every held cone facet whose box meets `B_S`, clipped exactly to `B_S`, has
   `psi > eps_S` at every clipped vertex. Equality refuses.

So no held cone facet touches the old cap or the prism, and the prism lies
outside the held cone. The held proxy is then the held tooth plus a held prism
that overlaps it only along the removed cap, so the Boolean cuts the same
non-extension facets at the same contacts and keeps the same pieces. No
extension facet is cut, carries a rim or survives. After the Boolean, any
result vertex with `psi < -max(eps_K, eps_S)` refuses. Both checks run before
the result is published. E5 proves the statement for the exact stitched
geometry. The post-Boolean check reads the result vertices after the final
float stitch (`ResultBeyondSlab`), which can move a rim vertex off its exact
cone facet and out of its half-space. It can therefore refuse a result
conservatively because of rounding. Such a refusal is not a failure of E5, and
the check never admits anything.

The proxy keeps the unextended tooth's occupied-volume bound and area slack,
and it adds no volume or area charge. It does add the largest extension-vertex
rounding gap (`CoordCharge`, a length) to the operand's coordinate bound.
The composition reads the operand's volume bound only through
`(T' ∩ K) Δ (heldT' ∩ heldK) ⊂ (T Δ heldT) ∪ (K Δ heldK)`, and the left side
equals `(T ∩ K) Δ (heldT ∩ heldK)` by the exact equality above and E5. Every
result facet is a piece of a non-extension facet, so the unextended area
allowance is the one the result composes. These two figures are valid only for
this Boolean under E5. The proxy is never published.

Extension walls carry the face id of the band wall they continue. Their own
ids would make every `(extension wall, cone)` a separate face pair whose held
facets come within the gate's slack of the cone at the ruling's endpoints, so
the ordinary gate would refuse every real tangent tooth. With the shared id the
gate examines the band wall's own crossing of the cone. E5 makes the id
irrelevant to the result, because no extension facet has a contact.

The held Intersect result must contain no facet from the end-cap group, which
now includes the extension cap. The result must keep one face for the heel
cone, and it keeps the trim record with the tangent end copied from the cap
record. A result without a single toe-cap face and a single heel-cone face
refuses before the body is built.

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
plane that misses its entire tool/root circle, or, for a tangent heel record,
touches it at exactly one point. These whole-cap checks rule out
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

Two checks of the first join have an exact cone-tangent branch beside the
strict one. Each runs only when the trimmed tooth carries a cone-tangent end
record that equals the cap record's by value, matches the heel tool cone's
exact apex and slope, and reproduces the final cap map (`spiralTangentRecordCurrent`).
A tooth without that record takes the strict branch alone.

- L2, the heel cap against the heel tool/root circle. The strict branch needs
  the final cap plane to miss the circle. The tangent branch needs the plane to
  meet the circle at exactly one point: with `phi = a + y*cos(theta) +
  z*sin(theta)` on the circle, `a² = y² + z²` and `a != 0`, and `a` has the sign
  that `phi` has at all four source corners of the preceding cap. Band
  interpolation then keeps the source on that side up to the cap, so the circle
  does not cross the source boundary at the touch point. Its membership in the
  source still changes only at the two transverse flank junctions. The toe cap
  half is shared by both branches.
- L3, the heel seam. The strict branch proves the final cap strictly outside
  the blank heel cone from the unit-slope two-term form. The tangent branch
  nests the cap in the closed outside of the tool cone, whose gap to the blank
  cone is positive over the tool cone's whole radius range, so the cap lies
  strictly outside the blank cone over the source rectangle.

A patterned Union accepts a cone-tangent first tooth only when every copy has
a direct `WithCopyRotation` record about the blank's X axis through the origin.
The exact rotation preserves the blank's cone, the cap's tangent relation and
the first tooth's strict toe and heel source gaps. The copied-tool proof checks
the source lineage and separation of complete tooth shells; the Union checks
the copied contact chains and its final boundary and occupied volume. A copy
without that exact axis rotation refuses. The generic Revolve-and-LoftFaces
route also declines a tooth with a tangent record.

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

## Cone-tangent end section

`LoftFacesToConeTangent(ctx, end, sections...)` lofts through the selected
caps and then through a last section that no selected cap supplies. The last
section is the plane that touches `end.Cone` along one ruling. It is built
from the cone's exact apex `A` and slope `k` (read by `exactPointCone`) and the
float direction `end.Radial = (0, y, z)`.

`loftface.NewTangentEnd` chooses a unit vector `e = (e_y, e_z)` by the
half-angle parameterisation: `e_y = (1-T^2)/(1+T^2)`, `e_z = 2T/(1+T^2)`, with
`e_y` negated when the radial `y` is negative. The radial is scaled by its
larger component before `T` is formed, so the norm and the half-angle
denominator neither overflow nor collapse. A much smaller component can still
underflow to zero after scaling, which only rounds the chosen direction.
Float rounding only chooses the rational `T`; `e_y^2 + e_z^2 = 1` then holds
exactly. `Rho` is the rational value of a float expression evaluated from the
float of `k`; it is not the rational nearest `1/sqrt(1+k^2)`. The end map
is

    P(u, v) = (A, 0, 0) + u*Rho*(k, -e_y, -e_z) + v*(0, -e_z, e_y)

with denominator 1, so it is an ordinary `loftface.Map` and the existing
positive-denominator, plane-order and band gates consume it unchanged. Two
identities hold for every `(u, v)` by `|e| = 1`:

- `x + k*(e_y*y + e_z*z) = A`, so every point lies on the tangent plane of the
  cone's ruling along `e`.
- `k^2*(y^2 + z^2) - (A - x)^2 = k^2*v^2`, so the cone pullback of the end
  plane is exactly `(0, 0, 0, 0, 0, k^2)`. The plane touches the cone along
  `v = 0` and lies in its closed outside everywhere else.

The map's `Stretch` is the Frobenius norm of its linear part, rounded up. The
`u` axis has length `1 + O(2^-53)`; that is the declared construction, as for
every rounded cap map, and is not an isometry claim. The held end ring is the
float64 rounding of the map at each source station, and the station-rounding
term of the cap bound charges that rounding.

The end face carries the cap-end role of the new loft only, with a `Plane` tag
rounded from the map, and no origin of a selected cap. `loftFacesCapRecord`
holds the `TangentEnd` beside `Final`, and the cone-trim record copies it, so
a later check can require that its cone and map match the construction
exactly. Placement drops both records. A source whose plane is not the tangent
plane still passes through the ordinary embedding gates, which refuse an
inconsistent end.

## Cones from a revolved face

`ConeFromFace(ctx, body, face, far)` builds a tool cone for a Revolve blank.
The cone is the nearest representable strict offset from the selected face's
exact line. This need not be one float step: the rounded value may already be
strictly sided, or the bound may be MaxFloat64. The construction guarantees
separation only from that line. Other blank and tool relations are left to
later checks. `body` must
pass `revolveshell.RecordedSection`: a live, unplaced, full Revolve about world X of
whole straight sides. That section has 4 to 128 recorded segments, strictly
positive X coordinates, nonnegative radii, no holes and one side of the axis;
it may be non-convex. `face` resolves to one non-axis side of that section
with both end radii positive, unequal end stations and unequal end radii.

The side's recorded endpoints `(x0, r0)`, `(x1, r1)` are floats, so the exact
rational line through them is `x + k*r = C` with `k = -(x1-x0)/(r1-r0)`, which
must be positive, and `C = x0 + k*r0`. The face's empty side is the outward
side of the section walk. For a closed walk of exact shoelace orientation `o`,
the side has sign `o * sign(r1 - r0)` of `f(x, r) = x + k*r - C`.

The cone apex `A` is the float64 nearest `C` that lies strictly on that side.
Its far radius `R` is the float64 nearest `(C - far)/k` strictly on that side.
Then `f(A, 0)` and `f(far, R) = k*(R - (C-far)/k)` both have the side's sign,
so the whole meridian segment from `(A, 0)` to `(far, R)` lies strictly off
the face's line. Since `k > 0`, the radius decreases as x rises. `strictFloat`
rounds toward the finite side when the exact value lies beyond the float64
range, by exact comparison against the largest finite float. `far` must be a
finite negative axial coordinate (below X = 0) and below both face ends, `A`
must be above `far`, `R` must be positive, and `A > 0 > far` so that
`exactPointCone` reads the result.

Revolve classifies a generator as a cylinder when `|dr| <= 1e-9*hypot(dx, dr)`,
and the directed rounding can cross that boundary. The built body is checked
with `exactPointCone`, which needs a face classified Cone, before it is
registered in the document. A failed check is `ErrUnsupported` and leaves the
document and its provenance numbering unchanged.

The result is the full Revolve of the triangle `(far, 0)`, `(far, R)`,
`(A, 0)` in a private world-XY sketch. It is an ordinary Revolve body that
depends on `body`. Every rounding step is decided by exact rational
comparison, so for identical recorded endpoint and far bits the strict gap to
the face is the same on every build. That establishes only the separation from
the selected face line; other blank and tool relations keep their own checks.
The spiral first-join gates (`spiralToolOuterGap`, `spiralFiniteConeSide`) read
the same strict signs on amd64 and arm64. `internal/pointcone/offset.go` holds
the arithmetic as plain data.

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
An interior fitted Bezier knot, a fitted-to-radial-arc endpoint and a
fitted-to-line endpoint have distinct contact kinds. The fitted-to-line gate
requires adjacent whole entities to share one live Sketch point. Both recorded
endpoints and their exact mapped meridian polynomial controls must agree.
It isolates that point's time roots, classifies their exact axial intervals
against the finite Revolve side, and proves one-sided source cells on both the
fitted and straight spans. If both endpoints of one span have time roots in
one band, their rational brackets must be disjoint. The manifest sorts them
and proves each open time gap with the complete source polynomial.
A nonnegative Bernstein box admits a strict gap with an exact zero on an excluded time endpoint only
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
and mapped boxes, not only the two selected circle crossings. Inside one call,
an arc that the face-pair census derived is compared with a private deep copy
of that derivation, which the census made from the same live producers
earlier in the call. The copy never leaves the census, and nothing writes to
it.

The source pair census rechecks every live Revolve face and retained LoftFaces
face against the current source-side and tool-cap records. A strict gap is a
miss; a mapped source crossing or retained vertical cell is a source candidate;
an occupied heel circle carries its ordered source-bound arc. Missing face
roles, duplicate pairs and undecidable side values refuse the whole census.
Each top-level mesh consumer derives the census from the live source before
using its cells, once per source certificate in its call.
These source verdicts do not prove a shared seam, retained patch or volume.
One census computes each distinct set of meridian rows once; equal inputs, compared exactly,
reuse the rows within that census only.

Each occupied heel cap arc endpoint also binds to one live source-side cell.
For a sloped Revolve side, the cap radial and side meridian polynomials
coincide at the authenticated LoftFaces band time; the selected transverse
cell has one root. For a vertical side, the cap and source cell share the
same exact root bracket and mapped XYZ box. The check compares the arc
with the census's own derivation when the census produced it, and rederives
any other arc from current producers. It reads the face-pair verdicts from
the census its caller derived from the same source certificate earlier in the
same call: the contact graph and the cap-ring constructor pass their own
census, and a caller without one derives a fresh census. It requires one live face and
source reference for each endpoint, and refuses ambiguous ownership or an
excluded time boundary. It does not place a mesh vertex or admit `Union`.

A whole radial Sketch arc contact can place one held seam row inside its
certified rational time bracket. Each point is rounded once and reused by
the open Revolve and LoftFaces source patches. Separate vertex bounds charge
the Revolve meridian residual and LoftFaces time, station and rounding error.
The patches retain their source origins. Their vertex bounds do not include
curve and twist error over whole faces, so they are not a closed shell or a
Boolean result.
The checked patch constructor uses the live source certificate and
face-pair census of its top-level call. It matches one whole-arc contact and
its rational root bracket, then requires every issued station of that arc
through its shared endpoint. The point-loft record compares station coordinates and their
rounding bound against the values issued at construction. Changed or missing
stations and partial arc runs refuse the patch.

The blank-shell constructor selects a whole radial-arc source cell from the
current face-pair census. It compares every supplied side cell with the live
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

The cap-ring constructor derives the live face-pair census and splits that shell
at each occupied heel-cap circle. It checks each arc against the live source,
through the census's own derivation for arcs that census produced,
requires its exact circle to lie strictly inside one straight section side,
and orders multiple cuts by their exact meridian fractions. Both bands next
to a cut reuse the same ring vertex indices. The complete shell then repeats
the crossing, Euler, volume, and face-bound audits. A changed arc refuses.
The neutral producer yields two cap rings, 690 vertices, and 1376 triangles.
The shell still does not admit `Union`.

For each occupied heel arc, the endpoint builder checks its live source
record the same way. It projects each source-box center onto the exact circle
and bounds the full movement to every point in that box, including float rounding.
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

This source certificate does not change the held mesh or grant a contact
exception. A certified pair currently returns `ErrUnsupported` at public
`Union` because the source-aligned held seam and its boundary, area, volume
and complete-contact audits have not been built. The same producer remains
unchanged after that refusal.

An unfinished source proof or face-pair census does not select this path.
Unless the caller's context was canceled, `Union` may continue through the
ordinary mesh Boolean and its independent contact and volume gates. A
completed source certificate and census still require the held-seam builder.
