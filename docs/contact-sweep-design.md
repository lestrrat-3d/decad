# Two-Body Contact Sweep Design

This document specifies the `Document.SweepPair` contract. Current affine
source-box sweeps certify first impact, immediate departure, persistent face
contact, and the first edge transition of a sliding patch. A source semicircle
sphere against a source box certifies affine or centered rotating first impact
and face-point departure or persistent touch while its projected radius remains strictly
inside one box face. A sphere against an exactly orthogonal rotated source
box also certifies a strict single-face affine clear span, first impact, or
separating departure. A full source cylinder certifies a clear path,
first face impact, and separating departure inside a source-box face, including
transverse translation that stays inside the face. A full circular vertical
source prism also certifies the same outcomes at its sidewall when its complete
axial contact line stays inside a broad source-box face.
One zero-bound faceted Boolean solid with a certified rectangular lower face
can certify an affine vertical clear path, first exact face touch, separating
departure, or persistent face contact against a containing source-box floor.
The same path accepts a translation-only `Placed` copy when its saved exact
source mesh proves the complete lower face despite positive held-mesh and
occupied-volume bounds. The event and replay retain the placed body's Face.
Both bodies must translate equally in X and Y, so the complete support patch
stays strictly inside the floor face. The lower support plane must start at or
above the floor's upper plane. A first-impact bracket ends at exact touch;
if its rational contact fraction is not a representable dyadic grid point,
the sweep returns `Undecided`. A bracket ending after support-plane crossing,
a lateral path, an unproved support face, and a footprint reaching the floor
edge also return `Undecided`. The contact track retains the original faceted
Face. Rounded replay checks the exact support-plane path and footprint proof.
A positive-bound faceted Boolean without that exact source proof can certify
only a strict clear span above a source-box floor. Both signed-axis paths must
have zero angular and transverse motion. The held vertex extrema, widened by
the certified boundary
displacement, stay inside the floor's projected face. The lower Z gap is affine;
both endpoints must exceed the displacement, including conversion error.
Both real endpoint contact queries must report separation. Replay charges the
rounded pose deviation and requires the widened actual extents to stay inside
the floor face with a positive gap. A path reaching the boundary allowance,
moving transversely, or rotating returns `SweepUndecided`.
Two source semicircle spheres certify affine first impact
through exact squared-distance motion, including transverse crossing, and
separating departure. Their exact isolated interior tangent publishes a
bounded point with strict separation on both sides. A touching source-sphere
pair with equal exact affine displacement certifies a full-span point track.
Two touching source spheres can also certify separating departure when one or
both follow centered rotating rigid drifts. Their sampled poses retain spin;
the occupied balls follow exact affine center paths.
Rotating source-box rigid drifts can also certify a clear path or bracket an
impact after exact oriented-box pose relations, a bounded float-to-ideal pose
difference, and whole-body travel bounds. An initial source-box face touch also certifies immediate
departure when both bodies have the same angular velocity and the bounded
normal separation rate is positive. Fixed oblique poses of co-oriented source
boxes also certify initial face touch and a persistent face patch when both
affine paths have the same translation. A partly overhanging horizontal
source-box face also certifies initial touch and a persistent clipped polygon
under the same translation. If their exact support-plane gap has
positive affine slope, the sweep certifies immediate departure and checks the
rounded endpoint against its ideal path. A box spinning about world Z can also
reach a contained horizontal face on a stationary box, then depart under
positive vertical velocity. A rotating `PoseSegment` whose read screw axis is
cardinal can certify a clear path or first impact. An axis-normal source face
can also certify departure when its support-plane gap increases throughout
the step. A source box spinning about Y can depart from a stationary horizontal
source-box face when every source corner has a positive bounded outward height
through the certified interval. Other unequal spins, rotating paths, and
payloads remain design contracts.
An extruded circular source prism or a full revolve of an axis-incident
rectangular half-profile can certify a strictly separated axial affine path
against a containing source-box face. The sweep uses the source-derived outer
disk box at both endpoints and over the full translation, and replay charges
the rounded pose deviation before returning a clear sample. Both sources
can also certify a first face impact and separating departure. Near contact,
lateral motion outside the containing face, partial revolutions, and other
revolved profiles return `Undecided` on this path.
`docs/collision-dynamics-design.md` owns the package
boundary and `docs/contact-geometry-design.md` owns relation and manifold
proofs at one pose. This document owns the paths, continuous clear certificate,
first-contact search, and sweep report. `docs/motion-check-design.md` supplies
the existing one-mover proof pattern; its `VerifyMotion` API stays separate.

Navigation only: paths and API → §1–2; pose accounting → §3; continuous bounds
→ §4; search and report → §5–6; work and errors → §7; computed tests → §8.

## 1. Claim and ownership

`SweepPair` reads two live, sound solid bodies of one `Document`, each under its
own prescribed path over one shared positive duration. It changes neither body
nor document. It returns the earliest **certifiable** outcome:

| Outcome | Claim |
|---|---|
| `SweepClear` | The bodies have strictly positive separation at every time in the closed step. |
| `SweepDepartedClear` | The pair starts in certified touch, departs immediately, and has strictly positive separation at every later time in the step. |
| `SweepPersistentTouch` | The pair stays in certified touch throughout the step, with a stable source feature set and a bounded manifold track. |
| `SweepContactTransitionBracket` | The pair starts touching and a bracket encloses the first change in contact features, patch structure, or departure from touch. The preceding contact track is certified. |
| `SweepImpactBracket` | The clear prefix may follow a certified departure from initial touch. The left pose is separated and the right pose certifies touch or overlap. Width is at most `TimeResolution`. |
| `SweepGrazingTouch` | One exact interior instant has certified touch. Both open sides of the closed step are strictly separated; no overlap or other touch occurs. |
| `SweepInitiallyTouching` | Contact is certified at time zero. It says nothing about closing velocity or a later impact. |
| `SweepInitiallyOverlapping` | Interior overlap is certified at time zero. |
| `SweepUndecided` | The available proof cannot establish one of the above. The earliest unresolved interval and a structured reason are returned. |

The pair order is the caller's `(a, b)` order. Normal directions in any contact
report therefore point from A toward B. A certified relation does not imply
that the contact kernel supplied a manifold usable for dynamics. The solver
must inspect manifold availability separately; it never steps through an
undecided interval or treats a later overlap sample as the first impact.

## 2. Paths and entry point

```go
func (d *Document) SweepPair(ctx context.Context, a, b *Body,
    pathA, pathB PairPath, req SweepRequest) (*SweepReport, error)

// PairPath is sealed. The permitted values are PoseSegment and
// RigidDriftSegment. Each has one Duration of Kind Time.
type PairPath interface {
    pairPath()
}

type PoseSegment struct {
    From, To r3.Transform // relative to the body's current placement
    Duration units.Value // positive Time
}

// QuantityVec belongs to decad so geometry cannot import dynamics.
// Every component has the stated kind; all constructors validate it.
type QuantityVec struct { X, Y, Z units.Value }

type RigidDriftSegment struct {
    From            r3.Transform // relative start pose
    Center          r3.Vec       // world pivot at the start, millimetres
    LinearVelocity  QuantityVec  // Velocity, world coordinates
    AngularVelocity QuantityVec  // Angle / Time, world coordinates
    Duration        units.Value  // positive Time
}

type SweepRequest struct {
    ContactRequest
    TimeResolution    units.Value // positive Time, <= Duration
    MaxPoseEvaluations uint64     // >= 2; counts distinct pair times
    StartPolicy       SweepStartPolicy
}

type SweepStartPolicy int // StopAtInitialContact (zero),
                          // ContinueSeparatingTouch, ContinueCertifiedTouch
```

Both paths must name equal physical durations, compared as exact rational base
time values read from their units, without a tolerance. Their time fraction
`s = t / Duration` is shared; their transforms are evaluated independently at
that fraction. A stationary path is legal. `PoseSegment{From: p, To: p}` is
constant, and a drift with both velocity vectors zero is constant. A mixed
handedness screw segment is invalid; two proper endpoints or two reflected
endpoints are legal. A drift composes proper rotations onto its `From`, so a
reflected start stays reflected. Dynamics may impose a narrower no-reflection
gate on its own state.

For a `PoseSegment`, read `rel = From.Inverse().Then(To)` and `rel.Screw()` once.
At `s` the float pose is `From.Then(screw.At(s))`. At exactly `s = 0` and `1`,
return the stated `From` and `To`, as `Between.PoseAt` does. `From == To` skips
decomposition and returns `From` at every sample. A nonidentical pair whose
read screw has zero angle and slide is likewise constant; its endpoint
deviation still covers the stated `To`. The path takes r3's deterministic
shorter arc, including its half-turn axis choice. A caller needing a longer
rotation splits it into segments.

For `RigidDriftSegment`, let `u` be elapsed time. Its world-space step rotates
by `|ω|u` about the line through `Center` along `ω`, then translates by `v u`.
The resulting pose is `From.Then(step)`, where `r3.RotationAround`,
`r3.Translation`, and `Transform.Then` construct the float step. A zero `ω`
uses translation alone. `Center` is a path pivot, not an inferred mass property;
the dynamics layer ensures it matches its bounded center-of-mass state. This
path realizes `c(u) = Center + v u` and `R(u) = Rotation(ω, u) R(0)` for an
isolated rigid drift. Replacing it with endpoint screw interpolation would
move the center along a different path when `v` is perpendicular to `ω`.

Both variants compose onto each body's existing placement in the same order
as `Body.Placed`. No path is constructed by subtracting one body's pose from
the other's at the endpoints. The relative transform can have nonlinear
motion even when both individual paths are simple.

## 3. Exact path and sampled pose

Treat every finite input float as the exact rational it denotes, and interpret
its unit through the exact or outward-enclosed unit factor. For a screw path,
the ideal transform is the exact screw of the **read** `Axis`, `Point`, `Angle`,
and `Slide`, composed after the exact `From`, exactly as motion-check §5.1
defines. For a drift, the ideal transform uses the stated world pivot and the
exact typed velocity components, with the axis normalization, angle, sine,
cosine, and any unit factor enclosed outward. The ideal body at each `s` is
that transform composed onto the exact read current placement.

The evaluator sees a float `r3.Transform` instead. For body `i` and evaluated
fraction `s`, calculate a point-displacement allowance `η_i(s)` between the
float placed body and its ideal body. Use motion-check §5.1's matrix-difference
times record-coordinate radius plus translation-difference form, with rational
intervals for every operation. Both bodies contribute: the ideal pair gap
interval is the float-pair gap interval widened by `η_A + η_B`. If either
radius or deviation is unbounded, the pose cannot prove a gap or contact.
For a rotating source box, compare all eight exact staged source corners under
the read query pose with their ideal-path intervals. The largest outward-rounded
corner distance bounds every point of the box. This uses the same staged
placement and pose operations as `ContactPair`; a float-composed transform can
round away an error that remains in those source corners.
The stated screw `To` gets the motion-check §5.1 endpoint allowance against
both the ideal end and exact `To`; a drift has no separate `To` promise.

A float-pose touching result is not automatically ideal touching. The
contact kernel must supply a certificate that survives both pose deviations
or evaluates the exact source geometry under the stated pose. Otherwise the
ideal relation is `Undecided`. A float-pose overlap transfers when both pose
deviations are zero, or when its bounded volume exceeds both bodies'
boundary-sweep allowances, using motion-check §5.1's area scaling for each.
A separate exact source-set proof may establish overlap at the ideal pose.
An overlap known only by a witness cannot cross a nonzero pose deviation
without such a proof.
Contact geometry's exact-rational source-box path provides the first placed
box touching certificate; ordinary `bodyGeom.delta` clearance §6 alone does
not certify touch under a displaced transient placement.

`ContactPair` speaks about the exact query pose named by its two read float
transforms. `SweepPair` separately transfers its result to the ideal paths and
records that adapted relation in `SweepEvent`. A manifold's point and normal
bounds must include placement and path-pose deviation before the sweep can
publish it. The adapted relation may be certified while those bounds are too
wide for the requested `PointResolution` or `NormalResolution`. Never infer a
contact normal from an overlap volume or from the direction between centers.

## 4. Conservative continuous bounds

### 4.1 Travel of each body

Read each body's `Bounds()` at its current placement, inflate its coordinates
by its outward `Bound`, and map all eight corners through the exact `From`.
Their coordinate hull contains the ideal start body. For a screw with axis line
through `Point` along `Axis`, let `ρ` be the maximum distance of those mapped
corners from that line, rooted upward. Distance from a line is convex, so no
interior point has a larger radius. Let `θ` and `d` be the read screw's angle
and slide. Over a fraction span `h`, every point travels at most

```text
τ_screw(h) = h (ρ θ + |d|).
```

For a drift, let `ρ` be the maximum distance of the same mapped corners from
`Center`; this is convex too. Let `V` and `Ω` be outward bounds on the norms
of the exact world linear and angular velocity vectors in mm/s and rad/s.
Over elapsed span `q`, every point travels at most

```text
τ_drift(q) = q (V + ρ Ω).
```

The same radius works throughout each path: screw motion preserves distance
to its axis; drift rotation preserves distance to its moving center, and the
center's translation is charged separately. Reflections in `From` do not alter
either argument. A stationary path has zero travel. Roots, products,
trigonometric factors, and final travel are rounded outward, never merely
evaluated in float and compared as though exact. If a finite bound cannot be
formed, the result is `SweepUndecided`, not a clear certificate.

### 4.2 Swept boxes

Build one conservative box per body from the exact `From`-mapped inflated
corner hull, expanded on every axis by its full-path travel bound. Compare
the two boxes using exact rational extremes. Strict separation on any axis
proves `SweepClear` without pair-pose evaluations. Box faces merely meeting
does not prove touch, clear, or overlap. Reading the original rest boxes and
expanding only by `To - From` is invalid when either `From` moves a body before
the path begins. A BVH or sweep-and-prune may discard only with this strict
box certificate; it cannot change pair or sample order.

### 4.3 Time interval certificate

At interval ends `l, r`, obtain proven positive lower gap bounds `g_l, g_r`
for the **ideal** placed pair. Let `T_A(l,r)` and `T_B(l,r)` be each body's
travel bound over the interval, and `T = T_A + T_B`, rounded upward. For every
time `u` inside, the true pair distance is at least each endpoint's lower gap
minus the travel from that endpoint. The interval has strictly positive gap
throughout when

```text
g_l + g_r > T.
```

The comparison uses exact rationals with outward bounds. A conservative lower
bound on the interval gap is `(g_l + g_r - T)/2`, rounded downward, and may be
loose. Neither two separated endpoint samples nor touching boxes prove this
inequality. No tolerance turns a failed strict comparison into `Clear`.
If an endpoint is touching, overlapping, or undecided, this positive-gap
certificate cannot use it. A different continuous geometric proof may settle
the interval only if the contact kernel explicitly supplies one.

For a Z-axis rigid drift against a stationary source box, compare the two
exact source-box Z supports after the stated vertical displacement at each
dyadic fraction. Rotation leaves these supports unchanged. A float-pose
horizontal manifold transfers to the ideal path only when every rotating
face corner stays strictly inside the stationary face after both pose
deviations and witness bounds are charged. A negative exact support gap
proves a shallow ideal overlap even if the rounded pose touches. A zero gap
proves ideal touch. Widen every transferred point by both pose deviations.
An exact initial support equality and positive vertical relative speed prove
a whole-body open-time gap; then the usual interval bounds certify the
remaining path. A partial patch or a tilted rotation has no such proof.

### 4.4 Source sphere and box face corridor

For the source ball admitted by contact geometry §4.2, an affine sweep can
use one box face when the sphere center plus and minus its radius stays
strictly inside both projected box intervals at the start and end. The four
signed endpoint inequalities prove that containment for every intervening
time. The normal support gap is then one exact affine function. Its positive
sign throughout proves clear; a positive start gap and a decreasing gap give
the exact first-contact root. Choose a dyadic bracket with a strictly clear
left endpoint and a right endpoint beyond that root, no wider than the caller's
time resolution. The right margin keeps the float query on the overlapping
side when ideal and rounded poses differ by a small amount.

For a shortened impact prefix, the final fraction may replace the usual right
dyadic sample when the root lies before that endpoint and the shortened
bracket fits `TimeResolution`. The real endpoint sample must still report a
matching shallow sphere manifold. Replay caches the original sphere and box,
the face corridor, and the exact affine support gap. At an interior fraction,
check the rounded placements against the ideal path within `PointResolution`,
then require the same face corridor and the outcome's support-gap relation.
Refuse replay when either check fails.

At each sample, classify the ideal rational sphere and box independently of
the float `ContactPair` query. Transfer the float manifold only when relation,
source features, and normal agree; widen its two witness bounds and separation
bound by the exact pose difference. An initial touch with an increasing normal
gap proves immediate departure. A zero normal gap and the same face corridor
prove a full-span point track. A path leaving the corridor returns
`SweepUndecided`; a center sample alone cannot certify the missing span.

A `RigidDriftSegment` may rotate the sphere when its pivot equals the source
sphere center, that center equals the query pose translation, and the other
path is affine. Its angular velocity changes the
sampled pose, but the occupied ball follows the exact affine center path from
its linear velocity. Form the sampled proper rotation through `r3`, then set
the center translation from that affine path so rotation rounding cannot turn
an exact touch into a crossing. Use the same face-corridor, support-gap, and
point-track proof as the translating sphere. Replay recomputes the rotating
pose and checks its center against the cached affine path and point resolution.
Refuse a different pivot, an unrepresentable center displacement, a rotating
box, or a path outside the face corridor.

For an exactly orthogonal rotated box, require the same exterior source face,
strict projected disk margins, and opposite-face clearance at both affine
endpoints. Those inequalities are affine, so both endpoint proofs cover the
whole span. Compare the squared exact center-to-face support with squared
radius times squared face normal; this decides clear, touch, and overlap
without rounding the normal length. Binary search dyadic fractions for the
first sign change, then leave a right-side overlap margin within
`TimeResolution`. The real right sample must retain the same bounded point
manifold. Replay compares rounded source corners and sphere center against
the cached ideal affine path, then rechecks the same face corridor and support
relation. A skew box, edge/corner corridor, non-affine pose, or lost rounded
manifold returns `SweepUndecided` or `ErrUnsupported` as appropriate.

### 4.5 Affine source-sphere pair

For two source balls admitted by contact geometry §4.3, require the relative
center displacement and initial center difference to use the same sole
cardinal axis. The signed center distance minus the radius sum is affine up
to first contact. A positive value throughout proves clear; an initial touch
with an increasing gap proves immediate departure. A decreasing positive
start gap has one exact first-contact root. Bracket it with dyadic fractions,
including a root at the final endpoint, with a clear left sample and a
touching or overlapping right sample. The endpoint can be ideal overlap
while the rounded pose is touch. Transfer the source manifold only when both
queries name the same source faces. For a cardinal center line, keep the exact
normal when both queries retain the same cardinal direction. For an off-axis
center line, bound the change in unit normal from the exact center-pose
difference and shorter center-line length. Charge that change to normal,
witness, angle, and separation bounds before publishing the event manifold.

For a transverse relative displacement, compare the exact quadratic squared
center distance minus squared radius sum over the held affine path. Its minimum
proves a full clear span or locates the earliest possible impact. Search the
decreasing side with dyadic fractions; require a separated left endpoint, a
touching or overlapping right endpoint, and width at most `TimeResolution`.
Find a hidden pass-through before reporting clear. An unrepresentable shallow
overlap returns `SweepUndecided`. An exact isolated tangent can use §4.5.1;
other tangent paths remain undecided. The bounded
center-line source manifold gate applies at the right sample. A zero-gap touch
with equal exact affine displacements and one bounded source point publishes a
full-span track. Its exact center-distance polynomial is identically zero.
Both endpoint samples must retain the same source faces. An interior track
query translates both cached spheres by their common exact displacement and
reduces them to a bounded point manifold. Replay requires exact touch of the
rounded pair. Unequal displacements remain undecided for persistent
continuation.

A centered rotating `RigidDriftSegment` may enter this sphere-pair proof after
initial point touch when `StartPolicy` is `ContinueSeparatingTouch`. Require
each rotating sphere's exact source center to equal both its query translation
and stated pivot. Require each full linear displacement to be representable as
a dyadic rational. Rotation then leaves the occupied ball unchanged; use the
same exact squared center-distance polynomial to prove an open-time gap and
sample both real rotating endpoint poses. An off-center pivot, a separated or
overlapping start, and an unresolved or closing departure return
`SweepUndecided`. This path does not publish a rotating graze or impact.

### 4.5.1 Isolated sphere-pair graze

For a separated affine source-sphere pair, form the exact rational polynomial
`q(s) = |(centerB−centerA) + s(deltaB−deltaA)|² − (radiusA+radiusB)²`
from the held source centers, radii, and translations. Publish
`SweepGrazingTouch` only when its quadratic coefficient is positive, its
discriminant is exactly zero, and its sole minimum `s*` is strictly inside
`(0,1)`. Prove `q(0)>0` and `q(1)>0`. These facts prove `q(s)>0` on both open
sides of `s*` and `q(s*)=0`; separated endpoint samples alone do not prove it.
The same source-sphere pair proof must classify the exact minimum as touching
with one bounded point manifold. The manifold's normal, source faces, and
witness bounds pass §4.5's float-to-ideal transfer at the minimum.

Require `s*` to have an exact finite `units.Scalar` representation and an
exactly representable positive event time `Duration × s*` before publishing
the outcome. The report keeps the private rational fraction even though its
public `SweepInstant` uses `units.Value`. Evaluate the real `ContactPair` at
`0`, `s*`, and `1`; require separated endpoints and a touching float pose at
`s*` with the same source faces. A float overlap, missing manifold, or
insufficient point or normal resolution makes the sweep `Undecided` with its
earliest unresolved interval. `MaxPoseEvaluations` must fund all three poses;
the request's existing minimum of two remains valid, but two poses cannot
certify this outcome.
Use `SweepEventUnrepresentable` when the exact fraction or event time cannot
be published, `SweepPoseBudget` when a required pose cannot be evaluated,
`SweepPoseRelation` when the float relation disagrees, and
`SweepContactUnsupported` when the event manifold cannot be transferred.
The unresolved interval contains `s*` and no later event is published.

This outcome applies to affine `PoseSegment` and zero-angular-velocity
`RigidDriftSegment` paths admitted by §4.5. It does not infer a graze for a
rotating path, source-box pair, sphere-box pair, nonrepresentable minimum,
initial touch, or final-instant touch. A positive quadratic minimum is
`SweepClear`; a negative minimum enters the existing earliest-impact search.
An initial touch follows §5.1, regardless of later motion. If a different
pair kernel cannot prove the full two-sided relation, it returns
`SweepUndecided` rather than this outcome.

### 4.6 Source-cylinder box-face path

For a full circular source prism or axis-incident rectangular full revolve
and one source box at signed-axis poses, construct exact outer boxes from the
source disk, axial limits, and recorded placements. Admit an affine clear
sweep only when the cylinder's projected disk stays strictly inside the box
face at both path endpoints. Check that
one axial gap between the complete swept outer boxes exceeds
`PointResolution`. The two endpoint `ContactPair` queries must also prove
separation, and their rounded pose differences must fit `PointResolution`.
Both admitted source cylinders enter the axial impact path after an
outer-box intersection. The impact path admits transverse relative translation
when the cylinder's outer disk box stays strictly inside the same box face at
both endpoints. Each transverse edge difference is affine, so strict endpoint
containment proves containment throughout the interval. The initial axial
support gap must be nonnegative.
The signed axial support gap is affine. A decreasing positive gap supplies
its exact first-contact root, including equality at the final endpoint. Use a
dyadic bracket with a separated left
sample and a touching or shallow-overlapping right sample whose original
face identities and signed normal match the posed `ContactPair` query.
Transfer that query's bounded witness to the ideal path by charging the
rounded pose difference. An initial touch with increasing support gap proves
immediate departure. A stationary touch, near gap without a clear margin,
or a path leaving the face corridor returns
`SweepUndecided`. Exhausting the pose budget at the bracket's right sample
returns `SweepUndecided` with `SweepPoseBudget` and the unresolved bracket.

Replay translates the cached source outer boxes with the held affine paths.
At each requested fraction, the rounded cylinder projection must remain
inside the rounded box face and the pose difference must fit
`PointResolution`. A clear path keeps a support gap above that difference.
Departure requires exact touch at the start and positive ideal and rounded
gaps afterward. Before an impact bracket, replay requires a positive gap;
inside the bracket, the ideal support gap must fit `PointResolution`.

For one full circular source prism with a vertical cylinder axis, the same
affine proof may select a horizontal axis instead of the cylinder axis. Its
relative axial displacement must be zero. Its complete axial interval
and other transverse diameter must stay strictly inside the box face at
both endpoints. The contact query supplies the source cylindrical Face,
box Face, one bounded midpoint on the complete axial contact line, and the
exact transverse normal. The selected circle support is the exact outer-box
extremum, so its signed gap is affine and the impact, departure, and replay
checks above apply. A revolved sidewall, rotating cylinder, or line reaching
the face edge returns `SweepUndecided` on this path.

## 5. Earliest-event search

Evaluate time zero first unless swept-box exclusion has already proven the
whole path clear. Return `InitiallyOverlapping` on certified initial overlap.
At initial touch, the default `StopAtInitialContact` returns
`SweepInitiallyTouching`; the two continuation policies use §5.1. If the zero
pose is undecided, return `SweepUndecided` with its cause. Evaluate the far
endpoint next and maintain the samples in increasing dyadic fraction order.

Process intervals from left to right. A certified clear interval extends the
clear prefix. For the first interval not certified clear:

1. If it has a separated left endpoint, a certified touching or overlapping
   right endpoint, and exact duration no greater than `TimeResolution`, return
   `SweepImpactBracket`. Its left endpoint is the end of the clear prefix.
2. If its duration exceeds `TimeResolution`, evaluate its dyadic midpoint and
   process the left half before the right half. This order searches for an
   earlier hidden contact even when the old right endpoint overlaps.
3. If its duration is at the floor, and no right endpoint certifies contact
   or overlap, return `SweepUndecided` for that interval.

Continue only when an interval has become certified clear. If every interval
becomes clear, return `SweepClear`, or `SweepDepartedClear` after a certified
departure from initial touch. A tangential graze may have no overlap
sample. The exact sphere-pair proof in §4.5.1 can publish
`SweepGrazingTouch`; otherwise it leaves an undecided floor interval.
Separated samples alone cannot turn it into `Clear`.
If midpoint arithmetic cannot produce a distinct dyadic fraction, return
`SweepUndecided` with `SweepFractionFloor`. The search never skips an earlier
uncertified interval to publish a later collision as the first event.

The procedure also handles two moving bodies that pass through one another
and separate between initial and final samples: their combined travel keeps
the interval undecided until a contact sample is found or the floor reports
uncertainty. A contact's impulse decision remains with dynamics; the sweep
proves only the geometric continuation of the stated paths.

### 5.1 Continuation from initial touch

`ContinueSeparatingTouch` and `ContinueCertifiedTouch` both require a new
one-sided certificate after an initial `Touching` relation. A private
departure proof supplies a duration `h > 0`, a proven positive gap lower bound
at `h`, and a global lower-gap function `L(u) > 0` for every `0 < u <= h`.
The proof must cover the **complete** initial contact set and all other body
features. Positive normal velocity at one manifold point is insufficient.
A general admissible form is `L(u) >= c*u - K*u*u` with proven `c > 0`,
`K >= 0`, and `c - K*h > 0`; a kernel may instead provide a stronger direct
bound. Select the largest representable dyadic fraction `h/Duration` within
the proved horizon, and evaluate its endpoint. If no positive fraction can be
represented, return `SweepUndecided` with `SweepFractionFloor`.

The initial source-box path in contact-geometry §4.1 proves departure for
certified pure affine translations. At time zero, an opposed support axis `n`
has equal A maximum and B minimum. An outward lower bound `w > 0` on
`(v_B - v_A)·n` proves the **whole-body** gap at least `w*u` for every `u > 0`.
Either body may move. The exact source boxes and placed poses are required;
transient AABBs with displacement bounds cannot establish the equality. A
rotation, tangential speed on that axis, an uncertain equality, or `w <= 0`
does not pass this source-box departure path.

For co-oriented source boxes at a fixed oblique pose, use the exact dyadic
cross product of two source edges as the common support-plane normal. A
four-point initial face manifold and exact equality of opposed support
projections identify the complete touching face. A positive exact dot product
of relative affine displacement with the outward normal proves a strictly
positive support gap for every positive path fraction. Check the chosen
rounded departure sample with its float-to-ideal pose bound. Every later
clear interval must keep this exact support gap positive at both endpoints;
affine motion then proves it positive throughout the interval.

For a stationary horizontal source-box face and a moving source box whose
only angular component is about world Y, read all eight exact staged moving
corners and the exact drift pivot. The initial four-point manifold must have
the same exact vertical normal at every point, and opposed full-box Z supports
must be equal. For each corner, bound its outward height derivative by the
exact `sign * (v_z - omega_y * (corner.x - center.x))`. Require the minimum
derivative `c` over all eight corners to be strictly positive. Bound the
absolute height acceleration of each corner by
`omega_y^2 * (abs(corner.x - center.x) + abs(corner.z - center.z))` and use the
maximum as `K`. The stationary support plane and the initially nonnegative
corner heights then give a full-body gap at least `c*u - K*u*u/2`. Admit a
dyadic horizon only when `K*h < c`; reduce the horizon when the full duration
fails. The rounded horizon sample must retain a positive gap after pose
deviation, and later intervals need their own continuous clear certificates.

After departure, mark `(0,h]` as certified clear, put a separated sample at
`h`, and run §5's earliest-first search on `[h, Duration]`. If no later
contact appears, return `SweepDepartedClear`; if later contact appears, return
`SweepImpactBracket` with the initial touch retained separately. The clear
claim is open at zero: neither result calls the closed `[0, Duration]` path
strictly separated. If the departure proof fails under
`ContinueSeparatingTouch`, return `SweepUndecided` with
`SweepDepartureUnproved`; never discard the initial touching pose and start
searching at a positive time by sample choice alone.

`ContinueCertifiedTouch` first accepts the same departure proof. Otherwise
it asks for a complete persistent-contact track. A valid track proves that
the pair stays `Touching`, with opposed material sides and no overlap, at
every instant in its interval. It must provide a bounded manifold at every
instant that the dynamics solver will use. If the track reaches `Duration`
without changing source features or patch structure, return
`SweepPersistentTouch`. A stationary resting stack is this case. If no such
track can be proved, return `SweepUndecided` with
`SweepContactTrackUnproved`. Tangential velocity alone is no proof of
persistent contact.

For two source-certified boxes under pure affine translation, contact-geometry
§4.1 supplies exact axis intervals. Persistent face touch requires equality
on one opposed support axis for the whole tracked interval and strictly
positive projected overlaps on both other axes. Each projected overlap is
the minimum of two affine upper endpoints minus the maximum of two affine
lower endpoints. Enumerate exact rational roots where endpoint order changes
or projected overlap reaches zero. Between consecutive roots, the active
source features and clipped patch vertices follow fixed affine formulas;
check their inequalities at both ends with outward bounds. This certifies
the complete contact set and a manifold track for resting or planar sliding.
An endpoint-order tie at time zero is resolved by its right-sided derivative
when the patch and normal are continuous. It is recorded in the initial
track, rather than returning a zero-time transition that a solver would hit
again on every restart. If the right-sided contact set or normal cannot be
certified, return `SweepContactTrackUnproved`.
If an edge or vertex limit occurs, classify that exact limit as touching,
then test the next open interval for persistent contact or positive gap.
Do not infer separation merely because the face patch shrank to zero.

Return `SweepContactTransitionBracket` at the **first** root that changes a
source feature, clipped patch structure, or touching/separated relation.
Bracket the exact rational root by adjacent representable dyadic times with
width at most `TimeResolution`; a dyadic root may use a zero-width bracket.
The report certifies the contact track before the bracket and the relation
at both bracket samples. This is a solver restart point, not an impulse by
itself. On edge exit, an exact equality at the root and a strict projected
gap afterward prove the one-sided departure; a later recontact is searched
on the remaining interval after the restart. If a root cannot be bracketed
within the resolution or budget, return `SweepUndecided` with the earliest
unresolved interval. Curved, rotating, or non-box contact may remain
undecided until their own contact-track proof exists.

The dynamics solver invokes `ContinueSeparatingTouch` after resolving an
impact whose next drift begins at certified touch. It invokes
`ContinueCertifiedTouch` for supported resting or sliding contacts. It may
advance across a contact transition bracket only when its time-travel and
state residual gates cover the bracket; otherwise the step is undecided.

## 6. Report shape and ordering

```go
type SweepOutcome int // Clear, DepartedClear, PersistentTouch,
                      // ContactTransitionBracket, ImpactBracket,
                      // InitiallyTouching, InitiallyOverlapping, Undecided,
                      // GrazingTouch; zero is invalid

type SweepCause int // None, PoseRelation, MissingBound, TimeFloor,
                    // FractionFloor, PoseBudget, ContactUnsupported,
                    // DepartureUnproved, ContactTrackUnproved,
                    // EventUnrepresentable

type SweepInstant struct {
    Fraction units.Value // Dimensionless dyadic fraction, canonical time
    Elapsed  Measurement // Time, outward bound on Fraction × Duration
}

type SweepInterval struct {
    From, To SweepInstant
}

type SweepDeparture struct {
    Until SweepInstant
    GapAtUntil Measurement // strictly positive lower gap at Until
}

// SweepContactTrack holds an immutable private continuous certificate.
// Accessors return copies; no exported field can change that certificate.
type SweepContactTrack struct { /* private source snapshot and proof */ }

func (t *SweepContactTrack) Start() SweepInstant
func (t *SweepContactTrack) End() SweepInstant
func (t *SweepContactTrack) Features() (ContactFeature, ContactFeature)
func (t *SweepContactTrack) Normal() VecMeasurement
func (t *SweepContactTrack) ManifoldAt(fraction units.Value) (*ContactManifold, error)

type SweepSample struct {
    At           SweepInstant
    PoseA        r3.Transform
    PoseB        r3.Transform
    FloatContact *ContactReport // query at the two read float transforms
    Ideal        SweepEvent     // relation transferred to the ideal paths
}

type SweepEvent struct {
    At       SweepInstant
    Relation ContactRelation
    Gap      *Measurement // separated or certified touching only
    Overlap  *Measurement // bounded ideal overlap, when proven
    Manifold *ContactManifold
    Reason   ContactReason // when relation or manifold is unavailable
}

type SweepReport struct {
    A, B            *Body
    PathA, PathB    PairPath
    Request         SweepRequest
    Outcome         SweepOutcome
    Bracket         *SweepInterval // ImpactBracket or ContactTransitionBracket
    Unresolved      *SweepInterval // only for Undecided
    InitialEvent    *SweepEvent // certified touch/overlap at time zero
    Event           *SweepEvent // later impact, graze, initial event, or transition right sample
    Departure       *SweepDeparture // only after one-sided proof
    ContactTrack    *SweepContactTrack // complete certified touching prefix
    Cause           SweepCause     // only for Undecided
    Samples         []SweepSample  // increasing Fraction, no duplicate time
    BoxExcluded     bool           // true only when both swept boxes prove clear
    PoseEvaluations uint64
}

func (r *SweepReport) HasAffineReplayProof() bool
func (r *SweepReport) BracketEndsAtDuration() bool
func (r *SweepReport) CertifiedPosesAt(elapsed units.Value) (r3.Transform, r3.Transform, error)
func (r *SweepReport) CertifiedPosesAtInterval(time, start, end units.Value) (r3.Transform, r3.Transform, error)
```

`BracketEndsAtDuration` compares the producer's private exact bracket right
fraction with one. The public `Fraction` may be rounded.

For `SweepGrazingTouch`, `Event` is the exact interior touching sample.
`Bracket`, `Unresolved`, `InitialEvent`, `Departure`, and `ContactTrack` are
nil; `Cause` is `SweepNoCause`. `Samples` includes the separated start and end
and the touching event in increasing fraction order. The outcome is not an
impact bracket: it supplies no closing velocity and authorizes no impulse.
Append its public enum value after `SweepUndecided` to preserve existing
numeric values. Append `SweepEventUnrepresentable` after the existing cause
values for the same reason. Reversing A and B keeps the same event fraction
and reverses the bounded contact normal.

`CertifiedPosesAt` evaluates the same float path used by the sweep at
the requested elapsed time. For affine paths it checks the read float poses
against the cached exact source boxes, spheres, or cylinder outer box.
The sphere-pair path keeps
both source centers, radii, exact translations, and impact bracket. At an
arbitrary interior fraction it compares both rounded centers with their ideal
rational centers, then checks the exact squared center distance against the
radius sum. For centered rotating departure, it evaluates the recorded rigid
rotation and exact affine center displacement before the same comparison. A
clear or departing sample must retain a positive gap after
charging the center displacement; an impact prefix stays clear before the
bracket and stays near contact at its right endpoint. A sample beyond the
bracket is unsupported. It refuses when total displacement exceeds
`PointResolution` or changes a clear or departing relation. For source-box
persistent contact and event brackets, it accepts only a rounded relation
within that resolution of the exact source-box relation. It does not call `ContactPair` or
read the document. It refuses times outside the sweep or outside a certified
event prefix. `CertifiedPosesAtInterval` maps exact held time values in a
specified interval onto the certified spatial path. A dynamics trace uses
its recorded event and step endpoints for that interval, avoiding a gap
when the rounded sweep duration differs from their exact difference.
For a grazing outcome, replay compares the exact squared center distance
with the squared radius sum on the whole interval. At `s*`, both ideal and
rounded squared center distances must equal the squared radius sum, and the
rounded source faces must match the event. At every other fraction, both
squared distances must be strictly greater than the squared radius sum, and
the rounded center deviation must fit `PointResolution`. The cached
quadratic establishes the ideal sign between queried times. If floating pose
construction moves the rounded pair across or away from touch at `s*`, replay
returns `ErrUnsupported`. The proof never turns a near graze into an exact
event.
For a clear rotating source-box drift, the report retains the sweep's exact
source corners and ideal-path bounds. Replay checks the rounded oriented boxes
with the exact separating-axis test. Their positive gap must exceed the total
bounded corner difference from the ideal poses, and that difference must fit
`PointResolution`. Rotating impact reports replay only the certified clear
prefix through the bracket's left edge; the unresolved bracket is refused.
Rotating departure reports replay their separated interior after the
producer's one-sided departure proof. Both paths check the exact oriented-box
gap against their staged corner deviation before returning a rounded pose.
For a co-moving oriented source-box persistent face track, replay uses the
same cached source corners and exact time fraction. The rounded poses must
fit `PointResolution` and yield a four-point patch on the producer's face
pair within its point and normal bounds. A changed face, missing patch, or
exceeded bound returns `ErrUnsupported`.

`Fraction` and the input `Duration` define the exact search time; `Elapsed`
is a bounded convenience reading for callers. Bracket width is checked from
those exact fractions and the exact read duration, not by subtracting rounded
`Elapsed.Value` fields. A solver uses the fractions and duration to make its
own conservative time choice. `SweepSample.FloatContact` is non-nil for every
evaluated pair pose, including an undecided one, and names the original bodies.
Samples are sorted after refinement, independent of evaluation order. The
bracket's right sample and `Event` are the same ideal-path finding. `InitialEvent`
keeps the time-zero relation when continuation finds a later event. A
`Departure` certifies positive gap on `(0, Until]`, not at time zero. A
`ContactTrack` certifies touch from time zero through `End()`. It owns copies
of the source geometry and proof, so changing fields in another report or
retiring a body later cannot alter its result. `ManifoldAt` requires a
Dimensionless fraction in `[Start().Fraction, End().Fraction]` and returns a
fresh bounded manifold. A
`ContactTransitionBracket` carries both `ContactTrack` and `Bracket`, and
`Event` holds the right sample's new relation. `BoxExcluded`
reports the whole-path shortcut; its `Samples` is empty and
`PoseEvaluations` is zero.

For an undecided pose at time zero, `Unresolved` is the zero-width interval at
zero. For a missing continuous proof, it is the earliest floor interval. For
budget exhaustion, it is the earliest not-yet-certified interval. The cause
does not replace the contact kernel's structured diagnostic; `Event` is nil
when no contact is proven. Repeated calls with identical inputs produce the
same outcome, sample order, bracket, and diagnostic order. Any spatial index
or parallel work must preserve that published order.

## 7. Validation, cancellation, and cost

Validate both bodies, both path variants, durations, request units, finite
values, and positive limits before reading `ctx`. Reject identical body
pointers, foreign or retired bodies, non-solid bodies, a zero/invalid
transform, a nonpositive duration, unequal durations, mixed-handedness screw
endpoints, a nonfinite pivot or velocity component, wrong quantity kinds, an
invalid resolution, or `MaxPoseEvaluations < 2` with the matching existing
sentinel (`ErrDegenerate`, `ErrForeignBody`, `ErrRetiredBody`, `ErrUnitKind`,
`ErrNotFinite`, `ErrNegativeMagnitude`, or `ErrUnsupported`). A valid sound
solid whose pair kernel lacks a proof returns `SweepUndecided`, not an input
error. An internal invariant failure remains an error with no report.

The dyadic grid depth is `ceil(log2(Duration / TimeResolution))`. Fully
exploring it evaluates at most `2^depth + 1` distinct times, if that count is
representable. `MaxPoseEvaluations` caps this work without silently relaxing
`TimeResolution`. When the cap is reached, return `SweepUndecided` with
`SweepPoseBudget` and the earliest unresolved interval. Reuse one contact
result per pair time; use the existing bounded kernel work counter and
`ctx` checks within each pose. A caller controls expensive kernel work with
the context deadline, as interference §7.2 does. Do not claim that a pose
budget caps each pose's geometric cost.

After validation, check `ctx` before each pose and at each subdivision;
pass it into the contact kernel. Cancellation returns `ctx.Err()` and a nil
report, even if an earlier prefix was clear. A contact-kernel invariant error
also returns no report. Never call public `Placed`, `PlacedCopy`, `Intersect`,
or a commit path. Snapshot tests must show that document membership/order,
body liveness, and next producer identity are unchanged after clear, impact,
undecided, error, and cancellation results.

## 8. Computed tests and first increment

Use real decad bodies, real pose evaluation, and the real contact kernel.
Assert computed locations, brackets, and bounds, not only enum values.

| Fixture | Required assertion |
|---|---|
| Two 10 mm boxes: A moves `+100 mm/s` from `x=[0,10]`; B is fixed at `x=[20,30]`; duration `0.2 s` | First contact is `0.1 s`. At `0.001 s` resolution, the bracket encloses it and names the original bodies. |
| Both boxes move: A as above, B starts at `x=[30,40]` and moves `-100 mm/s` | First contact is `0.1 s`. Dropping B's travel from the interval certificate must make the test fail. |
| A and B both drift at `+100 mm/s` with a 10 mm initial gap | `SweepClear` holds despite both bodies moving. |
| A box starts under `From=Translation(100,0,0)` and moves 30 mm toward a wall at `x=125` | Swept boxes cannot exclude the wall. A slab beside the original rest box at `x=[20,30]` is excluded. |
| A thin blade rotates while a small pin moves across its path | Coarse separated samples yield `SweepUndecided`; a finer resolution finds a bracket. Neither path may be replaced by endpoint relative interpolation. |
| A drift has `v=(10,0,0) mm/s`, `ω=(0,0,π) rad/s`, and center at the origin | At `0.5 s`, its center is `(5,0,0)` and an initial point `(1,0,0)` reaches `(5,1,0)`. Endpoint screw interpolation disagrees. |
| Initial cap touch, initial positive overlap, and an unsupported tangent graze | Touch and overlap return their distinct initial outcomes; the graze returns `Undecided`, never `Clear` from samples. |
| Two radius-5 mm source spheres: A stays at `(0,0,0)`, B starts at `(20,10,0)` and moves `(-40,0,0)` mm in 1 s | The exact polynomial is `1600(s−1/2)²`; `SweepGrazingTouch` reports a bounded point at `s=1/2`, separated endpoints, no bracket, and full-span interior replay. Reverse pair order and compare event time and reversed normal. |
| The same pair with B starting at `y=11`, `y=9`, or `y=10` and `x=20` moving to `x=0` at the final instant | The first path is `SweepClear`; the second enters impact search; the endpoint tangent remains `Undecided` in this increment. |
| A graze with an event fraction or elapsed time that `units.Value` cannot represent exactly, a rounded event pose that loses exact touch, or a two-pose budget | Return `SweepUndecided` with the earliest interval and specific cause; never publish `SweepClear` or a false graze. |
| A 10 mm box touches a fixed floor, then moves upward at `50 mm/s` for `0.1 s` | Default mode returns `InitiallyTouching`. `ContinueSeparatingTouch` returns `DepartedClear`, with a positive final gap enclosing `5 mm`. |
| The same touching boxes slide tangentially while their face patches overlap | `ContinueSeparatingTouch` returns `Undecided`; `ContinueCertifiedTouch` returns `PersistentTouch` until the first patch-feature change. |
| A top box starts at `x=[0,10]` on an equal fixed box, then slides `+5 mm/s` for `3 s` | The right-sided initial patch is admitted. `ContactTransitionBracket` encloses the edge exit at `2 s`; no earlier transition occurs. |
| A rotating body starts in touch without a one-sided contact-set proof | Both continuation policies return `Undecided`; positive velocity at one witness cannot certify the complete contact set. |
| Equal stationary segments and two same-handed reflected segments | Stationary paths are valid; reflected paths are accepted by geometry when the kernel can prove their relation. A mixed-handedness screw path returns `ErrDegenerate`. |
| Missing contact manifold at a proven overlap endpoint | The report keeps `ImpactBracket` and marks the event manifold unavailable; dynamics refuses to consume it. |
| Tight pose budget and cancellation during pair evaluation | The budget returns `Undecided` with earliest unresolved interval; cancellation returns `ctx.Err()` and nil report. Both preserve document state. |

The first end-to-end slice uses the first box fixture. It must send the
production sweep's certified contact event and manifold to the production
dynamics solver, then assert the computed post-impact position and velocity
from `docs/rigid-dynamics-design.md`'s Verification section. A hand-written
event fixture does not prove that
boundary. Later tests cover angular drift, graze, reflected geometry, and
kernel staging. Root-package test names update `.github/test-shards.txt`.
