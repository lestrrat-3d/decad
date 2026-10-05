# Rigid Dynamics Design

This document owns the `dynamics` subpackage's world, state, step, response,
and trace contracts. `docs/collision-dynamics-design.md` owns the package
boundary; `docs/contact-geometry-design.md` and `docs/contact-sweep-design.md` own
geometry results. A two-body world steps one pair with at least one dynamic
body using density-derived or supplied mass. An axis-aligned
certified contact normal determines the response component; tangent velocity
continues through an oblique impact. Centered impacts of two dynamic bodies
apply equal and opposite impulses.
An initial frictionless, half-width offset face impact between two dynamic
source boxes can produce equal Y spin. Four equal normal point impulses pass
the bounded velocity, angular, energy, and momentum checks before a rotating
departure sweep supplies both endpoint poses.
A fixed box and a dynamic source box at matching 45-degree Y poses can resolve
an initial centered frictionless face impact. The solver checks the bounded
normal and four point impulses against its momentum and angular residuals.
Zero restitution stops the box and requires ideal and rounded persistent-contact
tracks. Positive restitution reverses and scales its normal velocity; both
paths must certify one-sided departure and a separated endpoint. A second step
from that endpoint must certify a clear path. Tangential incoming motion and
off-center contact return `Undecided` in this tilted-pose path.
At a horizontal partly overhanging source-box face, a centered frictionless
fixed/dynamic pair with zero restitution can stop against the exact clipped
polygon. The response checks omitted spin and requires an ideal and rounded
persistent-contact track. An off-center patch requiring spin returns
`Undecided`.
`Trace.Sample` also returns interior states on a persistent-contact path. The
rounded source-box poses must stay within the cached oblique sweep's point
resolution and keep its bounded face track; otherwise replay returns
`ErrUnsupported`. Interior departure replay is not yet certified.
A source semicircle sphere with supplied or density-derived mass can rebound
from a fixed source box on an isolated face-point contact when its affine sweep
stays within that face corridor.
Two source semicircle spheres with supplied or density-derived mass can
rebound as a centered dynamic pair when the impact has a cardinal manifold.
Their approach may
include transverse motion when the sweep certifies first impact.
An isolated frictionless off-axis sphere-pair impact also admits a bounded
center-line normal with density-derived or supplied bounded mass and center
readings when neither body starts with spin. The solver checks both mass
interval endpoints against the impulse law and bounds torque from the contact
witness, center, and normal intervals against `AngularVelocityResidual`.
The center restitution error plus both omitted contact-point speeds must fit
one `VelocityResidual` budget. Both omitted rotational energies share the
impulse-times-remaining-residual allowance, and their full-step point travel
shares one `PointResolution` budget.
The impulse changes both velocity vectors along that normal. A small
separated position correction within the certified
bracket and contact slop is allowed when a full clear remainder sweep proves
the pair cannot meet again. Unresolved tangency or a noncentral mass stops the
step when its angular response exceeds that residual.
A two-body world's pair may be excluded; its bodies drift independently even
through overlap, and no pair material is mixed.
The current three-body step admits one dynamic body and two fixed bodies.
It sweeps each non-excluded dynamic/fixed pair. One active pair advances only
when every other pair has a certified clear path through the response. Two
simultaneous initial face contacts also advance when their certified normals
lie on distinct coordinate axes, both effective pair friction coefficients
are zero, and the fixed/fixed pair is separated or excluded. It applies each
zero-restitution normal impulse to the shared dynamic body, bounds their
combined omitted spin and whole-step point travel, and certifies both ideal
and rounded persistent tracks through the endpoint. Resting constraints may
receive zero impulse. Each impulse produces an ordered event; the step reports
their summed contact impulse and the dynamic body's conservation readings.
Later simultaneous impacts, coupled normals, separating initial contacts,
and simultaneous friction return `Undecided`. An excluded pair may overlap or
cross without a contact event or material mixing.
At an initial face touch, a fixed floor and dynamic source box can receive a
full-step kick from gravity and any center force, then a zero-restitution
support impulse. A zero-restitution impact can continue as certified
persistent contact, and a stationary touching pair can advance without an
impulse. These paths require full-span ideal and rounded contact
tracks. Each dynamic body accepts at most one center force and world-frame
torque; mass and inertia intervals must fit the kicked velocities within their
respective velocity residuals.
An initially touching pair with exactly zero relative normal speed can slide
without an impulse when both full-span paths certify persistent contact. The
first edge-exit step accepts a transition whose right bracket sample is
separated, then certifies a clear remainder. A transition with a touching
right sample or insufficient `MaxEvents` remains `Undecided`.
An affine translating kinematic driver can move through a certified clear
step, push an initially touching dynamic box with a zero-restitution response,
depart from an initially touching box without an impulse, or cause a centered
interior impact. Departure requires ideal and rounded full-path certificates;
a zero-restitution response requires certified persistent touch.
An axis-screw rotating driver can cross a centered source-box face and
rebound the dynamic box when its oriented contact witness and separating
remainder certify the step. Its event records the contact-point derivative,
while the prescribed body's stored velocity stays zero.
The driver's stored velocity remains zero while its derivative enters the
response. `World.Step` admits a positive effective pair friction coefficient
for a fixed floor and a dynamic source box in either world order. An initial
four-corner face touch
with zero incoming spin, positive X slip, and closing Z speed can use the joint
Coulomb solver. The response publishes zero Y/Z velocity, bounded corner and
aggregate impulses, a residual report, and a persistent-touch trace after
both ideal and rounded sweeps certify the full remainder. Zero X slip uses
the centered normal support path with zero tangent impulse.
The floor stays at identity placement. The box can start from any pure
translation whose real four-corner track passes the same bounds, including
the endpoint of a previous step. Its mass center is translated with exact
rational coordinate sums for the lever and torque certificates.
Off-center fixed/dynamic impulses that require spin return `Undecided`.
An initial two-dynamic face impact can publish bounded spin when its rotational
departure sweep certifies the full remainder. The step reports bounded
kinetic energy, linear momentum, and angular momentum for dynamic bodies
at input, after the full-step force kick, and at completion. It also reports
gravity, center-force, and fixed/kinematic contact impulses separately, plus
the drift-only energy, linear momentum, and angular momentum changes.
Torque-driven rotation advances when `SweepPair` certifies a clear full-span
drift and its rounded endpoint. Its trace replays clear interior poses against
the cached rotating source-box proof. A source box spinning about world Z can also
rebound from a wide fixed horizontal source-box face with positive
restitution and zero friction. Its normal impulse leaves the admitted spin
unchanged within `AngularVelocityResidual`; the returned pose uses the
certified rotating departure sweep. A rotating contact that lacks these
proofs returns `Undecided`. Other rotating kinematic drivers, broader
frictional stepping, stacks, broader contact-transition stepping, broader
spin response, and interior trace sampling across contacts remain design
contracts.

Navigation only; the named sections own the rules:

| Question | Section |
|---|---|
| Which bodies and state values can enter a world? | World and State |
| Which loads, drivers, and limits does a step take? | Step input and configuration |
| How does time advance to the next event? | Step and event schedule |
| How are impact, friction, and resting contact solved? | Response |
| What does failure return, and how is motion replayed? | Completion, conservation, and trace |
| Which computed fixtures gate implementation? | Verification |

## World and State

`World` copies one document's ordered rigid-body definitions, pair exclusions,
material overrides, and `StepConfig`. It stores no changing pose or velocity.
Construction rejects a nil/duplicate/foreign/retired/non-solid body. It checks
each body's liveness again at every step. A fixed or kinematic body needs no
mass properties; a dynamic body needs one of these complete sources:

- A positive exact density → call `Body.MassProperties(ctx, density)` once and
  cache its bounded mass, center, and six inertia components.
- A supplied `decad.MassProperties` → use the same bounded shapes. Its center
  and tensor refer to the body's committed world placement, with the tensor
  about that center in committed world axes. Reject a partial supplied record.

The density and supplied modes are exclusive. World construction copies the
selected input, including a supplied record, so later caller edits do not
change a world. It rejects an unknown exactness value and an `Exact` reading
with a nonzero bound. World construction requires
mass's lower bound to be positive and a certified positive lower eigenvalue of
the complete inertia interval tensor. A nominal positive determinant is
insufficient. Reject inconsistent bounds, a tensor lacking that proof, or a
non-finite inverse as `ErrInvalidMassProperties`. The mass design owns the
integrals and their bounds; `r3` owns symmetric-tensor rotation and inversion.
At a state pose, transform the certified center and tensor from the committed
placement into world coordinates. Recompute world inverse inertia after every
change of orientation.

```go
type BodyRole int // Fixed, Kinematic, Dynamic

type QuantityVec = decad.QuantityVec
// Each use validates all components against its required Kind and finiteness.

type Material struct {
    Restitution units.Value // Dimensionless, [0, 1]
    Friction    units.Value // Dimensionless, >= 0
}

type RigidBody struct {
    Body       *decad.Body
    Role       BodyRole
    Material   Material
    Density    *units.Value         // exactly one mass source for Dynamic
    Supplied   *decad.MassProperties
}

type BodyPair struct { A, B *decad.Body }
type PairMaterial struct { Pair BodyPair; Restitution, Friction units.Value }

type WorldConfig struct {
    Bodies    []RigidBody
    Excluded  []BodyPair
    Overrides []PairMaterial
    Step      StepConfig
}

func NewWorld(ctx context.Context, doc *decad.Document, cfg WorldConfig) (*World, error)
func (w *World) NewState(entries []BodyState) (State, error)

type BodyState struct {
    Body            *decad.Body
    Pose            r3.Transform // composes onto committed placement
    LinearVelocity  QuantityVec  // world Velocity
    AngularVelocity QuantityVec  // world Angle / Time
}
```

`NewState` requires exactly one entry for each world body, in any caller order;
it stores them in world insertion order and binds the state to that world.
`State` provides read-only, copy-returning accessors. `Step` rejects a state
from another world. Every pose must be valid and orientation-preserving:
`r3.Transform.IsValid()` true and `IsReflection()` false. `NewState` rejects
fixed and kinematic entries with nonzero velocity. A fixed pose is constant;
a kinematic pose comes from its step driver, and its stored velocity stays
zero. A dynamic pose and both velocity vectors must be finite. World order
fixes pair enumeration, event order, and trace order. The complete excluded
pair list appears in `World` inspection and each step report; an override for
an excluded pair is an input error.

The effective pair restitution is the smaller body's coefficient; pair
friction is the geometric mean. An explicit `PairMaterial` replaces both
values for that pair. Pair names are canonicalized by world order. Duplicate
or unknown exclusions/overrides fail construction. A pair with neither body
dynamic can still be checked when a kinematic driver moves; it cannot be
resolved by an impulse if closing contact occurs.

The current world accepts two bodies with at least one dynamic body, or three
bodies with exactly one dynamic and two fixed bodies. Every pair in a
three-body world can be excluded or given one material override. `NewWorld`
rejects a pair naming a body outside the world, including nil or repeated
bodies, with `ErrInvalidInput`. It also rejects a second override or exclusion
for the same pair, including reverse order. An override for an excluded pair
returns `ErrInvalidInput`. An excluded pair skips effective material mixing;
each body's material still passes input validation. Held exclusions and
reported event pairs use world order. `World.Excluded()` and
`StepReport.Excluded` return separate copies.

The current positive-friction slice accepts a fixed floor and dynamic box in
either world order, or two dynamic source boxes at an initial opposed face
touch. Without an override, the pair coefficient is the geometric
mean of the held body coefficients. The solver proposes impulses with a nominal
rounded mean, then checks the friction cone and slip law against rational
bounds that enclose the exact mean. A zero body coefficient selects the
frictionless response. A positive mean below the smallest positive float64 or
above the largest finite float64 returns `ErrUnsupported` at `NewWorld`.
With an override, its exact held coefficient replaces
the body values; a positive override goes to the patch solver even when both
body coefficients are zero. A zero override selects the frictionless response
even when the body coefficients differ. Other positive-friction body pairs
return `ErrUnsupported` at `NewWorld` when the pair is not excluded.
The fixed-body patch solver reads a floor-to-box witness in both orders. The
two-dynamic solver applies equal and opposite impulses through both masses
and inertias, then bounds each body's angular response. A nonzero outgoing
spin requires a certified rotational remainder before the step publishes it.
Each event keeps its original world-order manifold. Its normal impulse is nonnegative,
and its tangent and point impulses describe the impulse on world-order B.

## Step input and configuration

```go
type StepConfig struct {
    Contact             decad.ContactRequest
    TimeResolution      units.Value // positive Time
    ContactSlop         units.Value // nonnegative Length
    VelocityResidual    units.Value // positive Velocity
    AngularVelocityResidual units.Value // positive AngularVelocity
    ImpulseResidual     units.Value // positive Impulse
    PenetrationResidual units.Value // positive Length
    ImpactSpeed         units.Value // nonnegative Velocity
    MaxPoseEvaluations  uint64      // >= 2, per pair sweep
    MaxIterations       int         // positive, per event
    MaxEvents           int         // positive, per step
}

type BodyLoad struct {
    Body   *decad.Body
    Force  QuantityVec // world Force, at center of mass
    Torque QuantityVec // world Torque, about center of mass
}

type KinematicDriver struct {
    Body *decad.Body
    Path decad.PairPath // defined over the entire dt
}

type StepInput struct {
    Gravity QuantityVec // world Acceleration
    Loads   []BodyLoad
    Drivers []KinematicDriver
}

func (w *World) Step(ctx context.Context, from State,
    input StepInput, dt units.Value) (*StepReport, error)
```

The pinned `units` dependency provides Time and the composed kinds used above,
including AngularVelocity, Force, Torque, and Impulse, with named units for
public input.
All scalar public values use `units.Value`; no configuration tolerance or
material coefficient is implicit. Reject wrong kinds, non-finite values,
negative slop or impact speed, nonpositive resolutions or residual limits,
and nonpositive limits. `MaxPoseEvaluations < 2` is invalid. `dt` is positive
Time. A pair sweep uses `min(TimeResolution, remaining span)` as its time
resolution and passes `MaxPoseEvaluations` to `decad.SweepRequest`.

`Loads` has at most one entry per dynamic body. Missing loads mean zero;
loads on fixed or kinematic bodies are invalid. Gravity applies to every
dynamic body; force and torque are held constant in world coordinates for
this step. A force acts at the current center of mass. The caller converts
an off-center force to force plus torque. `Drivers` has exactly one path per
kinematic body and no path for another role. Each driver's duration equals
`dt`, its start pose equals the state's pose, and its start/end poses are
proper rotations. The driver supplies contact-point velocity as the
path's time derivative, with its certified numerical bound; absent or
unbounded derivatives make a potential contact undecided. It receives no
impulse. A fixed body has zero contact-point velocity.

The step accepts one typed, finite center force and torque per dynamic body.
A nil, duplicate, foreign, or nondynamic load body or a wrong-kind/non-finite
vector returns `ErrInvalidInput`. It bounds the linear kick over the admitted
mass interval and the angular kick over every admitted inertia component. A
published kick outside `VelocityResidual` or `AngularVelocityResidual` returns
`Undecided`.

The kinematic slice admits one `PoseSegment` with either constant orientation
and an exactly representable affine derivative or a cardinal read screw axis
with exactly representable full-step linear and angular rates. Its start and
duration must exactly match the state and step. A missing, duplicate, or
nonkinematic driver is `ErrInvalidInput`; a rotating path outside this proof or
a derivative that cannot meet the full-step speed proof is `ErrUnsupported`. An initially
touching, closing kinematic/dynamic pair can receive a zero-restitution
support impulse. An initially touching pair with strictly separating relative
normal speed can advance without an event only when the initial manifold is
bounded, both full-step sweeps certify departure, and the endpoint is separated.
This path does not consume `MaxEvents`. A centered interior impact can use
the bounded bracket-right sample and correct only the dynamic pose. A
separating response needs ideal and rounded sliced paths that certify
departure. A zero-restitution response
needs full-span bounded persistent-contact tracks on both sliced paths and a
bounded touching endpoint. A sliced affine driver derivative must equal the
admitted full-step derivative exactly.
Other contact schedules remain `Undecided`. The driver's effective contact
speed appears in the event, while `State` and `Trace` store zero kinematic
velocity. A separated pair can complete a clear driver path without an event.
For a cardinal rotating screw, the collision uses the full path's axial
translation rate at an axis-normal face. A bounded one-point oriented manifold
can supply an interior impact. The event's driver velocity is the screw's
linear derivative plus `ω×(point−axisPoint)` at that witness; the dynamic
body receives the impulse and the kinematic body receives none. A sliced
remainder may differ from the full-step read screw only within the configured
linear and angular velocity residuals. Its axis line must lie within
`PointResolution`, and its contact-point velocity must agree within
`VelocityResidual`. The original screw's support-plane gap must increase
strictly after the impact. Both ideal and rounded sliced sweeps must certify
departure. A rotated persistent contact, off-axis normal, or
unbounded contact-point derivative returns `Undecided`.

`units.Angle` is dimensionally distinct from `units.Dimensionless`. At the
physics boundary, validate every AngularVelocity component as Angle/Time,
convert its base-unit radians per second to an internal scalar, then use
`r3` vector operations for `ω×r`, torque, and `Iω`. Convert the result back
to Angle/Time before publishing state. The same explicit radian-to-scalar
rule applies to angular acceleration and angular impulse. Do not use
`Kind.Mul` to treat Angle/Time × Length as Velocity. Internal numeric vectors
carry a declared physical kind; conversion cannot silently change one.

## Step and event schedule

`Step` uses one semi-implicit force kick for the complete `dt`, then an
event-driven drift. It evaluates each dynamic body's world acceleration at
the initial state: `a = g + F/m` and `α = I_world^-1(τ − ω×I_world ω)`.
The gyroscopic term uses the initial orientation and radian scalar angular
speed. Set the kicked velocities to `v + a dt` and `ω + α dt` once. The
step's discrete force law is this kick; contact-event restarts do not repeat
it. Report the applied force and torque impulses. An overflow, an invalid
updated tensor, or a non-finite kicked velocity is undecided with no state.
Every event's pre-impact velocity is the kicked velocity plus impulses from
earlier events in this step. It is not a claim about velocity under a
continuous force law at that physical time.

For each remaining interval, construct a `decad.RigidDriftSegment` for each
dynamic body from its current pose, world center of mass, kicked/current
linear and angular velocity, and remaining duration. Its ideal center follows
`c(t)=c0+v t`; its orientation follows a world-axis rotation at constant
`ω`. This path is not generally the read screw between endpoint poses when
`v` has a component perpendicular to `ω`. `SweepPair` must consume the real
drift segment and certify that trajectory. Kinematic bodies retain their
original whole-step paths and are evaluated on the same elapsed-time slice;
fixed bodies use constant paths. All path poses compose onto each body's
committed placement. The geometry query's pair order is world order.

Sweep all non-excluded pairs that have a moving body. Do not discard a pair
from endpoint samples. New pairs use `StopAtInitialContact`. A pair whose
contact was just solved uses `ContinueSeparatingTouch` when every solved
normal speed exceeds `VelocityResidual`; otherwise it uses
`ContinueCertifiedTouch`. These policies require the one-sided departure
or persistent-touch proof owned by contact-sweep §5.1. They never skip an
initial touch by sampling at a positive time. `SweepDepartedClear` permits
the rest of the drift. `SweepPersistentTouch` permits the rest only when
the solver's normal/friction residuals stay within limits throughout its
`ContactTrack`. The first source-box implementation checks the track's
affine patch and constant normal/velocity; unsupported varying tracks are
undecided. `SweepContactTransitionBracket` stops the drift at the first
feature or patch change. It is a solver restart, not necessarily an impulse.

When the one world pair is excluded, validate input, driver, and force kick as
usual. Drift each dynamic body independently; put a kinematic body at its
driver endpoint. Do not call `SweepPair` or `ContactPair` for that pair, even
when its source solids overlap or cross. Publish no contact event and typed
zero contact impulse and kinematic work. The trace and conservation readings
still describe the completed step.

In a three-body world, sweep both non-excluded dynamic/fixed pairs before
choosing a response. The first simultaneous increment consumes both real
initial manifolds for orthogonal frictionless face contacts. A simultaneous
case outside that increment returns `Undecided` with no `Next`. For one active
response, certify the other pair's ideal and rounded paths before and after
the event; sweep any position correction against it. Query the fixed/fixed
pair at its constant pose unless it is excluded. The conservation report
includes the world's sole dynamic body. More than one dynamic body or a
kinematic body returns `ErrUnsupported` at three-body world construction.

A `SweepPair` `Undecided` that could precede the next event makes the step
`Undecided`; a later undecided interval can be revisited after an earlier
event changes the drift. `Clear` permits advancement across its whole span.
`InitiallyTouching` enters the contact set if closing or resting; separating
touch receives no impulse and needs a continuation proof before advancement.
`InitiallyOverlapping` is accepted only within the correction allowance
below. `ImpactBracket` provides a proven-clear lower time and a contact upper
time, not an exact impact time. Compare impact and contact-transition brackets
in world order, choose the earliest upper time, and gather every event whose
uncertainty interval overlaps that time.
The solver consumes each `SweepEvent.Manifold`, whose bounds include the
path-pose transfer.
`ContactPair` at the chosen float poses can refine a missing manifold only
if the sweep transfers its bounds to the ideal path. If any gathered pair
still has no adequate manifold, stop undecided. Recheck any pair whose bracket
starts before the chosen time but whose upper time lies later; do not advance
past a possible earlier contact. `MaxEvents` counts impacts and contact
transitions. Reaching it with remaining time returns `Undecided` and no `Next`.

Advance poses along the same certified paths to the chosen upper time. That
time is the numerical event time; the report retains the original bracket.
Every bracket's travel uncertainty contributes to the allowed penetration
and correction. If another pair impacts before a persistent track ends,
advance every body and sample that track's manifold at the earlier time.
Include its active constraints in the simultaneous solve, then discard the
unused track suffix and sweep all pairs under the new velocities. At a
contact-transition bracket, advance to the chosen time, recheck contact
features, and solve only constraints now closing or resting; the transition
itself applies no impulse. After solving, rebuild each remaining dynamic
drift from the new pose and velocity, and sweep again. No minimum time
quantum may silently skip a persistent contact: a zero-time repeat enters
the resting contact solve or stops undecided at `MaxEvents`. On an event-free
remainder, advance to `dt`. Run `ContactPair` on every active contact at
the completed pose and reject penetration beyond the configured residual.

## Response

At a contact point let `n` point from A to B; let `rA` and `rB` run from
each world center of mass to the geometry-certified point. Use the bounded
manifold's representative point and normal only when its point and angular
bounds meet `StepConfig.Contact`. A source feature may supply several
required normal constraints; solve them together in stable feature order.
Never derive a normal from overlap volume or a render mesh.

The relative velocity is
`u = (vB + ωB×rB) − (vA + ωA×rA)`. Closing means `u·n < 0`.
For one isolated frictionless contact, let
`K = mA^-1 + mB^-1 + (rA×n)·I_A^-1(rA×n) +
(rB×n)·I_B^-1(rB×n)`. Fixed and kinematic inverse mass and inertia are
zero. If pre-impact `u·n < −ImpactSpeed`, the target post-impact normal
speed is `−e(u·n)`; otherwise its target is zero. The isolated impulse is
`Jn=max(0,(target−u·n)/K)`. Apply `−Jn n` to A and `+Jn n` to B, with angular
impulses `r×J`. Reject `K <= 0` for a closing pair. Separating contacts
with no resting constraint receive zero impulse.

For simultaneous points, solve normal complementarity and the Coulomb disk
jointly, rather than applying the isolated formula point by point. Each
normal impulse is nonnegative. Each post-solve normal speed is at least its
target; an impulse above zero requires speed equal to the target within
`VelocityResidual`. Two orthogonal tangential components satisfy
`||J_t|| <= μ J_n`; sticking requires near-zero tangential velocity, while
slipping requires friction opposite tangent motion within the same velocity
gate. Construct the tangent basis deterministically from the least-aligned
world axis and `n`. Use a fixed-order projected iterative solve over the
whole contact island. Process islands, pairs, and manifold features in world
and source-feature order. Warm-start only from the input state's immutable
contact cache, keyed by body pair and certified source features; do not
key by topology slice index. Clamp a reused impulse to the current cone.

Publish the maximum normal-velocity, tangent-velocity, friction-cone
(Impulse kind), and penetration residuals, plus iteration count, for each
island. `VelocityResidual`, `ImpulseResidual`, and
`PenetrationResidual` gate success. If a residual exceeds its limit when
`MaxIterations` is reached, return `Undecided`. Use no random ordering or
solver tolerance hidden from `StepConfig`. Initial/resting contact uses zero
restitution, persists in the state cache, and passes the same gate every
step. A closing contact with no dynamic participant is undecided because
its prescribed trajectory cannot receive an impulse.
Evaluate each residual over the admitted mass/inertia, contact-point, and
normal bounds; a nominal solution whose uncertainty can exceed a limit is
`Undecided`. Exact source boxes and analytic mass can make these bounds
narrow; the arithmetic residual still applies.

The fixed/dynamic frictional step uses the identity-placed box's real
four-corner manifold, exact-rational impulse and torque sums, and the
mass/inertia bounds. It checks that the entire body cannot expose a larger
contact-point lever than the solver audited at the initial corners. It also
bounds omitted-spin travel over the full step. A narrow floor patch, an
interior frictional impact, a rotating state, and a contact transition
return `Undecided`.

The two-dynamic solver uses the same real four-point manifold but includes
both inverse inertias and both mass intervals in each impulse and residual.
For zero friction and zero incoming tangent speed, it proposes equal normal
impulses at all four points. It publishes only when every point reaches its
restitution target within `VelocityResidual`, the bounded angular laws pass,
and the rotational remainder proves departure.
It publishes neither a completed state nor an event when the solved outgoing
spin lacks a certified rotational remainder. The corresponding `World.Step`
returns `Undecided` with `Next == nil`.

For an admitted spinning response, the rotational `RigidDriftSegment` sweep
must certify departure and every later interval through the requested end.
`World.Step` takes each returned endpoint pose from that sweep's fraction-one
sample. The sample's relation must be certified separated after charging its
float-pose deviation against the ideal path. The trace records the same two
rigid drift paths and their sweep report. It returns stored event and endpoint
checkpoints; an interior sample requires a separate rotational replay proof.
An absent fraction-one sample, a nonpositive endpoint gap, or a trace path
that differs from the swept path returns `Undecided` with no `Next`.

The fixed-floor Z-spin impact uses the rotating sweep's four-point horizontal
manifold. Its spin axis is the certified contact normal, so contact-point
spin contributes no normal speed. The bounded four-corner lever check limits
unpublished angular impulse. The event records the pre-impact poses, pre/post
angular velocities, and equal normal point impulses so conservation checks
both angular momentum and rotational energy. If float position correction
leaves a positive exact gap within the original correction allowance, move the dynamic box
back by that gap and accept only a rechecked `Touching` relation. The
remaining `RigidDriftSegment` must prove immediate departure and a separated
fraction-one sample that equals the published endpoint. Zero-restitution
spinning support still needs a contact-track proof and returns `Undecided`.

The fixed/dynamic translating-box slice publishes zero spin. It bounds the
omitted angular speed from the real manifold's patch-center offset and point
bounds, the mass-center bound, an impulse upper bound, and a certified lower
inertia eigenvalue. It returns `Undecided` when that upper speed exceeds
`AngularVelocityResidual`. The impulse and velocity response residuals compare
the published floats with exact rational evaluation of the held input values;
one final arithmetic ULP does not cover all intermediate rounding.
For zero-slip support, four equal normal corner impulses use the same bounded
witnesses. The sum of their horizontal levers and each point and center bound
set an upper angular speed through the certified inertia lower bound. The step
also checks that this speed can move no box point beyond
`PenetrationResidual` over the full duration. A bound that exceeds either
limit returns `Undecided`.

Position correction may move a dynamic body by at most the sum of certified
geometry displacement, the sweep bracket's point-travel bound, and
`ContactSlop`. An initial overlap beyond that allowance is undecided.
Correct all simultaneous contacts together. Record every correction in the
event and trace; sweep the correction path against every other pair and
reject a new contact or undecided interval. Active contacts may remain
touching while their penetration decreases. A post-correction `ContactPair`
bounds residual penetration. At a separating impact at the final step time,
a separated corrected pose is accepted only when the contact report's upper gap is within
the same correction allowance. A nonzero remainder still requires a touching
corrected pose and a certified continuation. A correction does not alter
velocity and cannot claim to conserve mechanical energy. A failed correction
or new uncertain pair returns `Undecided`.

## Completion, conservation, and trace

`ContactEvent.Kind` distinguishes `ContactImpact` from `ContactTransition`.
An impact carries a bounded manifold and a normal impulse. A separated
contact transition carries its original bracket, the chosen right time,
unchanged velocities, and zero impulse; its `Manifold` is empty because the
right state is separated. `Trace.Sample` at that time returns the right state.
The frictional patch event also carries a typed aggregate tangent impulse,
one typed impulse per cloned manifold point, and bounded solver residuals with
the iteration count. A frictionless event and the centered static support
event carry a typed zero tangent impulse; the static support has no joint
solver report.

```go
type StepStatus int // Advanced, Undecided

type StepReport struct {
    Status      StepStatus
    Next        *State // non-nil exactly for Advanced
    Events      []ContactEvent
    Excluded    []BodyPair // canonical world order; copied per report
    Trace       Trace
    Diagnostics []StepDiagnostic
    Conservation *StepConservation // non-nil for Advanced
    // Effective input/config, elapsed time, residuals,
    // angular momentum, and other conservation diagnostics.
}
```

Input validation runs before reading `ctx`. Invalid input returns a typed
error and nil report; cancellation after validation returns `ctx.Err()` and
nil report. A valid call whose geometry, mass, arithmetic, event limit, or
solver cannot settle returns an `Undecided` report with `Next == nil`.
The diagnostic names the pair or island, time interval, failing certificate
or residual, and requested limit. `from`, `World`, and `Document` never
change. The report owns copies of all slices and state data. A report with
`Next == nil` may contain a diagnostic trace prefix for inspection, but it
cannot be passed as an advanced state.

`StepConservation.Input`, `AfterKick`, and `Completion` each contain a bounded
`KineticEnergy` scalar, `LinearMomentum` vector, and `AngularMomentum` vector
summed over dynamic bodies. `KineticEnergy` uses `units.Torque` because the
registered torque unit has the same `kg·mm²/s²` dimension as energy; the field
denotes energy, not a turning moment. Linear momentum components and bounds
have kind `units.Impulse`; angular momentum components and bounds have kind
`units.AngularMomentum`. Angular momentum sums the orbital term
`mass × (world mass center × linear velocity)` and the rotated inertia tensor
times world angular velocity. Source mass-center and inertia bounds widen
the reading.
`GravityImpulse`, `LoadImpulse`, and `ContactImpulse` are separate bounded
linear vectors. `TorqueImpulse` reports the exact held world torque times
step duration as an `AngularMomentum` vector. `KinematicWork` is a signed
bounded energy reading with kind
`units.Torque`. For each kinematic contact event, it sums the exact held
aggregate impulse delivered to the dynamic body dotted with the driver's
published effective event velocity. A step without a kinematic impact has
typed zero work. Contact impulse sums only events against fixed or kinematic
bodies;
the two impulses of a dynamic pair cancel in the world total. These readings
describe the discrete step and exclude fixed/kinematic bodies' own energy and
momentum. The contact reading encloses the published numerical event impulses;
it does not claim to enclose an uncomputed physical impulse. Mass uncertainty
and conversion rounding widen energy, momentum, and gravity readings. A
reading that cannot be enclosed by finite typed values makes the step
`Undecided` with no `Next`.

`StepConservation.DriftChange` contains bounded energy, linear momentum, and
angular momentum changes over recorded trace slices. Without an event,
it uses `AfterKick → Completion`. With an event, it sums `AfterKick → pre-event`
and `post-event → Completion`. It excludes the force kick, event impulse, and
event position correction. Each body's translation and velocity coefficients
sum before the one held mass interval is applied, so the same source-center
uncertainty cancels across each pure-translation slice. Energy and linear
momentum change by zero on pure-translation slices. A rotating slice subtracts
separately enclosed endpoint readings, including rotated inertia. A changed
velocity within a drift slice makes the reading `Undecided`.

Kinetic energy includes translation and spin. Each advanced contact event
checks every dynamic body's published velocity change
against its signed aggregate normal and tangent impulse. It checks both ends
of the body's held mass interval against `ImpulseResidual + massHigh ×
VelocityResidual` on each component. A zero-impulse transition must preserve
the published velocities exactly. Impacts also check kinetic
energy at the event. For each body, the code first subtracts squared pre-event
speed from squared post-event speed, then multiplies this one difference by
the mass endpoint that gives the largest energy change. The allowed numerical
gain is the sum, over each dynamic body and Cartesian component, of
`(massHigh × VelocityResidual + ImpulseResidual) ×
(|preVelocity| + |postVelocity| + VelocityResidual)`. A failed gate makes the
step `Undecided`. The full-step force kick remains outside these event checks.
For a kinematic impact, the energy gate subtracts the driver's work from the
dynamic energy gain. It adds `ImpulseResidual` times the sum of the absolute
driver velocity components to the numerical allowance. Events that publish
angular velocity and event poses also check spin-energy change against the
same held inertia interval and compare angular-momentum change with the
signed torque from every published point impulse. Angular-velocity and
contact-point residuals widen that comparison.

`Trace` contains the starting state, each certified drift slice, each
event's pre/post states, each position correction, and the ending state.
`Trace.Sample(t)` evaluates the recorded paths without running geometry or
the solver. At a numerical event time it returns the post-event velocity
and pose; an event exposes the pre-event values separately. Its event stores
the pair's original bodies, the sweep time bracket, chosen numerical time,
certified manifold and its bounds, impulses, residuals, and pre/post
velocities. `Trace.Sample` refuses a time outside `[0, dt]`. Feed `Next` to
the next `Step` for deterministic replay with the same inputs and config.
The trace never labels the chosen numerical time as an exact physical impact.
Contact transitions appear in the trace with zero impulse unless the
restarted solver finds a closing constraint at that transition.

The translating first slice re-sweeps every returned rounded drift endpoint
as a `PoseSegment`. A numerical path that lacks `Clear` or
`DepartedClear` proof returns `Undecided`, even if its ideal drift was clear.
The rotational response uses the certified `RigidDriftSegment` as its stored
path and returns that sweep's certified fraction-one float pose. It does not
claim that an independently interpolated `PoseSegment` is clear.
`Trace.Sample` evaluates clear, departed, persistent-contact, and transition
slices against each stored rounded sweep's cached source-box or source-sphere
certificate.
For an off-axis sphere-pair impact, the rounded impact prefix must reach its
own certified bracket endpoint with the same source faces, bounded normal,
and separation within `PenetrationResidual`. Interior trace samples before
and after the event use the corresponding sphere-pair sweep certificates.
It compares the exact held time values when selecting an event or endpoint.
An event at a proved final sweep fraction retains the input `dt` value;
interior event times must lie strictly inside the exact held duration.
For each slice it maps the exact time between its recorded global endpoints
onto the certified rounded path fraction, so subtraction rounding leaves no
unsampled gap before the step endpoint.
It returns `ErrUnsupported` for a slice without a rounded certificate, or
when the requested float pose exceeds the sweep's resolution. Impact prefixes
have a separate rounded sweep ending at the published pre-event pose, so
samples on both sides of an impact consume their own certificates. The
spinning source-box impact uses the original full-step rotating sweep for its
clear prefix and maps pre-event trace time against the full step duration.
An event-free clear rotating source-box drift also replays its interior poses.
The sweep retains its exact held source corners and path inputs. Each requested
rounded pose must lie within `PointResolution` of the ideal path under the
same staged source-corner transform used by contact geometry. The
rounded pair's exact separating-axis gap must exceed that pose error.
For a co-moving oriented source-box face track, replay checks the rounded
four-point manifold against the producer's face identities and point and
normal bounds. A fixed-floor spinning impact replays clear times before its
unresolved bracket from the original full-step sweep. Its separating
remainder replays from the cached departure sweep. Samples inside the
unresolved bracket return `ErrUnsupported`.

Publish kinetic energy, linear momentum, and angular momentum at the input,
after the full-step force kick, and at completion as typed numeric readings;
angular momentum includes orbital and spin terms about the fixed world
origin. Publish gravity/load impulse and kinematic/fixed contact impulse
separately. The full-step kick's energy and momentum change is charged once,
before any event. For an isolated dynamic pair, compare momentum immediately
before and after each impulse: equal/opposite impulses must leave total
linear momentum within the published numerical residual. Check angular
momentum about one world origin using both lever arms at that event. With
restitution in `[0,1]` and no kinematic participant, compare kinetic energy
immediately before and after an impulse; frictional impact cannot increase
it beyond its computed numerical residual. A failed event gate is undecided.
Also publish drift-only changes separately, since this first-order rotation
update need not conserve an asymmetric body's angular momentum exactly.
Kinematic work, the discrete force kick, and position correction are reported
so an energy difference in those cases is not mislabeled as collision loss.
These numeric checks apply to this discrete integrator; they are not
certified bounds on real continuous-force motion.

## Verification

First use a real `decad` box with top `z=0 mm` and a dynamic `10×10×10 mm`
box initially spanning `z=[10,20] mm`. Give the dynamic box density
`0.001 kg/mm³` → mass `1 kg`; set gravity and friction to zero, initial
downward velocity `100 mm/s`, restitution `0.5`, and `dt=0.2 s`. The real
`SweepPair` bracket and `ContactPair` manifold must reach the solver. The
contact time lies within the requested time resolution of `0.1 s`; normal
impulse is `150 kg·mm/s`; the final upward velocity is `50 mm/s`; and the
bottom ends at `z=5 mm`, within the stated integration/geometry tolerance.
Initial/final kinetic energies are `5000` and `1250 kg·mm²/s²`. The document
body set and placements are unchanged. A hand-written manifold does not
exercise this boundary.
With `dt=0.1 s`, the same real path impacts at the final fraction. Its
`150 kg·mm/s` impulse leaves upward velocity `50 mm/s`; any separated rounded
endpoint gap must fit the certified correction allowance.

The supplied-mass fixture passes that box's real `Body.MassProperties` result
through `RigidBody.Supplied`, `NewWorld`, `SweepPair`, and `Step`. It has the same
`150 kg·mm/s` impulse, `50 mm/s` outgoing speed, and `z=5 mm` final bottom.
Changing the caller's record after world construction does not change the step.

The oblique fixture uses a wide fixed floor and the same `1 kg` box, starting
`10 mm` above the floor with velocity `(50, 0, −100) mm/s`. At `0.1 s`, the
real sweep brackets face contact and the step applies `150 kg·mm/s` along the
certified vertical normal. At `0.2 s`, the box has velocity `(50, 0, 50) mm/s`
and translation `(10, 0, 5) mm`. The tangent speed stays `50 mm/s`; the ideal
and rounded departure paths are both certified.

The tilted-pose fixture rotates two touching `10 mm` boxes by 45 degrees
around world Y. One box is fixed; the `1 kg` dynamic box enters at
`(-50, 0, 50) mm/s`. Its four-point bounded manifold and initial-touch sweep
produce a `sqrt(5000) kg·mm/s` support impulse. The dynamic box stops and
both the ideal and rounded sweeps prove persistent contact through `0.01 s`.
The reported contact-impulse bound is positive; another stationary step
advances without an event. With restitution `0.5`, the initial impulse is
`1.5 sqrt(5000) kg·mm/s`, the outgoing velocity is `(25, 0, −25) mm/s`, and
the box translates `(0.25, 0, −0.25) mm` over `0.01 s`. Both ideal and rounded
paths prove one-sided departure; a second step advances clear without an event.

The center-force fixture starts the same `1 kg` box `10 mm` above the floor at
rest. A `−500 kg·mm/s²` center force over `0.2 s` gives a full-step kick of
`−100 mm/s`. The real sweep brackets contact at `0.1 s`; the step reports a
`150 kg·mm/s` contact impulse and finishes at `+50 mm/s` with its bottom at
`z=5 mm`. A supplied mass interval of `1 ± 0.01 kg` returns `Undecided` at a
`1e−6 mm/s` velocity residual because its kick cannot be bounded that tightly.

The torque fixture starts a `1 kg`, `10×10×10 mm` source box `100 mm` above
the floor. A `100 kg·mm²/s²` world-Z torque over `0.1 s` produces `0.6 rad/s`
spin and a `0.06 rad` drift rotation. The real full-span rotating sweep proves
separation; the step reports `10 kg·mm²/s` torque impulse and bounded spin
energy and angular momentum. A second step from its rotated endpoint, with
zero load, advances another `0.06 rad`. An approaching torque-driven box
returns `Undecided` when the rotating sweep cannot certify the contact.

The implemented resting-contact fixture starts a `1 kg` source box at rest on
a fixed floor and applies gravity `−1000 mm/s²` for `0.1 s`. The full-step
kick gives `v_z=−100 mm/s` before the zero-time contact event. The support
impulse is `100 kg·mm/s`, and the box finishes with zero velocity and its
bottom at `z=0 mm`. The second step repeats those results from the returned
state. Both steps consume the real face manifold and require a full-span
`SweepPersistentTouch` track for the ideal and rounded drift paths.

The zero-restitution impact fixture drops the same box from `10 mm` above
the floor at `100 mm/s` for `0.2 s`. It reaches contact at about `0.1 s`,
receives `100 kg·mm/s`, and finishes at rest with its bottom at `z=0 mm`.
The remaining drift passes ideal and rounded `SweepPersistentTouch` checks.
The stationary-touch fixture starts with the box already on the floor, uses
zero gravity for `0.1 s`, and returns the same pose and velocity with no
contact event or impulse. It also checks both full-span contact tracks.

The frictionless-slide fixture starts the same `1 kg` box touching a wide
fixed floor with `(50, 0, 0) mm/s` velocity and zero gravity. At `0.1 s`,
it has translated `(5, 0, 0) mm` with unchanged velocity and no contact
event or impulse. Real ideal and rounded sweeps each return a full-span
`SweepPersistentTouch` track, and `ContactPair` confirms both endpoints.
A narrower floor that changes the contact patch during the step returns
`Undecided` when the step cannot certify the transition and remainder. The
first implemented edge-exit case has a `10 mm` wide floor and touching box,
with the box moving at `5 mm/s` for `3 s`. The transition brackets `2 s`,
and a certified clear remainder ends at `x=15 mm` with unchanged velocity.
It reports one zero-impulse transition event. `MaxEvents=1` refuses the
remaining `1 s`; `MaxEvents=2` admits the step.
The solver also sweeps the rounded prefix directly to its published right
pose. The full-step rounded segment can sample a different pose by one ULP
at that time; the direct prefix must certify the same separated transition.

The kinematic-push fixture has two touching `10 mm` source boxes. Body A
follows a `PoseSegment` of `+1.25 mm` in `0.125 s`; body B has mass `1 kg`
and starts at rest. With zero restitution, friction, and gravity, the real
initial face manifold produces a `10 kg·mm/s` impulse. The ideal driver and
dynamic drift and their rounded paths certify persistent touch. Both bodies
finish `1.25 mm` to the right, B moves at `10 mm/s`, and A stores zero velocity.

The interior kinematic-impact fixture starts A at `x=[0,10] mm` and B at
`x=[20,30] mm`. A's driver moves `+20 mm` in `0.25 s` at `80 mm/s`; B has
mass `1 kg` and starts at rest. With restitution `0.5`, friction zero, and
gravity zero, the real sweep brackets impact near `0.125 s`. The step consumes
the bracket-right sample, applies `120 kg·mm/s`, and certifies departure on
both sliced remainder paths. B ends near `x=[35,45] mm` at `120 mm/s`; A ends
at `x=[20,30] mm` with zero stored velocity. The tiny difference from B's
ideal final coordinates is bounded by the impact bracket and correction.

With the same bodies and driver but restitution zero, the bracket-right
correction and `80 kg·mm/s` impulse give B `80 mm/s`. Both sliced remainder
paths certify persistent face touch. B ends near `x=[30,40] mm` and A ends at
`x=[20,30] mm`; the bracket and correction bound B's small offset from the
ideal coordinates. A still stores zero velocity.

For the no-impulse departure fixture, A starts at `x=[0,10] mm` touching B
at `x=[10,20] mm`. A's driver moves `−5 mm` in `0.125 s`, and B starts at
rest. Both full-step paths certify `SweepDepartedClear`. The step reports no
event, moves A to `x=[−5,5] mm`, leaves B at `x=[10,20] mm`, and stores zero
velocity for both. It advances with `MaxEvents=1` because no event occurs.

For the sliding-friction increment, put the same `1 kg` box
on a fixed floor wide enough for a `5 mm` slide. With gravity
`−1000 mm/s²`, initial horizontal speed `100 mm/s`, friction `0.5`, and
`dt=0.1 s`, the full-step kick gives `v_z=−100 mm/s`. The support impulse
is `100 kg·mm/s`; the friction impulse is `50 kg·mm/s` opposite motion.
The box then slides at `50 mm/s` and moves `5 mm` horizontally while its
bottom stays at `z=0 mm`. `SweepPersistentTouch` must certify the full
face-contact track. A second `0.05 s` step from that endpoint receives a
`50 kg·mm/s` support impulse and `25 kg·mm/s` friction impulse. It ends at
`x=6.25 mm` with `25 mm/s` horizontal speed. A run starting with zero
horizontal velocity must end with zero velocity and the same bottom height.
Every run uses the real contact manifold and sweep continuation, not a
touching sample alone.

Then run focused checks for an off-center impact with spin, two independent
moving bodies, a kinematic push, three simultaneous bodies, Coulomb stick
and slip, a resting stack across two steps, and exact trace replay. For each,
assert computed position or velocity, event ordering, the stated residuals,
and the unchanged document. Exercise reflected/invalid poses, bad mass
bounds, mixed units, missing drivers, an unresolved sweep, an inadequate
manifold, event/iteration limits, initial overlap, and cancellation. Test
energy and momentum on isolated dynamic pairs and report external work in
forced/kinematic cases. Run the real geometry producer through the real
solver in every integration check; direct impulse-equation unit tests may
use constructed contact data but cannot claim pair-query integration.
The first box-contact increment may end the off-center spin check at the
impact event. A longer remainder needs a real pair-clearance certificate;
co-oriented oblique source-box face contact has a bounded manifold, while
other rotated face arrangements may still return `Undecided`.
