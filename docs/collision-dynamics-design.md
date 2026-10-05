# Collision and Rigid Dynamics Design

This document is the system map and delivery order for collision-aware rigid
motion. Current code certifies source-box contact, source sphere-to-box face
contact, affine two-body sweeps,
persistent face contact and its first edge transition, and mass properties for
source boxes, admitted untapered prisms, full source spheres, and full
source cylinders revolved from axis-incident rectangles. It steps one
frictionless pair with a fixed and dynamic body or two centered dynamic bodies.
A three-body world with one or two dynamic bodies steps one active pair
when every other pair has a certified clear path. It also orders separated
frictionless positive-restitution source-box face impacts, re-sweeps active pairs after each response,
and replays the rounded path between events. A full circular source prism or a full
revolve of a rectangular half-profile can advance along a strictly separated
axial path above or below a source-box face and replay that clear path. The
two cylinders can also contact a wide source-box face along their axes,
bracket first impact, and rebound from that fixed face using supplied or
density-derived mass. The sweep and step replay the clear path before and after
impact. The revolved
cylinder publishes an axial disk-face manifold at a contained source-box
floor touch or shallow crossing.
A fixed source-box floor can hold two centered dynamic source boxes stacked
vertically. With zero friction and restitution, one gravity kick is followed
by a coupled two-contact response; all three pair paths receive continuous
proofs, and the trace replays two repeated resting steps.
A zero-bound faceted Boolean union with one certified rectangular lower face
can sweep vertically toward a wide source-box floor, depart from initial
touch, or retain a persistent face track. Its sweep can certify the first
exact touch. With supplied or density-derived mass and centered
frictionless motion, the rigid
solver consumes that first-impact bracket, rebounds or rests on the floor, and
replays the rounded path on either side of impact.
The same zero-bound Boolean can retain its exact lower-face proof after
translation-only `Placed` widens its held mesh. `ContactPair` recognizes that
placed face at floor touch. Its exact-source sweep brackets first axial impact
and certifies separating departure. A density-backed dynamic placed union
with centered frictionless motion and positive restitution rebounds from a
fixed floor; its trace replays the rounded approach and departure. Other
positive-bound faceted bodies can certify only strict clear motion here.
It also steps two orthogonal, frictionless
initial face contacts and their repeated resting response after real manifolds
and full-span tracks certify both pairs. A dynamic source sphere can touch
a fixed floor and source sphere at the same initial instant when its two
frictionless zero-restitution normal impulses pass one coupled response check;
each pair then needs a certified persistent or separating remainder. A source
box can hit a fixed floor obliquely while retaining tangential velocity. A fixed floor and
dynamic source box can complete an initial resting contact after a full-step
gravity kick or a zero-restitution impact. A stationary touching pair also
advances without an impulse. An initially touching box can slide tangentially
across a fixed floor without an impulse while full-span ideal and rounded
contact tracks certify the same face patch. Each touching interval requires a
certified contact track. A dynamic body's mass can come from
density or a caller-supplied bounded mass record. A full source sphere can use
its density-derived mass and inertia in a real fixed-floor rebound. One
world-frame center force and torque per dynamic body contribute to the step's
full-duration velocity kick.
Two co-oriented boxes at a 45-degree pose can resolve a centered frictionless
initial impulse from their four-point manifold. Zero restitution requires
full-span persistent tracks; positive restitution requires ideal and rounded
one-sided departure proofs. Tilted off-center impacts remain `Undecided`.
A horizontal source-box face rotated 45 degrees against an axis-aligned face
can publish all eight vertices of their clipped touching patch. A centered
zero-restitution fixed/dynamic impact stops after its persistent track proves
the complete step; an off-center patch requiring spin remains `Undecided`.
Two equal-mass source boxes with a half-width offset face patch can rebound
as a frictionless dynamic pair with equal nonzero Y spin when the rotating
sweep proves departure.
A fixed source box and a dynamic half-offset source box can also rebound with
Y spin when supplied mass places the dynamic center beyond the patch edge.
Two edge vertices receive the impulse, and a rotating sweep proves the
separating remainder.
Separated source boxes can rotate through a certified clear drift and advance
again from the returned spinning state. A Z-axis torque-driven box can also
rebound from a wide fixed horizontal box face and continue spinning from the
returned state when the real sweep certifies its impact and departure.
The source-sphere face corridor carries a real sphere-to-box point manifold
through an affine first-impact sweep and a centered fixed-floor rebound with
supplied or density-derived sphere mass. Two source spheres also have a bounded
center-line point manifold, an affine first-impact sweep including transverse
motion, and a centered dynamic-pair response at cardinal or off-axis contact
with a certified clear remainder. An interior off-axis impact can also advance
at zero restitution when its corrected gap and both remainder sweeps certify
separation. An exact isolated sphere-pair graze records
one zero-impulse event and replays the separated path on both sides.
An initially touching off-axis pair of dynamic source spheres rebounds under
positive restitution or resolves zero restitution to persistent touch or
tangential departure when its bounded point and full-span sweeps certify the
response and trace.
An initial cardinal X touch between two dynamic source spheres with exact
centered supplied mass can resolve a positive-restitution one-point friction
impulse. The response changes both linear velocities and both Z spins, then
the rotating sphere-pair sweep certifies separation and trace replay. Zero
friction retains the tangent velocities and zero spin on the same path.
A zero-friction, zero-restitution pair uses the existing response.
A separated pair can also resolve an interior positive-friction impact when its
certified bracket contains an exact cardinal source-sphere touch at a dyadic
time. Separate rounded prefix and rotating remainder sweeps certify replay.
An exactly orthogonal source box at a 45-degree Y pose also gives the sphere a
bounded oblique point, an affine first-impact sweep, and a centered
positive-restitution rebound with interior trace replay.
Other curved contact families still lack source
witnesses and continuous proofs.
The first separated edge exit records a zero-impulse contact transition and
advances through a certified clear remainder.
An affine kinematic driver can push an initially touching dynamic source box
through a certified persistent contact path, depart without an impulse, or
produce a centered interior impact with a certified separating or
persistent-contact remainder. A driver with a cardinal screw axis can produce
a centered interior face impact and a certified separating remainder. The event
records the driver's contact-point velocity and bounded work while its stored
velocity stays zero. A fixed floor and dynamic source box in either world order
with positive pair friction can slide repeatedly after a
four-corner Coulomb response. Body coefficients combine by geometric mean;
an explicit pair coefficient replaces them. The response checks rational
bounds around a nonexact mean. `World.Step` requires full ideal and rounded
persistent-contact tracks and reports normal and tangent impulses. A centered
zero-slip box receives normal support with zero tangent impulse. `World.Step`
also consumes an interior first-impact bracket for a centered translating
source box above a fixed floor with zero restitution and positive friction.
The corrected four-corner touch supplies a bounded normal and tangent response;
ideal and rounded sweeps certify the remaining persistent contact.
An initially touching source sphere with exact centered isotropic mass can
receive a one-point Coulomb impulse from an identity-placed fixed box floor.
The response publishes tangent impulse and Y spin, then a rotating sphere
sweep certifies the full persistent point track and trace replay. A later
step can apply the same response while the sphere is already spinning.
Every advanced step reports bounded dynamic-body kinetic energy, linear momentum, and angular
momentum at input, after the force kick, and at completion, plus gravity,
center-force, torque, and fixed/kinematic contact impulses, plus bounded
kinematic driver work.
Before publishing an advanced step, isolated responses check each contact
event's dynamic-body linear momentum against its impulse and reject an impact
whose kinetic energy gain exceeds its driver work plus the computed numerical
allowance. The coupled sphere island and centered two-box stack check their
simultaneous impulses together. Other frictional contacts, other rotating
kinematic drivers, broader stacks, and payload paths remain design
contracts. Each companion document owns its detail.

| Design | Ownership |
|---|---|
| [Contact geometry](contact-geometry-design.md) | `Document.ContactPair`, pair relation, and certified contact manifold. |
| [Contact sweep](contact-sweep-design.md) | `Document.SweepPair`, two-body paths, continuous proof, and first-contact bracket. |
| [Dynamic mass](dynamic-mass-design.md) | `Body.MassProperties`, bounded center and inertia, and mass admission. |
| [Rigid dynamics](rigid-dynamics-design.md) | `dynamics.World`, state, stepping, response, and replay trace. |

## 1. System boundary

`decad` owns immutable solid geometry and the proof that a pair is separated,
touching, or overlapping. It also owns a read-only sweep that finds the first
certifiable contact interval for two moving bodies. The `dynamics` subpackage
owns body roles, material parameters, loads, velocity changes, and the state
produced by each step. It consumes `decad` pair reports and sweep certificates.
The current `Document.VerifyMotion` remains a prescribed-motion verification
API with no response calculation. A viewer such as `kinetograph` may replay a
dynamics trace; it does not decide contact or change the solver result.

The detailed designs define the report fields, input gates, units, refusal
reasons, and numerical algorithms. If this system map omits or conflicts with a
detailed rule, the owning design controls that rule.

## 2. Shared rules

- Queries and steps do not mutate a `Document`, `Body`, or evaluator payload.
  A step returns a new state and a trace; the caller decides whether to retain
  them.
- Geometry certifies its claims. A numerical response may use a certified
  manifold, but an approximate impulse cannot establish that contact occurred.
  An unresolved pair stops a step with an explicit result and interval.
- Clear, touching, and overlapping are distinct relations. Touching alone
  does not prove an incoming impact. A resting or sliding pair needs its own
  certified continuation across time.
- Every dynamic body has a positive mass and invertible inertia with proved
  bounds. `units.Value` carries physical scalar kinds; `decad.QuantityVec`
  carries vectors whose components share a kind.
- Time resolution, pose evaluation limits, and response iteration limits are
  explicit. Running out of proof or work budget never silently means clear.
- Physics admits proper rigid transforms. The first stage handles rigid
  solids; joints, deformation, and fracture are outside this design.

## 3. Required dependencies

The pinned `units` dependency provides Time, Velocity, Acceleration, AngularVelocity,
Force, Torque, Impulse, and AngularMomentum for the public sweep and step APIs. The pinned `r3`
dependency provides a symmetric tensor with rotation and inversion support;
inertia-based angular response still needs its dynamics integration. The dynamics API is a subpackage of
this module, so it does not need a separate module or a geometry dependency
pointing back to it.

## 4. First real integration slice

Implement source-box contact and a two-body sweep, then pass their real
reports into one dynamics step. The first fixture uses a fixed box with its
top at `z=0` and a dynamic `10×10×10 mm` box starting at `z=[10,20] mm`.
The dynamic box moves down at `100 mm/s`; restitution is `0.5`, gravity is
zero, and the step lasts `0.2 s`. Contact occurs at `0.1 s`. The box leaves
at `50 mm/s` upward and finishes with its bottom at `z=5 mm`. A fixture that
constructs a manifold by hand cannot validate the geometry-to-solver boundary.

The source-box proof must cover posed face contact, the immediate departure
after rebound, and persistent planar contact for later resting and sliding
fixtures. It must report when a feature transition ends its proof. The
dynamic path uses linear center-of-mass travel and angular drift; endpoint
screw interpolation is reserved for prescribed kinematic motion.

## 5. Delivery order

1. Use the pinned physical units and add the tensor operations needed for
   angular response, plus their focused tests.
2. Complete exact source-box contact and sweep, then run the real rebound
   fixture through `dynamics.Step` before expanding shape coverage.
3. Add independent movers, kinematic paths, friction, resting contact, and
   contact transitions. Keep every state transition behind a sweep or contact
   certificate.
4. Extend mass and contact proofs to supported curved and mesh payloads,
   using each evaluator's existing bounds and refusal rules.
5. Improve candidate filtering and work limits without changing certified
   outcomes. Add replay checks against the same returned trace consumed by a
   viewer.

Each stage has focused local tests for the contracts it changes. The first
end-to-end run must use the real producer and consumer before adding a wider
test matrix or more payload kinds.
