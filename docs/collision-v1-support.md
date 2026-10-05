# Collision and dynamics support

`Document.ContactPair` checks two posed solids and can return a certified
contact manifold. `Document.SweepPair` checks their paths over time.
`dynamics.World.Step` changes velocities only when those geometry queries
prove the contact and its continuation. An advanced step includes a state,
events, conservation readings, and a trace that `Trace.Sample` can replay.
[The box collision example](../examples/dynamics_box_collision_example_test.go)
shows a box impact and computed rebound.

## Certified shape pairs

| Pair | Admitted contact and motion | Step response |
|---|---|---|
| Source boxes | Face patches under affine motion; selected oblique and rotating paths | Rebound, rest, sliding friction, edge exit, selected spin |
| Source sphere and source box | Point strictly inside one box-face corridor; selected centered sphere rotation or orthogonal box pose | Frictionless rebound and fixed-floor Coulomb response |
| Two source spheres | Center-line point, affine center paths, centered rotating departure | Rebound, rest, graze, and admitted planar Coulomb impact |
| Full source cylinder and source box | Axial disk in a wide face; vertical extruded sidewall against a broad face | Centered frictionless rebound |
| Verified faceted Boolean and source-box floor | One exact rectangular lower face, vertical affine path, contained footprint | Frictionless rebound or rest |

The source-box path starts from rectangular source prisms. Signed-axis
face patches cover affine approach, persistent contact, and the first edge
exit. Selected co-oriented oblique faces, clipped horizontal patches, and
rotating drifts have separate proofs. Fixed-floor friction and selected
two-dynamic face impulses can change linear velocity and spin.
[Box step tests](../dynamics/step_test.go) and
[friction tests](../dynamics/friction_step_test.go) exercise these responses.

The source-sphere path uses a full semicircle source. A sphere-box point
must stay inside one complete box face. An exactly orthogonal rotated box
has a separate bounded face-point proof. A centered sphere can slide or
spin on a fixed floor when its rotating point track and rounded replay pass.
[Sphere-floor tests](../dynamics/sphere_floor_friction_test.go) exercise
contact, sweep, friction response, and trace.

Two source spheres use their recorded centers and radii for a bounded point
and an exact affine center-distance sweep. Frictionless paths include
off-axis impact, persistent touch, and an isolated zero-impulse graze.
Positive-friction off-axis impact requires two dynamic spheres with exact
centered isotropic supplied mass, no incoming spin, planar XY motion,
positive restitution, and an exact positive pair coefficient. Initial or
exactly timed interior touch can then change both velocities and Z spins.
[Off-axis friction tests](../dynamics/sphere_pair_offaxis_friction_test.go)
check the complete response and replay.

For a **separated rotating sphere pair**, `SweepPair` also proves a clear
path when each sphere rotates about its exact source center, its full
translation is exactly representable, and the exact center-distance path
stays strictly separated. It replays rounded poses. An off-center pivot or
possible touch returns `SweepUndecided` on this path. The
[rotating clear-sweep test](../contact_sphere_pair_sweep_test.go)
checks the pair query. A three-dynamic world also consumes this proof for
the clear outer pair after a coupled sphere impact and for a later event-free
spinning step. Its [integration test](../dynamics/three_body_dynamic_friction_test.go)
samples both traces.

The cylinder path accepts a full circular extrusion or a full revolve of
an axis-incident rectangle. A disk must stay inside a broad source-box
face under signed-axis, zero-spin translation. A vertical circular
**extrusion** also has a complete sidewall line against a broad box face.
The solver admits centered frictionless rebound with supplied or
density-derived mass. [Cylinder impact tests](../dynamics/cylinder_sidewall_impact_test.go)
pass real contact and sweep reports through the step and trace.

The faceted floor path requires one complete rectangular lower face with
an exact source plane and a live Face identity. The proof admits a zero-bound
Boolean, its translation-only placed copy with a saved exact source mesh,
or a positive-bound mesh `Union` that retains a source-box lower face while
the other operand's certified lower extent stays strictly above it. The
footprint stays inside a wide source-box floor; both paths translate equally
in X and Y, with zero spin. [Faceted floor tests](../dynamics/faceted_floor_step_test.go)
exercise density-backed impact, rest, and trace replay.

## Bodies and response

`NewWorld` admits two or more sound solids and lists every body pair in
canonical world order (`World.Pairs`), with an exclusion or a material
override per pair. `World.Step` resolves two- and three-body worlds with at
least one dynamic body; a world of four or more bodies builds its pair table
and validates its step input, and `World.Step` returns `dynamics.Undecided`
for it. Each dynamic body uses either density-derived mass and inertia or a complete
caller-supplied bounded record. Density-derived properties currently cover
source boxes, admitted untapered prisms under any frame or rigid placement,
full source spheres, qualifying revolved cylinders, and verified faceted
Booleans. Other payloads may need
supplied properties, but those properties cannot replace a missing contact
proof.

Two-body worlds admit a fixed or kinematic body against a dynamic body, or
two dynamic bodies. An affine kinematic box driver and selected cardinal
screws can push, depart from, or impact a dynamic source box when their
pair paths pass. [Kinematic impact tests](../dynamics/kinematic_impact_test.go)
check driver work and replay.

Three-body worlds can order isolated events when every other pair has a
certified clear path. Current simultaneous paths include two orthogonal
frictionless box-face contacts on one dynamic box, a frictionless sphere
touching a fixed box and sphere, and a zero-friction two-dynamic box stack
on a fixed floor. One dynamic source sphere can also stick with positive
friction to two fixed orthogonal source-box faces at initial touch: it needs
exact centered isotropic mass, no incoming spin, closing X and Z speeds,
Y slip, zero restitution, exact positive friction, and two certified
rotating persistent tracks. [Three-body friction tests](../dynamics/three_body_friction_island_test.go)
check both impulses and trace replay.

Three dynamic source spheres also admit one symmetric simultaneous
positive-friction impact at initial touch. Body zero touches the equal-radius
outer spheres along positive X and Y, while the outer pair stays clear.
All three need equal exact centered isotropic supplied mass, no initial spin
or Z motion, and exact point witnesses. After the force kick, body zero has
equal positive X and Y speeds, and both outer spheres are still. The two
pairs need equal exact positive friction and equal restitution strictly
between zero and one. A joint sticking solution must fit both Coulomb cones;
`MaxEvents` must exceed two. Both active pairs must certify rotating
departure, and the outer pair must certify a rotating clear path. The
[three-dynamic friction test](../dynamics/three_body_dynamic_friction_test.go)
checks both impulses, all three rounded pair paths, trace samples, and a
second event-free spinning step. Asymmetric speeds, unequal masses or radii,
sliding impulses, and unproved pair paths return `dynamics.Undecided`.

## When a proof stops

| Call | Result when the request exceeds its certified path |
|---|---|
| `ContactPair` | `ContactUndecided`, or a proved relation with no manifold and a reason such as `ContactNoNormalProof` |
| `SweepPair` | `SweepUndecided` with a cause and unresolved interval |
| `Body.MassProperties` | `decad.ErrUnsupported` when density-derived bounded mass or inertia is unavailable |
| `NewWorld` | `dynamics.ErrUnsupported` for fewer than two bodies, a two- or three-body world with no dynamic body, or a three-body kinematic world |
| `World.Step` | `dynamics.Undecided` with `Next == nil` when contact, response, or replay lacks proof, and for a world of four or more bodies |
| `Trace.Sample` | `dynamics.ErrUnsupported` if a rounded pose loses its cached proof; `dynamics.ErrInvalidInput` for time outside the step |

A generic positive-bound faceted body can prove strict clearance above a
floor without proving a touching support face. Sphere contacts at box edges,
tilted cylinders, revolved-cylinder sidewalls, and other rotating contacts
also stop at their first missing proof. The detailed gates and algorithms are
in the [collision design](collision-dynamics-design.md) and its linked
contact, sweep, mass, and rigid-dynamics designs.
