# Collision and dynamics support

`Document.ContactPair` checks two posed solids and can return a certified
contact manifold. `Document.SweepPair` checks their paths over time.
`Document.SweptBox` encloses one body's whole path in an exact box, and
`SweptBox.StrictlyDisjoint` proves two such paths clear without a sweep.
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
| Full source cylinder sidewall, against a planar face or a parallel full source cylinder | Two-end ruling line at identity query poses only | None yet |
| Two exact planar solids | Gap, touch, or overlap at any pose with a positive-determinant basis; no manifold | None yet |

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

A full source cylinder's sidewall, including a revolved cylinder on its
side, uses the clearance kernel's ruling certificate at identity query poses.
Its carriers must be exact: an unplaced full revolve or extruded circle,
sketched on an XY plane. It must touch a planar face along its whole tangent
ruling, or touch a parallel full source cylinder with positive axial overlap.
The manifold publishes the ruling's two exact ends with the faces'
`Face.NormalAt` normals. No sweep or step consumes this contact yet. The
[ruling manifold tests](../contact_analytic_manifold_test.go) check both
contacts and their refusals.

The faceted floor path requires one complete rectangular lower face with
an exact source plane and a live Face identity. The proof admits a zero-bound
Boolean, its translation-only placed copy with a saved exact source mesh,
or a positive-bound mesh `Union` that retains a source-box lower face while
the other operand's certified lower extent stays strictly above it. The
footprint stays inside a wide source-box floor; both paths translate equally
in X and Y, with zero spin. [Faceted floor tests](../dynamics/faceted_floor_step_test.go)
exercise density-backed impact, rest, and trace replay.

Two exact planar solids are prisms over straight-edged sections with no
recorded displacement, or zero-bound faceted Booleans, directly or after a
translation-only `Placed`. `ContactPair` proves their relation under any
rotation: an enclosed gap, a touch at a vertex, edge or face, or an overlap,
including one body nested in the other's material. It publishes no manifold
yet, so no step response uses it. The report names `ContactNonConvex` when
neither body is convex. A touch it cannot resolve locally, such as two
flush bodies meeting at a saddle point, stays `ContactUndecided`.
[Planar pair tests](../contact_faceted_pair_test.go) check each relation.

## Bodies and response

`NewWorld` admits two or more sound solids and lists every body pair in
canonical world order (`World.Pairs`), with an exclusion or a material
override per pair. `World.Step` resolves two- and three-body worlds with at
least one dynamic body. Each dynamic body uses either density-derived mass and inertia or a complete
caller-supplied bounded record. Density-derived properties currently cover
source boxes, admitted untapered prisms under any frame or rigid placement,
full source spheres, qualifying revolved cylinders, full or partial revolves
about an exact in-plane axis, sweeps and cups built from those prisms and
revolves, and verified faceted Booleans. Every other solid
whose `VerifyAll` mesh carries an occupied-volume proof is integrated over
that mesh: solid lofts, unplaced exact stitched solids, and sweeps, cups and
revolves the analytic paths refuse, refined until the tensor interval proves positive. A
placed or certificate-welded stitched solid, and any payload whose mesh
carries no such proof, returns `decad.ErrUnsupported`. Those payloads may need
supplied properties, but those properties cannot replace a missing contact
proof.

A world of four or more bodies, in any role mix, kicks every dynamic body
once and moves every body along its drift or driver over the step. Its broad
phase sweeps only the pairs whose `SweptBox` values are not strictly
disjoint, and the step advances only when every swept pair is proved clear;
a contact event returns `dynamics.Undecided` with `StepUnsupported`. Its
kinematic bodies take `PoseSegment` drivers.
[Schedule tests](../dynamics/schedule_test.go) check the swept pairs, the
drift poses and trace samples.

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
| `SweptBox` | `decad.ErrUnsupported` when the body's bounds or the path's travel bound are not finite |
| `Body.MassProperties` | `decad.ErrUnsupported` when density-derived bounded mass or inertia is unavailable |
| `NewWorld` | `dynamics.ErrUnsupported` for fewer than two bodies, a two- or three-body world with no dynamic body, or a three-body kinematic world |
| `World.Step` | `dynamics.Undecided` with `Next == nil` when contact, response, or replay lacks proof, and for any contact event in a world of four or more bodies |
| `Trace.Sample` | `dynamics.ErrUnsupported` if a rounded pose loses its cached proof; `dynamics.ErrInvalidInput` for time outside the step |

A generic positive-bound faceted body can prove strict clearance above a
floor without proving a touching support face. Sphere contacts at box edges,
tilted cylinders, posed or swept cylinders on their sides, and other rotating
contacts also stop at their first missing proof. The detailed gates and algorithms are
in the [collision design](collision-dynamics-design.md) and its linked
contact, sweep, mass, and rigid-dynamics designs.
