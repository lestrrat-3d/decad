# Dynamic Mass Design

This document owns mass, center-of-mass, and inertia readings used by rigid
dynamics. It supplies the mass gate in `docs/collision-dynamics-design.md` §2.
The source-box query is implemented; other payload paths remain design contracts.
`docs/evaluator-design.md` §4 owns
the existing planar area moments; this document owns the additional volume
moments and their use by dynamics.

## 1. Boundary and inputs

`decad` computes mass properties of its immutable `BodySolid` geometry for a
uniform density. `dynamics` chooses one of two explicit input modes per dynamic
body: density-derived properties or caller-supplied mass, center, and inertia.
The modes cannot be combined. Fixed and kinematic bodies need neither mode.
Multiple lumps and void shells contribute to one body's net mass properties;
they do not become separate dynamic bodies.

```go
func (b *Body) MassProperties(ctx context.Context, density units.Value) (MassProperties, error)

type MassProperties struct {
    Mass    Measurement        // units.Mass
    Center  VecMeasurement     // world position, millimetres
    Inertia InertiaReading      // about Center, in world axes
}

type InertiaReading struct {
    XX, YY, ZZ Measurement     // units.MomentOfInertia
    XY, XZ, YZ Measurement     // units.MomentOfInertia; signed products
}
```

Each `Measurement.Bound` is an absolute, nonnegative error of the same kind as
its value. `Center.Bound` is a length radius of a ball around `Center.Value`.
An `Exact` component has a zero bound; an inexact component is `Approximate`.
The six inertia components describe the symmetric tensor
`[[XX,XY,XZ],[XY,YY,YZ],[XZ,YZ,ZZ]]`. The products already include the
physical inertia sign: for example, `XY = -ρ∫(x-Cx)(y-Cy)dV`. Callers do not
negate them again. Each component has its own bound; one bound for all six
would hide which entries are certified tightly enough to invert.

`density` has kind `units.Density`, is finite, and is strictly positive. It is
the caller's stated uniform material parameter, not a measured interval.
Conversion from its registered unit to the base `kg/mm³` charges any numerical
rounding in the computed readings. A caller with uncertain physical density
must widen the returned readings itself or use a future bounded-density input;
the current signature cannot represent that uncertainty. The same density
applies to all material lumps and excludes all cavities.

`MassProperties` is a read-only query. It may read a retired body, as `Volume`
and `Centroid` do; the `dynamics.World` live-body gate is separate. A nil body,
a body without an evaluator payload, or a sheet is rejected. The method does
not call `Placed`, commit a body, consume a producer identity, or change the
document. An uncanceled repeated call on the same body and density produces
the same values, bounds, and error.

## 2. Geometric integrals

Choose a finite anchor `O` near the body's proven bounds, before integration.
For coordinates `q = x - O`, compute certified intervals for

```text
V       = ∫ dV
P_i     = ∫ q_i dV
Q_ij    = ∫ q_i q_j dV, i <= j
C       = O + P/V
S_ij    = Q_ij - P_i P_j/V
I_ij    = ρ (trace(S) δ_ij - S_ij)
M       = ρ V
```

`V` has kind Volume, `P` has Length × Volume, and `Q` has Length² × Volume.
`S` is the second volume moment about `C`; `I` has kind MomentOfInertia.
All interval operations round outward, including division and conversion to
the public `float64` readings. The body publishes no `MassProperties` unless
the certified volume interval has a strictly positive lower end. Signed
shell, cell, and lump contributions may cancel algebraically only after each
contribution has its own outward interval. An uncertainty term never cancels
because a neighboring term has the opposite sign.

The evaluator computes the integrals from its own admitted records. It does
not ask `sketch` to infer a missing cut or replace an uncertain curve with a
sampled one. The existing `Volume()` and `Centroid()` readings remain
independent public calls; the new query may reuse their certified internal
integrals when those carry every term needed for `P` and `Q`. It must not infer
`Q` from volume, centroid, `Bounds`, or a bounding box's inertia.

### 2.1 Analytic records

An untapered prism integrates each section's area, first moments, and second
moments over its signed axial interval. For a section at `(u,v)` and axial
coordinate `z`, `∫u²dV = (z1-z0)∫u²dA`, `∫uzdV =
(z1²-z0²)∫u dA/2`, and `∫z²dV = (z1³-z0³)A/3` before rotation into world
axes. The corresponding `v` and `uv` terms follow the same rule. Holes use
their recorded winding. `moments.go` already owns section moments through
second order; this path must reuse that validated record and its bounds.

A revolve requires section integrals through third polynomial order because
the cylindrical Jacobian contributes one radius and a transverse second
volume moment contributes two more. Extend the recorded-section moment engine
for `r³`, `r²z`, and `rz²`, then integrate the stated angular interval with
certified trigonometric intervals. Full turns may simplify terms by symmetry;
partial turns must retain their nonzero mixed terms. Circular and free-form
segments retain their existing exactness tiers and admission rules. A sphere,
cylinder, cone, or torus may use an equivalent closed-form primitive integral
only when it represents that payload's exact denotation, including its cuts,
holes, and placement.

Other analytic payloads may provide certified `V`, `P`, and `Q` directly.
Otherwise they use §2.2 if they have a suitable occupied-volume proof. A
payload with neither path returns `ErrUnsupported`; it does not borrow a nearby
primitive's formula merely because the boundary looks similar.

### 2.2 Faceted and curved records

For a truly faceted solid, audit its closed, consistently outward, embedded
boundary before integrating signed tetrahedra from `O` to its oriented
triangles. Exact arithmetic over held vertex coordinates yields each
tetrahedron's `V`, `P`, and `Q`, followed by outward conversion. A signed
tetrahedron sum alone is insufficient if the mesh self-intersects or fails
the solid topology audit. The faceted body may also carry a separate
displacement from its held triangles to its denoted solid; charge that
displacement through a certified occupied-volume error.

For a curved or approximated body, request a `VerifyAll` tessellation with
`BoundaryVerified()` and `VolumeVerified()`. Read its private certified
`volSymDiff = E`, where `E` bounds the occupied volume of
`TrueBody △ MeshSolid`. Both regions must lie within a proven radius `R`
about `O`, derived from the body's bounded `Bounds()`, the mesh vertices, and
their displacement bounds. Widen the mesh integrals by at least `E` for `V`,
`R E` for each `P_i`, and `R² E` for each `Q_ij`, plus outward arithmetic and
coordinate-construction terms. These are componentwise absolute bounds;
signed-volume closeness and a two-sided boundary distance alone do not imply
them. Refine the tessellation when permitted and useful. If the payload does
not publish an occupied-volume proof, or a finite `R` or finite moment bound
cannot be certified, return `ErrUnsupported`.

`VolumeVerified()` is a proof about the mesh and its source body, not a license
to use its triangles without the embedding and provenance audits. A render
mesh at `VerifyNone` or `VerifyBoundary` cannot provide density-derived
inertia. The same rule applies to placed faceted bodies and Boolean results:
their inherited occupied-volume allowances contribute to `E`.

## 3. Center, tensor, and numerical admission

Compute `S` with interval arithmetic from one common `V`, `P`, and `Q`
enclosure. Subtracting `P_iP_j/V` using point floats and attaching an error
afterward can lose the center-of-mass uncertainty. The near-body anchor limits
cancellation from a distant world origin; interval width still decides
whether a reading is usable. Center bounds include division by uncertain
volume and the anchor's coordinate or placement bound. Each inertia bound
includes density conversion, the `V/P/Q` intervals, center subtraction, and
the final tensor conversion.

Prove the entire admissible inertia interval positive definite before a
density-derived result is accepted for dynamics. A positive center tensor
alone is insufficient if its component bounds admit a singular tensor.
Use a certified lower eigenvalue bound or interval Cholesky test on the
six-component interval. Its lower bound must be strictly positive. A failed
proof may trigger tighter analytic arithmetic or a finer certified mesh;
exhausting a stated work limit returns `ErrUnsupported`. Inversion in
`dynamics` uses the certified interval, and the solver rejects a step if
resulting acceleration or impulse uncertainty cannot meet its residual gate.
All diagnostics identify whether volume positivity, a moment bound, or tensor
positivity blocked admission.

The parallel-axis rule is applied to each separate lump or primitive before
combining it about the body's common center: `I_at_C = I_at_c +
m[(d·d)1 - ddᵀ]`, where `d = c-C`. A cavity subtracts its `V`, `P`, and `Q`
before the body's final center and tensor are formed. A void is never a
negative-mass dynamic body. A disjoint-lump body with a common uniform density
returns one combined mass, center, and tensor.

## 4. Placement and supplied properties

The returned center and inertia refer to the body's **current world
placement**. For a later rigid pose `T`, transform the center with `T.Apply`
and rotate the centroidal tensor as `R I Rᵀ` using `T`'s orthogonal basis.
Translation leaves centroidal inertia unchanged. Reflection follows the same
tensor rule; it does not change mass. Carry interval bounds through each
basis product and its rounding. `Body.PlacedCopy` or `Body.Placed` must agree
with that transformed reading within both reported bounds. The dynamics state
applies its pose to the body's current placement exactly once.

The caller-supplied mode in `dynamics` contains a positive bounded `Mass`
reading, a bounded `Center` world position at the body's current placement,
and six bounded `Inertia` readings about that center in current world axes.
It is not an unlabelled diagonal or an inertia about the origin. All six
components and their bounds have kind `units.MomentOfInertia`; the mass value
and bound have kind `units.Mass`; the center bound has kind `units.Length`.
The lower mass endpoint and certified lower tensor eigenvalue are strictly
positive. The mode is explicit, so a zero density value never means “use the
supplied tensor.” The caller may use measured values that differ from the
geometry-derived values; dynamics reports that input mode in its world
description and does not silently replace them with geometric readings.

`r3` needs a symmetric 3×3 numerical tensor with rotation by an orthogonal
transform, positive-definite validation, inversion, and vector action.
`decad` and `dynamics` use those operations rather than duplicating coordinate
algebra. The public six-component readings stay typed `units.Value`; `r3`'s
numerical tensor is an internal arithmetic representation, not a public
physical quantity.

## 5. Refusals and cancellation

Validation runs before reading `ctx`. A nil or payload-less receiver returns
`ErrDegenerate` or `ErrUnsupported`, respectively. A non-solid receiver
returns `ErrNotSolid`. Wrong density kind returns `ErrUnitKind`; non-finite
density returns `ErrNotFinite`; negative density returns
`ErrNegativeMagnitude`; zero density returns `ErrDegenerate`. A valid but
unsupported payload, absent occupied-volume certificate, interval volume
containing zero, or unproved positive tensor returns `ErrUnsupported` with
the specific cause. Arithmetic overflow that prevents a finite computed
reading returns `ErrNotFinite`. No failure returns partial properties.

After validation, every potentially long integration, tessellation,
refinement, and proof loop polls `ctx`. Cancellation returns `ctx.Err()` and
zero properties, including when cancellation arrives before publication.
The query leaves the document and body's geometric measurements unchanged on
success, refusal, and cancellation. `dynamics` validates caller-supplied
readings before a step; wrong kinds and non-finite values use its typed input
errors, and a mass or inertia interval that is not provably positive uses
`ErrInvalidMassProperties`.

## 6. Dependencies and computed checks

The pinned `units` version in `go.mod` already defines `Mass`, `Density`, and
`MomentOfInertia`. The time-based `dynamics` API separately needs the planned
`units` upgrade; this document does not require time for the query itself.
The pinned `r3` version has rigid transforms but no symmetric-tensor type.
Add that capability to `r3` and pin a release before dynamics uses it. Neither
change is made in a sibling repository by this design.

Computed tests must cover these results, with independent formulas or
quadrature and assertions against the published bounds:

| Body and input | Expected geometric result |
|---|---|
| `a × b × c` box with density `ρ` | `M = ρabc`; center at box midpoint; diagonal inertia `M(b²+c²)/12`, `M(a²+c²)/12`, `M(a²+b²)/12`; zero mixed terms in aligned axes. |
| Same box after a non-axis-aligned rigid rotation and translation | Mass unchanged; center transformed; all six components enclosed by `R I Rᵀ`; translation adds no centroidal inertia. |
| Full solid cylinder, radius `r`, height `h` | Axial inertia `Mr²/2`; transverse inertia `M(3r²+h²)/12`; bounds enclose the independent π-based values. |
| Partial revolve with an off-axis section | At least one nonzero mixed component agrees with independently integrated `r³`, `r²z`, and `rz²` terms. |
| Two separated equal boxes in one body | Common-center inertia includes the `m d²` parallel-axis contribution; no per-lump tensor is returned as the body tensor. |
| Hollow solid with a centered cavity | Mass and tensor equal outer solid minus cavity integrals; cavity material contributes no positive mass. |
| Verified faceted box and a curved body at coarse and fine tessellation | True mass and every true tensor component lie in both reported intervals; refined proof narrows at least one bound. |
| Curved body without occupied-volume proof | Query returns `ErrUnsupported`; a render mesh never supplies an inertia estimate. |
| Bad density, sheet, cancellation, and ill-conditioned tensor interval | Each returns the specified typed refusal and publishes no partial properties. |

The first end-to-end dynamics check uses a real decad box, its density-derived
mass and inertia, a real pair sweep, and the real contact solver. Supplying a
hand-written tensor or contact to that check would not exercise this seam.
