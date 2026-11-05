# Shell variants design

One part per Shell route that has its own construction. Each part keeps its own decision IDs and `§` numbers; cite a `§` number as "Part name §N". The general opening route is `agent-docs/shell-opening-design.md`.

## Faceted boss

This route shells the complete top cap of one ordinary two-prism mesh Union.
It covers a rectangular plate and a strictly nested rectangular boss whose
horizontal placement rotates and may carry the stored similarity scale. It
rebuilds a faceted cup from the Union's authenticated sources and held mesh.

### Admission

The Union must retain its two original `prismPayload` sources and remain
unplaced. The plate is an exact positive XY rectangular prism with identity
placement. The boss is another exact positive XY rectangular prism. Its
placement has horizontal basis `(c,s),(-s,c)` and unchanged positive Z;
`sqrt(c²+s²)` is enclosed within `[0.99,1.01]`. No source section or level
displacement is admitted. The selected face must be exactly the boss's
`CapEnd`: one group of two triangles, four source corners, and no other
fragment of that cap. All four true boss corners, widened by their source
point bounds, must lie strictly inside the plate's inset by thickness `t`.

The boss bottom is exactly `t` below the plate top. The plate floor below it
and the boss's two horizontal dimensions must leave positive material after
the offset. All levels and offset coordinates used in the reconstruction
must be represented exactly where the route does not carry a numeric bound.
Other Union shapes and placed Union results keep the ordinary Shell refusal.

### Cavity and held boundary

Write `L=sqrt(c²+s²)`, and let `W,H` be the boss's local rectangle widths.
The true boss width and height are `LW,LH`. A world-distance offset `q`
therefore becomes a local inset `q/L`. The cavity has a plate-inset floor at
`z_plate_bottom+t`, lower walls up to `z_boss_bottom`, a ledge from the plate
inset to the boss footprint, the rounded root band up to the plate top,
upper boss-inset walls, and a rim around the removed top cap.

The root edge, the boss outline at the plate top, is a concave edge of the
source, so the reflex-corner row of `agent-docs/modify-design.md` §8 applies
to it: the cavity keeps distance `t` from the root edge. On the root
interval, with `s=z-z_boss_bottom`, the true world inset is

```text
q(s) = sqrt(t² - (t-s)²),       0 ≤ s ≤ t.
```

In the `(q,s)` plane this is the quarter circle of radius `t` centred on the
root edge at `(0,t)`. It leaves the ledge's boss-outline ring at `(0,0)`
tangent to the ledge and meets the upper inset walls at `(t,t)` tangent to
them. Each section is the boss rectangle inset by `q(s)` with sharp corners,
so adjacent sides meet on mitred diagonal seams.

The held band uses 16 stations at `w=k/16` with rational coordinates
`q=2tw/(1+w²)` and `s=2tw²/(1+w²)`. The square-root bracket for `L` and the
rational-to-float station error enter each new vertex's source point bound.
The planar curve `(q(w),s(w))` has second-derivative norm at most `4t`.
Linear interpolation at the same `w` across one station cell differs by at
most `t/(2·16²)`. Each rectangle corner moves in two inward coordinates,
so the payload charges `3t/(4·16²)` for the curved band. Parameter matching
allows Z to move near the horizontal tangent at `w=0`. The root volume
bound uses swept surface area rather than a same-Z inset difference. The
source Union mesh loses only the selected top cap triangles. Its other facets
and vertex bounds remain unchanged.

The held mesh is checked for positive facet area, exact facet contact,
directed-edge closure, components, and signed volume before the new body is
built. Strict source containment and the monotone nested rectangle sections
keep the new cavity apart from the exterior. A failed audit leaves the Union
live.

### Occupied volume and readings

The true outer volume is plate area times plate height plus `L²WH` times the
boss height above the plate. The lower and upper cavity prisms have ordinary
rectangle volumes. Integrating the root rectangle area gives

```text
V_root = WHL²t - (W+H)Lπt²/2 + 8t³/3.
```

The code evaluates this expression as a rational interval with an outward
interval for `L` and `π`. Its held signed volume must lie inside the stated
occupied-volume allowance. The allowance adds the source Union's symmetric
difference bound, a root-surface sweep term below four times perimeter times
thickness times the chord and station gap, an upper-wall station term, and a
level-slab term. These
terms bound the actual symmetric difference; the final volume comparison is
an independent consistency gate. The faceted result carries the maximum
source and cavity boundary displacement. Its area slack includes both sides
of a conservative root-patch area bound.

## Twisted sweep

`Shell` admits an inward, one-cap opening on a one-line twisted solid whose
Sketch section is a smooth, centrally symmetric, strictly convex sequence of
whole Lines and Arcs. The path starts at the origin, follows positive Z, and
uses the origin XY profile plane. The opening must select `capEnd`. Thickness
must have an exact millimetre value, leave a positive floor, and pass the
bounded offset and mesh proofs below. The result has one outer wall, one inner
wall, a bottom cap, a cavity floor, and a top rim. Other twisted Sweep Shell
requests return `ErrUnsupported` without retiring the receiver.

### Denoted offset

Let `p(s)` follow the section counterclockwise at unit speed, with tangent
`T`, inward normal `n = J T`, curvature `κ`, and `J` the quarter-turn. At
height `z`, the source wall is `R(kz)p(s)`, where `k = θ/H`. Put
`f = p·T`, `h = p·n`, `g = sqrt(1+k²f²)`, `a = t/g`, and `β = k²tf/g`.
Moving the source wall inward by its unit surface normal and then naming
the result by its own height gives the cavity contour
`q(s) = R(β)(p(s)+a n(s))`. The cavity wall is `R(kz)q(s)` for
`t ≤ z ≤ H`; its floor is at `z=t`. This curve remains denoted by the
authenticated Lines and Arcs. It is never replaced by the stored polygon.

On each source segment, interval arithmetic encloses `f`, `h`, `κ`, the
source length and the denoted angle. The derivative is
`q' = λ R(β)T`, with
`λ = 1-aκ-β'(h+a)` and `β' = k²t(1+κh)/g³`.
Admission requires positive lower bounds for `λ`, `κ+β'`, and the inward
support gap `a-R|β|`, where `R` encloses the source radius. It also requires
smooth source joins and positive contour area. These checks exclude a cusp,
a reversed offset, and contact with the outer contour in the admitted
class. Half-turn symmetry gives zero first moments for the cavity section.
The Green integral encloses its area by summing
`½ λ(-h-a) ds` over 64 bounded parts of each source edge. The true cup
volume lies in `H*A(P) - (H-t)*A(Q)`, with both areas enclosed by the source
certificates.

### Held boundary and certificates

Each Line contributes eight contour cells and each Arc contributes sixteen.
The builder encloses every source and offset station with rational
trigonometric intervals. A second-derivative bound gives the offset's
maximum chord departure on each cell; the source Arc uses its circular
sagitta. Eight equal-height stations rotate paired outer and inner
contours. The held shell has two triangles per contour cell and axial cell
on each wall, a fan on the outside bottom and inside floor, and a paired
top annulus. It keeps the outer and inner wall as separate faces.

The wall boundary bound adds contour chord departure, angular interpolation
`R*step²/8`, the diagonal's `edge*step/4` displacement, and outward-rounded
station error. Planar caps and rim use their contour chord and vertex errors.
For each wall cell, a straight homotopy to its held triangles moves points
by at most its boundary bound. Its swept-volume allowance is that bound
times the sums of certified source and held contour lengths and axial
travel lengths. The build adds every nonnegative cell allowance. Planar
bottom, floor, and rim homotopies sweep zero occupied volume. The enclosed
true volume must be positive and lie within the resulting held-volume
allowance.

Each source wall cell and held triangle receives a non-cancelling area
allowance. The planar patches contribute at most `2*A(P)` of source area;
the held triangles contribute their outward area upper bounds. This coarse
sum remains valid when a later operation keeps only part of one face.
The builder audits held facet crossings, edge use, vertex links, and
orientation before it commits. `Tessellate(VerifyAll)` checks the stored
boundary and occupied-volume certificates at the requested tolerance.

`Copy` and `Transform` can place the source Sweep under a rigid motion. Shell
first certifies the offset cup in the Sweep's local profile and path frame.
The stored floating-point rotation has a small exact metric defect, so an
affine image of a local unit-normal offset is not exactly the world-normal
offset. The exact stored matrix supplies `e = ||LᵀL-I|| < 1/2`. Its normal
map differs from the local unit normal by at most `4e/(1-e)` and its tangent
map by at most `100e/(1-e)³`. The certificate compares the two cavity walls
at each height. It requires the height map to remain increasing, the new
contours to keep their positive radial winding, the outer-wall clearance to
remain positive, and the world-offset floor to stay between the two caps.
The floor's local height differs from `t` by at most `te/(1-e)`.

The maximum contour movement plus the moved floor bounds boundary distance.
The contour homotopy's swept area bounds each section's symmetric difference;
its height integral and the floor strip bound occupied-volume difference.
Separate absolute wall-density, floor, and rim terms bound area difference
without cancellation. These three charges augment the local cup proof before
the ordinary faceted placement path moves the held vertices, audits contact
and orientation, and adds its own rounding, area, and volume charges. A proof
refusal leaves the source Sweep and document unchanged. `capEnd` remains
selectable on the placed Sweep.

The public gate uses a solved 8×6 mm rounded rectangle with 1.5 mm corners,
a 5 mm path, 30° twist, and 1 mm inward Shell thickness. Its 3,456-triangle
cup reports a 0.015314 mm boundary bound and a 16.076 mm³ volume allowance;
`Tessellate(0.1 mm, VerifyAll)` reports boundary and volume verified.
A 0.01 mm request refuses at the same fixed mesh. Starting-cap, outward,
and over-thick calls also refuse without changing the document.
The placed public gate applies a 23° rotation about `(1,2,3)` followed by a
`(9,-4,7)` mm translation. Both `Copy` and `Transform` Sweep sources select
one `capEnd` and shell successfully. Their cups pass `VerifyAll` at 0.1 mm
with a boundary bound below 0.015314 mm and a volume allowance below
16.076 mm³.
