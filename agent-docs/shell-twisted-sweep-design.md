# Shell of a straight twisted Sweep

`Shell` admits an inward, one-cap opening on a one-line twisted solid whose
Sketch section is a smooth, centrally symmetric, strictly convex sequence of
whole Lines and Arcs. The path starts at the origin, follows positive Z, and
uses the origin XY profile plane. The opening must select `capEnd`. Thickness
must have an exact millimetre value, leave a positive floor, and pass the
bounded offset and mesh proofs below. The result has one outer wall, one inner
wall, a bottom cap, a cavity floor, and a top rim. Other twisted Sweep Shell
requests return `ErrUnsupported` without retiring the receiver.

## Denoted offset

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

## Held boundary and certificates

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
