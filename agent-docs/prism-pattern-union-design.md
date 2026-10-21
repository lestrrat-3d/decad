# Circular prism and copied-section Union

## Admission

`Union(blank, section, copies...)` takes one centered circular XY prism and
two to 32 axial extrusions from one recorded section. All bodies share the
same positive interval starting at Z = 0. The blank has one circle and no
holes. Each section has no holes, no section or axial displacement, a final
centered inward root arc, and rigid radial `Copy` placement without
translation or reflection. Copies retain the first section's exact source
record identity. A separately drawn similar section does not enter this
path.

The root arc's denoted radius comes from its recorded start. Its recorded
end can differ slightly, so both endpoints must have squared radius
strictly below the blank's squared radius after multiplying by the copy's
exact dyadic XY scale squared. A tangent or outward root refuses before a
joined shell is built. Every other boundary curve stays outside the blank:
lines and fitted-spline Bezier spans use a strict support line for all their
controls; centered arcs use their denoted radius. The two lines next to the
root start inside and end outside the circle, with a positive radial
derivative at the inner end. These tests prove each connector has one
circle crossing and the root arc enters the blank.

Each section's `Tessellate(VerifyAll)` mesh supplies its source-to-held
boundary and occupied-volume bounds. A candidate separating line through
the difference of two held root endpoint sums must leave every held vertex
of one section strictly beyond every held vertex of the other, after
charging both mesh bounds times the line's exact length. The boundary and
therefore each filled section lie in their separate halfplanes. Every pair
must pass. The root endpoint angle intervals must leave an ordered gap
between adjacent copies.

## Joined shell

The build keeps each section's exterior wall facets and omits its inward
root wall. It connects consecutive root endpoints with a wall sampled on
the blank circle at steps no wider than 0.02 radians. Endpoint angles use
`Atan2Interval`, and interior points charge both their angle interpolation
and `RadSinCosInterval` rounding. The two caps triangulate the same ordered
exterior and circle-gap outline used by the walls.

`tessellation.OrientJoinedMesh` proves every edge pairs and the result has one orientable
component. The bounded exact triangle crossing audit checks the complete
shell. Each cap audit proves a simple counterclockwise outline containing
the axis and an exact tiling by its cap triangles. A separate exact
point-in-polygon and edge-contact check keeps every omitted held root chord
strictly inside both cap outlines, except for its two shared endpoints.
Any failed check returns `ErrUnsupported` and leaves all inputs live.

## Boundary and occupied volume

For a connector from inner point `q` along step `d`, squared radius grows
at first derivative `2 q·d > 0`. Its unique circle crossing parameter is
at most `(R²−|q|²)/(2 q·d)`. The copy's exact XY scale enters both terms;
the crossing distance multiplies that parameter bound by the scaled step
length. Nonconnector held wall triangles must remain outside
`R + mesh.Bound()` by their exact minimum projected radius.

The seam band adds the source root-radius gap, both connector crossing
reaches, any held root-vertex excess, circle-chord sagitta, and ring-point
rounding. The joined boundary bound adds that band to the largest section
mesh bound and must be at most 0.1 mm. The complete ring and connector
changes lie in the symmetric annulus from `R−band` to `R+band`; admission
requires `band < R`. Its occupied volume is at most `4πR·band·height`.
The output's symmetric-difference bound adds that annulus volume to each
source section mesh's already verified occupied-volume bound. Its area
slack includes a perturbed area upper bound for the joined shell and upper
areas of the input faces. The two cap groups retain the blank's planar
surfaces and receive the result's own cap roles; other faces are faceted.

## Axial edge selection

The authenticated inputs and their joined shell are one XY section swept
through a common Z interval. The result records this source proof. A faceted
edge chain receives a Z direction for `ParallelTo` only when every held
chord has exactly fixed X/Y and moves strictly in one Z direction. Bent,
stationary, reversed, and unproved faceted chains have no selector direction.
The edge remains a bounded `FacetedCurve`; this certificate does not make its
held XY position exact. A rigid placement drops the source-specific axial
selector certificate until that placed result has its own proof.

## Local root Fillet

The joined body retains its authenticated circular blank, copied section
records, source meshes and audited shell. `Fillet` admits selected
source-root axial chains on that body. Other faceted chains and placed joins
refuse. The radius must have zero input displacement.

For each selected corner, exact dyadic connector and placement coordinates
define an outward line. Its material-side offset by the requested radius
meets the circle of radius `R+r` at the outward quadratic root. Rational
interval arithmetic brackets the center, the line foot and the circle foot.
The line foot must lie beyond the blank and before the next source vertex;
the circle foot must lie in a strictly bracketed shared-ring step. The bracket
may follow several consumed ring stations. Held station order and source
circle station order must both be certified, including ring-point rounding.
The entire tangent arc fits within `3r` of the source root. The source control-hull
certificate keeps every nonconnector wall beyond `R+2r`, the farthest radial
reach of the whole fillet circle. Pinned source arcs charge their radial
residual. The exact contact center, line foot and source root define a narrow
polar sector for the connector cutback, circular-wall cutback and short
fillet arc. The circle's tangent cone bounds the complete arc. Sectors of
selected contacts and all other copied connectors must be strictly separate
modulo a full turn. The connector belonging to each selected root is the only
exception. Sector overlap or an ambiguous angle refuses.

The shell moves each selected connector endpoint to its line foot, removes
the ring facet pairs before the circle foot, and moves the near endpoint of
the retained bracket pair to that foot. It removes the consumed stations
from both cap outlines, then joins the feet with a sampled circular wall.
Overlapping cuts and missing facet pairs refuse. The edge retains its bounded
faceted type. Endpoint angle intervals,
trigonometric intervals and chord sagitta bound every new arc station and
facet. The rebuilt shell must be connected and orientable with paired edges,
pass the full exact triangle-crossing audit, tile both caps exactly, and
keep the omitted source-root chords inside its caps.

Unchanged source and ring walls keep the Union boundary bound. The new
boundary bound adds the arc station and sagitta allowance and must stay
within 0.1 mm. The occupied-volume certificate sums two complete `3r`
cylindrical disks per selected root over the old Union certificate, enlarging
the held disk by the old boundary and arc bounds. These disks may overlap;
the sum remains an upper bound. The area allowance charges both old and new
walls and caps in those disks. `Tessellate(VerifyAll)` checks the detached
result before Fillet commits and retires its input.
