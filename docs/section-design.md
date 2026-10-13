# Planar section design

`Body.Section` reads a solid at a requested plane and returns a new Sketch whose
profiles denote the material in that plane. It does not change the body or its
document. The returned Sketch may be passed to `MeasureProfile`, `Extrude`, or
another feature that consumes a Sketch profile.

## Public result

```go
func (b *Body) Section(ctx context.Context, plane r3.Frame) (*sketch.Sketch, []*sketch.Profile, error)
```

The returned Sketch uses `plane` as its construction frame. The profiles are
fresh, valid Sketch snapshots; changing the Sketch makes them stale under the
ordinary seam rules. A later body class may return several regions, so the
result is a slice even though the admitted prism carries one material region.
The caller may read a retired body. A nil body or context, or an invalid frame,
is `ErrDegenerate`; a sheet or unsound solid is `ErrNotSolid`.

## Parallel exact prism class

The parallel class admits a `prismPayload` with zero `sectionDelta`,
`z0Delta`, and `z1Delta`. It refuses a plane that touches either cap. A body
whose payload is not this prism class returns `ErrUnsupported` without
changing the document.

The gate reads the stored source frame and rigid placement as exact dyadic
values. It proves that the placed images of source `U`, `V`, and `N` equal the
requested frame's `U`, `V`, and `N`, component by component. It proves that the
placed source origin differs from the requested origin by a scalar multiple of
the requested normal. The scalar gives the requested sweep level, which must
lie strictly between the recorded sweep levels. These are exact comparisons
on stored values, not a parallelism or distance tolerance. A placed body is
admitted when its transform passes the same identities.

For every admitted level the section is the prism's recorded profile, including
its holes. No 3D edge intersection is needed. Reconstruct its entities on a new
Sketch at the requested frame. Ask Sketch for its profiles, record the fresh
boundary through the seam's `TExact` checks, then require its segments to match
the prism record. A source segment that cannot be recreated or whose
trim Sketch does not certify returns `ErrUnsupported`. A reconstructed profile
that does not match the source never becomes a section result.

## Cardinal vertical plane of an exact cylinder

The second class reads an exact straight prism whose authenticated source is
one whole, counterclockwise `CircleSeg` with positive radius and no holes.
The source section and both cap levels carry zero displacement. The exact
placed images of the source U, V and normal must each be a signed world
coordinate axis; an image that does not have unit length and cardinal direction
returns `ErrUnsupported`.

The requested plane's V axis must equal either sense of the placed sweep
axis. Its U axis must equal either sense of one placed source radial axis.
The requested normal must equal the exact U×V cross product. The plane may
be offset from the placed circle centre's sweep axis. Compute its signed
normal offset `d` in exact dyadic arithmetic. For a recorded radius `r`,
require `q = r² − d² > 0`. At zero offset, retain the recorded radius as the
half-chord. At nonzero offset, use the downward dyadic square root and an
exact square comparison to require a binary64 half-chord `h` with `h² = q`.
A tangent plane, an exterior plane, a chord without such an exact `h`, or a
frame that fails the cardinal equalities returns `ErrUnsupported`.

The source circle denotes a disk; sweeping it through the recorded interval
denotes disk × interval. The admitted plane cuts the disk on a chord,
so the section is the Cartesian product of the exact interval
`[uCenter − h, uCenter + h]` and the two recorded cap levels. Compute the
four corners in the requested frame using
exact dyadic arithmetic. A corner whose local coordinate is not finite and
exactly representable as `float64` returns `ErrUnsupported`; no rounded
intersection is published as an exact Sketch line. Reconstruct the four-line
profile on the requested plane through Sketch and require its fresh recorded
profile to match that exact rectangle. The parallel class keeps its existing
circle result and its cap-plane refusal.

## Cardinal vertical plane of an exact rectangular prism

This class admits a hole-free exact straight prism whose source profile has
four complete lines forming one axis-aligned rectangle in its own section
coordinates. A blend record, nonzero section or cap displacement, or a
nonpositive sweep interval returns `ErrUnsupported`.

Prove with exact dyadic arithmetic that the placed source U, V, and sweep axes
are signed world coordinate axes. The requested V must equal either sense of
the sweep axis; its U must equal either sense of one source radial axis; its
normal must equal their cross product. Compute the plane's signed normal
coordinate at opposite rectangle corners and require strict opposite signs.
This refuses tangent and exterior planes without a distance tolerance.

The section is the product of the uncut source radial interval and the sweep
interval. Derive both local interval ends from exact images of the source
corners and cap levels. Refuse any corner coordinate that is not finite and exactly
representable as `float64`. Reconstruct the resulting four-line profile in a
fresh Sketch and require Sketch's certified record to match before returning
it. The parallel prism and cylinder classes keep their existing gates.

## Verification

- Extrude a 10×10 mm square by 5 mm. Section at 2.5 mm must give one 100 mm²
  profile. Extruding that returned Sketch/profile by 2 mm gives 200 mm³ and
  passes `Tessellate(VerifyAll)`.
- A rigidly translated prism must section on the correspondingly translated
  plane. A source profile with a hole must return the same outer and hole
  boundaries when Sketch can certify their reconstruction.
- Extrude a 1 mm whole circle by 5 mm. Section it by a cardinal plane through
  its axis and check the exact 2×5 mm rectangle. Re-extrude the returned
  profile by 2 mm and require `Tessellate(0.1 mm, VerifyAll)` to certify the
  resulting 20 mm³ prism. Check a translated cardinal cylinder too.
- Extrude a 5 mm whole circle by 10 mm. Section it by a cardinal plane 3 mm
  from its axis and check the exact 8×10 mm rectangle. Re-extrude the returned
  profile by 2 mm and require `Tessellate(0.1 mm, VerifyAll)` to certify the
  resulting 160 mm³ prism. Check another exact chord, both offset signs, and
  a rigidly placed cylinder.
- Extrude a 10×10 mm rectangle by 5 mm. Its cardinal vertical section through
  the interior is 50 mm². Extrude the returned Sketch/profile by 2 mm and
  require 100 mm³ and `Tessellate(0.1 mm, VerifyAll)`.
- Check a translated, quarter-turned rectangular prism with both radial axes
  and a reversed sweep axis. Refuse tangent, exterior, noncardinal and tapered
  rectangular cases.
- Refuse a tangent or exterior cardinal plane, a positive `q` whose square
  root is not exactly binary64, a non-cardinal plane, and a radius or frame
  whose derived rectangle corner cannot be held exactly.
- A cap plane, oblique plane, sheet body, invalid input, and
  canceled context must return their stated errors without changing the
  body's document.
