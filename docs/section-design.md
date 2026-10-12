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

## Cardinal axis plane of an exact cylinder

The second class reads an exact straight prism whose authenticated source is
one whole, counterclockwise `CircleSeg` with positive radius and no holes.
The source section and both cap levels carry zero displacement. The exact
placed images of the source U, V and normal must each be a signed world
coordinate axis; an image that does not have unit length and cardinal direction
returns `ErrUnsupported`.

The requested plane's V axis must equal either sense of the placed sweep
axis. Its U axis must equal either sense of one placed source radial axis.
The requested normal must equal the exact U×V cross product. The plane
must contain the placed circle centre's sweep axis: the exact dot product
of its normal with the displacement to that centre is zero. A plane offset
from the axis or a frame that fails any equality returns `ErrUnsupported`.

The source circle denotes a disk; sweeping it through the recorded interval
denotes disk × interval. The admitted plane cuts the disk on a diameter,
so the section is the Cartesian product of a radius interval and the two
recorded cap levels. Compute the four corners in the requested frame using
exact dyadic arithmetic. A corner whose local coordinate is not finite and
exactly representable as `float64` returns `ErrUnsupported`; no rounded
intersection is published as an exact Sketch line. Reconstruct the four-line
profile on the requested plane through Sketch and require its fresh recorded
profile to match that exact rectangle. The parallel class keeps its existing
circle result and its cap-plane refusal.

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
- Refuse a parallel-offset cardinal plane, a non-cardinal axis plane, and a
  radius or frame whose derived rectangle corner cannot be held exactly.
- A cap plane, oblique plane, sheet body, invalid input, and
  canceled context must return their stated errors without changing the
  body's document.
