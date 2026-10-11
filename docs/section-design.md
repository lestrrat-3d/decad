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

## Exact prism class

The first increment admits a `prismPayload` with zero `sectionDelta`,
`z0Delta`, and `z1Delta`. It refuses a plane that touches either cap. It also
refuses every body whose payload is not this prism class. These misses return
`ErrUnsupported` without changing the document.

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

## Verification

- Extrude a 10×10 mm square by 5 mm. Section at 2.5 mm must give one 100 mm²
  profile. Extruding that returned Sketch/profile by 2 mm gives 200 mm³ and
  passes `Tessellate(VerifyAll)`.
- A rigidly translated prism must section on the correspondingly translated
  plane. A source profile with a hole must return the same outer and hole
  boundaries when Sketch can certify their reconstruction.
- A cap plane, oblique plane, sheet body, invalid input, and
  canceled context must return their stated errors without changing the
  body's document.
