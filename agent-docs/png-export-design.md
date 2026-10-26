# PNG export design

`export.PNG(ctx, w, body, tol, view, opts...)` draws one body's tessellation as a PNG image, so a program can look at a
part. It takes the chord tolerance and `decad.TessellateOption` values of the other writers, defaults to `VerifyNone`,
and adds a `View`. The drawing is plain-data code in `internal/raster/`: an orthographic camera, a depth buffer and
flat shading, with no dependency beyond the standard library's `image/png`.

## 1. Contract

```go
type View struct {
    Direction, Up r3.Vec
    Width, Height int
}

func IsometricView(width, height int) View
func PNG(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, view View,
    opts ...decad.TessellateOption) error
```

| Decision | Rule |
|---|---|
| P1 | `Direction` is the way the camera looks, from the eye into the scene. The image's right is `Direction × Up` and its up is `right × Direction`. Neither vector need be a unit vector. A zero `Direction`, or an `Up` parallel to it, is `ErrDegenerate`. |
| P2 | `Width` and `Height` are pixels, each from 16 to 8192. Another value is `ErrDegenerate`. |
| P3 | `IsometricView` looks along (-1, -1, -1) with +Z up, so the top, front and right faces of a part with its corner at the origin show. |
| P4 | The camera is orthographic. The mesh's bounds in the image plane are centred and scaled by one factor, the largest that leaves a 5% margin of the smaller image side on every side, so proportions hold. A mesh with no extent along one image axis is fitted by the other. |
| P5 | A pixel takes the colour of the nearest triangle whose area covers the pixel's centre. A pixel centre on a shared edge belongs to both triangles, and the depth test keeps one. Every other pixel is opaque white. |
| P6 | A triangle with a zero-area normal draws nothing. A face whose outward normal looks toward the camera takes the blue base colour, and one that looks away takes the orange base colour. Both sides are shaded the same way, as if the normal faced the camera. |
| P7 | The shade is `0.3 + 0.7·max(0, n·L)` of the base colour, with `n` the unit normal turned to face the camera and `L` the unit vector of the camera's reverse direction raised by 0.5 and moved 0.35 to the left of the image. The offset makes faces at equal angles to the view take different shades, as an isometric view needs. |
| P8 | No outline, axis, label or anti-aliasing is drawn. A pixel is either covered or white. |
| P9 | A nil context, writer or body is `ErrDegenerate`, and tessellation errors, including a wrong unit kind for `tol` and cancellation, propagate. The image is fully drawn and encoded before the first write, so a failure leaves the writer untouched. |

## 2. Determinism

The shade and the pixel coverage use float64 arithmetic and a fused multiply-add may differ between platforms, so
equal input gives equal bytes on one platform. The tests compare pixels away from triangle edges and compare rendered
colours against `raster.Shaded`, never against a literal colour.

## 3. Verification

The raster tests draw squares and triangles whose image positions are computed by hand: they check the margin, the
direction of the image's axes, the depth order in both drawing orders and the shade of faces turned toward, away from,
up, down, left and right of the light. The export tests draw a block from above, a drilled plate through whose hole the
background shows, and an isometric block whose three faces differ. A depth test written the other way round, or an
image axis flipped, turns the tests red.

## 4. Not covered

- Perspective cameras, outlines, hidden or feature edges, axes, dimensions and text.
- Colours, materials, textures, shadows and anti-aliasing.
- A transparent background and image formats other than PNG.
- Drawing more than one body, or a mesh body.
