# PNG export design

`export.PNG(ctx, w, body, tol, view, opts...)` draws one body's tessellation as a PNG image, and `export.PNGBodies` draws several in one (§5), so a program can look at a
part. It takes the chord tolerance and `decad.TessellateOption` values of the other writers, defaults to `VerifyNone`,
and adds a `View`. The drawing is plain-data code in `internal/raster/`: an orthographic camera, a depth buffer and
flat shading, with no dependency beyond the standard library's `image/png`.

## 1. Contract

```go
type View struct {
    Direction, Up r3.Vec
    Width, Height int
    Outlines      bool
    FieldOfView   units.Value
    Colors        []color.RGBA
    HiddenEdges   bool
    Shadows       bool
    BackColor     *color.RGBA
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
| P4 | The camera is orthographic unless `View.FieldOfView` is set (§7). The mesh's bounds in the image plane are centred and scaled by one factor, the largest that leaves a 5% margin of the smaller image side on every side, so proportions hold. A mesh with no extent along one image axis is fitted by the other. |
| P5 | A pixel takes the colour of the nearest triangle whose area covers the pixel's centre. A pixel centre on a shared edge belongs to both triangles, and the depth test keeps one. Every other pixel is opaque white. |
| P6 | A triangle with a zero-area normal draws nothing. A face whose outward normal looks toward the camera takes the blue base colour, and one that looks away takes the orange base colour. Both sides are shaded the same way, as if the normal faced the camera. |
| P7 | The shade is `0.3 + 0.7·max(0, n·L)` of the base colour, with `n` the unit normal turned to face the camera and `L` the unit vector of the camera's reverse direction raised by 0.5 and moved 0.35 to the left of the image. The offset makes faces at equal angles to the view take different shades, as an isometric view needs. |
| P8 | No axis, label or anti-aliasing is drawn, and no outline unless `View.Outlines` is set (§6). A pixel is either covered or white. |
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

- Axes, dimensions and text.
- Materials, textures and anti-aliasing and soft or multiple-light shadows.
- A transparent background and image formats other than PNG.
- Drawing a mesh body together with bodies.

## 5. Several bodies

```go
func PNGBodies(ctx context.Context, w io.Writer, bodies []*decad.Body, tol units.Value, view View,
    opts ...decad.TessellateOption) error
```

| Decision | Rule |
|---|---|
| B1 | Each body is tessellated as `PNG` tessellates one, at the one tolerance and with the same options. The meshes are joined into one, each body's triangles keeping their own vertices shifted past those before, and drawn as P4 to P8 say. |
| B2 | The scene's bounds, not any one body's, set the centring and the scale, so the whole scene fits with the margin of P4. |
| B3 | A nearer body hides a farther one by the depth test of P5, so the order of `bodies` changes nothing in the image. |
| B4 | All bodies share the colour scheme of P6. One body gives the bytes `PNG` gives. |
| B5 | A nil context or writer, an empty list and a nil body are `ErrDegenerate`. A wrong unit kind for `tol` and a canceled context propagate, and a failure writes nothing. |

## 6. Outlines

`View.Outlines` draws the body's visible feature edges over the shading. The edges come from `drawing.Project`, the
projection `export.DXF` uses (`agent-docs/dxf-export-design.md`), so they are the same edges that drawing shows.

| Decision | Rule |
|---|---|
| O1 | An edge is a feature as `drawing.Project` says: an edge used once or three or more times, a crease of more than 30 degrees, or a silhouette. The diagonal of a flat face is not drawn. |
| O2 | Only the visible runs are drawn. A hidden run, one behind a surface, is not. |
| O3 | A segment is drawn one pixel wide in dark grey, `(32, 32, 32)`, by `raster.DrawSegments`: points about every half pixel along it, each in the pixel that holds it, so an edge at a pixel's boundary lies in the pixel to its right or below. The outline is drawn over the shading and hides it. |
| O4 | The edges are mapped with the same projection as the shading, so each outline pixel lies on the edge between its shaded faces, to a pixel. |
| O5 | The zero value draws no outline, and the bytes are those of §1 to §5. `PNGBodies` and `MeshPNG` take the field too. |

## 7. Perspective

`View.FieldOfView` makes the camera perspective. The zero `units.Value` is the orthographic camera of P1 to P8.

| Decision | Rule |
|---|---|
| V1 | `FieldOfView` is an angle strictly between 0 and 180 degrees. A negative value is `ErrNegativeMagnitude`, a wrong unit kind `ErrUnitKind`, and zero (as an angle) or 180 degrees or more `ErrDegenerate`. |
| V2 | The eye sits on the line through the middle of the scene's bounding box along the view direction, at the distance `r / sin(fov/2)` behind it, where `r` is half the box's diagonal. The scene's bounding sphere then lies inside the field of view, and no point is behind the eye. |
| V3 | A point projects to its camera-right and camera-up components over its distance from the eye along the view direction. The projected scene is then centred and scaled to fit with the margin of P4. |
| V4 | The depth test reads minus the reciprocal of that distance, which is linear in screen space across a triangle, so a nearer triangle hides a farther one in either drawing order. |
| V5 | A face looks away when its normal points along the line of sight from the eye to the face's centre, not along the camera's axis, so two faces of one body can differ in facing where an orthographic camera would agree. |
| V6 | Shading uses the camera's axes and headlight as P7 does. Outlines take the field as §12 says. |
| V7 | `PNGBodies` and `MeshPNG` take the field too. |

## 8. A colour for each body

`View.Colors` sets the colour of the faces that look toward the camera, one colour for each body drawn.

| Decision | Rule |
|---|---|
| K1 | An empty `Colors` is the default scheme, and the bytes are those of §1 to §7. Otherwise `Colors` has one entry for each body drawn: `PNG` and `MeshPNG` draw one body and take one, and `PNGBodies` takes one for each of its bodies, in order. Any other length is `ErrDegenerate` and nothing is written. |
| K2 | A triangle of body `i` is shaded with `Colors[i]` in place of the default blue, by the same headlight and ambient level (`raster.Shaded`'s). A face that looks away keeps the orange of §1, so an open sheet or an inside-out mesh still shows. The alpha channel of a colour is ignored: the image is opaque. |
| K3 | The default blue, 120, 160, 200, as a colour gives the bytes of no colour. Outlines and the perspective camera draw as before. |

## 9. Hidden edges

`View.HiddenEdges` draws the feature edges that lie behind surfaces, dashed, so the far side of a part reads through
its shading. They come from `drawing.Project` as the outlines of §6 do: its hidden runs are the ones a depth buffer
finds behind a surface (`agent-docs/dxf-export-design.md` D5, D6).

| Decision | Rule |
|---|---|
| E1 | Each hidden run is drawn in light grey, `(150, 150, 150)`, by `raster.DrawDashed`: along it, from its start, 4 pixels drawn and 3 skipped, in the half-pixel steps of `DrawSegments`. A run shorter than 4 pixels is drawn whole. |
| E2 | The dashes are drawn over the shading and under the outlines, so with `Outlines` set a visible edge covers a dash where the two cross. |
| E3 | The zero value draws none, and the bytes are those of §1 to §8. `HiddenEdges` is independent of `Outlines`. |
| E4 | `HiddenEdges` takes `FieldOfView` as §12 says. `PNGBodies` and `MeshPNG` take the field too, and a hidden run is hidden by any body. |

## 10. Shadows

`View.Shadows` casts the key light's shadows, so a tall part visibly shades the ground or a neighbour beside it.

| Decision | Rule |
|---|---|
| H1 | The key light is the one of §1: raised 0.5 and 0.35 to the left of the camera in the camera's own axes. A shadow map is the scene seen from the light along that direction by an orthographic camera, drawn into a depth buffer of side `2·max(Width, Height)` clamped to 512 to 4096 pixels. |
| H2 | Each covered pixel's world point is rebuilt from its place and depth, mapped into the light's view and compared with the nearest depth there. The pixel is in shadow when the map's depth is nearer by more than a bias of `1.5 + 2·tan θ` pixels of the map, `θ` being the angle between the face's normal and the light, so no face shadows itself. |
| H3 | A pixel in shadow takes its face's base colour at the ambient level, 0.3, which is the level of a face turned from the light. A face turned from the light is left as it was. So a convex part shows no shadow on itself, and the bytes of a lone block are those without `Shadows`. |
| H4 | The shadow of a body falls on the others drawn in the same image by `PNGBodies`. The shadow is hard, one light, one sample, so its edge is as fine as the map and the image. |
| H5 | The zero value draws none, and the bytes are those of §1 to §9. `Shadows` takes `FieldOfView` as §13 says. `MeshPNG` and `PNGBodies` take the field too. |

## 11. A colour for the faces that look away

`View.BackColor` sets the base colour of the faces that look away from the camera, which are orange by default.

| Decision | Rule |
|---|---|
| B1 | Nil keeps the orange, `(220, 130, 60)`, and the bytes are those of §1 to §10. A set colour replaces it for every face that looks away, shaded at the same light level, so an open sheet or an inside-out mesh still shows, in the colour the caller picked. Its alpha is ignored. |
| B2 | A face that looks toward the camera is unchanged, so `Colors` and the default blue are untouched. With `Shadows`, a shadowed face that looks away takes the back colour at the ambient level. |
| B3 | `BackColor` is one colour for the image, not one for each body: `Colors` colours the faces toward the camera, one for each body. |

## 12. Edges under a perspective camera

`Outlines` and `HiddenEdges` draw with `FieldOfView` set, from the same `drawing.Project` as the orthographic camera, given the perspective projection.

| Decision | Rule |
|---|---|
| Q1 | An edge is a feature as §6 says, except that a silhouette is an edge whose two faces differ in facing, a face looking toward the camera when its normal points against the line of sight from the eye to the face's centre (V5), not against the camera's axis. |
| Q2 | An edge's ends are placed in the plane at distance 1 from the eye, as V3 places any point, and the edge is the straight line between them: a straight edge projects to a straight line under a pinhole camera. The result is in those plane units and meant for the pixels of the same view. |
| Q3 | An edge is cut into visible and hidden runs by the depth buffer of V4. A sample is hidden when a face in front of it covers its pixel or a neighbouring one, within a tolerance of two pixels: the depth there is minus the reciprocal of the distance, so two pixels' length at distance `d` is `2·PixelSize/d` in depth, and the tolerance is that at the sample's own depth. |
| Q4 | A view's other fields and the order of drawing, dashes under outlines, are those of §6 and §9. |
| Q5 | The tests state the near and far squares' pixel positions of a block seen from above from the distances of V2, and require that the dashed far edge lies there and that the near outline lies on the near face, and `raster`'s own tests state the plane point, the facing and the tolerance by their formulas. |

## 13. Shadows under a perspective camera

`Shadows` draws with `FieldOfView` set. The light stays the directional key light of H1, so the shadow map is the orthographic one.

| Decision | Rule |
|---|---|
| W1 | A pixel's world point is on its ray from the eye: the point at distance `-1/depth` along the view direction, where `depth` is the buffer's value (V4), and the camera's right and up by the pixel's plane position times that distance. `raster.Projection.World` is the inverse of `Project` for either camera. |
| W2 | A face seen from behind is decided against the line of sight from the eye to the pixel's world point (V5), not the camera's axis, so a face the axis sees edge on can still look toward or away from the eye. The shadow's colour is that face's base colour, as H3 says. |
| W3 | Every other rule of §10 holds: the bias, the map's size, a convex part shadowing none of itself and the bytes without `Shadows`. |
| W4 | The tests place the shadow of a pillar on a plate from the distances of V2 and the key light's direction and read the pixel's colour, which is the plate's blue at the ambient level, and `raster`'s own tests state the world point and the sight line. |

## 14. Soft shadows

`View.ShadowSoftness`, an angle, makes the key light an area light.

| Decision | Rule |
|---|---|
| F1 | The value is the angular radius of the light. It needs `Shadows` (otherwise `ErrDegenerate`), is a positive angle of at most 45 degrees (`ErrUnitKind` for another kind, `ErrDegenerate` beyond the bound), and the zero `units.Value` is the point light of §10, whose bytes are unchanged. |
| F2 | The light is sampled in 32 directions over a disc of radius `tan(softness)` about the key direction, placed on a Vogel spiral so they fill the disc evenly. Each direction has its own shadow map, 2048 pixels at most, built one at a time so only one is held. |
| F3 | A pixel counts the samples whose light a nearer triangle blocks, with §10's bias. A pixel blocked in every sample takes the ambient shade of §10 exactly, one blocked in none keeps its lit shade, and any other takes the lit and ambient shades mixed in the blocked fraction. A face turned from the key light is at the ambient level as before. |
| F4 | A shadow so grades from dark to light across a penumbra whose width grows with the distance from the occluder, about twice that distance times `tan(softness)`. The tests count the pixels strictly between the lit and shadowed plate levels: a hard shadow has a handful at its edge, a 5 degree one hundreds, and a 12 degree one more; the pillar's top and the far plate are unchanged. A render that mixes by a fixed fraction fails them. |

### Row shading

Hard and soft shadows shade images of at least 65,536 pixels with up to four workers, subject to the shared
`orderedwork` worker budget and `GOMAXPROCS`. Smaller images shade serially. Each worker reads the completed camera
and light depth buffers and writes only its assigned rows' blocked counts and ambient colours. All rows join before
the next light sample starts, so each pixel accumulates samples in the same order as serial rendering. Camera
rasterization, light depth buffers and final colour mixing run serially. Cancellation joins the workers before
returning an error and publishes no image.

The row tests compare every pixel for hard, soft and perspective shadows with body colours, back colours and gloss.
The export tests also compare the complete PNG bytes across worker counts using tessellated plate and pillar bodies.

## 15. Gloss

`View.Gloss`, a dimensionless strength from 0 to 1, gives lit faces a specular highlight.

| Decision | Rule |
|---|---|
| L1 | The zero `units.Value` is matte, and the bytes of every render without it are unchanged. A value outside [0, 1] is `ErrDegenerate` and a non-dimensionless one `ErrUnitKind`. |
| L2 | A face the key light of §1 reaches (positive diffuse term) adds white to each channel: `255 · gloss · (n · h)^32`, with `h` the unit halfway vector of the unit direction to the key light and the unit direction to the eye (along the line of sight to the face for a perspective camera), clamped at 255. A face the light does not reach, and a back face seen from behind, take none. |
| L3 | A pixel in a shadow (§10, §14) is at the ambient shade with or without a gloss, since the highlight is added to the lit shade only, and a soft shadow mixes the glossy lit shade with the ambient one. |
| L4 | The shading is flat per triangle, so a flat face takes one highlight and a tessellated curved face takes it in bands. The tests read the top face of a block viewed from above against the formula of L2 at gloss 1 and one half, where the highlight is about 61 of 255, leave the background and a face off the highlight unchanged, keep every shadowed pixel of a plate and pillar scene as dark as without a gloss while the lit plate takes the highlight, and refuse the bad values. A sharpness exponent of 8 in place of 32 fails them. |

## 16. Opacity

`View.Opacity` lets a body show what is behind it: one dimensionless value for each body drawn, in order as `Colors` is.

| Decision | Rule |
|---|---|
| T1 | An empty `Opacity`, and one whose values are all 1, draw as without it, byte for byte. A count other than the bodies' (a single `PNG` body takes one) is `ErrDegenerate`, a value outside (0, 1] is `ErrDegenerate` and a non-dimensionless one `ErrUnitKind`. It does not combine with `Shadows`, which is `ErrDegenerate`. |
| T2 | The triangles of opacity 1 are drawn as before, shaded flat and depth tested. The others are blended over that image from the farthest to the nearest, ordered by the mean depth of their vertices, each at the pixels where it is nearer than the opaque surface: a pixel becomes `α` of the triangle's shade and `1 − α` of what was there, rounded to the nearest 8 bit value. A translucent body's own far side, which a back colour shades, therefore shows through its near side. |
| T3 | A pixel whose centre lies on an edge two translucent triangles share is blended once, by the top-left rule: the triangle is wound one way and an edge it runs upward, or level toward the right, owns such a pixel. The opaque pass needs no rule, its depth test hides a second draw. |
| T4 | The order is by triangle, not exact: two translucent triangles that cross are blended in one order throughout, and a perspective camera orders by the same depth. |
| T5 | The tests lay a plate 1 mm above a block seen from above, and read the pixel at the plate's centre, which lies on the diagonal of its two triangles, as a half-clear plate over the block: green half of the near face's, blue `(1 − α)α` of the far face's and red `(1 − α)²` of the block's. The pixels beside the plate and the background are as without opacity, an opacity of 1 matches the plain bytes, and the refusals are listed. A blend of the shared diagonal twice, and an order of nearest first, each fail them. |
