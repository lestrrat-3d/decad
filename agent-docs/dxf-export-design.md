# DXF export design

`export.DXF(ctx, w, body, tol, view, opts...)` writes a 2D drawing of one body as an ASCII DXF file: the feature edges
of its tessellation, projected orthographically, with hidden edges on their own layer. It takes the chord tolerance
and `decad.TessellateOption` values of the other writers and defaults to `VerifyNone`. The projection and the
hidden-line test are plain-data code in `internal/drawing/`, over the depth buffer of `internal/raster/`.

## 1. Contract

```go
type DrawingView struct{ Direction, Up r3.Vec }

func IsometricDrawingView() DrawingView
func DXF(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, view DrawingView,
    opts ...decad.TessellateOption) error

const (
    DXFVisibleLayer = "VISIBLE"
    DXFHiddenLayer  = "HIDDEN"
)
```

| Decision | Rule |
|---|---|
| D1 | `Direction` is the way the camera looks. The drawing's X axis is `Direction × Up` and its Y axis is the component of `Up` square to `Direction`. Neither need be a unit vector. A zero `Direction`, or an `Up` parallel to it, is `ErrDegenerate`. |
| D2 | Coordinates are millimetres at full scale, measured along those axes from the world origin's projection. There is no fitting, margin or sheet border. |
| D3 | A feature edge is an edge of the tessellation that one triangle uses or three or more do, one whose two triangles' normals turn by more than 30 degrees (`drawing.CreaseCosine`), or one shared by a triangle looking toward the camera and one looking away (a silhouette). The diagonals of a flat face are not features. A triangle of zero area contributes none. |
| D4 | A curved face is drawn as the chords of its tessellation. Its crease-free facet edges are not drawn, so a cylinder shows its two silhouette lines and its rim polygons. A circle is a polygon of LINE entities. |
| D5 | An edge is cut into runs by sampling it once per pixel of a 2048 x 2048 depth buffer of all the triangles. A sample is hidden when its depth exceeds the nearest depth in the 3 x 3 pixels around it by more than two pixels of the drawing's size plus twice the depth change per pixel of that nearest surface. |
| D6 | A run shorter than three samples is the buffer's noise where an edge meets a corner. It joins its longer neighbour, and neighbours of one visibility join. An edge of one run stays whole. |
| D7 | Visible runs are LINE entities on layer `VISIBLE` (colour 7, linetype `CONTINUOUS`) and hidden runs LINE entities on layer `HIDDEN` (colour 8, linetype `HIDDEN`, dashed). Entities are ordered by layer, then by the mesh edge's vertex indices, then along the edge. Equal input gives equal bytes. |
| D8 | The file is release 12 (`AC1009`): a header with the version, a `TABLES` section with the `CONTINUOUS` and `HIDDEN` linetypes and the two layers, an `ENTITIES` section of LINEs, and `EOF`. Numbers are plain decimals with negative zero written as zero. |
| D9 | A nil context, writer or body is `ErrDegenerate`, and tessellation errors, including a wrong unit kind for `tol` and cancellation, propagate. The file is fully built before the first write, so a failure leaves the writer untouched. |

## 2. What the drawing does not claim

Hidden-line removal reads a depth buffer, so it is exact to a pixel of that buffer and no better: a drawing of a part
a metre long resolves about half a millimetre. An edge that passes behind a surface by less than the tolerance of D5
reads as visible. The drawing shows the mesh's edges, so a curve is as coarse as `tol` makes it. The DXF carries no
analytic circles, arcs, ellipses or splines.

## 3. Verification

The projection tests take a cube from a corner and count nine visible and three hidden edges, the three hidden edges
meeting at the far corner and the total projected length matching the analytic one, so no edge is split or lost, and a
24-sided prism whose silhouette spans its diameter. The export tests read the DXF back as group-code pairs, check the
sections, the layers, the full-scale coordinates of the far corner and a cylinder's width and height, and the
determinism of the bytes. A crease rule that marks every edge, or no absorption of corner noise, turns them red.

## 4. Not covered

- Dimensions, text, hatching, centre lines and title blocks.
- Analytic circles, arcs and ellipses, and DXF versions after release 12.
- Scale, paper size, view labels and borders on a sheet, and a layout other than one row.
- A tolerance on the hidden-line test finer than the depth buffer's pixel.

## 5. Several views on one sheet

```go
func DXFViews(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, views []DrawingView,
    gap units.Value, opts ...decad.TessellateOption) error
```

| Decision | Rule |
|---|---|
| V1 | The body is tessellated once and projected along each view as D1 to D6 say. |
| V2 | Each view is moved so the least x of its edges falls on the sheet's running x position and the least y of its edges falls on y = 0. The first view's left edge is at x = 0, and each later view starts `gap` to the right of the previous view's greatest x. |
| V3 | The views share the `VISIBLE` and `HIDDEN` layers and the file of D8, so the entities of the views follow one another in view order, visible runs then hidden runs for each layer as D7 orders them. |
| V4 | `gap` is a non-negative length: a negative value is `ErrNegativeMagnitude` and a wrong unit kind `ErrUnitKind`. An empty `views`, an invalid view, a nil context, writer or body are `ErrDegenerate`, and the file is fully built before the first write. |
| V5 | A view is moved by its drawn edges, so a view's extent is that of its feature edges, not of any frame. Views that differ in how far their edges reach from the origin still line up on the baseline. |

