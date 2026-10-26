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
| D4 | A curved face is drawn as the chords of its tessellation. Its crease-free facet edges are not drawn, so a cylinder shows its two silhouette lines and its rim polygons. A circle or arc is a polygon of LINE entities, unless §7 takes it as a CIRCLE or an ARC. |
| D5 | An edge is cut into runs by sampling it once per pixel of a 2048 x 2048 depth buffer of all the triangles. A sample is hidden when its depth exceeds the nearest depth in the 3 x 3 pixels around it by more than two pixels of the drawing's size plus twice the depth change per pixel of that nearest surface. |
| D6 | A run shorter than three samples is the buffer's noise where an edge meets a corner. It joins its longer neighbour, and neighbours of one visibility join. An edge of one run stays whole. |
| D7 | Visible runs are LINE entities on layer `VISIBLE` (colour 7, linetype `CONTINUOUS`) and hidden runs LINE entities on layer `HIDDEN` (colour 8, linetype `HIDDEN`, dashed). Entities are ordered by layer, then by the mesh edge's vertex indices, then along the edge. Equal input gives equal bytes. |
| D8 | The file is release 12 (`AC1009`): a header with the version, a `TABLES` section with the `CONTINUOUS` and `HIDDEN` linetypes and the two layers, an `ENTITIES` section of LINEs, and `EOF`. Numbers are plain decimals with negative zero written as zero. |
| D9 | A nil context, writer or body is `ErrDegenerate`, and tessellation errors, including a wrong unit kind for `tol` and cancellation, propagate. The file is fully built before the first write, so a failure leaves the writer untouched. |

## 2. What the drawing does not claim

Hidden-line removal reads a depth buffer, so it is exact to a pixel of that buffer and no better: a drawing of a part
a metre long resolves about half a millimetre. An edge that passes behind a surface by less than the tolerance of D5
reads as visible. The drawing shows the mesh's edges, so a curve is as coarse as `tol` makes it. The DXF carries no
ellipses or splines, and a circle or arc only where §7 draws one whole.

## 3. Verification

The projection tests take a cube from a corner and count nine visible and three hidden edges, the three hidden edges
meeting at the far corner and the total projected length matching the analytic one, so no edge is split or lost, and a
24-sided prism whose silhouette spans its diameter. The export tests read the DXF back as group-code pairs, check the
sections, the layers, the full-scale coordinates of the far corner and a cylinder's width and height, and the
determinism of the bytes. A crease rule that marks every edge, or no absorption of corner noise, turns them red.

## 4. Not covered

- Hatching other than a section cut (§10), centre lines, dimensions other than the overall size, diameters and radii, and a title block with more than a title, three fields and a scale.
- Arcs and ellipses, a circle only partly seen, and DXF versions after release 12.
- View labels and a layout other than one row.
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

## 6. Paper, scale and a title block

```go
type DXFPaper struct {
    Views         []DrawingView
    Width, Height units.Value
    Scale         units.Value
    Margin, Gap   units.Value
    Title         string
    Dimensions    bool
    Fields        []DXFField
}

type DXFField struct{ Label, Value string }

func DXFSheet(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, paper DXFPaper,
    opts ...decad.TessellateOption) error

const (
    DXFBorderLayer    = "BORDER"
    DXFTextLayer      = "TEXT"
    DXFDimensionLayer = "DIMENSION"
)
```

| Decision | Rule |
|---|---|
| P1 | Coordinates are paper millimetres, the paper's lower left corner at the origin. The views are projected as D1 to D6 say, then every coordinate is moved to its least x and least y and multiplied by `Scale`, a dimensionless ratio of paper to model: 0.5 is 1:2. |
| P2 | The border is the paper inset by `Margin`. A title block, 14 mm deep, fills the bottom of the border: a line across its top, and a divider 60 mm from the right edge. `Title` is a `TEXT` 5 mm high at 3 mm in, and `SCALE 1:n` or `SCALE n:1` sits right of the divider, `n` rounded to two decimals. |
| P3 | The row of views, `Gap` apart, is centred in the area above the title block. With `Dimensions` the row also takes 12 mm to the left and 12 mm below it, for each view's dimensions. |
| P4 | A view's width is dimensioned below it: two extension lines, a dimension line with a 45 degree tick at each end, and a 3.5 mm `TEXT` of the model's width, not the drawn one, rounded to two decimals. Its height is dimensioned the same way left of it, with the text turned 90 degrees. Two or more dimensioned views need `Gap` of at least 12 mm. |
| P5 | The border, the title block and their divider are `LINE`s on `BORDER` and every `TEXT` is on `TEXT`. A dimension's lines are on `DIMENSION`, colour 3. The views keep the `VISIBLE` and `HIDDEN` layers. Entities are ordered border, texts, then each view's visible lines, hidden lines and dimension, so equal input gives equal bytes. |
| P6 | `Width`, `Height` and `Scale` must be positive: zero is `ErrDegenerate`, a negative value `ErrNegativeMagnitude` and a wrong unit kind `ErrUnitKind`. `Margin` and `Gap` are lengths that are not negative. A title with a character outside printable ASCII is `ErrDegenerate`. |
| P7 | Views that do not fit the area, with their dimensions, are `ErrDegenerate` and nothing is written. An empty `Views`, an invalid view, and a nil context, writer or body are `ErrDegenerate`. The file is fully built before the first write. |

## 7. True circles

`DXF`, `DXFViews` and `DXFSheet` draw a whole circular edge as a `CIRCLE` entity where the drawing sees it as a circle and sees it
whole, in place of its polygon of `LINE`s. `MeshDXF` has no edges to read and draws chords as ever.

| Decision | Rule |
|---|---|
| X1 | A candidate is a `Circle3` edge of the body that starts and ends at one vertex and whose axis lies along the view direction (a cross product of at most 1e-9). It projects to a circle of the same radius about its centre's right and up components. A circle seen edge on, or at an angle, is not a candidate and is drawn as its chords. |
| X2 | A candidate is taken when the visible segments with both ends on the circle, within `1e-7` of its radius (at least 1 mm), together run all the way round it: the sum of the angles they span reaches 2π less 1e-6. Every chord is then a whole visible segment, and those segments are removed and a `CIRCLE` on `VISIBLE` written. The same test is made on the hidden segments for a `CIRCLE` on `HIDDEN`. A candidate that is cut by a surface in part, so that a chord's run ends inside it, fails the test and stays as its segments. |
| X3 | The test only chooses between two drawings of edges the body states: whether the polygon is the circle is the body's, not a residual. A near circle in the mesh with no `Circle3` edge is never replaced. |
| X4 | A `CIRCLE` has layer, centre and radius, `10`, `20`, `30` and `40` as the file's group codes. `DXFViews` moves each by its view's offset, `DXFSheet` moves and scales it with the view, and both count its extent with the segments' so a view is placed by its whole drawing. The circles come before the lines in the file, visible then hidden, and are ordered as the body's edges are. |

### Arcs

| Decision | Rule |
|---|---|
| X5 | An `Arc3` edge whose axis lies along the view direction is an arc candidate, by X1's test. It is swept counter-clockwise about its axis from its start vertex to its end vertex. The drawing's toward-the-viewer axis is `-Direction`, so the arc is counter-clockwise in the drawing, from the start's angle to the end's, when the axis has a negative component along `Direction`, and from the end's angle to the start's when it has a positive one. |
| X6 | An arc is taken when the visible (or hidden) segments with both ends on its circle and inside its sweep, within X2's tolerance, together span its sweep less 1e-6: each chord of the arc's polygon is then a whole segment. They are removed and an `ARC` is written with the centre, the radius and the start and end angles in degrees (group codes 10, 20, 30, 40, 50, 51), counter-clockwise from the start. A chord cut by a surface, or one missing, leaves the arc as segments. |
| X7 | `DXFViews` and `DXFSheet` place a view by its arcs' own boxes (the end points and each axis extreme the sweep passes through), not their circles', and move and scale the arcs with the view. |

## 8. Diameters and radii on a sheet

With `DXFPaper.Dimensions`, `DXFSheet` also states the size of each circle and arc a view draws whole (§7), on the `DIMENSION` and
`TEXT` layers.

| Decision | Rule |
|---|---|
| M1 | Each visible `CIRCLE` of a view gets a leader across its diameter, a `LINE` along the drawing's X axis through its centre, and a 3.5 mm `TEXT` `DIA d` just above the centre, `d` the model's diameter, not the drawn one, to two decimals. A hidden circle gets none. |
| M2 | Each visible `ARC` gets a leader from its centre to the middle of its sweep, a `LINE`, and a 3.5 mm `TEXT` `R r` (written `Rr`) half way along it, `r` the model's radius to two decimals. A hidden arc gets none. |
| M3 | The positions are the view's, moved and scaled as its lines are, so a dimension sits on the circle it states whatever the scale. The text is ASCII, so a diameter is `DIA` and not the diameter sign. |
| M4 | The sizes come from the body's own circle and arc edges (§7 X1, X5), so a polygon the mesh draws as a circle is never dimensioned. A circle seen at an angle, or one a surface hides in part, is drawn as lines and has no dimension. |

## 9. Title block fields

`DXFPaper.Fields` adds up to three labelled entries to the title block: a part number, an author, a revision.

| Decision | Rule |
|---|---|
| T1 | A `DXFField` is a label and a value, each printable ASCII, the label not empty: `PART` and `BK-100` are written `PART: BK-100`. More than three fields, an empty label or a character outside printable ASCII is `ErrDegenerate` and nothing is written. |
| T2 | The fields are `TEXT` 2.5 mm high on the `TEXT` layer, stacked from the top of the title block, the first 4 mm below it and each 4 mm under the last, at 0.55 of the way from the sheet's left border to the scale's divider. The title keeps its place at the left. |
| T3 | No fields adds nothing, so a sheet without them is as it was. The tests read the texts back and require the stack's spacing and that each lies inside the block and left of the scale. |

## 10. A hatched section view

`export.DXFSection` writes the cut of a body by a plane, hatched, as one drawing. The outline of the cut comes from
`Body.SectionOutline` (`agent-docs/section-outline-design.md`) and the hatching from `internal/drawing.Hatch`.

| Decision | Rule |
|---|---|
| S1 | The drawing's X and Y are the plane's U and V in millimetres at full scale, measured from the plane's origin, so `plane.ToLocal` of an outline point is its drawing point. The loops are `LINE` entities on the `SECTION` layer, one for each side of the tessellated outline, and the hatch lines are `LINE` entities on the `HATCH` layer. The file is release 12 like the others, with those two layers and the continuous linetype. |
| S2 | The hatch fills the region the loops enclose by the even-odd rule, so a hole inside an outline is empty whichever way the loops run. Lines run at `Angle` from the X axis, 45 degrees for the zero `units.Value`, and sit at whole multiples of `Spacing` across their direction measured from the drawing's origin, so two sections of one spacing and angle carry one family of lines. A line along a boundary edge is drawn when the region lies on the side of increasing offset and not when it lies on the other. Each segment is the part of one line inside the region, ordered by line and then along it. |
| S3 | `Spacing` is a positive length (`ErrDegenerate` for zero, `ErrNegativeMagnitude` for a negative, `ErrUnitKind` for a wrong kind, and for a missing one, a dimensionless zero). `Angle` is an angle; its sign is a direction. A cut that needs over 100000 hatch lines is `ErrDegenerate`. A nil context, writer or body and an invalid plane are `ErrDegenerate`, and a refusal writes nothing. |
| S4 | A plane that misses the body gives a complete file with no entity. `SectionOutline`'s own refusals, a sheet or unsound solid (`ErrNotSolid`) and a mesh whose cut does not close (`ErrUnsupported`), return as they are. |
| S5 | The outline is a reading of the tessellation, not of the exact solid (`SectionOutline`'s contract), so a curved wall is chords and the hatch fills the polygon. The tests read the entities back: lines of the right layer, spacing and ends for a block, a hole left empty with the hatch's length times the spacing equal to the plate's area less the hole's within 1 percent, the plane's origin and angle honoured, and unit tests of `Hatch` on its own. |
