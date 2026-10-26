# Rib and Web design

`Rib` and `Web` add thin fins to a solid. Each builds one rectangular tool per sketch line with `Extrude` and joins
the tools to the body with `Union`. They add line-to-outline arithmetic, validation and one atomic commit, and no new
geometry kernel. The public contract is in `agent-docs/api-design.md` §8; this document owns the decisions behind it.

## 1. Contract

```go
func Rib(ctx context.Context, b *Body, s *sketch.Sketch, line *sketch.Line, thickness units.Value, e Extent, opts ...RibOption) (*Body, error)
func Web(ctx context.Context, b *Body, s *sketch.Sketch, lines []*sketch.Line, thickness units.Value, e Extent, opts ...RibOption) (*Body, error)
func RibArc(ctx context.Context, b *Body, s *sketch.Sketch, arc *sketch.Arc, thickness units.Value, e Extent, opts ...RibOption) (*Body, error)
func WebPath(ctx context.Context, b *Body, s *sketch.Sketch, path []sketch.Entity, thickness units.Value, e Extent, opts ...RibOption) (*Body, error)
func WithRibOverrun(overrun units.Value) RibOption
func WithRibDraft(angle units.Value) RibOption
```

`Rib` is `Web` with one line. Both retire `b` and return the joined body.

| Decision | Rule |
|---|---|
| R1 | A rib of `Rib` and `Web` is one straight `*sketch.Line` of the sketch `s`. A chain or a polyline is not accepted, and an arc is `RibArc`'s (§5). A line that is nil, not an entity of `s`, stale or of zero length is `ErrDegenerate`. |
| R2 | The rib outline is the line thickened across its width by `thickness`, centred on the line, and lengthened by the overrun past both ends. `ribplan.Footprint` computes its four corners in sketch-plane coordinates. |
| R3 | `thickness` is a positive length and the overrun a non-negative length. The last `WithRibOverrun` wins and the default is zero. A violation is `ErrDegenerate`, `ErrUnitKind` or `ErrNegativeMagnitude`. |
| R4 | The outline is extruded by `e` from the plane of `s`. `e` is any `Extent` that `Extrude` accepts, resolved against that plane exactly as for an `Extrude` of the outline. |
| R5 | A rib whose bounding box is certifiably disjoint from the body's is `ErrDegenerate`. The test grows both boxes by their own bounds, so it can refuse and never admits. |
| R6 | A result whose volume is certifiably no larger than the body's is `ErrDegenerate`: the ribs lie inside the material. The test compares the volume readings with their bounds, so it can refuse and never admits. |
| R7 | A rib end that only touches a wall is a coplanar contact, which `Union` refuses as `BooleanUnsupportedContact`. The caller buries the ends with `WithRibOverrun`, and buries the base by drawing the sketch plane below the surface the rib stands on. |

## 2. Joining

- Each tool is built with `Extrude` on a private `sketch` world whose plane has the frame of `s`.
- `Web` unites the tools with one another in line order first, and joins the single merged tool to `b` once. Crossing
  ribs share a base plane and a top plane. Uniting a second rib into a body that already holds the first meets those
  shared planes in a mesh boolean, which refuses them as a tangent contact (`apitest/rib_test.go`,
  `TestWebCrossesRibs`); uniting the two equal-extent ribs with each other first does not.
- The merge, the join and the commit run through `combineTools` (`tool_combine.go`), which `Hole` also uses. No
  body is registered until the join and the volume check succeed. A failure removes the tools and leaves `b` live and
  the document unchanged. The tools are live in the document between their build and the removal.

## 3. Exactness

`Rib` returns what `Union` returns for the same bodies. A rib joined to a block on straight walls reads
`Approximate` with a bound of the order of 1e-11 mm³: the result is a faceted body (`TestRibAcrossPocket`).

## 4. Not covered

- A rib that follows the surface of the body, and a rib along a spline or any entity but a line and an arc.
- A rib that extends itself to the body. The caller draws the line and the extent so that the rib reaches the walls.
- A rib thickened perpendicular to the sketch plane. The thickness lies in the sketch plane.

## 5. Rib along an arc

`RibArc` thickens one `*sketch.Arc` into an annular band and joins it as `Rib` joins a rectangle. `ribplan.ArcBand`
computes the band.

| Decision | Rule |
|---|---|
| A1 | The arc sweeps counter-clockwise from its `Start` to its `End` about its `Center`, as every sketch arc does. An arc whose end points share an angle sweeps a full turn, and any rib that would wrap a full turn once lengthened is `ErrDegenerate`. |
| A2 | The arc's end points MUST lie on one circle about its centre, within 1e-9 of the radius, and the radius is positive. Otherwise the arc is `ErrDegenerate`. |
| A3 | The band lies between the radii `R − thickness/2` and `R + thickness/2`. An inner radius of zero or less is `ErrDegenerate`. |
| A4 | `WithRibOverrun` lengthens the band at each end by that arc length along the arc, which is `overrun / R` radians. |
| A5 | The tool is the band's two arcs and two straight ends on fixed sketch points, extruded by `e` from the plane of `s`. R3 to R7 apply, with the rib's outline in place of the rectangle. |
| A6 | `RibArc` and `Rib` record one `OperationRib` entry each. |

The volume a rib adds, above the body it stands on, is the band's area times its height: `θ·R·t·h` for a sweep of `θ`
radians. The tests compare against it and check on which side of the arc material lies, so a rib swept the wrong way
round the circle turns them red.

## 6. A run of lines and arcs

`WebPath` adds one rib along each line and arc of a run, in order, in one commit: `Web` for lines and arcs mixed.

| Decision | Rule |
|---|---|
| P1 | `path` holds `*sketch.Line` and `*sketch.Arc` entities of `s`. Each is built as `Rib` or `RibArc` builds it (R2 to R4, A1 to A5). Any other entity, a nil entity and an entity of another sketch are `ErrDegenerate`, as is an empty path. |
| P2 | `Web` is `WebPath` over its lines, and `Rib` over one line, so the three share every rule of §1 and §5. |
| P3 | The ribs are united with one another in path order and the single merged tool joins `b` once (§2), so a rib that crosses or meets another needs no special case. |
| P4 | Ribs that meet end to end, the legs of a polyline or an arc and the lines either side of it, overlap only through the overrun of R7. With the default of zero their end faces touch in a coplanar contact that `Union` refuses, so a run that is one chain sets an overrun. |
| P5 | A corner is not mitred: the overrun of each leg fills the corner on the outside and the legs overlap inside it. |

## 7. Draft

`WithRibDraft` leans every rib's walls. It passes `WithTaper` to the `Extrude` that builds each tool, so the lean, its
sign and its refusals are `Extrude`'s (`agent-docs/draft-design.md` §2).

| Decision | Rule |
|---|---|
| F1 | The lean is measured from the sketch plane, upward along the extent: at the height `t` above the plane a drafted line rib is `thickness − 2·t·tan(angle)` thick, and its ends lean in the same way. A positive angle makes the rib thinner as it rises and a negative one thicker. |
| F2 | The overrun (R3, R7) is its length at the sketch plane. The ends of a rib that spans a cavity shorten by `t·tan(angle)` at height `t`, so the overrun MUST bury them at the top of the rib as well as at the plane. |
| F3 | The angle is `WithTaper`'s: a wrong unit kind is `ErrUnitKind` and an angle at or past a right angle is `ErrDegenerate`. The last occurrence wins and the default is a straight wall. |
| F4 | `WithRibDraft` applies to `Rib`, `Web`, `RibArc` and `WebPath`. A rib whose outline has arcs, such as the band of `RibArc`, meets the drafted `Extrude`'s refusals, so a corner of an arc and a line that is not tangent is `ErrUnsupported`. |
| F5 | The tests compare the volume a drafted line rib adds against `length · ∫ (thickness − 2·t·tan(angle)) dt` over the height above the cavity floor within a relative `1e-9`. |

