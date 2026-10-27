# G-code perimeter export design

`export.GCodePerimeters` writes the first stage of a slicer: the toolpath that prints a body's outline layer by layer.
It reads the same section outline as `export.DXFSection` (`agent-docs/section-outline-design.md`).

## 1. Contract

```go
func GCodePerimeters(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, spec GCodeSpec) error
```

| Decision | Rule |
|---|---|
| G1 | The file is millimetres, absolute positioning and absolute extrusion: `G21`, `G90`, `M82`, `G92 E0`, then each layer under a `; layer i z=` comment. |
| G2 | With `zmin` the body's lowest point and `h` the layer height, there are `floor((zmax − zmin)/h)` layers; layer `i` is traced along the section at `zmin + (i + 1/2)·h` and printed at `zmin + (i + 1)·h`. A body lower than one layer, or needing over 100000 layers, is `ErrDegenerate`. The part above the last whole layer is not printed. |
| G3 | The section is `Body.SectionOutline` at `tol`: every loop, an outer loop and a hole alike, is a travel move (`G0`) to its first point at the layer's height and then extruding moves (`G1`) along it and back to the start. The outline is the mesh's, so a curved wall is chords. |
| G4 | The extrusion of a move is its length times `LayerHeight·LineWidth` over the filament's cross-section `π(d/2)²`, accumulated into the absolute `E`. The feed rate, a velocity, is written in millimetres a minute on each extruding move. |
| G5 | A length of the spec is positive (`ErrDegenerate` for zero, `ErrNegativeMagnitude`, `ErrUnitKind`), as is the feed rate; a nil context, writer or body is `ErrDegenerate`. Every failure writes nothing. |
| G6 | The tests print a 20 x 10 x 6 mm block as twelve layers, require each layer top, each coordinate inside the block's outline, the feed rate and an `E` that never backs up, and read the final `E` against the 720 mm of line and the filament's area. A wrong filament area fails them. |

## 2. Not covered

- Infill, top and bottom skin, support, perimeter offset by half the line width, retraction, temperatures, fan and a start or end sequence.
- A toolpath for a mill or a lathe, and a body whose section the outline refuses.
