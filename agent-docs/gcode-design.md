# G-code perimeter export design

`export.GCodePerimeters` writes the first stage of a slicer: the toolpath that prints a body's outline layer by layer, with rectilinear infill when asked.
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
| G7 | With `InfillSpacing` set (a positive length, else `ErrDegenerate`, `ErrNegativeMagnitude` or `ErrUnitKind`; the zero value writes none) each layer follows its perimeters with rectilinear infill: lines parallel to X on even layers and to Y on odd ones, one every spacing across the outline's extent from half a spacing in. A line is cut at every crossing with a loop, an outer loop and a hole alike, by the even-odd rule, so it runs through material between crossings in pairs, and each piece is shortened by `LineWidth` at both ends so that it does not pile onto a perimeter; a piece no longer than that is dropped. Each piece is a `G0` to its first point and one `G1`, and alternate lines run opposite ways. |
| G9 | `SolidLayers` (a count, zero for none, negative is `ErrDegenerate`) fills the first and last that many layers solid: their infill is G7's with the lines `LineWidth` apart whatever `InfillSpacing` is, and it is written even when `InfillSpacing` is zero, so the solid layers can stand alone. The layers between keep G7's sparse infill. The count is of the print's layers and no skin is found from what lies above or below a layer. |
| G10 | `PerimeterInset` moves each perimeter loop half a `LineWidth` into the material (`insetLoops`): the outer loops inward and the loops of holes outward, so that the printed line's edge lies on the outline. A loop is wound with the material on its left, outer loops counter-clockwise and holes clockwise, by how many other loops enclose it. Each corner is the meeting of its two moved sides, and a corner whose sides turn back by more than 170 degrees is cut off at the inset from the corner instead. The infill's lines are unchanged: they end a full line width from the outline, which is the inset perimeter's edge. |
| G6 | The tests print a 20 x 10 x 6 mm block as twelve layers, require each layer top, each coordinate inside the block's outline, the feed rate and an `E` that never backs up, and read the final `E` against the 720 mm of line and the filament's area. A wrong filament area fails them. |
| G11 | The inset tests print the 20 x 10 block, whose perimeter reaches 0.2 and 19.8 mm in x and 0.2 and 9.8 mm in y and runs 58.4 mm a layer, and the 20 x 20 plate with its 6 x 6 hole, whose outer loop reaches 0.2 and 19.8 and whose hole loop 6.8 and 13.2, 104 mm a layer. A loop wound the wrong way, or an inset the wrong way, fails them. |
| G8 | The infill tests print the block with a 5 mm spacing (two lines of 19.2 mm on an even layer and four of 9.2 mm on an odd one, at the stated coordinates, 451.2 mm of infill in all) and a 20 x 20 plate with a 6 x 6 hole (the lines through the hole break into two pieces of 6.2 mm, 63.2 mm of infill a layer), read the final `E` against the lengths, and refuse a negative or wrongly typed spacing. An infill without the inset, or one that ignores the hole, or one that never turns, fails them. A 12 layer print with two solid layers at each end reads 25 and 50 lines on the solid even and odd layers and the sparse ones between, with the solid layers alone as well, and fails with solid layers ignored. |

## 2. Not covered

- Infill other than rectilinear lines, a skin found from what lies above or below a layer, support, retraction, temperatures, fan and a start or end sequence.
- A toolpath for a mill or a lathe, and a body whose section the outline refuses.
