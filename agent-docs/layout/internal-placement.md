# decad file layout: internal packages, placement

Internal packages that place bodies: layout on a bed and spacing along a path.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/arrangeplan/` | Shelf packing of rectangular footprints onto a bed with a gap; plain millimetre data. See arrange design. |
| `internal/pathsample/` | Equal-arc-length points along a path of lines and circular arcs; plain millimetre vectors. See path-pattern design. |
| `internal/glyphs/` | TrueType layout of a line of text and flattening of its glyph outlines to closed polygons. See text design. |
| `internal/igesread/` | IGES 5.3 fixed-format records into entities, delimiters and the unit. See iges-import design. |
| `internal/igesbrep/` | Planar-faced manifold solid B-rep entities into a welded, consistently wound triangle mesh. See iges-import design. |
| `internal/stepread/` | ISO 10303-21 file syntax into entity numbers, names and parameter values. See step-import design. |
| `internal/stepbrep/` | Planar-faced MANIFOLD_SOLID_BREP and BREP_WITH_VOIDS into a welded triangle mesh. See step-import design. |
| `internal/polytri/` | Ear-clipping triangulation of a planar polygon with holes. See step-import design. |

