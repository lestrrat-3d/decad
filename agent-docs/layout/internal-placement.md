# decad file layout: internal packages, placement

Internal packages that place bodies: layout on a bed and spacing along a path.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/arrangeplan/` | Shelf packing of rectangular footprints onto a bed with a gap; plain millimetre data. See arrange design. |
| `internal/pathsample/` | Equal-arc-length points along a path of lines and circular arcs; plain millimetre vectors. See path-pattern design. |
