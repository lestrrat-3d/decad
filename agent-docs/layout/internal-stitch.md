# decad file layout: Stitch packages

Internal packages for Stitch and Unstitch assembly, bounds and mass.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/stitchweld/` | Stitch vertex and edge welding, topology and closure checks. See surface §6.2–6.4. |
| `internal/stitchbounds/` | Bounded component separation, vertex-bound summaries, and unstitch placement boxes. See surface §6.2–6.4. |
